#!/usr/bin/env python3
"""Shoulder gravity moment M(q) and the joint path q_a -> q_b that minimises it (2026-11-18, user's notation).

Chain:  shoulder --theta1--> L1 + Lm (elbow motor m1) --theta2--> L2 --theta3--> L3 (long bracket) -> m2
        phi1 = theta1 + psi, phi2 = phi1 + theta2, phi3 = phi2 + theta3
        psi: angle from the horizontal to theta1 = 0

M = sum_i r_i x F_i, F_i = (0, -m_i g):
M(q) = -g [ M_L1 r1 cos phi1 + m1 (L1 + Lm/2) cos phi1
          + M_L2 ((L1+Lm) cos phi1 + r2 cos phi2)
          + M_L3 ((L1+Lm) cos phi1 + L2 cos phi2 + r3 cos phi3)
          + m2   ((L1+Lm) cos phi1 + L2 cos phi2 + L3 cos phi3) ]

solve_path(qa, qb, P, objective):
  'peak': min_{q_1..q_{N-1}, s} s            s.t. -s <= M(q_i) <= s
  'heat': min_{q_1..q_{N-1}} sum M(q_i)^2 dt + lam sum |q_{i+1}-q_i|^2 / dt
  both:   joint limits, |dtheta| <= w_max dt, z_tip(q_i) >= z_min, x_tip(q_i) >= x_min (optional); SLSQP.

python3 shoulder_path.py              self-test + example (EXAMPLE values, not measured)
python3 shoulder_path.py --rviz       also publish the example path to RViz (/shoulder_path/markers, frame 'shoulder')
"""
import math
import sys

import numpy as np
from scipy.optimize import minimize

G = 9.80665

# EXAMPLE parameters only (not measured): [kg], [m], [rad]. Replace with the real ones.
EXAMPLE = dict(M_L1=0.040, m1=0.082, M_L2=0.030, M_L3=0.050, m2=0.250,
               L1=0.124, Lm=0.0465, L2=0.124, L3=0.100,
               r1=0.062, r2=0.062, r3=0.050, psi=0.0,
               q_min=np.radians([-20.0, -150.0, -150.0]), q_max=np.radians([160.0, 150.0, 150.0]),
               w_max=np.radians([30.0, 60.0, 60.0]))


def rm2(P):
    """Distance from the theta3 axis to m2 along L3. Default L3 (m2 at the tip, the user's notation); the URDF model
    (urdf_arm.py) puts m2 at the gripper's centre of mass while L3 still ends at the TCP."""
    return P.get('rm2', P['L3'])


def phis(q, P):
    q = np.asarray(q, float)
    p1 = q[..., 0] + P['psi']
    p2 = p1 + q[..., 1]
    p3 = p2 + q[..., 2]
    return p1, p2, p3


def positions(q, P):
    """r_i (x, z) of the 5 masses [M_L1, m1, M_L2, M_L3, m2], plus elbow, theta3 axis and tip. q: (3,)."""
    p1, p2, p3 = phis(q, P)
    e1, e2, e3 = (np.array([math.cos(a), math.sin(a)]) for a in (p1, p2, p3))
    E = (P['L1'] + P['Lm']) * e1
    W = E + P['L2'] * e2
    r = [P['r1'] * e1, (P['L1'] + P['Lm'] / 2) * e1, E + P['r2'] * e2, W + P['r3'] * e3, W + rm2(P) * e3]
    return r, dict(L1_end=P['L1'] * e1, elbow=E, joint3=W, tip=W + P['L3'] * e3)


def masses(P):
    return [P['M_L1'], P['m1'], P['M_L2'], P['M_L3'], P['m2']]


def M_of(q, P):
    """Shoulder moment M = sum r_i x F_i [N m] (written out, no lumped coefficients). q: (..., 3)."""
    p1, p2, p3 = phis(q, P)
    a = P['L1'] + P['Lm']
    c1, c2, c3 = np.cos(p1), np.cos(p2), np.cos(p3)
    return -G * (P['M_L1'] * P['r1'] * c1 + P['m1'] * (P['L1'] + P['Lm'] / 2) * c1
                 + P['M_L2'] * (a * c1 + P['r2'] * c2)
                 + P['M_L3'] * (a * c1 + P['L2'] * c2 + P['r3'] * c3)
                 + P['m2'] * (a * c1 + P['L2'] * c2 + rm2(P) * c3))


def M_cross(q, P):
    """Same M from the vectors: sum (r_x F_z - r_z F_x). Used to check M_of."""
    r, _ = positions(q, P)
    return sum(ri[0] * (-m * G) - ri[1] * 0.0 for ri, m in zip(r, masses(P)))


def dM(q, P):
    """[dM/dtheta1, dM/dtheta2, dM/dtheta3]."""
    p1, p2, p3 = phis(q, P)
    a = P['L1'] + P['Lm']
    s1, s2, s3 = math.sin(p1), math.sin(p2), math.sin(p3)
    d3 = G * (P['M_L3'] * P['r3'] * s3 + P['m2'] * rm2(P) * s3)
    d2 = G * (P['M_L2'] * P['r2'] * s2 + P['M_L3'] * (P['L2'] * s2 + P['r3'] * s3)
              + P['m2'] * (P['L2'] * s2 + rm2(P) * s3))
    d1 = G * (P['M_L1'] * P['r1'] * s1 + P['m1'] * (P['L1'] + P['Lm'] / 2) * s1
              + P['M_L2'] * (a * s1 + P['r2'] * s2)
              + P['M_L3'] * (a * s1 + P['L2'] * s2 + P['r3'] * s3)
              + P['m2'] * (a * s1 + P['L2'] * s2 + rm2(P) * s3))
    return np.array([d1, d2, d3])


# ---------------------------------------------------------------- dynamics (2026-11-19)
# Each point mass sits at r_i = sum_k a_ik u(phi_k), u(phi) = (cos phi, sin phi). Rows: [M_L1, m1, M_L2, M_L3, m2].
def arm_coeffs(P):
    a = P['L1'] + P['Lm']
    return np.array([[P['r1'], 0.0, 0.0],
                     [P['L1'] + P['Lm'] / 2, 0.0, 0.0],
                     [a, P['r2'], 0.0],
                     [a, P['L2'], P['r3']],
                     [a, P['L2'], rm2(P)]])


def link_spin(P):
    """Own (spin) inertia of each link about its centre of mass [kg m^2], uniform rod M L^2 / 12, unless P gives
    I_L1..I_L3. Point masses m1, m2 have none here. J_r: shoulder rotor inertia seen at the output (N^2 J_rotor)."""
    return np.array([P.get('I_L1', P['M_L1'] * P['L1'] ** 2 / 12),
                     P.get('I_L2', P['M_L2'] * P['L2'] ** 2 / 12),
                     P.get('I_L3', P['M_L3'] * P['L3'] ** 2 / 12)])


def M_dyn(q, qd, qdd, P):
    """Shoulder load with motion [N m]: M_dyn = sum r_i x (F_i - m_i r_i'') - sum I_Lk phi_k'' - J_r theta1''.
    The shoulder motor gives tau = -M_dyn (as -M at rest; qd = qdd = 0 gives M_dyn = M).
      r_i x r_i'' = sum_k sum_j a_ik a_ij [ phi_j'' cos(phi_j - phi_k) - phi_j'^2 sin(phi_j - phi_k) ]
      phi1' = theta1', phi2' = theta1' + theta2', phi3' = theta1' + theta2' + theta3' (same for '').
    q, qd, qdd: (..., 3) [rad, rad/s, rad/s^2]."""
    q, qd, qdd = (np.asarray(v, float) for v in (q, qd, qdd))
    ph = np.stack(phis(q, P), axis=-1)
    w = np.cumsum(qd, axis=-1)
    al = np.cumsum(qdd, axis=-1)
    D = ph[..., None, :] - ph[..., :, None]           # D[..., k, j] = phi_j - phi_k
    Kc = al[..., None, :] * np.cos(D) - (w ** 2)[..., None, :] * np.sin(D)
    W = sum(m * np.outer(a, a) for m, a in zip(masses(P), arm_coeffs(P)))   # sum_i m_i a_ik a_ij
    inertial = np.sum(W * Kc, axis=(-2, -1))
    return M_of(q, P) - inertial - al @ link_spin(P) - P.get('J_r', 0.0) * qdd[..., 0]


def M_dyn_path(Q, t, P):
    """M_dyn along a sampled path (rest at both ends), speeds and accelerations by central differences."""
    Q, t = np.asarray(Q, float), np.asarray(t, float)
    qd = np.gradient(Q, t, axis=0)
    qd[0] = qd[-1] = 0.0
    qdd = np.gradient(qd, t, axis=0)
    return M_dyn(Q, qd, qdd, P), qd, qdd


def rest_derivs(Q, dt):
    """Speeds and accelerations of an evenly sampled path that starts and ends at rest (ghost points q_-1 = q_1,
    q_N+1 = q_N-1):  qd_i = (q_i+1 - q_i-1) / 2dt,  qdd_i = (q_i+1 - 2 q_i + q_i-1) / dt^2."""
    Q = np.asarray(Q, float)
    G_ = np.vstack([Q[1], Q, Q[-2]])
    return (G_[2:] - G_[:-2]) / (2 * dt), (G_[2:] - 2 * Q + G_[:-2]) / dt ** 2


def M_dyn_rest(Q, dt, P):
    qd, qdd = rest_derivs(Q, dt)
    return M_dyn(Q, qd, qdd, P)


def dM_dyn_rest(Q, dt, P, h=1e-6):
    """d M_dyn_i / d q_k for an evenly sampled rest-to-rest path: (N+1, N+1, 3), only k = i-1, i, i+1 are nonzero."""
    Q = np.asarray(Q, float)
    n = len(Q)
    qd, qdd = rest_derivs(Q, dt)
    J = np.zeros((n, n, 3))
    E = np.eye(3) * h
    gq = np.stack([M_dyn(Q + e, qd, qdd, P) - M_dyn(Q - e, qd, qdd, P) for e in E], -1) / (2 * h)   # (n, 3)
    gv = np.stack([M_dyn(Q, qd + e, qdd, P) - M_dyn(Q, qd - e, qdd, P) for e in E], -1) / (2 * h)
    ga = np.stack([M_dyn(Q, qd, qdd + e, P) - M_dyn(Q, qd, qdd - e, P) for e in E], -1) / (2 * h)
    for i in range(n):
        J[i, i] += gq[i] - 2 * ga[i] / dt ** 2
        lo, hi = (i + 1 if i == 0 else i - 1), (i - 1 if i == n - 1 else i + 1)  # ghost points mirror
        J[i, lo] += -gv[i] / (2 * dt) + ga[i] / dt ** 2
        J[i, hi] += gv[i] / (2 * dt) + ga[i] / dt ** 2
    return J


def z_tip(q, P):
    p1, p2, p3 = phis(q, P)
    return (P['L1'] + P['Lm']) * math.sin(p1) + P['L2'] * math.sin(p2) + P['L3'] * math.sin(p3)


def x_tip(q, P):
    """Horizontal position of the L3 end from the shoulder axis (+ = the side phi = 0 points to)."""
    p1, p2, p3 = phis(q, P)
    return (P['L1'] + P['Lm']) * math.cos(p1) + P['L2'] * math.cos(p2) + P['L3'] * math.cos(p3)


def dx_tip(q, P):
    p1, p2, p3 = phis(q, P)
    s3 = P['L3'] * math.sin(p3)
    s2 = P['L2'] * math.sin(p2) + s3
    return -np.array([(P['L1'] + P['Lm']) * math.sin(p1) + s2, s2, s3])


def dz_tip(q, P):
    p1, p2, p3 = phis(q, P)
    c3 = P['L3'] * math.cos(p3)
    c2 = P['L2'] * math.cos(p2) + c3
    return np.array([(P['L1'] + P['Lm']) * math.cos(p1) + c2, c2, c3])


def solve_path(qa, qb, P, objective='peak', T=4.0, N=40, z_min=None, lam=1e-3, inits=None, x_min=None, dynamic=False,
               z_max=None, z_max_x_slope=0.0, z_max_x_start=0.0):
    """Joint path qa -> qb (rad, (3,)) over T seconds in N steps minimising the shoulder moment.

    objective 'peak': max_i |M(q_i)|;  'heat': sum M^2 dt + lam sum |dq|^2 / dt.
    dynamic (peak only): bound M_dyn(q_i, qd_i, qdd_i) instead of M, rest at both ends (rest_derivs).
    Returns dict(path (N+1, 3), t, M, peak, heat, success, message)."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    dt, m = T / N, N - 1
    dmax = np.asarray(P['w_max']) * dt
    if np.any(np.abs(qb - qa) > dmax * N + 1e-9):
        raise ValueError('T too short for w_max')
    peak_mode = objective == 'peak'
    off = 1 if peak_mode else 0

    def path(x):
        return np.vstack([qa, x[off:].reshape(m, 3), qb])

    def f(x):
        Q = path(x)
        smooth = lam * np.sum(np.diff(Q, axis=0) ** 2) / dt
        return (x[0] if peak_mode else np.sum(M_of(Q, P) ** 2) * dt) + smooth

    def df(x):
        Q = path(x)
        g = np.zeros_like(Q)
        d = np.diff(Q, axis=0)
        g[:-1] -= 2 * lam * d / dt
        g[1:] += 2 * lam * d / dt
        if not peak_mode:
            for i in range(1, N):
                g[i] += 2 * M_of(Q[i], P) * dM(Q[i], P) * dt
        return np.concatenate([[1.0] if peak_mode else [], g[1:-1].ravel()])

    nv = off + 3 * m

    def col(i):  # first variable index of q_i (interior only)
        return off + 3 * (i - 1)

    def cons(x):
        Q = path(x)
        c = [(dmax ** 2 - np.diff(Q, axis=0) ** 2).ravel()]
        if peak_mode:
            Mi = M_dyn_rest(Q, dt, P) if dynamic else M_of(Q, P)
            c += [x[0] - Mi, x[0] + Mi]
        if z_min is not None:
            c.append(np.array([z_tip(q, P) for q in Q[1:-1]]) - z_min)
        if z_max is not None:
            samples = np.vstack([Q[1:-1], (Q[:-1] + Q[1:]) / 2])
            c.append(np.array([z_max + z_max_x_slope * max(0.0, x_tip(q, P) - z_max_x_start)
                               - z_tip(q, P) for q in samples]))
        if x_min is not None:  # tip never further back than x_min (e.g. the rearmost carousel cell centre)
            c.append(np.array([x_tip(q, P) for q in Q[1:-1]]) - x_min)
        return np.concatenate(c)

    def dcons(x):
        Q = path(x)
        d = np.diff(Q, axis=0)
        J = []
        Jd = np.zeros((3 * N, nv))
        for i in range(N):  # step i: q_{i+1} - q_i
            for j in range(3):
                if i >= 1:
                    Jd[3 * i + j, col(i) + j] = 2 * d[i, j]
                if i + 1 <= m:
                    Jd[3 * i + j, col(i + 1) + j] = -2 * d[i, j]
        J.append(Jd)
        if peak_mode:
            Jm = np.zeros((N + 1, nv))
            if dynamic:
                Jd3 = dM_dyn_rest(Q, dt, P)
                for k in range(1, N):
                    Jm[:, col(k):col(k) + 3] = Jd3[:, k, :]
            else:
                for i in range(1, N):
                    Jm[i, col(i):col(i) + 3] = dM(Q[i], P)
            one = np.zeros((N + 1, nv)); one[:, 0] = 1
            J += [one - Jm, one + Jm]
        if z_min is not None:
            Jz = np.zeros((m, nv))
            for i in range(1, N):
                Jz[i - 1, col(i):col(i) + 3] = dz_tip(Q[i], P)
            J.append(Jz)
        if z_max is not None:
            def upper_grad(q):
                g = -dz_tip(q, P)
                if x_tip(q, P) > z_max_x_start:
                    g += z_max_x_slope * dx_tip(q, P)
                return g

            Jupper = np.zeros((m + N, nv))
            for i in range(1, N):
                Jupper[i - 1, col(i):col(i) + 3] = upper_grad(Q[i])
            for i in range(N):
                g = upper_grad((Q[i] + Q[i + 1]) / 2) / 2
                if i >= 1:
                    Jupper[m + i, col(i):col(i) + 3] = g
                if i + 1 <= m:
                    Jupper[m + i, col(i + 1):col(i + 1) + 3] = g
            J.append(Jupper)
        if x_min is not None:
            Jx = np.zeros((m, nv))
            for i in range(1, N):
                Jx[i - 1, col(i):col(i) + 3] = dx_tip(Q[i], P)
            J.append(Jx)
        return np.vstack(J)

    # the arm holds still at qa before and at qb after the move: s never below those static holds (with dynamic, the
    # end accelerations alone could make M_dyn there smaller than the hold that follows)
    hold = max(abs(M_of(qa, P)), abs(M_of(qb, P)))
    bounds = ([(hold if dynamic else 0, None)] if peak_mode else []) + [(lo, hi) for _ in range(m)
                                                  for lo, hi in zip(P['q_min'], P['q_max'])]
    if inits is None:
        k = N // 2  # straight joint line, and "elbow/wrist first, shoulder last"
        mid = np.array([qa[0], qb[1], qb[2]])
        inits = [np.linspace(qa, qb, N + 1),
                 np.vstack([np.linspace(qa, mid, k + 1), np.linspace(mid, qb, N - k + 1)[1:]])]
    best = None
    for Q0 in inits:
        Mof = (lambda Q_: M_dyn_rest(Q_, dt, P)) if (peak_mode and dynamic) else (lambda Q_: M_of(Q_, P))
        x0 = np.concatenate([[max(np.abs(Mof(Q0)).max(), hold)] if peak_mode else [], Q0[1:-1].ravel()])
        r = minimize(f, x0, jac=df, method='SLSQP', bounds=bounds,
                     constraints=[{'type': 'ineq', 'fun': cons, 'jac': dcons}],
                     options={'maxiter': 500, 'ftol': 1e-12})
        feas = np.all(cons(r.x) > -1e-6)
        Q = path(r.x)
        Ms = Mof(Q)
        score = max(np.abs(Ms).max(), hold) if peak_mode else np.sum(Ms ** 2) * dt
        if feas and (best is None or score < best[0]):
            best = (score, Q, r)
    if best is None:
        return dict(success=False, message='no feasible path')
    _, Q, r = best
    Ms = Mof(Q)
    return dict(path=Q, t=np.linspace(0, T, N + 1), M=Ms, peak=float(max(np.abs(Ms).max(), hold)),
                heat=float(np.sum(Ms ** 2) * dt), success=True, message=r.message)


def solve_path_timed(qa, qb, P, N=20, lam=1e-3, rho=0.01, T0=3.0, T_max=20.0, z_min=None, x_min=None):
    """MinHeat with the move time T free, plus a time term:

        min_{T, q_1..q_{N-1}}  sum_i M(q_i)^2 dt  +  lam sum_i |q_{i+1}-q_i|^2 / dt  +  rho T,     dt = T / N
        s.t. |q_{i+1,j}-q_{i,j}| <= w_max_j dt,  limits,  z_tip >= z_min,  T_min <= T <= T_max

    M^2 dt grows with T (holding longer), the smoothness term falls with T (slower = gentler), rho T charges the
    time itself: rho [N^2 m^2] = how much heat one second is worth. Returns solve_path's dict plus T."""
    qa, qb = np.asarray(qa, float), np.asarray(qb, float)
    w = np.asarray(P['w_max'])
    T_min = max(1.0001 * np.max(np.abs(qb - qa) / w), 1e-3)
    T_max = max(T_max, 1.5 * T_min)
    m = N - 1

    def unpack(x):
        return x[0], np.vstack([qa, x[1:].reshape(m, 3), qb])

    def f(x):
        T, Q = unpack(x)
        dt = T / N
        return np.sum(M_of(Q, P) ** 2) * dt + lam * np.sum(np.diff(Q, axis=0) ** 2) / dt + rho * T

    def df(x):
        T, Q = unpack(x)
        dt = T / N
        Ms = M_of(Q, P)
        d = np.diff(Q, axis=0)
        g = np.zeros_like(Q)
        g[:-1] -= 2 * lam * d / dt
        g[1:] += 2 * lam * d / dt
        for i in range(1, N):
            g[i] += 2 * Ms[i] * dM(Q[i], P) * dt
        gT = np.sum(Ms ** 2) / N - lam * np.sum(d ** 2) * N / T ** 2 + rho
        return np.concatenate([[gT], g[1:-1].ravel()])

    def col(i):
        return 1 + 3 * (i - 1)

    def cons(x):
        T, Q = unpack(x)
        c = [((w * T / N) ** 2 - np.diff(Q, axis=0) ** 2).ravel()]
        if z_min is not None:
            c.append(np.array([z_tip(q, P) for q in Q[1:-1]]) - z_min)
        if x_min is not None:  # tip never further back than x_min (e.g. the rearmost carousel cell centre)
            c.append(np.array([x_tip(q, P) for q in Q[1:-1]]) - x_min)
        return np.concatenate(c)

    def dcons(x):
        T, Q = unpack(x)
        d = np.diff(Q, axis=0)
        nv = 1 + 3 * m
        Jd = np.zeros((3 * N, nv))
        for i in range(N):
            for j in range(3):
                Jd[3 * i + j, 0] = 2 * w[j] ** 2 * T / N ** 2
                if i >= 1:
                    Jd[3 * i + j, col(i) + j] = 2 * d[i, j]
                if i + 1 <= m:
                    Jd[3 * i + j, col(i + 1) + j] = -2 * d[i, j]
        J = [Jd]
        if z_min is not None:
            Jz = np.zeros((m, nv))
            for i in range(1, N):
                Jz[i - 1, col(i):col(i) + 3] = dz_tip(Q[i], P)
            J.append(Jz)
        if x_min is not None:
            Jx = np.zeros((m, nv))
            for i in range(1, N):
                Jx[i - 1, col(i):col(i) + 3] = dx_tip(Q[i], P)
            J.append(Jx)
        return np.vstack(J)

    bounds = [(T_min, T_max)] + [(lo, hi) for _ in range(m) for lo, hi in zip(P['q_min'], P['q_max'])]
    T0 = float(np.clip(T0, T_min, T_max))
    best = None
    for Q0 in (np.linspace(qa, qb, N + 1),):
        x0 = np.concatenate([[T0], Q0[1:-1].ravel()])
        r = minimize(f, x0, jac=df, method='SLSQP', bounds=bounds,
                     constraints=[{'type': 'ineq', 'fun': cons, 'jac': dcons}],
                     options={'maxiter': 500, 'ftol': 1e-12})
        if np.all(cons(r.x) > -1e-6) and (best is None or r.fun < best.fun):
            best = r
    if best is None:
        return dict(success=False, message='no feasible path')
    T, Q = unpack(best.x)
    Ms = M_of(Q, P)
    return dict(path=Q, t=np.linspace(0, T, N + 1), M=Ms, T=float(T), peak=float(np.abs(Ms).max()),
                heat=float(np.sum(Ms ** 2) * T / N), success=True, message=best.message)


# ---------------------------------------------------------------- IK candidates

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def ik_all(tip, P, tool=None, step_deg=1.0):
    """All joint solutions (theta1, theta2, theta3) putting the L3 end at tip = (x, z) [m, shoulder frame], inside
    q_min..q_max. tool: absolute tool angle phi3 [rad] from the horizontal (fixed tool, e.g. a grasp: two solutions,
    elbow up / down) or None (free tool: phi3 scanned every step_deg, two solutions each).
    Each item: (q (3,), phi3, 'up'|'down')."""
    a, b = P['L1'] + P['Lm'], P['L2']
    tip = np.asarray(tip, float)
    phi3s = [tool] if tool is not None else np.radians(np.arange(-180.0, 180.0, step_deg))
    out = []
    for f3 in phi3s:
        W = tip - P['L3'] * np.array([math.cos(f3), math.sin(f3)])
        c2 = (W @ W - a * a - b * b) / (2 * a * b)
        if abs(c2) > 1:
            continue
        for name, t2 in (('up', -math.acos(c2)), ('down', math.acos(c2))):
            f1 = math.atan2(W[1], W[0]) - math.atan2(b * math.sin(t2), a + b * math.cos(t2))
            q = np.array([wrap(f1 - P['psi']), t2, wrap(f3 - f1 - t2)])
            if np.all(q >= P['q_min']) and np.all(q <= P['q_max']):
                out.append((q, f3, name))
    return out


def ik_default(tip, P, tool):
    """The plain IK answer: elbow 'up' at the given tool angle (no load reasoning). None if unreachable."""
    for q, _, name in ik_all(tip, P, tool):
        if name == 'up':
            return q
    sols = ik_all(tip, P, tool)
    return sols[0][0] if sols else None


def ik_min_M(tip, P, tool=None):
    """Of ik_all, the solution with the smallest |M| (sets the lower bound of any path's peak)."""
    sols = ik_all(tip, P, tool)
    if not sols:
        return None
    return min(sols, key=lambda s: abs(M_of(s[0], P)))


# ---------------------------------------------------------------- pick and load scenario

def with_payload(P, m_puck):
    """P with the puck added to m2 (wrist + end effector carry it)."""
    Q = dict(P)
    Q['m2'] = P['m2'] + m_puck
    return Q


def wp_candidates(wp, P, step_deg=5.0):
    """IK candidates of one waypoint {tip, tool [rad], free, tool_range [rad, rad]}: list of q."""
    if not wp['free']:
        return [q for q, _, _ in ik_all(wp['tip'], P, wp['tool'])]
    lo, hi = wp['tool_range']
    out = []
    for f3 in np.arange(lo, hi + 1e-9, math.radians(step_deg)):
        out += [q for q, _, _ in ik_all(wp['tip'], P, f3)]
    return out


def choose_ik(wps, Ps, mu=0.05, mode='peak'):
    """One IK solution per waypoint, by dynamic programming over the candidates.
    mode 'peak': minimise  max_k |M_k(q_k)|  (the hold the paths cannot get under), ties broken by
                 sum_k |M_k| + mu * sum_k |q_{k+1} - q_k|;
    mode 'sum':  minimise  sum_k |M_k(q_k)| + mu * sum_k |q_{k+1} - q_k|.
    wps: waypoints, Ps: the P (payload) at each waypoint. Returns list of q, or raises if a tip is out of reach."""
    cands = []
    for wp, P in zip(wps, Ps):
        c = wp_candidates(wp, P)
        if not c:
            raise ValueError(f"{wp['name']}: tip out of reach")
        cands.append(np.array(c))
    absM = [np.abs(M_of(c, P)) for c, P in zip(cands, Ps)]
    peak, add = absM[0].copy(), absM[0].copy()   # max so far, additive cost so far
    back = []
    for k in range(1, len(wps)):
        step = np.linalg.norm(cands[k][:, None, :] - cands[k - 1][None, :, :], axis=2)  # (n_k, n_{k-1})
        add_t = add[None, :] + mu * step
        if mode == 'peak':
            peak_t = np.maximum(peak[None, :], absM[k][:, None])
            key = peak_t + 1e-6 * add_t      # peak first, the additive cost only between equal peaks
        else:
            peak_t, key = np.maximum(peak[None, :], absM[k][:, None]), add_t
        b = np.argmin(key, axis=1)
        r = np.arange(len(b))
        back.append(b)
        peak, add = peak_t[r, b], add_t[r, b] + absM[k]
    i = int(np.argmin(peak + 1e-6 * add if mode == 'peak' else add))
    pick = [i]
    for b in reversed(back):
        i = int(b[i]); pick.append(i)
    pick.reverse()
    return [cands[k][i] for k, i in enumerate(pick)]


def plain_ik(wps, Ps):
    """Plain IK per waypoint, no load reasoning: tool = the given angle (free: the middle of tool_range, else the
    nearest reachable angle in 1 deg steps); of the elbow up / down solutions the first waypoint takes 'up', every
    later one the solution nearest the previous waypoint's joints (as a robot solving IK move by move would; taking
    'up' everywhere flipped the elbow between above_slot and place, 2026-11-18)."""
    out = []
    for wp, P in zip(wps, Ps):
        if not wp['free']:
            tools = [wp['tool']]
        else:
            mid, half = sum(wp['tool_range']) / 2, (wp['tool_range'][1] - wp['tool_range'][0]) / 2
            tools = [mid]
            for d in np.radians(np.arange(1.0, math.degrees(half) + 1e-9, 1.0)):
                tools += [mid + d, mid - d]
        q = None
        for tool in tools:
            sols = ik_all(wp['tip'], P, tool)
            if not sols:
                continue
            if not out:
                q = next((s_[0] for s_ in sols if s_[2] == 'up'), sols[0][0])
            else:
                q = min((s_[0] for s_ in sols), key=lambda c: np.linalg.norm(c - out[-1]))
            break
        if q is None:
            raise ValueError(f"{wp['name']}: tip out of reach with the plain tool angle")
        out.append(q)
    return out


def payload_flags(wps):
    """carrying[k]: the puck is held while moving from waypoint k to k+1 and while holding waypoint k+1.
    grasp at a waypoint: held from it on; release: not held after it."""
    held, flags = False, []
    for wp in wps:
        if wp.get('grasp'):
            held = True
        if wp.get('release'):
            held = False
        flags.append(held)
    return flags


def scenario_path(wps, P, m_puck, qs, objective, N_seg=20, z_min=None, lam=1e-3, rho=None, x_min=None, dynamic=False):
    """Joint path through qs (one per waypoint), segment k -> k+1 solved with the payload of segment k.
    objective None: straight joints at the yaml T; 'heat': MinHeat at the yaml T; with rho (not None) and 'heat':
    the time is free too (solve_path_timed, yaml T as the start guess).
    Returns (Q (n, 3), carrying (n,), t (n,) [s], segment start indices, message)."""
    flags = payload_flags(wps)
    Q, carry, t, starts, msg = [qs[0]], [flags[0]], [0.0], [0], ''
    for k in range(len(wps) - 1):
        Pk = with_payload(P, m_puck) if flags[k] else P
        T = float(wps[k + 1].get('T', 3.0))
        need = np.max(np.abs(qs[k + 1] - qs[k]) / np.asarray(P['w_max']))
        T = max(T, 1.05 * need)  # never faster than the joint speed limit allows
        line = np.linspace(qs[k], qs[k + 1], N_seg + 1)
        if objective is None or wps[k + 1].get('from_') == wps[k]['name'] or wps[k].get('from_') == wps[k + 1]['name']:
            seg = line   # straight joints (a shoulder-only move stays shoulder-only in every lane)
        else:
            if rho is not None:
                r = solve_path_timed(qs[k], qs[k + 1], Pk, N_seg, lam, rho, T0=T, z_min=z_min, x_min=x_min)
            else:
                r = solve_path(qs[k], qs[k + 1], Pk, objective, T, N_seg, z_min=z_min, lam=lam, x_min=x_min,
                               dynamic=dynamic)
            seg = r['path'] if r['success'] else line
            if r['success'] and rho is not None:
                T = r['T']
            if not r['success']:
                msg = f"{wps[k + 1]['name']}: {r['message']}"
        Q += list(seg[1:])
        carry += [flags[k]] * N_seg
        t += list(t[-1] + np.linspace(0, T, N_seg + 1)[1:])
        starts.append(len(Q) - 1)
    return np.array(Q), np.array(carry), np.array(t), starts, msg


def M_dyn_scenario(Q, carry, t, starts, P, m_puck):
    """M_dyn along a scenario path: every segment (starts[k]..starts[k+1]) rests at both ends."""
    out = np.zeros(len(Q))
    P1 = with_payload(P, m_puck)
    for k in range(len(starts) - 1):
        a, b = starts[k], starts[k + 1]
        Pk = P1 if carry[a + 1] else P
        seg = M_dyn_rest(Q[a:b + 1], (t[b] - t[a]) / (b - a), Pk)
        if k > 0 and abs(out[a]) > abs(seg[0]):  # a waypoint ends one segment and starts the next: keep the larger
            seg[0] = out[a]
        out[a:b + 1] = seg
    for a in starts:  # the arm holds still at every waypoint: never report less than that static hold
        for Pk in (P, P1) if m_puck else (P,):
            if (Pk is P1) == bool(carry[a] or carry[min(a + 1, len(Q) - 1)]) and abs(M_of(Q[a], Pk)) > abs(out[a]):
                out[a] = M_of(Q[a], Pk)
    return out


def M_scenario(Q, carry, P, m_puck):
    P1 = with_payload(P, m_puck)
    return np.where(carry, M_of(Q, P1), M_of(Q, P))


def report(name, Q, P, T):
    Ms = M_of(Q, P)
    dt = T / (len(Q) - 1)
    print(f'{name:22s} peak |M| {np.abs(Ms).max():.4f} N m   int M^2 dt {np.sum(Ms**2)*dt:.5f}   '
          f'z_tip min {min(z_tip(q, P) for q in Q)*1000:.0f} mm')


def selftest(P):
    rng = np.random.default_rng(1)
    for _ in range(200):
        q = rng.uniform(P['q_min'], P['q_max'])
        assert abs(M_of(q, P) - M_cross(q, P)) < 1e-12
        num = [(M_of(q + h, P) - M_of(q - h, P)) / 2e-7 for h in np.eye(3) * 1e-7]
        assert np.allclose(num, dM(q, P), atol=1e-6)
        numx = [(x_tip(q + h, P) - x_tip(q - h, P)) / 2e-7 for h in np.eye(3) * 1e-7]
        assert np.allclose(numx, dx_tip(q, P), atol=1e-8)
        assert abs(x_tip(q, P) - positions(q, P)[1]['tip'][0]) < 1e-12
        numz = [(z_tip(q + h, P) - z_tip(q - h, P)) / 2e-7 for h in np.eye(3) * 1e-7]
        assert np.allclose(numz, dz_tip(q, P), atol=1e-8)
    for _ in range(100):
        q = rng.uniform(P['q_min'], P['q_max'])
        tip = positions(q, P)[1]['tip']
        f3 = phis(q, P)[2]
        sols = ik_all(tip, P, f3)
        assert sols and all(np.allclose(positions(s[0], P)[1]['tip'], tip, atol=1e-9) for s in sols)
        assert any(np.allclose(wrap(s[0] - q), 0, atol=1e-9) for s in sols)
    # M_dyn against the rate of change of angular momentum, built only from positions (finite differences in time)
    A, ms, I = arm_coeffs(P), masses(P), link_spin(P)
    for _ in range(50):
        q0, w0, a0 = rng.uniform(-1, 1, (3, 3)) * [[2], [1.5], [4]]
        qt = lambda tt: q0 + w0 * tt + 0.5 * a0 * tt ** 2

        def Lmom(tt, h=1e-5):  # shoulder-axis angular momentum: sum m r x r' + sum I phi' + J_r theta1'
            r0 = positions(qt(tt), P)[0]
            rp, rm = positions(qt(tt + h), P)[0], positions(qt(tt - h), P)[0]
            lin = sum(m * (ri[0] * (a[1] - b[1]) - ri[1] * (a[0] - b[0])) / (2 * h)
                      for m, ri, a, b in zip(ms, r0, rp, rm))
            return lin + I @ np.cumsum(w0 + a0 * tt) + P.get('J_r', 0.0) * (w0[0] + a0[0] * tt)
        dL = (Lmom(1e-4) - Lmom(-1e-4)) / 2e-4
        assert abs(M_dyn(q0, w0, a0, P) - (M_of(q0, P) - dL)) < 1e-6, (M_dyn(q0, w0, a0, P), M_of(q0, P) - dL)
        assert abs(M_dyn(q0, np.zeros(3), np.zeros(3), P) - M_of(q0, P)) < 1e-12
    print('selftest ok: M formula = sum r x F, dM, dz_tip, IK, M_dyn = M - dL/dt')


def example():
    P = EXAMPLE
    selftest(P)
    qa = np.radians([30.0, 60.0, -90.0])   # start: arm folded up, tool down
    qb = np.radians([120.0, -30.0, -60.0])  # end: reaching over to the other side
    T, N = 6.0, 40
    lin = np.linspace(qa, qb, N + 1)
    print('EXAMPLE values (not measured)')
    report('straight joints', lin, P, T)
    out = {}
    for obj in ('peak', 'heat'):
        r = solve_path(qa, qb, P, obj, T, N, z_min=0.0)
        assert r['success'], r['message']
        report(f'optimised ({obj})', r['path'], P, T)
        out[obj] = r
    lb = max(abs(M_of(qa, P)), abs(M_of(qb, P)))
    print(f'lower bound (end holds) {lb:.4f} N m')
    for i in range(0, N + 1, 8):
        print(f'  t {out["peak"]["t"][i]:4.1f}s  q[deg] {np.round(np.degrees(out["peak"]["path"][i]), 1)}'
              f'  M {out["peak"]["M"][i]:+.4f}')
    return P, lin, out, T


def rviz(P, paths, T):
    """Loop the paths on /shoulder_path/markers (MarkerArray, frame 'shoulder', x horizontal, z up)."""
    import rclpy
    from geometry_msgs.msg import Point
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init()
    node = rclpy.create_node('shoulder_path_viz')
    pub = node.create_publisher(MarkerArray, '/shoulder_path/markers', 1)
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    tf = TransformStamped()  # 'shoulder' must exist in TF or RViz shows nothing (fixed frame error)
    tf.header.stamp = node.get_clock().now().to_msg()
    tf.header.frame_id, tf.child_frame_id = 'world', 'shoulder'
    tf.transform.rotation.w = 1.0
    static_tf = StaticTransformBroadcaster(node)
    static_tf.sendTransform(tf)
    names = list(paths)
    colours = {'straight': (0.6, 0.6, 0.6), 'peak': (0.1, 0.6, 0.45), 'heat': (0.85, 0.35, 0.2)}
    offsets = {n: 0.5 * k for k, n in enumerate(names)}  # side by side along y

    def pt(xz, y):
        return Point(x=float(xz[0]), y=float(y), z=float(xz[1]))

    def mk(mid, ns, typ, rgb, scale, a=1.0):
        m = Marker()
        m.header.frame_id = 'shoulder'
        m.header.stamp = node.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = float(scale)
        m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in (*rgb, a))
        return m

    n_frames = len(next(iter(paths.values())))
    k = 0
    print('publishing /shoulder_path/markers (frame shoulder); Ctrl-C to stop')
    try:
        while rclpy.ok():
            arr = MarkerArray()
            mid = 0
            for name in names:
                Q, y, rgb = paths[name], offsets[name], colours[name.split()[0]]
                q = Q[k]
                r, J = positions(q, P)
                arm = mk(mid, name, Marker.LINE_STRIP, rgb, 0.012); mid += 1
                arm.points = [pt((0, 0), y), pt(J['L1_end'], y), pt(J['elbow'], y), pt(J['joint3'], y), pt(J['tip'], y)]
                trail = mk(mid, name, Marker.LINE_STRIP, rgb, 0.003, 0.5); mid += 1
                trail.points = [pt(positions(qq, P)[1]['tip'], y) for qq in Q[:k + 1]]
                arr.markers += [arm, trail]
                for ri, mi in zip(r, masses(P)):
                    s = mk(mid, name, Marker.SPHERE, (0.85, 0.35, 0.2), 0.012 + 0.08 * mi); mid += 1
                    s.pose.position = pt(ri, y)
                    rv = mk(mid, name, Marker.ARROW, (0.1, 0.6, 0.45), 0.004, 0.7); mid += 1
                    rv.scale.y, rv.scale.z = 0.008, 0.01
                    rv.points = [pt((0, 0), y), pt(ri, y)]
                    fv = mk(mid, name, Marker.ARROW, (0.85, 0.35, 0.2), 0.004); mid += 1
                    fv.scale.y, fv.scale.z = 0.008, 0.01
                    fv.points = [pt(ri, y), pt((ri[0], ri[1] - 0.1 * mi / 0.1), y)]
                    arr.markers += [s, rv, fv]
                txt = mk(mid, name, Marker.TEXT_VIEW_FACING, (1, 1, 1), 0.03); mid += 1
                txt.pose.position = pt((0, 0.45), y)
                txt.text = f'{name}  M = {M_of(q, P):+.3f} N m'
                arr.markers.append(txt)
            pub.publish(arr)
            k = (k + 1) % n_frames
            rclpy.spin_once(node, timeout_sec=T / (n_frames - 1))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    P, lin, out, T = example()
    if '--rviz' in sys.argv:
        rviz(P, {'straight joints': lin, 'peak (minimax)': out['peak']['path'],
                 'heat (int M^2)': out['heat']['path']}, T)
