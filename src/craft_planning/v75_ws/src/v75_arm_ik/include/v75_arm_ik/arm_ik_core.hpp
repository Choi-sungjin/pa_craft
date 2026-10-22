// URDF 기반 팔 IK 코어 (header-only). CLI(arm_ik)와 ROS 노드(ik_rviz_node)가 함께 사용.
//
// 로봇 치수는 코드에 없다. 링크 길이, 관절 위치·축·한계, 끝점(TCP)을 모두 URDF에서 읽는다.
//   - 끝점: URDF의 `tool_tcp` 링크 (없으면 tool_roll_link)
//   - IK 관절: 기준 링크(base_link)에서 끝점까지 이어진 관절 체인을 자동 탐색
//   - 체인 밖 관절은 q=0으로 고정
#pragma once

// Pinocchio 헤더는 다른 헤더(Boost 사용)보다 먼저 포함해야 함
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/config.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

namespace v75 {

namespace pin = pinocchio;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

constexpr double kDeg = M_PI / 180.0;

struct IkParams {
  double pos_tol = 1e-4;   // [m]   0.1 mm
  double rot_tol = 1e-3;   // [rad] 약 0.06°
  int max_iters = 300;     // 시도 1회당 반복 수
  int max_restarts = 30;   // 무작위 초기값 재시도 횟수
  int want_solutions = 4;  // 이만큼 해를 모은 뒤 seed와 가장 가까운 해를 고름
  double damping = 1e-6;   // Damped Least Squares 감쇠
  double step = 1.0;       // 적분 스텝
  bool pos_only = false;   // true면 위치만 맞춤 (자세 무시)
};

struct IkResult {
  bool ok = false;
  VectorXd q;
  double pos_err = 0, rot_err = 0;
  int attempts = 0, solutions = 0;
  double ms = 0;
};

struct Arm {
  pin::Model model;                         // 끝점까지의 체인만 남긴 축소 모델
  pin::FrameIndex tip = 0, base = 0;
  std::string tip_name, base_name, urdf;
  std::vector<bool> prismatic;              // q 인덱스별 직선 관절 여부
  std::vector<std::string> chain_joints;    // IK 관절 이름 (q 순서)
  std::vector<std::string> all_joints;      // URDF의 움직이는 관절 전체 (joint_states 발행용)
};

inline pin::JointIndex frameParentJoint(const pin::Model& m, pin::FrameIndex f) {
#if PINOCCHIO_VERSION_AT_LEAST(3, 0, 0)
  return m.frames[f].parentJoint;
#else
  return m.frames[f].parent;
#endif
}

inline bool isPrismatic(const std::string& shortname) {
  return shortname.find("Prismatic") != std::string::npos || shortname.rfind("JointModelP", 0) == 0;
}

inline Arm buildArm(const std::string& urdf, std::string tip_name = "tool_tcp",
                    const std::string& base_name = "base_link") {
  pin::Model full;
  pin::urdf::buildModel(urdf, full);

  if (!full.existFrame(tip_name)) {
    if (tip_name == "tool_tcp" && full.existFrame("tool_roll_link")) {
      std::cerr << "[arm_ik] URDF에 tool_tcp 링크가 없어 tool_roll_link 원점을 끝점으로 사용\n";
      tip_name = "tool_roll_link";
    } else {
      throw std::runtime_error("URDF에 끝점 링크가 없음: " + tip_name);
    }
  }
  if (!full.existFrame(base_name)) throw std::runtime_error("URDF에 기준 링크가 없음: " + base_name);

  auto ancestors = [&](pin::JointIndex j) {
    std::vector<pin::JointIndex> out;
    for (; j > 0; j = full.parents[j]) out.push_back(j);
    return out;
  };
  std::vector<pin::JointIndex> chain = ancestors(frameParentJoint(full, full.getFrameId(tip_name)));
  const std::vector<pin::JointIndex> base_anc = ancestors(frameParentJoint(full, full.getFrameId(base_name)));
  chain.erase(std::remove_if(chain.begin(), chain.end(),
                             [&](pin::JointIndex j) {
                               return std::find(base_anc.begin(), base_anc.end(), j) != base_anc.end();
                             }),
              chain.end());
  if (chain.empty()) throw std::runtime_error("기준 링크와 끝점 사이에 움직이는 관절이 없음");
  for (pin::JointIndex j : chain)
    if (full.joints[j].nq() != 1)
      throw std::runtime_error("체인 관절 '" + full.names[j] + "'이 1자유도가 아님 (continuous 등은 IK 체인 불가)");

  std::vector<pin::JointIndex> lock;
  for (pin::JointIndex j = 1; j < (pin::JointIndex)full.njoints; ++j)
    if (std::find(chain.begin(), chain.end(), j) == chain.end()) lock.push_back(j);

  Arm arm;
  arm.model = pin::buildReducedModel(full, lock, pin::neutral(full));
  arm.tip = arm.model.getFrameId(tip_name);
  arm.base = arm.model.getFrameId(base_name);
  arm.tip_name = tip_name;
  arm.base_name = base_name;
  arm.urdf = urdf;
  for (int i = 1; i < arm.model.njoints; ++i) {
    arm.prismatic.push_back(isPrismatic(arm.model.joints[i].shortname()));
    arm.chain_joints.push_back(arm.model.names[i]);
  }
  for (int i = 1; i < full.njoints; ++i) arm.all_joints.push_back(full.names[i]);
  return arm;
}

// 자세 입력: R = Rx(pitch) * Ry(roll)  (기준 링크 좌표계)
inline pin::SE3 makeTarget(double x, double y, double z, double pitch, double roll) {
  const Matrix3d R = (Eigen::AngleAxisd(pitch, Vector3d::UnitX()) *
                      Eigen::AngleAxisd(roll, Vector3d::UnitY())).toRotationMatrix();
  return pin::SE3(R, Vector3d(x, y, z));
}

// R = Rx(p) Ry(r) 로 분해. yaw_residual 은 이 형태에서 벗어난 정도
inline void decompose(const Matrix3d& R, double& pitch, double& roll, double& yaw_residual) {
  pitch = std::atan2(R(2, 1), R(1, 1));
  roll = std::atan2(R(0, 2), R(0, 0));
  yaw_residual = std::asin(std::clamp(-R(0, 1), -1.0, 1.0));
}

// 기준 링크 좌표계에서 본 끝점 자세
inline pin::SE3 forward(const Arm& arm, pin::Data& data, const VectorXd& q) {
  pin::forwardKinematics(arm.model, data, q);
  pin::updateFramePlacement(arm.model, data, arm.tip);
  pin::updateFramePlacement(arm.model, data, arm.base);
  return data.oMf[arm.base].actInv(data.oMf[arm.tip]);
}

inline void clampToLimits(const pin::Model& m, VectorXd& q) {
  q = q.cwiseMax(m.lowerPositionLimit).cwiseMin(m.upperPositionLimit);
}

// IK 1회: Damped Least Squares CLIK + 관절 한계 클램프
inline bool solveOnce(const Arm& arm, pin::Data& data, const pin::SE3& target_in_base, VectorXd& q,
                      const IkParams& p, double& pos_err, double& rot_err) {
  const pin::Model& m = arm.model;
  pin::Data::Matrix6x J(6, m.nv);
  for (int it = 0; it < p.max_iters; ++it) {
    pin::forwardKinematics(m, data, q);
    pin::updateFramePlacement(m, data, arm.tip);
    pin::updateFramePlacement(m, data, arm.base);
    const pin::SE3 target = data.oMf[arm.base] * target_in_base;  // world 기준 목표
    const pin::SE3 cur = data.oMf[arm.tip];

    pos_err = (target.translation() - cur.translation()).norm();
    rot_err = p.pos_only ? 0.0 : pin::log3(cur.rotation().transpose() * target.rotation()).norm();
    if (pos_err < p.pos_tol && rot_err < p.rot_tol) return true;

    VectorXd dq;
    J.setZero();
    if (p.pos_only) {
      pin::computeFrameJacobian(m, data, q, arm.tip, pin::LOCAL_WORLD_ALIGNED, J);
      const Eigen::MatrixXd Jp = J.topRows(3);
      const Vector3d e = target.translation() - cur.translation();
      const Matrix3d A = Jp * Jp.transpose() + p.damping * Matrix3d::Identity();
      dq = Jp.transpose() * A.ldlt().solve(e);
    } else {
      const pin::SE3 err_se3 = cur.actInv(target);
      const pin::Motion::Vector6 e = pin::log6(err_se3).toVector();
      pin::computeFrameJacobian(m, data, q, arm.tip, pin::LOCAL, J);
      pin::Data::Matrix6 Jlog;
      pin::Jlog6(err_se3.inverse(), Jlog);
      const Eigen::MatrixXd Je = -Jlog * J;
      Eigen::Matrix<double, 6, 6> A = Je * Je.transpose();
      A.diagonal().array() += p.damping;
      dq = -Je.transpose() * A.ldlt().solve(e);
    }
    q = pin::integrate(m, q, dq * p.step);
    clampToLimits(m, q);  // 물리적 관절 범위 밖으로 못 나가게
  }
  // 실패: 마지막 q 기준으로 오차를 다시 계산
  const pin::SE3 cur = forward(arm, data, q);
  pos_err = (target_in_base.translation() - cur.translation()).norm();
  rot_err = p.pos_only ? 0.0 : pin::log3(cur.rotation().transpose() * target_in_base.rotation()).norm();
  return false;
}

// seed에서 먼저 시도 -> 실패하거나 해가 부족하면 무작위 초기값으로 재시도.
// 여러 해 중 seed와 가장 가까운 해 선택 (불필요한 큰 동작 방지)
inline IkResult solveIk(const Arm& arm, const pin::SE3& target, const VectorXd& seed,
                        const IkParams& p, std::mt19937& rng) {
  const auto t0 = std::chrono::steady_clock::now();
  const pin::Model& m = arm.model;
  pin::Data data(m);
  std::uniform_real_distribution<double> U(0.0, 1.0);

  IkResult best;
  double best_dist = 1e18, best_fail = 1e18;
  for (int a = 0; a <= p.max_restarts; ++a) {
    VectorXd q(m.nq);
    if (a == 0) {
      q = seed;
    } else {
      for (int i = 0; i < m.nq; ++i)
        q[i] = m.lowerPositionLimit[i] + U(rng) * (m.upperPositionLimit[i] - m.lowerPositionLimit[i]);
    }
    clampToLimits(m, q);
    double pe = 0, re = 0;
    const bool ok = solveOnce(arm, data, target, q, p, pe, re);
    best.attempts = a + 1;
    if (ok) {
      ++best.solutions;
      const double d = (q - seed).norm();
      if (d < best_dist) {
        best_dist = d;
        best.ok = true;
        best.q = q;
        best.pos_err = pe;
        best.rot_err = re;
      }
      if (a == 0 || best.solutions >= p.want_solutions) break;
    } else if (!best.ok && pe + re < best_fail) {
      best_fail = pe + re;
      best.q = q;
      best.pos_err = pe;
      best.rot_err = re;
    }
  }
  best.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  return best;
}

// 사용자 단위 변환 (직선 관절 mm, 회전 관절 deg)
inline double toUser(const Arm& a, int i, double v) { return a.prismatic[i] ? v * 1000.0 : v / kDeg; }
inline double fromUser(const Arm& a, int i, double v) { return a.prismatic[i] ? v / 1000.0 : v * kDeg; }

}  // namespace v75
