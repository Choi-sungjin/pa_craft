// arm_ik: URDF 기반 팔 IK 명령줄 도구 (ROS 없이 단독 실행)
//
// 사용법:
//   arm_ik [opts] info
//   arm_ik [opts] fk Q1 Q2 ...            (체인 순서, 직선 관절 mm / 회전 관절 deg)
//   arm_ik [opts] ik X Y Z PITCH ROLL     (mm, deg) [--seed Q1 Q2 ...] [--pos-only]
//   arm_ik [opts] selftest [N]
// opts:
//   --urdf PATH   URDF 경로 (우선순위: --urdf > 환경변수 ARM_IK_URDF > 빌드 시 기본 경로)
//   --tip LINK    끝점 링크 (기본 tool_tcp, 없으면 tool_roll_link)
//   --base LINK   기준 링크 (기본 base_link)
//
// 자세 입력: R = Rx(pitch) * Ry(roll). pitch = roll = 0 이면 q=0 (CAD 저장 자세)의 끝점 방향.

#include "v75_arm_ik/arm_ik_core.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>

#ifndef DEFAULT_URDF
#define DEFAULT_URDF "urdf/v75_robot.urdf"
#endif

using namespace v75;

// 수직 하향 잡기에서 물체를 놓을 기준 앞뒤 위치 (base_link y, mm). downmap 결과로 정함
#ifndef YREF_DEFAULT_MM
#define YREF_DEFAULT_MM -110.0
#endif

static void printJoints(const Arm& a, const VectorXd& q) {
  const pin::Model& m = a.model;
  std::cout << std::fixed;
  for (int i = 0; i < m.nq; ++i) {
    const double lo = toUser(a, i, m.lowerPositionLimit[i]), hi = toUser(a, i, m.upperPositionLimit[i]);
    const bool at_limit = std::abs(q[i] - m.lowerPositionLimit[i]) < 1e-6 ||
                          std::abs(q[i] - m.upperPositionLimit[i]) < 1e-6;
    std::cout << "  " << std::left << std::setw(18) << a.chain_joints[i] << std::right << std::setw(9)
              << std::setprecision(2) << toUser(a, i, q[i]) << " " << (a.prismatic[i] ? "mm " : "deg")
              << "   [" << std::setw(8) << lo << ", " << std::setw(8) << hi << "]"
              << (at_limit ? "  <- 한계 도달" : "") << "\n";
  }
}

static void printPose(const pin::SE3& T) {
  double p, r, y;
  decompose(T.rotation(), p, r, y);
  const Vector3d t = T.translation() * 1000.0;
  std::cout << std::fixed << std::setprecision(2) << "  위치 [mm]   x=" << t.x() << "  y=" << t.y()
            << "  z=" << t.z() << "\n  자세 [deg]  pitch=" << p / kDeg << "  roll=" << r / kDeg
            << "   (yaw 잔차 " << std::setprecision(4) << y / kDeg << ")\n";
}

static VectorXd jointsFromUser(const Arm& a, const std::vector<double>& v) {
  VectorXd q(a.model.nq);
  for (int i = 0; i < a.model.nq; ++i) q[i] = fromUser(a, i, v[i]);
  return q;
}

static void usage() {
  std::cerr << "사용법:\n"
               "  arm_ik [opts] info\n"
               "  arm_ik [opts] fk Q1 Q2 ...            (직선 관절 mm, 회전 관절 deg, info의 순서)\n"
               "  arm_ik [opts] ik X Y Z PITCH ROLL     (mm, deg) [--seed Q1 Q2 ...] [--pos-only]\n"
               "  arm_ik [opts] selftest [N]\n"
               "  arm_ik [opts] downmap [X_MM] [ROLL_DEG]     수직 하향 도달 범위표\n"
               "  arm_ik [opts] plan X Y Z [ROLL] [--yref MM] 바퀴(y)/LM(x)/팔(z) 분담 계획\n"
               "opts:\n"
               "  --urdf PATH   URDF 경로 (--urdf > 환경변수 ARM_IK_URDF > 빌드 기본값)\n"
               "  --tip LINK    끝점 링크 (기본 tool_tcp)\n"
               "  --base LINK   기준 링크 (기본 base_link)\n";
}

int main(int argc, char** argv) {
  std::string urdf = DEFAULT_URDF;
  if (const char* env = std::getenv("ARM_IK_URDF")) urdf = env;
  std::string tip = "tool_tcp", base = "base_link";
  IkParams params;
  std::vector<std::string> args;
  std::vector<double> seed_user;
  bool reading_seed = false;
  double yref_mm = YREF_DEFAULT_MM;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--urdf" && i + 1 < argc) { urdf = argv[++i]; reading_seed = false; }
    else if (a == "--tip" && i + 1 < argc) { tip = argv[++i]; reading_seed = false; }
    else if (a == "--base" && i + 1 < argc) { base = argv[++i]; reading_seed = false; }
    else if (a == "--pos-only") { params.pos_only = true; reading_seed = false; }
    else if (a == "--yref" && i + 1 < argc) { yref_mm = std::atof(argv[++i]); reading_seed = false; }
    else if (a == "--seed") reading_seed = true;
    else if (reading_seed) seed_user.push_back(std::atof(a.c_str()));
    else args.push_back(a);
  }
  if (args.empty()) { usage(); return 1; }

  Arm arm;
  try {
    arm = buildArm(urdf, tip, base);
  } catch (const std::exception& e) {
    std::cerr << "모델 로드 실패 (" << urdf << "): " << e.what() << "\n";
    return 1;
  }
  const pin::Model& m = arm.model;
  const int n = m.nq;
  pin::Data data(m);
  std::mt19937 rng(42);
  const std::string cmd = args[0];

  if (cmd == "info") {
    std::cout << "URDF : " << arm.urdf << "\n기준 : " << arm.base_name << "\n끝점 : " << arm.tip_name
              << "\nIK 관절 " << n << "개 (체인 순서) / 한계:\n";
    printJoints(arm, VectorXd::Zero(n));
    std::cout << "q=0 (CAD 자세) 끝점:\n";
    printPose(forward(arm, data, VectorXd::Zero(n)));
    return 0;
  }

  if (cmd == "fk") {
    if ((int)args.size() != n + 1) {
      std::cerr << "fk 에는 관절값 " << n << "개가 필요함 (info 참고)\n";
      return 1;
    }
    std::vector<double> v;
    for (int i = 1; i <= n; ++i) v.push_back(std::atof(args[i].c_str()));
    VectorXd q = jointsFromUser(arm, v), qc = q;
    clampToLimits(m, qc);
    if ((qc - q).norm() > 1e-9) std::cout << "경고: 관절 한계를 벗어난 입력이 있음\n";
    std::cout << "관절값:\n";
    printJoints(arm, q);
    std::cout << "끝점 (" << arm.tip_name << "):\n";
    printPose(forward(arm, data, q));
    return 0;
  }

  if (cmd == "ik") {
    if (args.size() != 6) { usage(); return 1; }
    const double x = std::atof(args[1].c_str()) / 1000.0, y = std::atof(args[2].c_str()) / 1000.0,
                 z = std::atof(args[3].c_str()) / 1000.0;
    const double pitch = std::atof(args[4].c_str()) * kDeg, roll = std::atof(args[5].c_str()) * kDeg;
    const pin::SE3 target = makeTarget(x, y, z, pitch, roll);
    VectorXd seed = VectorXd::Zero(n);
    if (!seed_user.empty()) {
      if ((int)seed_user.size() != n) { std::cerr << "--seed 에는 " << n << "개 값이 필요함\n"; return 1; }
      seed = jointsFromUser(arm, seed_user);
    }

    std::cout << "목표 (" << arm.tip_name << (params.pos_only ? ", 위치만" : "") << "):\n";
    printPose(target);
    const IkResult r = solveIk(arm, target, seed, params, rng);
    std::cout << std::fixed << std::setprecision(2);
    if (r.ok)
      std::cout << "\n해 찾음 (" << r.ms << " ms, 시도 " << r.attempts << "회, 찾은 해 " << r.solutions
                << "개 중 seed와 가장 가까운 해)\n";
    else
      std::cout << "\n해 없음: 도달 불가능하거나 관절 한계 밖 (" << r.ms << " ms, 시도 " << r.attempts
                << "회)\n가장 가까운 결과 (위치 오차 " << std::setprecision(1) << r.pos_err * 1000
                << " mm, 자세 오차 " << r.rot_err / kDeg << " deg):\n";
    printJoints(arm, r.q);
    std::cout << "FK 검증:\n";
    printPose(forward(arm, data, r.q));
    return r.ok ? 0 : 2;
  }

  if (cmd == "selftest") {
    const int N = args.size() > 1 ? std::atoi(args[1].c_str()) : 1000;
    std::uniform_real_distribution<double> U(0.0, 1.0);
    int ok = 0, limit_violation = 0;
    double t_sum = 0, t_max = 0, pe_max = 0, re_max = 0;
    for (int k = 0; k < N; ++k) {
      VectorXd qt(n);
      for (int i = 0; i < n; ++i)
        qt[i] = m.lowerPositionLimit[i] + U(rng) * (m.upperPositionLimit[i] - m.lowerPositionLimit[i]);
      const pin::SE3 target = forward(arm, data, qt);
      const IkResult r = solveIk(arm, target, VectorXd::Zero(n), params, rng);
      t_sum += r.ms;
      t_max = std::max(t_max, r.ms);
      if (r.ok) {
        ++ok;
        pe_max = std::max(pe_max, r.pos_err);
        re_max = std::max(re_max, r.rot_err);
        VectorXd qc = r.q;
        clampToLimits(m, qc);
        if ((qc - r.q).norm() > 1e-9) ++limit_violation;
      }
    }
    std::cout << std::fixed << std::setprecision(2) << "selftest " << N << "회 (끝점 " << arm.tip_name
              << ", seed = q0)\n"
              << "  성공률        " << 100.0 * ok / N << " %\n"
              << "  평균 / 최대   " << t_sum / N << " / " << t_max << " ms\n"
              << "  최대 위치오차 " << std::setprecision(4) << pe_max * 1000 << " mm\n"
              << "  최대 자세오차 " << re_max / kDeg << " deg\n"
              << "  관절한계 위반 " << limit_violation << " 건\n";
    return ok == N ? 0 : 3;
  }

  if (cmd == "downmap") {
    // 그리퍼 수직 하향(pitch -90)으로 도달 가능한 (y, z) 범위표. x 는 LM 가이드가 맡으므로 고정
    const double x = args.size() > 1 ? std::atof(args[1].c_str()) / 1000.0 : 0.149;
    const double roll = args.size() > 2 ? std::atof(args[2].c_str()) * kDeg : 0.0;
    IkParams p = params;
    p.max_restarts = 8;
    std::cout << "수직 하향 도달 범위 (x=" << x * 1000 << " mm, roll=" << roll / kDeg << " deg)\n"
              << "   y[mm]   z 범위[mm]\n";
    for (int yi = -250; yi <= 450; yi += 10) {
      int zlo = 99999, zhi = -99999;
      for (int zi = -50; zi <= 450; zi += 5) {
        const pin::SE3 T = makeTarget(x, yi / 1000.0, zi / 1000.0, -M_PI / 2, roll);
        if (solveIk(arm, T, VectorXd::Zero(n), p, rng).ok) { zlo = std::min(zlo, zi); zhi = std::max(zhi, zi); }
      }
      if (zhi >= zlo) std::cout << "  " << std::setw(6) << yi << "   " << std::setw(4) << zlo << " ~ " << std::setw(4) << zhi << "\n";
    }
    return 0;
  }

  if (cmd == "plan") {
    // 수직 하향 잡기 계획: 앞뒤(y)는 바퀴, 좌우(x)는 LM 가이드, 높이(z)·각도는 팔
    // arm_ik plan X Y Z [ROLL] [--yref MM]
    if (args.size() < 4) { usage(); return 1; }
    const double x = std::atof(args[1].c_str()) / 1000.0, y = std::atof(args[2].c_str()) / 1000.0,
                 z = std::atof(args[3].c_str()) / 1000.0;
    const double roll = args.size() > 4 ? std::atof(args[4].c_str()) * kDeg : 0.0;
    const double yref = yref_mm / 1000.0;
    const double dy = y - yref;
    std::cout << std::fixed << std::setprecision(1) << "물체 (" << x * 1000 << ", " << y * 1000 << ", " << z * 1000
              << ") mm, roll " << roll / kDeg << " deg\n"
              << "1) 바퀴: 로봇을 +y 방향으로 " << dy * 1000 << " mm 이동 (물체가 y = " << yref * 1000
              << " mm 에 오도록)\n";
    const pin::SE3 T = makeTarget(x, yref, z, -M_PI / 2, roll);
    const IkResult r = solveIk(arm, T, VectorXd::Zero(n), params, rng);
    if (!r.ok) {
      std::cout << "2) 팔: 해 없음 (x 는 LM 범위, z 는 downmap 의 범위 안이어야 함)\n";
      printJoints(arm, r.q);
      return 2;
    }
    std::cout << "2) LM 가이드 + 팔 (수직 하향):\n";
    printJoints(arm, r.q);
    std::cout << "FK 검증:\n";
    printPose(forward(arm, data, r.q));
    return 0;
  }

  usage();
  return 1;
}
