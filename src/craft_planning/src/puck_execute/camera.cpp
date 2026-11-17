#include "camera.hpp"
#include "runtime.hpp"
#include "plan.hpp"
#include "puck_observations.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

extern char **environ;

namespace puck::execute {
namespace {
struct Lease {
  Runner *node;
  std::string role;
  Lease(Runner *n, std::string r) : node(n), role(std::move(r)) { if (node) node->lease_camera(role); }
  ~Lease() { if (node) node->release_camera(role); }
};
void drain(int &fd, std::string &dest) {
  if (fd < 0) return;
  char buf[8192];
  for (;;) {
    auto n = read(fd, buf, sizeof(buf));
    if (n > 0) dest.append(buf, static_cast<std::size_t>(n));
    else if (n == 0) { close(fd); fd = -1; break; }
    else if (errno == EINTR) continue;
    else if (errno == EAGAIN || errno == EWOULDBLOCK) break;
    else { close(fd); fd = -1; throw std::runtime_error("camera pipe read failed"); }
  }
}
}
CameraResult camera_run(Runner *node, const std::vector<std::string> &command, double timeout_s) {
  if (command.empty() || timeout_s < 0) throw std::invalid_argument("invalid camera command or timeout");
  const auto role = std::filesystem::path(command.front()).filename() == "upper_puck" ? "uppercam" : "d435";
  const auto stamp = node ? node->now() : rclcpp::Time(0);
  Lease lease(node, role);
  int out[2], err[2];
  if (pipe(out) != 0) throw std::runtime_error("camera pipe failed");
  if (pipe(err) != 0) { close(out[0]); close(out[1]); throw std::runtime_error("camera pipe failed"); }
  auto invocation = command;
  const auto binary = std::filesystem::path(command.front()).filename();
  if (binary == "upper_puck" || binary == "grip_yaw") {
    invocation.insert(invocation.end(), {"--data-dir", camera_data_dir().string()});
  }
  std::vector<char *> args; args.reserve(invocation.size() + 1);
  for (const auto &s : invocation) args.push_back(const_cast<char *>(s.c_str()));
  args.push_back(nullptr);
  posix_spawn_file_actions_t actions;
  posix_spawn_file_actions_init(&actions);
  posix_spawn_file_actions_addclose(&actions, out[0]);
  posix_spawn_file_actions_addclose(&actions, err[0]);
  posix_spawn_file_actions_adddup2(&actions, out[1], STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, err[1], STDERR_FILENO);
  posix_spawn_file_actions_addclose(&actions, out[1]);
  posix_spawn_file_actions_addclose(&actions, err[1]);
  pid_t pid = -1;
  const int spawn_error = posix_spawnp(&pid, args[0], &actions, nullptr, args.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  if (spawn_error) {
    close(out[0]); close(out[1]); close(err[0]); close(err[1]);
    throw std::runtime_error(std::string("camera spawn failed: ") + std::strerror(spawn_error));
  }
  close(out[1]); close(err[1]);
  int a = out[0], b = err[0];
  fcntl(a, F_SETFL, fcntl(a, F_GETFL) | O_NONBLOCK);
  fcntl(b, F_SETFL, fcntl(b, F_GETFL) | O_NONBLOCK);
  const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
  CameraResult result{0, {}, {}};
  int status = 0; bool exited = false;
  try {
    while (!exited || a >= 0 || b >= 0) {
      drain(a, result.out); drain(b, result.err);
      if (!exited) {
        const auto w = waitpid(pid, &status, WNOHANG);
        if (w == pid) exited = true;
        else if (w < 0 && errno != EINTR) throw std::runtime_error("camera waitpid failed");
      }
      if (exited && a < 0 && b < 0) break;
      if (is_interrupted()) throw std::runtime_error("interrupted");
      if (timeout_s > 0 && std::chrono::steady_clock::now() >= until) throw std::runtime_error("camera command timed out");
      pollfd fds[2]{{a, POLLIN | POLLHUP, 0}, {b, POLLIN | POLLHUP, 0}};
      (void)poll(fds, 2, 20);
    }
  } catch (...) {
    if (!exited) { kill(pid, SIGKILL); while (waitpid(pid, &status, 0) < 0 && errno == EINTR) {} }
    if (a >= 0) close(a);
    if (b >= 0) close(b);
    throw;
  }
  result.returncode = WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status);
  if (node && std::find(command.begin(), command.end(), "--photo") == command.end()) {
    try {
      auto msg = role == std::string("uppercam")
        ? puck::upper_observations(result.out, (puck::camera_data_dir() / "upper_cam_pose.txt").string(), number("UPPER_AHEAD_BIAS_MM"), number("UPPER_LATERAL_BIAS_MM"))
        : puck::grip_observations(result.out);
      msg.header.stamp = stamp;
      node->publish_observation(role, std::move(msg));
    } catch (const std::exception &e) { std::cout << "   puck display: " << e.what() << '\n'; }
  }
  return result;
}
}

#include <cmath>
#include <Eigen/SVD>
#include <limits>
#include <cctype>
#include <fstream>
#include <regex>
#include <sstream>
namespace puck::execute {
namespace {
CameraResult capture(CameraContext &ctx, const std::vector<std::string> &command, double timeout) {
  return ctx.capture ? ctx.capture(command, timeout) : camera_run(ctx.node, command, timeout);
}
fs::path camera_bin(const std::string &name) { return camera_executable(name); }
fs::path camera_out() { return camera_data_dir() / "out"; }
fs::path cal_path() { return camera_data_dir() / "stand_cal.json"; }
std::string timestamp(const char *format) {
  std::time_t t = std::time(nullptr); std::tm local{}; localtime_r(&t, &local);
  char buf[80]; std::strftime(buf, sizeof(buf), format, &local); return buf;
}
Json read_cal() { std::ifstream f(cal_path()); if (!f) throw std::runtime_error("cannot read " + cal_path().string()); return Json::parse(f); }
std::string grip_capture(CameraContext &ctx, const std::string &prefix, const std::vector<std::string> &extra = {}) {
  auto cmd = std::vector<std::string>{camera_bin("grip_yaw").string()};
  cmd.insert(cmd.end(), extra.begin(), extra.end());
  cmd.insert(cmd.end(), {"--save", (camera_out() / (prefix + timestamp("%H%M%S"))).string()});
  auto r = capture(ctx, cmd, 30);
  return r.out + r.err;
}
std::string detector_line(const std::string &text,const std::string &pattern) {
  for(const auto &line:policy::splitlines(text))if(line.find(pattern)!=std::string::npos){auto pos=line.find(": ");return pos==std::string::npos?line:line.substr(pos+2);}
  std::string trimmed=text;while(!trimmed.empty()&&std::isspace(static_cast<unsigned char>(trimmed.back())))trimmed.pop_back();if(trimmed.size()>200)trimmed=trimmed.substr(trimmed.size()-200);return trimmed;
}
std::optional<double> yaw_read(const std::string &text, bool lying) {
  const std::regex re(lying ? R"re(LYING YAW ([+-]?\d+(?:\.\d+)?) deg)re" : R"re(: YAW ([+-]?\d+(?:\.\d+)?) deg)re");
  std::smatch m; if (std::regex_search(text, m, re)) return std::stod(m[1]); return std::nullopt;
}
std::optional<Pose> prior_pose(const Steps &steps, std::size_t i) {
  while (i > 0) { --i; if (steps[i].kind == TC::MANIPULATOR) { auto path = path_arg(steps[i].arg); if (!path.empty()) return path.back(); } }
  return std::nullopt;
}
}
CameraStepResult stand_yaw_step(CameraContext &ctx, Steps &steps, std::size_t i) {
  std::optional<double> yaw;
  for (int k = 0; k < 2 && !yaw; ++k) { auto text = grip_capture(ctx, "gripyaw_"); std::cout << "   " << detector_line(text, ": YAW ") << '\n'; yaw = yaw_read(text, false); }
  if (!yaw) return {false, "no standing prism read in the depth (2 shots)"};
  auto prev = prior_pose(steps, i); if (!prev) return {false, "no arm pose before the yaw step"};
  if (std::abs(*yaw) < number("STAND_YAW_MIN_DEG")) return {true, "yaw " + fmt(*yaw,1,true,true) + " deg: no turn"};
  auto q = *prev; q[4] += M_PI / 180. * number("STAND_ROLL_PER_YAW") * *yaw;
  auto [lo, hi] = chain().limits("tool_roll_joint");
  if (q[4] < lo || q[4] > hi) return {false, "yaw: tool_roll is outside its limits"};
  for (std::size_t j = i + 1; j < steps.size(); ++j) if (steps[j].kind == TC::MANIPULATOR && steps[j].name.rfind("stand pick:", 0) == 0) {
    auto path = path_arg(steps[j].arg); for (auto &p : path) p[4] = q[4]; steps[j].arg = path;
  }
  steps.insert(steps.begin() + static_cast<std::ptrdiff_t>(i + 1), {"stand yaw: turn the jaws to the prism, tool_roll " + fmt((*prev)[4] * 180. / M_PI, 1, true, true) + " -> " + fmt(q[4] * 180. / M_PI, 1, true, true) + " deg (yaw " + fmt(*yaw,1,true,true) + ")", TC::MANIPULATOR, Path{q}});
  return {true, "yaw " + fmt(*yaw,1,true,true) + " deg: jaws turn next"};
}
}
namespace puck::execute {
namespace {
std::vector<std::string> target_args(CameraContext &ctx, double ahead, const std::string &posture, bool cylinder) {
  const auto cal = read_cal();
  auto ref = interp_cells(ahead, [&](int c) { return cal.at(std::to_string(c)).at("grip_ref").at("centre_mm"); }).get<Pose>();
  if (posture == "lying") {
    auto cell = std::to_string(options.integer("--cell", 0));
    if (cal.contains(cell) && cal.at(cell).contains("lie_ref") && cal.at(cell).at("lie_ref").contains("floor_mm")) ref = cal.at(cell).at("lie_ref").at("floor_mm").get<Pose>();
  }
  std::vector<std::string> args{"--posture", posture, "--target", std::to_string(ref[0]), std::to_string(ref[1]), "40"};
  if (ctx.target) {
    Json mat = interp_cells(ahead, [&](int c) { return cal.at(std::to_string(c)).at("cam_per_arm"); });
    for (const auto &other : ctx.candidates) if (!(other == *ctx.target)) {
      const double x = -(other.lateral - ctx.target->lateral), y = other.ahead - ctx.target->ahead;
      args.insert(args.end(), {"--avoid", std::to_string(ref[0] - mat[0][0].get<double>() * x - mat[0][1].get<double>() * y), std::to_string(ref[1] - mat[1][0].get<double>() * x - mat[1][1].get<double>() * y)});
    }
  }
  if (cylinder) args.push_back("--no-yaw");
  return args;
}
std::pair<double,double> solve_cam(const Json &mat, double x, double y) {
  Eigen::Matrix2d m; m << mat[0][0].get<double>(), mat[0][1].get<double>(), mat[1][0].get<double>(), mat[1][1].get<double>();
  Eigen::FullPivLU<Eigen::Matrix2d> lu(m); lu.setThreshold(0.);
  if (!lu.isInvertible()) throw std::runtime_error("singular cam_per_arm matrix");
  const Eigen::Vector2d v = lu.solve(Eigen::Vector2d{x,y}); return {v[0],v[1]};
}
}
// TODO(port): Some diagnostic strings are abbreviated compared with execute.py; computed poses and stopping thresholds must remain equivalent.
CameraStepResult lie_yaw_step(CameraContext &ctx, Steps &steps, std::size_t i, const Json &arg) {
  std::optional<double> yaw; std::optional<std::pair<double,double>> floor;
  const int cell = arg.at("cell").get<int>();
  const double ahead = cell == 2 ? 172.5 : cell == 5 ? 112.5 : 52.5;
  for (int k = 0; k < 2 && !yaw; ++k) {
    auto text = grip_capture(ctx, "lieyaw_", ctx.target ? target_args(ctx,ahead,"lying",false) : std::vector<std::string>{});
    std::cout << "   " << detector_line(text, "YAW") << '\n'; yaw = yaw_read(text, true);
    std::smatch m; if (yaw && std::regex_search(text,m,std::regex(R"re(FLOOR ([+-]?\d+(?:\.\d+)?) ([+-]?\d+(?:\.\d+)?))re"))) floor = {std::stod(m[1]),std::stod(m[2])};
    if (!yaw && text.find(": YAW ") != std::string::npos) return {false,"the gripper cam sees a STANDING prism here: use --stand / --find"};
  }
  if (!yaw) return {false,"no lying prism read (2 shots)"};
  auto prev = prior_pose(steps,i); if (!prev) return {false,"no arm pose before the yaw step"};
  auto [lo,hi] = chain().limits("tool_roll_joint");
  const double r1 = arg.at("roll0").get<double>() + number("STAND_ROLL_PER_YAW") * *yaw * M_PI / 180.;
  std::vector<double> rolls; for (int k=-3;k<=3;++k) { double r = r1 + k*M_PI; if (lo<=r && r<=hi) rolls.push_back(r); }
  if (rolls.empty()) return {false,"tool_roll (+-180) is outside its limits"};
  auto nearest = [](const std::vector<double> &v, double target) { return *std::min_element(v.begin(),v.end(),[&](double a,double b){ return std::abs(a-target)<std::abs(b-target); }); };
  const double roll = nearest(rolls,(*prev)[4]);
  rolls.clear(); const double base = lie_grasp(8)[4] + M_PI/2.;
  for (int k=-3;k<=3;++k) { double r=base+k*M_PI; if(lo<=r && r<=hi) rolls.push_back(r); }
  if (rolls.empty()) return {false,"no tool_roll inside the limits puts the puck down along x at cell 8"};
  const double place = nearest(rolls,roll);
  Json cal = fs::exists(cal_path()) ? read_cal() : Json::object();
  if (options.has("--lie-ref")) {
    if (!floor) return {false,"no FLOOR in the reading: nothing saved"};
    // The calibration file is user data: retain its original contents before overwriting.
    auto backup = home()/"port_backup/tools/realsense_check/stand_cal.json";
    if (fs::exists(cal_path()) && !fs::exists(backup)) { fs::create_directories(backup.parent_path()); fs::copy_file(cal_path(),backup); }
    cal[std::to_string(cell)]["lie_ref"] = {{"floor_mm",{floor->first,floor->second}},{"yaw_deg",*yaw},{"time",timestamp("%Y-%m-%d %H:%M:%S")}};
    std::ofstream f(cal_path()); f << cal.dump(1); if (!f) throw std::runtime_error("cannot write stand calibration");
    return {false,"reference saved (--lie-ref stops here on purpose)"};
  }
  double dlm=0, dahead=0; const auto key=std::to_string(cell);
  if (floor && cal.contains(key)) {
    auto &c=cal[key]; Json ref;
    if (c.contains("lie_ref") && c["lie_ref"].contains("floor_mm")) ref=c["lie_ref"]["floor_mm"];
    else if (c.contains("grip_ref") && c["grip_ref"].contains("floor_mm")) ref=c["grip_ref"]["floor_mm"];
    if (!ref.is_null()) std::tie(dlm,dahead)=solve_cam(c.at("cam_per_arm"),ref[0].get<double>()-floor->first,ref[1].get<double>()-floor->second);
    if (std::abs(dlm)>number("LIE_MAX_LM_FIX_MM")) return {false,"gripper cam: sideways correction exceeds 25 mm"};
  }
  if (ctx.target && std::abs(dahead)>number("LIE_MAX_LM_FIX_MM")) return {false,"selected lying puck needs ahead correction outside local calibration"};
  for (std::size_t j=i+1;j<steps.size();++j) if(steps[j].kind==TC::MANIPULATOR) {
    const auto &n=steps[j].name; bool pick=n.rfind("lie pick:",0)==0;
    bool move=n.rfind("lie move:",0)==0 || n.rfind("lie regrasp 2a",0)==0 || n.rfind("lie regrasp 2b",0)==0;
    if (!pick && !move) continue;
    auto path=path_arg(steps[j].arg);
    for (auto &q:path) {
      if (pick) { q[0]+=dlm/1000.; q[4]=roll; if(ctx.target && std::abs(dahead)>=.05) { auto [ok, shifted]=shift_try(q,dahead); if(!ok) return {false,"selected lying position cannot be reached by the local taught-pose correction"}; q=shifted; } if(q[0]<-.043 || q[0]>.0627) return {false,"corrected lying pose is outside LM reach"}; }
      else q[4]=place;
    }
    steps[j].arg=path;
  }
  Pose q=*prev; q[4]=roll;
  steps.insert(steps.begin()+static_cast<std::ptrdiff_t>(i+1),{"lie: turn the jaws across the puck, tool_roll " + fmt((*prev)[4]*180./M_PI,1,true,true) + " -> " + fmt(roll*180./M_PI,1,true,true) + " deg (long axis " + fmt(*yaw,1,true,true) + ")",TC::MANIPULATOR,Path{q}});
  return {true,"lying puck, long axis " + fmt(*yaw,1,true,true) + " deg: tool_roll " + fmt(roll*180./M_PI,1,true,true) + " deg; LM " + fmt(dlm,1,true,true) + " mm; put down at cell 8 with tool_roll " + fmt(place*180./M_PI,1,true,true) + " deg (long axis along x)"};
}
}
namespace puck::execute {
namespace {
fs::path save_dir(CameraContext &ctx) { auto p=ctx.run_dir.empty()?camera_out():ctx.run_dir; fs::create_directories(p); return p; }
void log_entry(CameraContext &ctx,const std::string &key,const Json &value) {
  if (ctx.run_dir.empty()) return;
  fs::create_directories(ctx.run_dir); auto path=ctx.run_dir/"log.json"; Json log;
  if (fs::exists(path)) { std::ifstream f(path); log=Json::parse(f); }
  else {
    Json constants=Json::object();
    for(const auto &name:{"UPPER_AHEAD_BIAS_MM","UPPER_LATERAL_BIAS_MM","STAND_GRASP_Y_FIX_M","STAND_X_FIX_TABLE","STAND_Y_FIX_TABLE","STAND_YAW_Y_FIX_TABLE","STAND_LM_Y_FIX_TABLE","STAND_YAW_X_FIX_TABLE","STAND_APPROACH_UP_MM","STAND_ROLL_PER_YAW"})constants[name]=C.at(name);
    log={{"start",timestamp("%Y-%m-%d %H:%M:%S")},{"argv",options.args},{"constants",constants},{"user_eye","(fill in: where the gripper should have gone, e.g. \"motor 3, robot 2\")"}};
  }
  log[key]=value; std::ofstream f(path); f<<log.dump(1); if (!f) throw std::runtime_error("cannot write pick log");
}
std::string upper_capture(CameraContext &ctx,const std::string &prefix,int timeout=60) {
  auto r=capture(ctx,{camera_bin("upper_puck").string(),"--save",(save_dir(ctx)/(prefix+timestamp("%H%M%S"))).string()},timeout); return r.out;
}
void replace_pose(Steps &steps,std::size_t i,const std::string &prefix,const Pose &q) {
  for(std::size_t j=i+1;j<steps.size();++j) if(steps[j].name.rfind(prefix,0)==0) { steps[j].arg=Path{q}; return; }
  throw std::runtime_error("missing future step " + prefix);
}
std::optional<std::array<double,4>> parse_standing(const std::string &text) {
  std::smatch m; static const std::regex re(R"re(: YAW ([+-]?\d+(?:\.\d+)?) deg CENTRE ([+-]?\d+(?:\.\d+)?) ([+-]?\d+(?:\.\d+)?) ([+-]?\d+(?:\.\d+)?) mm)re");
  if (!std::regex_search(text,m,re)) return std::nullopt;
  return std::array<double,4>{std::stod(m[1]),std::stod(m[2]),std::stod(m[3]),std::stod(m[4])};
}
std::optional<std::array<double,4>> find_grip_capture(CameraContext &ctx,const std::vector<std::string> &extra,const std::string &prefix) {
  // Record-only colour photo at the wait pose (user 2026-11-16: a failed 05:53 shot left only depth, so which puck
  // was under the camera could not be seen). Saved as <prefix><time>_photo_color.png next to the depth stack.
  if(prefix=="grip_wait_"){auto r=capture(ctx,{camera_bin("grip_yaw").string(),"--photo",(save_dir(ctx)/(prefix+timestamp("%H%M%S")+"_photo")).string()},30);
    std::cout<<"   wait-pose photo: "<<(r.out.find("PHOTO saved")!=std::string::npos?"saved":"not saved")<<std::endl;}
  for(int k=0;k<2;++k) { std::vector<std::string> cmd{camera_bin("grip_yaw").string()}; cmd.insert(cmd.end(),extra.begin(),extra.end()); cmd.insert(cmd.end(),{"--save",(save_dir(ctx)/(prefix+timestamp("%H%M%S"))).string()}); auto r=capture(ctx,cmd,30); auto text=r.out+r.err; std::cout<<"   "<<detector_line(text,": YAW ")<<'\n'; auto shot=parse_standing(text); if(shot)return shot; }
  return std::nullopt;
}
}
policy::Candidate survey_pick(CameraContext &ctx) {
  std::optional<int> expected; if(options.has("--expected-pucks")) expected=options.integer("--expected-pucks",0);
  policy::Candidate chosen;
  for(int attempt=0;attempt<2;++attempt) {
    auto text=upper_capture(ctx,"survey_"+std::to_string(attempt)+"_"); std::cout<<text<<'\n';
    auto found=policy::candidates(text,number("UPPER_AHEAD_BIAS_MM"),number("UPPER_LATERAL_BIAS_MM"));
    try { chosen=policy::select(found,expected); ctx.candidates=found; break; }
    catch(const policy::ValueError &) { if(attempt==1) throw; }
  }
  Json records=Json::array(); for(const auto &c:ctx.candidates) records.push_back(c.record());
  log_entry(ctx,"selection",{{"rule","standing then distance from base_link"},{"candidates",records},{"picked",chosen.record()}});
  std::cout<<"PICK POLICY: "<<chosen.cls<<" "<<chosen.posture<<", distance "<<fmt(chosen.distance,1,false,true)<<" mm (ahead "<<fmt(chosen.ahead,1,false,true)<<", lateral "<<fmt(chosen.lateral,1,true,true)<<"); "<<ctx.candidates.size()<<" detected\n";
  if(chosen.ahead<C.at("FIND_AHEAD_RANGE_MM")[0].get<double>() || chosen.ahead>C.at("FIND_AHEAD_RANGE_MM")[1].get<double>()) throw std::runtime_error("highest-priority puck is outside the calibrated forward reach");
  ctx.target=chosen; return chosen;
}
// Puck re-dock (user 2026-11-16): any surveyed puck past the LM rail -> back out, dock again shifted sideways, survey
// again. The shift puts the middle of all pucks in the middle of the reach when they fit in it together, else the
// chosen puck. Once per pick; still past the rail afterwards, find_step fails as before.
// upper_puck lateral: + = away from the LM motor = robot left, the same side as the dock offset (+y left).
policy::Candidate redock_for_reach(CameraContext &ctx, const policy::Candidate &chosen) {
  if (!ctx.node || !ctx.node->planned() || ctx.candidates.empty()) return chosen;
  const double c=dice_lm("centre").first; auto lim=teach::motor_limits()[0];
  const double a=teach::raw_to_target({lim.first*180./M_PI,0,0,0,0,0})[0],b=teach::raw_to_target({lim.second*180./M_PI,0,0,0,0,0})[0];
  const double margin=number("REDOCK_LM_MARGIN_MM");
  const double lo=(c-std::max(a,b))*1000.+margin,hi=(c-std::min(a,b))*1000.-margin;  // reachable lateral [mm]
  double mn=1e9,mx=-1e9;for(const auto &p:ctx.candidates){mn=std::min(mn,p.lateral);mx=std::max(mx,p.lateral);}
  if(mn>=lo&&mx<=hi) return chosen;
  const double mid=(lo+hi)/2.,cap=number("REDOCK_MAX_MM");
  const bool both=mx-mn<=hi-lo;
  const double shift=std::clamp((both?(mn+mx)/2.:chosen.lateral)-mid,-cap,cap);
  std::cout<<"RE-DOCK: puck lateral "<<fmt(mn,1,true,true)<<" .. "<<fmt(mx,1,true,true)<<" mm, reach "<<fmt(lo,1,true,true)<<" .. "<<fmt(hi,1,true,true)
           <<" mm: dock "<<fmt(shift,1,true,true)<<" mm sideways ("<<(both?"all pucks":"the chosen puck")<<")\n";
  log_entry(ctx,"redock",{{"lateral_min",mn},{"lateral_max",mx},{"reach",{lo,hi}},{"shift_mm",shift},{"all",both}});
  const auto box=options.value("--box");
  if(!ctx.node->drive(box,true)) throw std::runtime_error("re-dock: back-out failed");
  if(!ctx.node->drive(box,false,shift/1000.)) throw std::runtime_error("re-dock: dock failed");
  ctx.target.reset(); return survey_pick(ctx);
}
CameraStepResult find_step(CameraContext &ctx,Steps &steps,std::size_t i,const Json &arg) {
  const auto what=arg.at("what").get<std::string>();
  if(what=="upper") {
    std::vector<policy::Candidate> found; policy::Candidate chosen;
    if(ctx.target) chosen=*ctx.target;
    else { auto text=upper_capture(ctx,"upper_"); found=policy::candidates(text,number("UPPER_AHEAD_BIAS_MM"),number("UPPER_LATERAL_BIAS_MM")); chosen=policy::select(found); ctx.target=chosen;ctx.candidates=found; }
    if(chosen.posture!="standing") return {false,"selected lying prism requires --collect (automatic standing/lying dispatch)"};
    if(chosen.ahead<C.at("FIND_AHEAD_RANGE_MM")[0].get<double>() || chosen.ahead>C.at("FIND_AHEAD_RANGE_MM")[1].get<double>()) return {false,"highest-priority puck is outside the calibrated forward reach"};
    double lm=dice_lm("centre").first-chosen.lateral/1000.; auto lim=teach::motor_limits()[0];
    double a=teach::raw_to_target({lim.first*180./M_PI,0,0,0,0,0})[0],b=teach::raw_to_target({lim.second*180./M_PI,0,0,0,0,0})[0];
    if(lm<std::min(a,b)||lm>std::max(a,b)) return {false,"upper cam: puck is past the LM rail"};
    ctx.find={{"cls",chosen.cls},{"ahead",chosen.ahead},{"lat",chosen.lateral},{"lm",lm}};
    auto wait=wait_at(chosen.ahead,lm); wait.push_back(teach::control_params()["gripper_open_angle"].as<double>()); replace_pose(steps,i,"standfind 2:",wait);
    Json records=Json::array();for(const auto &c:found)records.push_back(c.record());
    log_entry(ctx,"upper",{{"all",records},{"picked",{{"cls",chosen.cls},{"score",chosen.score},{"ahead_mm",chosen.ahead},{"lateral_mm",chosen.lateral}}},{"lm",lm},{"wait_pose",wait}});
    return {true,chosen.cls+" "+fmt(chosen.score,2,false,true)+": "+fmt(chosen.ahead,1,false,true)+" mm ahead, "+fmt(chosen.lateral,1,true,true)+" mm sideways (bias taken off) -> LM "+fmt(lm*1000.,1,false,true)+" mm"};
  }
  if(what=="photo") { auto tag=arg.at("tag").get<std::string>(); auto r=capture(ctx,{camera_bin("grip_yaw").string(),"--photo",(save_dir(ctx)/tag).string()},30); bool ok=r.out.find("PHOTO saved")!=std::string::npos; log_entry(ctx,"photo_"+tag,ok?tag+"_color.png":"failed"); return {true,std::string(ok?"saved":"photo failed (record only, going on)")+" -> "+save_dir(ctx).string()}; }
  auto cal=read_cal();
  if(what=="grip3") {
    for(int cell:{2,5,8}) if(!cal.at(std::to_string(cell)).contains("grasp_ref") || cal.at(std::to_string(cell))["grasp_ref"].is_null()) return {true,"no grasp-pose reference yet (--standcal again): grasp as planned"};
    auto shot=find_grip_capture(ctx,{},"grip_final_"); if(!shot) return {true,"gripper cam at the grasp pose: nothing read (jaws in the way / too near?): grasp as planned"};
    double ahead=ctx.find.at("ahead2"),lm=ctx.find.at("lm2"),roll=ctx.find.at("roll");
    Json ref=interp_cells(ahead,[&](int c){return cal.at(std::to_string(c)).at("grasp_ref").at("centre_mm");});
    Json mat=interp_cells(ahead,[&](int c){return cal.at(std::to_string(c)).at("cam_per_arm");});
    auto [dlm,dahead]=solve_cam(mat,ref[0].get<double>()-(*shot)[1],ref[1].get<double>()-(*shot)[2]); double d=std::max(std::abs(dlm),std::abs(dahead));
    if(d>6.) return {false,"final look: correction is over 6 mm (the jaws would hit it)"};
    if(d<1.5) return {true,"final look: no move"};
    auto g=grasp_at(ahead+dahead,lm+dlm/1000.,roll);steps.insert(steps.begin()+static_cast<std::ptrdiff_t>(i+1),{"stand pick: final shift at the grasp pose",TC::MANIPULATOR,Path{grasp_up(g,g)}});return {true,"final look: shift before closing"};
  }
  if(what!="grip") throw std::runtime_error("unknown find step: "+what);
  double ahead=ctx.find.at("ahead"),lm=ctx.find.at("lm"); std::string cls=ctx.find.at("cls");
  bool cylinder=cls.size()>=8 && cls.substr(cls.size()-8)=="cylinder";
  auto shot=find_grip_capture(ctx,target_args(ctx,ahead,"standing",cylinder),"grip_wait_"); if(!shot)return {false,"gripper cam: no standing prism (2 shots)"};
  Json ref=interp_cells(ahead,[&](int c){return cal.at(std::to_string(c)).at("grip_ref").at("centre_mm");});
  Json mat=interp_cells(ahead,[&](int c){return cal.at(std::to_string(c)).at("cam_per_arm");});
  double yaw0=interp_cells(ahead,[&](int c){return cal.at(std::to_string(c)).at("grip_ref").at("yaw_deg");}).get<double>();
  auto [dlm,dahead]=solve_cam(mat,ref[0].get<double>()-(*shot)[1],ref[1].get<double>()-(*shot)[2]);
  if(std::max(std::abs(dlm),std::abs(dahead))>number("FIND_MAX_FIX_MM"))return {false,"gripper cam: correction exceeds 25 mm"};
  double yaw=cls.find("prism")!=std::string::npos?fold((*shot)[0]-yaw0):0.; double ahead2=ahead+dahead,lm2=lm+dlm/1000.; auto w=wait_at(ahead2,lm2);double roll=w[4]+M_PI/180.*number("STAND_ROLL_PER_YAW")*yaw;
  auto [lo,hi]=chain().limits("tool_roll_joint"); if(roll<lo||roll>hi)return {false,"tool_roll is outside its limits"};
  double ag=ahead2+stand_x_fix(ahead2)+stand_yaw_x_fix(yaw), lg=lm2-(stand_y_fix(ahead2)+stand_yaw_y_fix(yaw))/1000.;
  auto [app,down,how]=descent(grasp_at(ag,lg,roll),approach_at(ag,lg,roll)); auto [shifted,app_shift]=shift_ahead(app,stand_approach_x_fix(ahead2));app=shifted;
  auto lim=teach::motor_limits()[0]; double a=teach::raw_to_target({lim.first*180./M_PI,0,0,0,0,0})[0],b=teach::raw_to_target({lim.second*180./M_PI,0,0,0,0,0})[0];
  if(app[0]<std::min(a,b)||app[0]>std::max(a,b)||down.back()[0]<std::min(a,b)||down.back()[0]>std::max(a,b))return {false,"the puck is out of the LM reach"};
  replace_pose(steps,i,"standfind 4:",app); bool replaced=false;for(std::size_t j=i+1;j<steps.size();++j)if(steps[j].name.rfind("stand pick: descend",0)==0){steps[j].name="stand pick: descend onto the puck ("+how+")";steps[j].arg=down;replaced=true;break;} if(!replaced)throw std::runtime_error("missing descend step");
  w[4]=roll;w[0]=down.back()[0];  // lift straight up at the grasp LM; stand load 2 takes it to the centre (user 2026-11-16)
  replace_pose(steps,i,"stand pick: up to 75 mm",w);
  ctx.find.update({{"yaw",yaw},{"d_lm",dlm},{"d_ahead",dahead},{"ahead2",ahead2},{"lm2",lm2},{"roll",roll}});
  log_entry(ctx,"grip",{{"yaw_read",(*shot)[0]},{"yaw_zero",yaw0},{"yaw",yaw},{"centre",{(*shot)[1],(*shot)[2]}},{"ref",{ref[0],ref[1]}},{"d_lm_mm",dlm},{"d_ahead_mm",dahead},{"ahead2",ahead2},{"lm2",lm2},{"roll",roll},{"x_fix_mm",stand_x_fix(ahead2)},{"approach_x_fix_mm",stand_approach_x_fix(ahead2)},{"approach_x_done_mm",app_shift},{"y_fix_mm",stand_y_fix(ahead2)},{"yaw_y_fix_mm",stand_yaw_y_fix(yaw)},{"yaw_x_fix_mm",stand_yaw_x_fix(yaw)},{"approach_pose",app},{"grasp_pose",down.back()}});
  std::string note=(ag<47.5||ag>177.5)?" !! "+fmt(ag,1,false,true)+" mm is outside the taught rows (52.5-172.5): poses of the end row":"";
  return {true,"puck off by LM "+fmt(dlm,1,true,true)+" mm, ahead "+fmt(dahead,1,true,true)+" mm -> "+fmt(ahead2,1,false,true)+" mm ahead, LM "+fmt(lm2*1000.,1,false,true)+" mm"+note+"; yaw "+fmt((*shot)[0],1,true,true)+" - zero "+fmt(yaw0,1,true,true)+" = "+fmt(yaw,1,true,true)+" deg"+(cylinder?" (cylinder: no turn)":"")};
}
}
namespace puck::execute {
CameraStepResult standcal_step(CameraContext &ctx,const Json &arg) {
  auto what=arg.at("what").get<std::string>();
  if(what=="grip") {
    auto tag=arg.at("tag").get<std::string>(); auto shot=find_grip_capture(ctx,{},"cal_"+tag+"_");
    if(!shot) return {arg.value("optional",false),arg.value("optional",false)?"gripper cam: no standing prism (2 shots); optional shot, skipped":"gripper cam: no standing prism (2 shots)"};
    ctx.cal_shots[tag]={{"centre",{(*shot)[1],(*shot)[2],(*shot)[3]}},{"yaw",(*shot)[0]},{"dlm",arg.at("dlm")},{"dahead",arg.at("dahead")}};
    return {true,"centre "+fmt((*shot)[1],1,true,true)+" "+fmt((*shot)[2],1,true,true)+" "+fmt((*shot)[3],1,false,true)+" mm, yaw "+fmt((*shot)[0],1,true,true)};
  }
  if(what=="upper") {
    auto text=upper_capture(ctx,"cal_upper_"); Json pucks=Json::array();
    static const std::regex re(R"re(^PUCK (\S+) ([+-]?\d+(?:\.\d+)?) ahead ([+-]?\d+(?:\.\d+)?) lateral ([+-]?\d+(?:\.\d+)?) top ([+-]?\d+(?:\.\d+)?))re");
    for(const auto &line:policy::splitlines(text)) { std::cout<<"   "<<line<<'\n'; std::smatch m;if(std::regex_search(line,m,re))pucks.push_back({{"cls",m[1].str()},{"score",std::stod(m[2])},{"ahead",std::stod(m[3])},{"lateral",std::stod(m[4])},{"top",std::stod(m[5])}}); }
    if(pucks.empty())return {false,"upper cam: no puck"};
    ctx.cal_shots["upper"]=pucks.size()==1?pucks[0]:pucks;
    return {true,std::to_string(pucks.size())+" puck(s)"};
  }
  if(what!="solve") throw std::runtime_error("unknown standcal step: "+what);
  if(!ctx.cal_shots.contains("ref"))return {false,"not enough shots (no reference)"};
  auto ref=ctx.cal_shots["ref"]; std::vector<Json> shots;
  for(auto it=ctx.cal_shots.begin();it!=ctx.cal_shots.end();++it) { auto key=it.key();if(key!="ref" && key!="upper" && key.rfind("ap_",0)!=0 && key.rfind("gr_",0)!=0)shots.push_back(it.value()); }
  if(shots.size()<3)return {false,"not enough shots"};
  Eigen::MatrixXd A(shots.size(),2),B(shots.size(),2);
  for(std::size_t j=0;j<shots.size();++j) { A(j,0)=shots[j]["dlm"].get<double>(); A(j,1)=shots[j]["dahead"].get<double>(); for(int k=0;k<2;++k)B(j,k)=shots[j]["centre"][k].get<double>()-ref["centre"][k].get<double>(); }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A,Eigen::ComputeThinU|Eigen::ComputeThinV);
  svd.setThreshold(std::numeric_limits<double>::epsilon()*std::max(A.rows(),A.cols()));
  Eigen::Matrix2d m=svd.solve(B).transpose(); double rms=std::sqrt((A*m.transpose()-B).squaredNorm()/(2.*shots.size()));
  Json matrix=Json::array();for(int x=0;x<2;++x){Json row=Json::array();for(int y=0;y<2;++y)row.push_back(std::nearbyint(m(x,y)*10000.)/10000.);matrix.push_back(row);}
  Json upper=nullptr; if(ctx.cal_shots.contains("upper")){ upper=ctx.cal_shots["upper"];if(upper.is_array()){auto best=std::min_element(upper.begin(),upper.end(),[&](const Json &a,const Json &b){auto score=[&](const Json &p){return std::abs(p["ahead"].get<double>()-arg["ahead"].get<double>())+std::abs(p["lateral"].get<double>());};return score(a)<score(b);});Json selected=*best;upper=std::move(selected);}}
  Json grasp=nullptr; if(ctx.cal_shots.contains("gr_ref")){auto g=ctx.cal_shots["gr_ref"];grasp={{"centre_mm",g["centre"]},{"yaw_deg",g["yaw"]}};}
  Json up=nullptr;if(!upper.is_null())up={{"ahead_mm",upper["ahead"]},{"lateral_mm",upper["lateral"]},{"cls",upper["cls"]},{"score",upper["score"]},{"ahead_err_mm",std::stod(fmt(upper["ahead"].get<double>()-arg["ahead"].get<double>(),1,false,true))}};
  Json cal=fs::exists(cal_path())?read_cal():Json::object();auto cell=std::to_string(arg.at("cell").get<int>()); Json shot_records=ctx.cal_shots;shot_records.erase("upper");
  cal[cell]={{"time",timestamp("%Y-%m-%d %H:%M:%S")},{"row",arg["row"]},{"ahead_mm",arg["ahead"]},{"lm",arg["lm"]},{"grasp","stand_grasp_c"+cell},{"grip_ref",{{"centre_mm",ref["centre"]},{"yaw_deg",ref["yaw"]}}},{"cam_per_arm",matrix},{"fit_rms_mm",std::stod(fmt(rms,2,false,true))},{"shots",shot_records},{"grasp_ref",grasp},{"upper",up}};
  auto backup=home()/"port_backup/tools/realsense_check/stand_cal.json";if(fs::exists(cal_path())&&!fs::exists(backup)){fs::create_directories(backup.parent_path());fs::copy_file(cal_path(),backup);}
  std::ofstream f(cal_path());f<<cal.dump(1);if(!f)throw std::runtime_error("cannot write stand calibration");
  return {true,"cam_per_arm "+matrix.dump()+" (fit rms "+fmt(rms,2,false,true)+" mm) -> "+cal_path().string()};
}
}
namespace puck::execute {
namespace {
fs::path grip_locate() {
  if (!std::getenv("CRAFT_REPO") && !std::getenv("PUCK_PICK_DIR"))
    throw std::runtime_error("legacy grip_locate modes require the source workflow (CRAFT_REPO); installed picking supports collect/find/lie");
  return camera_data_dir()/"grip_locate.py";
}
void backup_data(const fs::path &p) {
  if(!fs::exists(p))return;
  auto backup=(home()/"port_backup/tools/realsense_check")/p.filename();
  if(!fs::exists(backup)){fs::create_directories(backup.parent_path());fs::copy_file(p,backup);}
}
CameraResult run_legacy(CameraContext &ctx,const std::vector<std::string> &cmd,double timeout=60.) {
  // The Python utility was invoked directly, without the ROS camera lease or observation publication.
  return ctx.capture?ctx.capture(cmd,timeout):camera_run(nullptr,cmd,timeout);
}
struct GripMeasurement {std::optional<Eigen::Vector3d> cam,tag;std::optional<Eigen::Vector2d> off;std::optional<double> dz,turn;std::string line,prefix;};
GripMeasurement grip_measure(CameraContext &ctx) {
  auto r=run_legacy(ctx,{"/usr/bin/python3",grip_locate().string(),"--capture","--under-jaws"});auto text=r.out+r.err;GripMeasurement m;
  for(const auto &line:policy::splitlines(text))if(line.find("camera x")!=std::string::npos || line.find("no puck")!=std::string::npos){m.line=line;break;}
  if(m.line.empty())m.line=text.substr(text.size()>200?text.size()-200:0);
  std::smatch match;static const std::regex cam(R"re(camera x ([+-]?\d+(?:\.\d+)?) y ([+-]?\d+(?:\.\d+)?) z ([+-]?\d+(?:\.\d+)?) mm)re"),off(R"re(dx ([+-]?\d+(?:\.\d+)?) dy ([+-]?\d+(?:\.\d+)?) dz ([+-]?\d+(?:\.\d+)?) mm)re"),tag(R"re(tag x ([+-]?\d+(?:\.\d+)?) y ([+-]?\d+(?:\.\d+)?) z ([+-]?\d+(?:\.\d+)?))re"),turn(R"re(arm yaw ([+-]?\d+(?:\.\d+)?) deg)re"),prefix(R"re((?:^|\n)(\S+): \w+ [\d.]+ px)re");
  if(std::regex_search(text,match,cam))m.cam=Eigen::Vector3d(std::stod(match[1]),std::stod(match[2]),std::stod(match[3]));
  if(std::regex_search(text,match,off)){m.off=Eigen::Vector2d(std::stod(match[1]),std::stod(match[2]));m.dz=std::stod(match[3]);}
  if(std::regex_search(text,match,tag))m.tag=Eigen::Vector3d(std::stod(match[1]),std::stod(match[2]),std::stod(match[3]));
  if(std::regex_search(text,match,turn))m.turn=std::stod(match[1]);
  if(std::regex_search(text,match,prefix))m.prefix=match[1];
  return m;
}
}
CameraStepResult gripzero_step(CameraContext &ctx,const std::string &what) {
  if(what=="look") {auto p=camera_out()/("gripclose_"+timestamp("%H%M%S"));auto r=run_legacy(ctx,{camera_bin("rs_snap").string(),p.string(),"1280","720","261622073363"});return {r.returncode==0,"saved "+p.filename().string()};}
  backup_data(grip_locate().parent_path()/"grip_zero.json");
  auto r=run_legacy(ctx,{"/usr/bin/python3",grip_locate().string(),"--capture","--under-jaws","--set-zero"});auto text=r.out+r.err;std::string line;for(const auto &s:policy::splitlines(text))if(s.find("zero")!=std::string::npos){line=s;break;}if(line.empty())line=text.substr(text.size()>200?text.size()-200:0);return {line.find("zero stored")!=std::string::npos,line};
}
CameraStepResult gripcal_step(CameraContext &ctx,const std::string &what) {
  if(what!="solve") {auto m=grip_measure(ctx);std::cout<<"   "<<m.line<<'\n';if(!m.cam)return {false,"gripper cam: no puck"};ctx.gripcal_shots[what]={(*m.cam)[0],(*m.cam)[1]};return {true,"shot "+what+": camera x "+fmt((*m.cam)[0],1,true,true)+" y "+fmt((*m.cam)[1],1,true,true)+" mm"};}
  if(!ctx.gripcal_shots.contains("c0")||!ctx.gripcal_shots.contains("c1")||!ctx.gripcal_shots.contains("c2"))return {false,"a shot is missing"};
  Eigen::Matrix2d m;for(int k=0;k<2;++k){m(k,0)=(ctx.gripcal_shots["c1"][k].get<double>()-ctx.gripcal_shots["c0"][k].get<double>())/8.;m(k,1)=(ctx.gripcal_shots["c2"][k].get<double>()-ctx.gripcal_shots["c0"][k].get<double>())/8.;}
  double n1=m.col(0).norm(),n2=m.col(1).norm(),cosang=m.col(0).dot(m.col(1))/std::max(n1*n2,1e-9);
  if(!(n1>.6&&n1<1.4&&n2>.6&&n2<1.4&&std::abs(cosang)<.35))return {false,"the shifts do not look like 8 mm moves at right angles: nothing written (puck moved? wrong puck detected?)"};
  auto p=grip_locate().parent_path()/"grip_axes.json";backup_data(p);Json data={{"cam_per_arm",{{m(0,0),m(0,1)},{m(1,0),m(1,1)}}},{"move_mm",8.},{"stamp",timestamp("%Y-%m-%d %H:%M")},{"plan",plan_file}};std::ofstream f(p);f<<data.dump(1);if(!f)throw std::runtime_error("cannot write grip axes");return {true,"written: "+p.string()};
}
}
#include "plan_generator.hpp"
#include <yaml-cpp/yaml.h>
namespace puck::execute {
namespace {
std::string step_plan(const std::string &name) {
  std::smatch m;static const std::regex re(R"re(\[puck (\d+)\])re");auto index=std::regex_search(name,m,re)?static_cast<std::size_t>(std::stoul(m[1])-1):0;
  return plan_files.at(index);
}
bool is_prism(const std::string &name) {
  try {std::ifstream f(here()/"plan_cam_pucks.json");auto data=Json::parse(f);for(const auto &p:data)if(p.at("plan").get<std::string>()==step_plan(name))return p.at("class").get<std::string>().find("prism")!=std::string::npos;}catch(const std::exception &){}return false;
}
std::optional<Eigen::Matrix2d> grip_axes() {
  auto p=grip_locate().parent_path()/"grip_axes.json";if(!fs::exists(p))return std::nullopt;std::ifstream f(p);auto j=Json::parse(f).at("cam_per_arm");Eigen::Matrix2d m;m<<j[0][0].get<double>(),j[0][1].get<double>(),j[1][0].get<double>(),j[1][1].get<double>();return m;
}
}
CameraStepResult gripcheck_step(CameraContext &ctx,Steps &steps,std::size_t i) {
  const auto name=steps[i].name; bool again=name.find("again after the correction")!=std::string::npos;
  if(!again){ctx.fixed.setZero();ctx.tries=0;ctx.shelf_fix=0;ctx.roll_fix.reset();}
  ctx.learn_ref.reset();auto shot=grip_measure(ctx);std::cout<<"   "<<shot.line<<'\n';auto zero=grip_locate().parent_path()/"grip_zero.json";
  if(fs::exists(zero)&&shot.cam&&!shot.off&&!shot.tag&&ctx.previous_cam&&ctx.previous_tag&&ctx.previous_plan==step_plan(name)&&again){
    std::ifstream f(zero);auto z=Json::parse(f);auto sh=is_prism(name)?"prism":"cylinder";
    if(z.contains(sh)&&z[sh].value("point","")=="tag") {auto v=z[sh]["mm"].get<Pose>();Eigen::Vector3d estimate=*ctx.previous_tag+(*shot.cam-*ctx.previous_cam);Eigen::Vector3d d=estimate-Eigen::Vector3d(v[0],v[1],v[2]);if(std::abs(d[2])<=30.){shot.off=d.head<2>();shot.dz=d[2];}}
  }
  for(int k=0;k<3&&fs::exists(zero)&&shot.cam&&!shot.off;++k){shot=grip_measure(ctx);std::cout<<"   "<<shot.line<<'\n';}
  if(shot.cam&&shot.tag){ctx.previous_cam=shot.cam;ctx.previous_tag=shot.tag;ctx.previous_plan=step_plan(name);}
  if(!shot.cam)return {false,"gripper cam: no puck"};
  bool ok=true;std::string reason;double dz_h=0.;auto axes=grip_axes();
  if(!shot.off&&fs::exists(zero)) {
    ok=false;reason="gripper cam: the top tag was not read in 4 frames (hidden by a jaw? glare?): not going down blind";
    if(!options.has("--no-gripfix"))return {false,reason};
  } else if(!shot.off){ctx.learn_ref=shot.prefix;reason="gripper cam sees the puck; no reference at this height yet, this frame becomes it if the grasp holds";}
  else {double lim=axes?3.:10.;if(shot.dz&&axes){double side=(-axes->inverse()*(*shot.off))[0];dz_h=*shot.dz+.375*side;if(std::abs(dz_h)>4.){ok=false;reason="gripper cam: puck top is off the planned height";}}
    if(reason.empty()){ok=shot.off->cwiseAbs().maxCoeff()<=lim;reason="gripper cam: puck "+fmt((*shot.off)[0],1,true,true)+" / "+fmt((*shot.off)[1],1,true,true)+" mm off the lined-up spot (limit "+fmt(lim,0)+")";}}
  if(options.has("--no-gripfix")) {
    if(is_prism(name)&&!shot.turn&&!again){for(int k=0;k<2&&!shot.turn;++k){auto retry=grip_measure(ctx);std::cout<<"   top tag not read, shot "<<k+2<<": "<<retry.line<<'\n';shot.turn=retry.turn;}if(!shot.turn)reason+="; PRISM YAW NOT READ in 3 shots, the jaws are not turned";}
    ok=true;reason="NOT CORRECTED (--no-gripfix): "+reason;
  }
  std::optional<double> turn_fix;
  if(is_prism(name)&&shot.turn){auto yaml=YAML::LoadFile((here()/step_plan(name)).string());double have=ctx.roll_fix.value_or(yaml["assume"]["tool_roll"]?yaml["assume"]["tool_roll"].as<double>():0.);double want=-*shot.turn*M_PI/180.;double d=py_mod(want-have+M_PI/4.,M_PI/2.)-M_PI/4.;if(std::abs(d)>6.*M_PI/180.)turn_fix=have+d;}
  bool need=!options.has("--no-gripfix")&&!ok&&shot.off&&axes&&shot.off->cwiseAbs().maxCoeff()<=35.;
  if((need||(turn_fix&&(ok||need)))&&ctx.tries<3){Eigen::Vector2d d=Eigen::Vector2d::Zero();if(need)d=-axes->inverse()*(*shot.off);double up=0.;if(need&&std::abs(dz_h)>4.){up=-dz_h*1.08;d.setZero();ctx.shelf_fix+=up;}if(turn_fix)ctx.roll_fix=turn_fix;
    auto poses=replan(ctx.fixed[0]+d[0],ctx.fixed[1]+d[1],"plan_cam_corr.yaml",step_plan(name),ctx.roll_fix,ctx.shelf_fix);if(!poses)return {false,reason+"; no pose for the correction"};ctx.fixed+=d;++ctx.tries;auto [approach,grasp]=*poses;
    if(ctx.shelf_fix){auto g0=replan(ctx.fixed[0],ctx.fixed[1],"plan_cam_corr0.yaml",step_plan(name),ctx.roll_fix,0.);if(g0)grasp=g0->second;}
    auto pos=name.find(" [puck");auto tag=pos==std::string::npos?std::string():name.substr(pos);
    for(std::size_t j=i+1;j<steps.size();++j){if(steps[j].name.find("pick: descend")!=std::string::npos)steps[j].arg=grasp;else if(steps[j].name.find("pick: ascend")!=std::string::npos){steps[j].arg=Path{approach};break;}}
    steps.insert(steps.begin()+static_cast<std::ptrdiff_t>(i+1),{"bigbro fix "+std::to_string(ctx.tries)+": over the puck, arm "+fmt(d[0],1,true,true)+" mm sideways "+fmt(d[1],1,true,true)+" mm ahead "+fmt(up,1,true,true)+" mm up"+tag,TC::MANIPULATOR,Path{approach}});
    steps.insert(steps.begin()+static_cast<std::ptrdiff_t>(i+2),{"bigbro check: gripper cam again after the correction"+tag,GRIPCHECK,nullptr});
    return {true,"CORRECTING "+reason};
  }
  if(!ok)return {false,reason};
  if(name.rfind("bigbro check",0)==0)for(std::size_t j=i+1;j<steps.size();++j)if(steps[j].name.find("pick: ascend")!=std::string::npos){auto q=path_arg(steps[j].arg).back();if(std::abs(q[4])>M_PI/180.){auto pos=name.find(" [puck");auto tag=pos==std::string::npos?std::string():name.substr(pos);steps.insert(steps.begin()+static_cast<std::ptrdiff_t>(i+1),{"bigbro pick: turn the jaws to the puck, tool_roll "+fmt(q[4]*180./M_PI,1,true,true)+" deg"+tag,TC::MANIPULATOR,Path{q}});}break;}
  return {true,reason};
}
std::string held_bad(CameraContext &ctx,const std::string &name,const std::string &shot,const Steps &steps,std::size_t i) {
  if(options.has("--no-held-check"))return {};
  for(std::size_t j=i;j<steps.size();++j)if(steps[j].name.find("pick: ascend")!=std::string::npos){auto q=path_arg(steps[j].arg).back();if(std::abs(q[4])>10.*M_PI/180.){std::cout<<"   held check skipped: the jaws are turned (no reference for that yet)\n";return {};}break;}
  auto saved=shot.rfind("saved ",0)==0?shot.substr(6):shot;auto p=camera_out()/saved;auto r=run_legacy(ctx,{"/usr/bin/python3",grip_locate().string(),"--held",p.string(),"--shape",is_prism(name)?"prism":"cylinder"});
  for(const auto &line:policy::splitlines(r.out))if(line.find("HELD")!=std::string::npos){std::cout<<"   "<<line<<'\n';auto pos=line.find("HELD BAD ");if(pos!=std::string::npos)return "held wrong ("+line.substr(pos+9)+")";break;}
  return {};
}
void record_loaded(const std::string &name,bool grasped) {
  std::smatch match;static const std::regex re(R"re(\[puck (\d+)\])re");std::size_t k=std::regex_search(name,match,re)?std::stoul(match[1])-1:0;std::string cls="unknown";Json score=nullptr;
  try{std::ifstream f(here()/"plan_cam_pucks.json");for(const auto &p:Json::parse(f))if(p["plan"]==plan_files.at(k)){cls=p["class"].get<std::string>();if(p.contains("score"))score=p["score"];break;}}catch(const std::exception &){}
  auto p=here()/"carousel.yaml";auto backup=home()/"port_backup/demo/puck_pick/carousel.yaml";if(fs::exists(p)&&!fs::exists(backup)){fs::create_directories(backup.parent_path());fs::copy_file(p,backup);}YAML::Node y=fs::exists(p)?YAML::LoadFile(p.string()):YAML::Node(YAML::NodeType::Map);auto row=y[bigbro_slots.at(k)];row["class"]=cls;if(score.is_number())row["score"]=score.get<double>();else row["score"]=YAML::Node();row["time"]=timestamp("%m-%d %H:%M:%S");row["grasped"]=grasped;std::ofstream f(p);f<<y;if(!f)throw std::runtime_error("cannot write carousel");std::cout<<"   carousel: slot "<<bigbro_slots.at(k)<<" = "<<cls<<'\n';
}
}

namespace puck::execute {
void learn_reference(CameraContext &ctx) {
  if (!ctx.learn_ref || ctx.learn_ref->empty()) return;
  backup_data(grip_locate().parent_path()/"grip_ref.json");
  auto r=run_legacy(ctx,{"/usr/bin/python3",grip_locate().string(),"--under-jaws","--set-ref",*ctx.learn_ref},0.);
  auto lines=policy::splitlines(r.out);std::cout<<"   gripper cam: "<<(lines.empty()?"reference not stored":lines.back())<<'\n';
}
}
