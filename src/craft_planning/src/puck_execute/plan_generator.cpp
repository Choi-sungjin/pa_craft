#include "plan_generator.hpp"
#include "plan.hpp"
#include "teach.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace puck::execute {
namespace {
double rounded(double x, int n) { std::ostringstream s; s << std::fixed << std::setprecision(n) << x; return std::stod(s.str()); }
YAML::Node defaults() { return YAML::Load(R"(
lidar_center_x: 0.128
puck_x_offset: -0.01
lidar_center_y: 0.0525
lidar_front_y: 0.034
lidar_z: [0.020, 0.053]
floor_z: 0.0
hw_z_correction: -0.010
shelf_size: [0.205, 0.208]
shelf_height: 0.07
puck_ahead: 0.13
puck_size: [0.03, 0.03, 0.06]
puck_height: 0.06
finger_beyond_tcp: 0.027
finger_clearance: 0.003
approach_above: 0.04
grasp_coverage: 0.8666666666666667
max_tilt_deg: 30.0
limit_margin_deg: 3.0
tool_roll: 0.0
jaw_open: -0.021
jaw_grip: -0.0018
lying: false
tool_tilt_deg: null
shelf_ahead: null
)"); }
double get(const YAML::Node &n, const char *k) { return n[k].as<double>(); }
std::vector<JointMap> solve(const Eigen::Vector3d &p, const YAML::Node &a) {
  const auto &ch = chain(); const double mg = get(a,"limit_margin_deg") * M_PI/180.;
  const double roll = get(a,"tool_roll");
  for (int t=0; t<=static_cast<int>(get(a,"max_tilt_deg")); ++t) {
    auto sols=ik_top_down(ch,p,t);
    for (auto &q:sols) { q["tool_roll_joint"]=roll; bool ok=true;
      for (const auto &n:Chain::NAMES) {auto [lo,hi]=ch.limits(n); double m=n=="slide_joint"?0:mg; if(q[n]<lo+m||q[n]>hi-m) ok=false;}
      if(ok) return {q};
    }
  }
  auto sols=ik_top_down(ch,p,0); for (auto &q:sols) q["tool_roll_joint"]=roll; return sols;
}
YAML::Node joints(const JointMap &q,double jaw) { YAML::Node n; for (const auto &k:Chain::NAMES) n[k]=q.at(k); n["jaw_upper_joint"]=jaw; return n; }
bool valid_motor(const JointMap &q,const YAML::Node &c) {
  for(size_t i=1;i<Chain::NAMES.size();++i) { double x=c["arm_joint_offsets"][i-1].as<double>()+c["arm_joint_directions"][i-1].as<double>()*q.at(Chain::NAMES[i]);
    if(x<c["arm_min_angle"][i-1].as<double>()||x>c["arm_max_angle"][i-1].as<double>()) return false; }
  double lm=c["lm_motor_offset"].as<double>()+c["lm_motor_direction"].as<double>()*q.at("slide_joint")/c["lm_meters_per_rad"].as<double>();
  return lm>=c["lm_min_angle"].as<double>()&&lm<=c["lm_max_angle"].as<double>();
}
std::vector<double> pose(const YAML::Node &frame) {std::vector<double> p; for(const auto &n:Chain::NAMES) p.push_back(frame["joints"][n].as<double>()); p[0]+=0.002; return p;}
}
// TODO(port): replan diagnostics do not reproduce plan.py stdout/stderr detail verbatim.
std::optional<std::pair<std::vector<double>,std::vector<std::vector<double>>>> replan(double side_mm,double ahead_mm,const std::string &out,const std::string &plan_file,std::optional<double> roll,double shelf_mm) {
  auto in=YAML::LoadFile((here()/plan_file).string())["assume"];
  const double reach=get(in,"puck_ahead")+ahead_mm/1000.;
  if(reach>0.150) {std::cout << "   the corrected puck is " << reach*1000 << " mm ahead: no correction; park closer\n"; return std::nullopt;}
  auto a=defaults(); a["puck_ahead"]=rounded(reach,4); a["puck_x_offset"]=rounded(get(in,"puck_x_offset")+side_mm/1000.,4);
  a["shelf_height"]=rounded(get(in,"shelf_height")+shelf_mm/1000.,4);
  a["grasp_coverage"]=rounded(get(in,"grasp_coverage")*get(in,"puck_height")*1000.,1)/1000./get(a,"puck_height");
  a["tool_roll"]=rounded(roll.value_or(in["tool_roll"] ? get(in,"tool_roll") : 0),4);
  a["approach_above"]=rounded(in["approach_above"] ? get(in,"approach_above") : 0.04,3);
  const Eigen::Vector3d grasp(get(a,"lidar_center_x")+get(a,"puck_x_offset"), get(a,"lidar_front_y")-get(a,"puck_ahead"),get(a,"floor_z")+get(a,"shelf_height")+std::max(get(a,"puck_height")*(1-get(a,"grasp_coverage"))+get(a,"finger_beyond_tcp"),get(a,"finger_beyond_tcp")+get(a,"finger_clearance"))+get(a,"hw_z_correction"));
  const Eigen::Vector3d center(grasp.x(),grasp.y(),get(a,"floor_z")+get(a,"shelf_height")+get(a,"puck_height")/2.);
  auto ctrl=teach::control_params(); YAML::Node doc; doc["assume"]=a; for(int i=0;i<3;i++) doc["puck_grasp_urdf"].push_back(grasp[i]);
  doc["waypoints"]=YAML::Node(YAML::NodeType::Sequence); doc["frames"]=YAML::Node(YAML::NodeType::Sequence); bool ok=true;
  for(int i=0;i<3;i++) {auto p=grasp; if(i==0)p.z()+=get(a,"approach_above"); auto sols=solve(p,a); if(sols.empty()){ok=false;continue;}
    auto best=sols.front(); size_t nbad=100; for(const auto &q:sols){size_t n=0;for(const auto &name:Chain::NAMES){auto [lo,hi]=chain().limits(name);if(q.at(name)<lo||q.at(name)>hi)++n;}if(n<nbad){best=q;nbad=n;}}
    if(nbad||!valid_motor(best,ctrl)) ok=false;
    YAML::Node w; w["name"]=(i==0?"approach":i==1?"grasp":"close"); w["joints"]=joints(best,get(a,i==2?"jaw_grip":"jaw_open")); doc["waypoints"].push_back(w);
  }
  if(ok) {
    auto frame=[&](const JointMap &q,double jaw) {YAML::Node f; f["joints"]=joints(q,jaw); for(int i=0;i<3;i++) f["puck"].push_back(center[i]); return f;};
    auto top=grasp;top.z()+=get(a,"approach_above"); auto initial=solve(top,a); if(initial.empty()) throw std::runtime_error("plan trajectory approach unreachable"); JointMap prev=initial.front();
    for(int i=0;i<20;i++) doc["frames"].push_back(frame(prev,get(a,"jaw_open")));
    for(int i=0;i<40;i++){auto p=top+(grasp-top)*(double(i)/39.);auto sols=solve(p,a);if(sols.empty())throw std::runtime_error("plan trajectory descent unreachable");
      auto cost=[&](const JointMap &q){double s=0;for(const auto &n:Chain::NAMES)s+=std::abs(q.at(n)-prev.at(n));return s;}; prev=*std::min_element(sols.begin(),sols.end(),[&](const auto &x,const auto &y){return cost(x)<cost(y);});doc["frames"].push_back(frame(prev,get(a,"jaw_open")));}
    for(int i=0;i<20;i++)doc["frames"].push_back(frame(prev,get(a,"jaw_open")+(get(a,"jaw_grip")-get(a,"jaw_open"))*i/19.));
    for(int i=0;i<30;i++)doc["frames"].push_back(frame(prev,get(a,"jaw_grip")));
  }
  const auto output = here()/out;
  // Preserve the original user-generated plan before replacing it.
  if (fs::exists(output) && output.is_relative() == false) {
    std::error_code ec;
    auto rel=fs::relative(output,ws(),ec);
    if (!ec && !rel.empty() && *rel.begin() != "..") {
      auto backup=home()/"port_backup"/rel;
      if (!fs::exists(backup)) { fs::create_directories(backup.parent_path()); fs::copy_file(output,backup); }
    }
  }
  std::ofstream f(output); if(!f)throw std::runtime_error("cannot write plan "+out); f<<doc;f.close();
  if(!ok){std::cout<<"   plan.py found no pose for the corrected puck\n";return std::nullopt;}
  auto frames=doc["frames"]; int knee=59;for(int i=20;i<59;i++){double lean=0;for(int j=1;j<4;j++)lean+=frames[i]["joints"][Chain::NAMES[j]].as<double>()-frames[59]["joints"][Chain::NAMES[j]].as<double>();if(std::abs(lean)<M_PI/360.){knee=i;break;}}
  std::vector<std::vector<double>> down; if(knee>21&&knee<57){down.push_back(pose(frames[knee]));down.push_back(pose(frames[(knee+59)/2]));}down.push_back(pose(frames[59]));return std::make_pair(pose(frames[0]),down);
}
}
