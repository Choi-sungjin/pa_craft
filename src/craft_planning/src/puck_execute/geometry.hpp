#pragma once
#include "plan.hpp"
#include "teach.hpp"
#include <nlohmann/json.hpp>
#include <craft_interfaces/msg/task_command.hpp>
#include <functional>
#include <optional>
#include <tuple>
namespace puck::execute {
using Json = nlohmann::ordered_json;
using Path = std::vector<Pose>;
using TC = craft_interfaces::msg::TaskCommand;
struct Step { std::string name; int kind; Json arg; };
using Steps = std::vector<Step>;
enum PseudoKind { STANDFIND=100, STANDCAL, PAUSE, LIEYAW, STANDYAW, GRIPCHECK, GRIPZERO, GRIPCAL };
struct Options {
 std::vector<std::string> args;
 bool has(const std::string &flag) const;
 std::string value(const std::string &flag,const std::string &fallback="") const;
 double real(const std::string &flag,double fallback) const;
 int integer(const std::string &flag,int fallback) const;
 void set(const std::string &flag,const std::string &value);
};
extern Options options;
extern Json C;
extern std::vector<std::string> plan_files;
extern std::string plan_file;
extern std::vector<int> bigbro_slots;
extern int bigbro_slot;
extern std::map<std::string,std::vector<double>> slow_seconds;
void configure(const Options &opts);
double number(const std::string &name);
Pose values(const std::string &name);
std::string fmt(double v,int precision=6,bool sign=false,bool fixed=false);
Pose first5(Pose q);
Pose nominal_pose(); Pose via_pose(); Pose carry_via_pose(); Path carry_via_path();
Step wrist_first(const Pose &prev,const Pose &target,const std::string &what);
bool close_but_shoulder(const Pose &a,const Pose &b,double joint_deg=15.,double slide_m=.005);
Steps out_to_via(const Pose &prev);
Steps with_via(Steps steps,const Pose &start,bool out=true);
Pose load_pose(); Pose below(const Pose &arm,double dz,double dy=0); Pose above(const Pose &arm,double dz);
using PoseAt=std::function<Pose(int)>;
PoseAt plan_pose(const std::string &name="");
Path frames_path(const PoseAt &pose,int i0,int i1);
Path descent_path(const PoseAt &pose,const std::string &name="");
Path lying_line(const Pose &heights); Path tcp_path(const Pose &arm0,const Pose &arm1,int n);
Path path_arg(const Json &arg);
Steps stage(Steps steps,std::optional<Pose> prev=std::nullopt);
std::pair<double,std::string> dice_lm(const std::string &col);
Pose jaw_fix(const Pose &q,std::optional<double> off_m=std::nullopt);
double table(Json t,double x); double fold(double a,double period=90.);
double stand_x_fix(double a); double stand_y_fix(double a); double stand_lm_y_fix(double a);
double stand_approach_x_fix(double a); double stand_yaw_x_fix(double yaw); double stand_yaw_y_fix(double yaw,double ahead=97.);
Eigen::Vector3d plane_state(const Pose &q);
std::pair<bool,Pose> shift_try(const Pose &q,double d_mm);
std::pair<Pose,double> shift_ahead(const Pose &q,double d_mm);
Pose taught_target(const std::string &name); Pose stand_grasp(int cell);
Pose wait_at(double ahead,double lm);
Json interp_cells(double ahead,const std::function<Json(int)> &value);
std::optional<Path> line_up(const Pose &q,std::optional<double> dz_mm=std::nullopt,std::optional<int> n=std::nullopt);
Pose stand_above_arm(int cell); Pose grasp_up(const Pose &grasp,const Pose &approach);
std::tuple<Pose,Path,std::string> descent(const Pose &grasp,const Pose &approach);
Pose grasp_at(double ahead,double lm,double roll); Pose approach_at(double ahead,double lm,double roll);
Steps stand_yaw_tail(); Steps stand_load_tail(const Pose &start);
Steps stand_pick_tail(const std::string &row,double lm,const Pose &start,std::optional<int> cell=std::nullopt);
Steps standcal_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps find_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Pose lie_grasp(int cell,std::optional<double> roll=std::nullopt);
std::pair<Pose,std::string> lie_approach(int cell,std::optional<double> roll=std::nullopt);
Steps lie_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
Steps stand_steps(std::optional<double> current_slide=std::nullopt,std::optional<Pose> start=std::nullopt);
double step_tolerance(const std::string &name); std::vector<int32_t> step_i_gains(const std::string &name);
double step_contact(const std::string &name); double step_seconds(const std::string &name,int k=0);
std::string seconds_text(const std::string &name); std::string describe(int kind,const Json &arg);
void smooth_steps(Steps &steps,const Pose &previous,const Pose &start);
}
