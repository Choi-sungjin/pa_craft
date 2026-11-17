#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <craft_interfaces/msg/task_command.hpp>
#include <craft_interfaces/msg/task_result.hpp>
#include <craft_interfaces/msg/control_status.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "craft_control/lateral_controller.hpp"
#include "craft_control/longitudinal_controller.hpp"
using Command = craft_interfaces::msg::TaskCommand;
using Result = craft_interfaces::msg::TaskResult;
using Clock = std::chrono::steady_clock;
class Plant : public rclcpp::Node {
 public:
  Plant() : Node("drive_sim_plant"), tf_(*this) {
    auto file = declare_parameter<std::string>("control_yaml", "");
    auto start = declare_parameter<std::vector<double>>("start", {0.56, 0.98, 1.57079632679});
    if (file.empty() || file[0] != '/' || start.size()!=3 || !std::all_of(start.begin(),start.end(),[](double v){return std::isfinite(v);})) throw std::runtime_error("absolute control_yaml and finite start [x,y,yaw] required");
    sx_=x_=start[0]; sy_=y_=start[1]; sa_=a_=start[2];
    auto p=YAML::LoadFile(file)["control_master"]["ros__parameters"];
    auto d=[&](const char *key){return p[key].as<double>();};
    radius_=d("wheel_radius"); max_accel_=d("max_accel"); max_wheel_=d("max_wheel");
    if (radius_<=0 || max_accel_<=0 || max_wheel_<=0) throw std::runtime_error("invalid drive configuration");
    lat_.configure(radius_,d("wheel_track"),d("wheelbase"),d("skid_steer_factor"),p["lookahead_speeds"].as<std::vector<double>>(),p["lookahead_distances"].as<std::vector<double>>(),d("wheel_goal_tolerance"),d("path_spacing"),d("wheel_heading_tolerance"),d("wheel_rotate_angular_speed"),d("wheel_tracking_gain"));
    lat_.set_goal_rotate_angular_speed(d("wheel_goal_rotate_angular_speed"),d("wheel_goal_rotate_min_speed"));
    lat_.set_lateral_integral(d("wheel_lateral_integral_gain"),d("wheel_lateral_integral_limit"),0.02);
    lat_.set_heading_offset(d("wheel_heading_offset_deg")*M_PI/180.0);
    auto errors=p["wheel_tracking_gain_errors_deg"].as<std::vector<double>>(); for(auto &v:errors) v*=M_PI/180.0;
    lat_.set_gain_schedule(errors,p["wheel_tracking_gain_scales"].as<std::vector<double>>());
    lat_.set_slow_tracking_gain(d("wheel_slow_speed"),d("wheel_slow_tracking_gain"));
    lat_.set_dock_straight(d("wheel_dock_straight_speed"),d("wheel_dock_heading_gain"));
    lon_.configure(radius_,d("max_v"));
    status_pub_=create_publisher<craft_interfaces::msg::ControlStatus>("/control/status",10);
    result_pub_=create_publisher<Result>("/control/task_result",10);
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("/drive_sim/truth",10);
    joint_pub_=create_publisher<sensor_msgs::msg::JointState>("/joint_states",10);
    path_pub_=create_publisher<nav_msgs::msg::Path>("/drive_sim/planned_path",rclcpp::QoS(1).transient_local());
    trail_pub_=create_publisher<nav_msgs::msg::Path>("/drive_sim/trail",10);
    look_pub_=create_publisher<geometry_msgs::msg::PointStamped>("/drive_sim/lookahead",10);
    state_pub_=create_publisher<std_msgs::msg::String>("/drive_sim/plant_state",10);
    hold_sub_=create_subscription<std_msgs::msg::Bool>("/planning/hold",10,[this](const std_msgs::msg::Bool &m){held_=m.data; heartbeat_=Clock::now(); have_heartbeat_=true;});
    task_sub_=create_subscription<Command>("/planning/task_command",10,[this](const Command &m){receive(m);});
    collision_sub_=create_subscription<std_msgs::msg::Bool>("/drive_sim/collision",10,[this](const std_msgs::msg::Bool &m){if(m.data){fault_=true; if(active_){active_=false; result_.status=Result::FAILED; result_.reason="simulated collision";}}});
    cmd_sub_=create_subscription<std_msgs::msg::String>("/drive_sim/command",10,[this](const std_msgs::msg::String &m){command(m.data);});
    trail_.header.frame_id="map"; last_tick_=Clock::now(); timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){tick();});
  }
 private:
  bool inhibited() const {return paused_ || fault_ || held_ || !have_heartbeat_ || std::chrono::duration<double>(Clock::now()-heartbeat_).count()>0.5;}
  void reject(const Command &c,const std::string &why){Result r; r.header.stamp=now(); r.task_id=c.task_id; r.revision=c.revision; r.status=Result::REJECTED; r.reason=why; result_pub_->publish(r);}
  void receive(const Command &c) {
    ++commands_;
    if(c.operation==Command::STOP){if(c.task_id && c.task_id!=result_.task_id){reject(c,"unknown stop task"); return;} active_=false; result_.status=Result::STOPPED; result_.reason="stopped"; lat_.clear_target(); return;}
    if(c.operation==Command::EXECUTE && c.task_id==result_.task_id && c.task_id){return;}
    bool update=c.operation==Command::UPDATE_PATH;
    if(!c.task_id || !c.revision || c.kind!=Command::NAVIGATE){reject(c,"only NAVIGATE with nonzero ID and revision supported"); return;}
    if(update && (!active_ || c.task_id!=result_.task_id)){reject(c,"no matching active navigation task"); return;}
    if(update && c.revision<=result_.revision) return;
    if(!update && (c.operation!=Command::EXECUTE || active_ || c.task_id<=last_id_)){reject(c,"busy, invalid operation or retired task ID"); return;}
    if(inhibited()){reject(c,"plant held or unavailable"); return;}
    bool valid=c.path.header.frame_id=="map" && lat_.is_valid_target(c.path) && lon_.is_valid_target(c.speed) && (update || c.speed!=0) && !c.camera_distance_control && std::isfinite(c.goal_tolerance) && c.goal_tolerance>=0 && std::isfinite(c.tolerance_rad) && c.tolerance_rad>=0 && c.tolerance_rad<=M_PI && std::isfinite(c.tracking_reference_x) && c.tracking_reference_x>=0 && std::isfinite(c.lookahead_distance) && c.lookahead_distance>=0 && (c.lookahead_distance>0 || c.tracking_reference_x==0) && std::isfinite(c.duration_s) && c.duration_s>=0 && c.duration_s<=3600 && (c.speed_limit.empty() || (c.speed_limit.size()==c.path.poses.size() && std::all_of(c.speed_limit.begin(),c.speed_limit.end(),[](double v){return std::isfinite(v)&&v>=0;})));
    if(!valid){if(update){active_=false; result_.status=Result::FAILED; result_.reason="invalid path update";} else reject(c,"invalid path or unsupported camera mode"); return;}
    if(!update){result_=Result(); result_.task_id=c.task_id; last_id_=c.task_id; active_=true; task_start_=Clock::now(); deadline_=c.continuous_tracking ? c.duration_s : 0;}
    result_.revision=c.revision; result_.status=Result::RUNNING; result_.reason="running"; speed_=c.speed;
    lat_.set_task_tolerances(c.goal_tolerance,c.tolerance_rad); lat_.set_task_tracking_reference(c.tracking_reference_x,c.lookahead_distance); lat_.set_continuous_tracking(c.continuous_tracking); lat_.set_camera_distance_mode(false); lat_.set_target(c.path,c.require_goal_heading,c.speed_limit); path_pub_->publish(c.path);
  }
  void command(const std::string &s) {
    if(s=="pause") paused_=true;
    else if(s=="play") paused_=false;
    else if(s=="reset") {if(active_){RCLCPP_WARN(get_logger(),"reset rejected while task active; stop first"); return;} x_=sx_;y_=sy_;a_=sa_; fault_=false; paused_=false; wl_pos_=wr_pos_=0; trail_.poses.clear(); lat_.clear_target(); RCLCPP_INFO(get_logger(),"plant reset");}
    else RCLCPP_WARN(get_logger(),"unknown plant command: %s",s.c_str());
  }
  nav_msgs::msg::Odometry odom(){nav_msgs::msg::Odometry o; o.header.stamp=now(); o.header.frame_id="map"; o.child_frame_id="base_link"; o.pose.pose.position.x=x_;o.pose.pose.position.y=y_;o.pose.pose.position.z=0.034346;o.pose.pose.orientation.z=std::sin(a_/2);o.pose.pose.orientation.w=std::cos(a_/2);o.twist.twist.linear.x=vel_;o.twist.twist.angular.z=omega_;return o;}
  void tick(){
    auto t=Clock::now(); double dt=std::clamp(std::chrono::duration<double>(t-last_tick_).count(),0.005,0.05); last_tick_=t;
    auto o=odom(); lat_.set_odom(o); lat_.set_speed(vel_);
    if(active_ && deadline_>0 && std::chrono::duration<double>(t-task_start_).count()>deadline_){active_=false;result_.status=Result::FAILED;result_.reason="task deadline";}
    wl_=wr_=0;
    if(active_ && !inhibited()){
      if(lat_.is_target_reached()){active_=false; result_.status=Result::SUCCEEDED;result_.reason="target reached";}
      else if(speed_!=0){
        if(lat_.can_track(speed_)){double dir=speed_<0?-1:1;double v=std::min({std::abs(speed_),lat_.speed_limit(),dir*vel_+max_accel_*dt});lon_.set_target(dir*std::max(v,0.0));}else lon_.clear_target();
        auto v=lon_.compute_command(); auto w=lat_.compute_command(radius_*(v[0]+v[1])/2,speed_); wl_=v[0]+w[0];wr_=v[1]+w[1];double k=std::max({1.0,std::abs(wl_)/max_wheel_,std::abs(wr_)/max_wheel_});wl_/=k;wr_/=k;
      }
    }
    vel_=radius_*(wl_+wr_)/2; omega_=radius_*(wr_-wl_)/lat_.effective_separation(); x_+=vel_*std::cos(a_+omega_*dt/2)*dt; y_+=vel_*std::sin(a_+omega_*dt/2)*dt;a_=std::remainder(a_+omega_*dt,2*M_PI);wl_pos_+=wl_*dt;wr_pos_+=wr_*dt;
    o=odom(); odom_pub_->publish(o); geometry_msgs::msg::TransformStamped tf;tf.header=o.header;tf.child_frame_id="base_link";tf.transform.translation.x=x_;tf.transform.translation.y=y_;tf.transform.translation.z=0.034346;tf.transform.rotation=o.pose.pose.orientation;tf_.sendTransform(tf);
    sensor_msgs::msg::JointState j;j.header.stamp=now();j.name={"lm_slide","shoulder","elbow","wrist_pitch","tool_roll","jaw_upper","jaw_lower","carousel","front_left_wheel","rear_left_wheel","front_right_wheel","rear_right_wheel"};j.position={0,0,0,0,0,0,0,0,wl_pos_,wl_pos_,wr_pos_,wr_pos_};joint_pub_->publish(j);
    if(trail_.poses.empty() || std::hypot(x_-trail_.poses.back().pose.position.x,y_-trail_.poses.back().pose.position.y)>0.01){geometry_msgs::msg::PoseStamped p;p.header=o.header;p.pose=o.pose.pose;trail_.poses.push_back(p);if(trail_.poses.size()>10000)trail_.poses.erase(trail_.poses.begin());}trail_.header.stamp=now();trail_pub_->publish(trail_);
    if(active_){auto point=lat_.lookahead_point();if(point){geometry_msgs::msg::PointStamped p;p.header=o.header;p.point=*point;look_pub_->publish(p);}}
    craft_interfaces::msg::ControlStatus st;st.header.stamp=now();st.ready=true;st.fault=fault_;st.held=inhibited();st.stopped=wl_==0 && wr_==0;st.reason=fault_?"simulated collision":(st.held?"held":"ready");status_pub_->publish(st);
    if(result_.task_id){result_.header.stamp=now(); result_.controllers[2]=active_?Result::ACTIVE:(result_.status==Result::SUCCEEDED?Result::DONE:(result_.status==Result::FAILED?Result::ERROR:Result::HALTED));result_pub_->publish(result_);}
    std_msgs::msg::String state;std::ostringstream ss;ss<<"{\"x\":"<<x_<<",\"y\":"<<y_<<",\"yaw\":"<<a_<<",\"speed\":"<<vel_<<",\"left_wheel\":"<<wl_<<",\"right_wheel\":"<<wr_<<",\"active_id\":"<<(active_?result_.task_id:0)<<",\"fault\":"<<(fault_?"true":"false")<<",\"held\":"<<(inhibited()?"true":"false")<<",\"paused\":"<<(paused_?"true":"false")<<",\"command_count\":"<<commands_<<"}";state.data=ss.str();state_pub_->publish(state);
  }
  craft_control::LateralController lat_;craft_control::LongitudinalController lon_;tf2_ros::TransformBroadcaster tf_;
  double sx_=0,sy_=0,sa_=0,x_=0,y_=0,a_=0,radius_=0,max_accel_=0,max_wheel_=0,speed_=0,vel_=0,omega_=0,wl_=0,wr_=0,wl_pos_=0,wr_pos_=0,deadline_=0;
  bool held_=true,have_heartbeat_=false,paused_=false,fault_=false,active_=false;uint64_t last_id_=0,commands_=0;Clock::time_point heartbeat_,last_tick_,task_start_;Result result_;nav_msgs::msg::Path trail_;
  rclcpp::Publisher<craft_interfaces::msg::ControlStatus>::SharedPtr status_pub_;rclcpp::Publisher<Result>::SharedPtr result_pub_;rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_,trail_pub_;rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr look_pub_;rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hold_sub_,collision_sub_;rclcpp::Subscription<Command>::SharedPtr task_sub_;rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv){rclcpp::init(argc,argv);try{rclcpp::spin(std::make_shared<Plant>());}catch(const std::exception &e){fprintf(stderr,"drive_plant: %s\n",e.what());rclcpp::shutdown();return 1;}rclcpp::shutdown();return 0;}
