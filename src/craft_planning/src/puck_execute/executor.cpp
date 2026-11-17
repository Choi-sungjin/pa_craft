#include "executor.hpp"
#include "legacy.hpp"
#include "motion.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cctype>
#include <csignal>
#include <iostream>
#include <thread>
#include <poll.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>
using namespace std::chrono_literals;
namespace puck::execute {
namespace {
std::atomic<bool> interrupted{false};
void on_signal(int) { interrupted.store(true); }
std::string input_line() {
  while(!interrupted.load()) { pollfd p{STDIN_FILENO,POLLIN|POLLHUP,0}; int n=std::cin.rdbuf()->in_avail()>0?1:poll(&p,1,100); if(n>0){std::string s;if(!std::getline(std::cin,s)) throw std::runtime_error("EOF when reading a line");auto a=s.find_first_not_of(" \t\r\n");auto b=s.find_last_not_of(" \t\r\n");s=a==std::string::npos?"":s.substr(a,b-a+1);std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});return s;} if(n<0 && errno!=EINTR) throw std::runtime_error("input poll failed"); }
  return "q";
}
bool starts(const std::string &s,const std::string &prefix) { return s.rfind(prefix,0)==0; }
std::vector<std::string> arm_names() { return {"slide_joint","shoulder_joint","elbow_joint","wrist_pitch_joint","tool_roll_joint"}; }
Steps generate(std::optional<double> slide={},std::optional<Pose> start={}) {
  if(options.has("--goto")) {
    const auto g=options.value("--goto");
    if(g=="cell") { auto s=stand_steps(slide,start);s.erase(std::remove_if(s.begin(),s.end(),[](const Step &x){return starts(x.name,"stand 0");}),s.end());return s; }
    if(g=="nominal") return {{"to the nominal pose",TC::MANIPULATOR,nominal_pose()}};
    if(g=="basic"||g=="via") { auto s=out_to_via(nominal_pose());if(g=="basic")s.push_back({"to the nominal pose",TC::MANIPULATOR,nominal_pose()});return s; }
    throw std::runtime_error("--goto nominal|via|basic|cell");
  }
  if(options.has("--lie"))return lie_steps(slide,start);
  if(options.has("--find")||options.has("--collect"))return find_steps(slide,start);
  if(options.has("--standcal"))return standcal_steps(slide,start);
  if(options.has("--stand"))return stand_steps(slide,start);
  if(options.has("--gripzero"))return gripzero_steps(slide,start);
  if(options.has("--gripcal"))return gripcal_steps(slide,start);
  if(options.has("--bigbro"))return bigbro_steps(slide,start);
  if(options.has("--dice"))return dice_steps(slide,start);
  if(options.has("--unload"))return unload_steps(slide,start);
  if(options.has("--lying"))return lying_steps_from_plan(slide,start);
  return steps_from_plan(slide,start);
}
void show(const Steps &s) { for(const auto &x:s)std::cout<<"  "<<x.name<<"  "<<describe(x.kind,x.arg)<<" "<<seconds_text(x.name)<<'\n'; }
TC command(int kind,const Json &target,double secs,double tol,const std::vector<int32_t> &gains,double contact) {
  TC c;c.kind=kind;
  if(kind==TC::MANIPULATOR) { c.duration_s=secs;c.tolerance_rad=tol;c.position_i_gains.assign(gains.begin(),gains.end());c.speed=contact;auto q=target.get<Pose>();c.joints.name=arm_names();if(q.size()==6)c.joints.name.push_back("gripper");c.joints.position=q; }
  else if(kind==TC::GRIPPER) { if(target.is_boolean())c.close=target.get<bool>();else {c.joints.name={"gripper"};c.joints.position={target.get<double>()};} }
  else if(kind==TC::CLOCK)c.clock_slot=target.get<int>();
  return c;
}
}
bool is_interrupted() { return interrupted.load(); }

bool run_steps(Runner &node,CameraContext &camera,Steps &steps) {
  bool automatic=options.has("--auto") || (std::find(options.args.begin(),options.args.end(),"choi")!=options.args.end() || std::find(options.args.begin(),options.args.end(),"auto")!=options.args.end());
  bool grasped_at_close=false;
  bool box_known=true;
  std::optional<Pose> last_manipulator_target;
  for(std::size_t i=0;i<steps.size();++i) {
    const auto name=steps[i].name;const int kind=steps[i].kind;Json arg=steps[i].arg;
    if(interrupted.load())return false;
    if(automatic)std::cout<<">> "<<name<<'\n';
    else {std::cout<<">> "<<name<<"? "<<std::flush;std::string answer=input_line();if(answer=="q")return false;if(answer=="bro")automatic=true;}
    if(starts(name,"bigbro pick: close") && options.real("--hold-before-close",0)>0) {
      double hold=options.real("--hold-before-close",0);std::cout<<"   holding "<<hold<<" s at the grasp pose with the jaws open (take the photo now)\n";gripzero_step(camera,"look");
      auto until=std::chrono::steady_clock::now()+std::chrono::duration<double>(hold);while(!interrupted.load()&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(50ms);
    }
    if(kind==PAUSE) {
      if(starts(name,"lie regrasp 1:"))box_known=false;
      if(starts(name,"lie regrasp 3:"))box_known=true;
      if(node.planned()) {if(!starts(name,"lie regrasp 1:")&&!starts(name,"lie regrasp 3:"))throw std::runtime_error("unhandled mission pause: "+name);if(!node.drive(options.value("--box"),starts(name,"lie regrasp 1:")))return false;}
      else {std::cout<<"   "<<arg.get<std::string>()<<" -- Enter when done (q = stop): "<<std::flush;std::string answer=input_line();if(answer=="q")return false;}
      continue;
    }
    if(kind>=100) {
      CameraStepResult result;
      if(kind==STANDFIND)result=find_step(camera,steps,i,arg);
      else if(kind==STANDCAL)result=standcal_step(camera,arg);
      else if(kind==STANDYAW)result=stand_yaw_step(camera,steps,i);
      else if(kind==LIEYAW)result=lie_yaw_step(camera,steps,i,arg);
      else if(kind==GRIPZERO)result=gripzero_step(camera,arg.get<std::string>());
      else if(kind==GRIPCAL)result=gripcal_step(camera,arg.get<std::string>());
      else if(kind==GRIPCHECK)result=gripcheck_step(camera,steps,i);
      else throw std::runtime_error("legacy gripper-camera step not yet ported: "+name);
      std::cout<<"   -> "<<(result.first?"SUCCEEDED ":"FAILED ")<<result.second<<'\n';if(!result.first)return false;continue;
    }
    if(kind==TC::MANIPULATOR && (starts(name,"bigbro pick: move 1")||starts(name,"bigbro pick: move 2")||starts(name,"bigbro fix"))) {auto p=path_arg(arg);for(auto &q:p)q[4]=0.;arg=p;}
    std::vector<Json> targets;
    if(kind==TC::MANIPULATOR)for(const auto &q:path_arg(arg))targets.emplace_back(q);else targets.push_back(arg);
    for(std::size_t k=0;k<targets.size();++k) {
      double tol=step_tolerance(name),secs=step_seconds(name,static_cast<int>(k));
      if(kind==TC::MANIPULATOR && name.find("the vertical line")!=std::string::npos && k+1<targets.size())tol=number("VIA_TOLERANCE_DEG")*M_PI/180.;
      if(kind==TC::MANIPULATOR && name.find("pick: descend")!=std::string::npos && targets.size()>1) {secs=number(starts(name,"stand pick: descend")?"STAND_DESCEND_PIECE_S":"DESCEND_PIECE_S");if(k+1<targets.size())tol=number("DESCEND_PASS_DEG")*M_PI/180.;}
      std::vector<motion::Move> moves{{kind==TC::MANIPULATOR?targets[k].get<Pose>():Pose{},secs,tol,false}};
      if(kind==TC::MANIPULATOR && motion::candidate(name)) {
        auto start=last_manipulator_target;
        auto feedback=start?std::optional<sensor_msgs::msg::JointState>{}:node.joints();
        if(feedback){
          const int64_t stamp=static_cast<int64_t>(feedback->header.stamp.sec)*1000000000LL+feedback->header.stamp.nanosec;
          const int64_t age=node.now().nanoseconds()-stamp;
          std::map<std::string,double> values;bool unique=feedback->name.size()==feedback->position.size();
          for(std::size_t j=0;j<std::min(feedback->name.size(),feedback->position.size());++j)unique=values.emplace(feedback->name[j],feedback->position[j]).second&&unique;
          Pose pose;if(unique&&age>=0&&age<=500000000LL)for(const auto &n:arm_names()){if(!values.count(n)){pose.clear();break;}pose.push_back(values[n]);}
          if(pose.size()==5)start=pose;
        }
        try {auto prepared=motion::prepare_motion(name,start,targets[k].get<Pose>(),secs,tol,node.grasped().value_or(true),box_known);moves=std::move(prepared.moves);if(!prepared.reason.empty())std::cout<<"   shoulder-last skipped: "<<prepared.reason<<'\n';}
        catch(const std::exception &e){std::cout<<"   shoulder-last rejected: "<<e.what()<<'\n';return false;}
        if(moves.size()>1 && !last_manipulator_target) {
          auto latest=node.joints();bool valid=latest.has_value();std::map<std::string,double> values;
          if(latest){const int64_t stamp=static_cast<int64_t>(latest->header.stamp.sec)*1000000000LL+latest->header.stamp.nanosec;const int64_t age=node.now().nanoseconds()-stamp;
            valid=age>=0&&age<=500000000LL&&latest->name.size()==latest->position.size();
            for(size_t j=0;j<std::min(latest->name.size(),latest->position.size());++j)valid=values.emplace(latest->name[j],latest->position[j]).second&&valid;
            auto names=arm_names();for(size_t j=0;j<names.size();++j){auto it=values.find(names[j]);if(it==values.end()||!std::isfinite(it->second)||std::abs(it->second-(*start)[j])>(j==0?.0005:.5*M_PI/180.))valid=false;}
          }
          if(!valid){std::cout<<"   shoulder-last rejected: feedback changed or became stale during clearance check\n";return false;}
        }
      }
      for(const auto &move:moves) {
        auto c=command(kind,kind==TC::MANIPULATOR?Json(move.target):targets[k],move.seconds,move.tolerance,kind==TC::MANIPULATOR?step_i_gains(name):std::vector<int32_t>{},kind==TC::MANIPULATOR&&!move.intermediate?step_contact(name):0.);
        auto id=node.send(c);
        if(kind==TC::MANIPULATOR)last_manipulator_target=Pose(move.target.begin(),move.target.begin()+5);
        auto [status,reason]=node.wait(id);std::cout<<"   -> "<<status<<" "<<reason<<'\n';if(status!="SUCCEEDED")return false;
      }
      if(starts(name,"bigbro 5:"))record_loaded(name,grasped_at_close);
      if(kind==TC::GRIPPER && arg.is_boolean() && arg.get<bool>()) {
        // Result and grasp feedback use different DDS topics. A healthy completed-step hold now
        // keeps publishing the grasp; allow that feedback to arrive before requiring confirmation.
        const auto grasp_deadline=std::chrono::steady_clock::now()+500ms;
        while(!node.grasped().value_or(false) && !is_interrupted() && std::chrono::steady_clock::now()<grasp_deadline)
          std::this_thread::sleep_for(10ms);
        grasped_at_close=node.grasped().value_or(false);
        if(node.planned() && !grasped_at_close){std::cout<<"mission pick: no confirmed grasp; stopping\n";return false;}
        if(starts(name,"bigbro pick: close")){auto [shot_ok,shot]=gripzero_step(camera,"look");(void)shot_ok;std::cout<<"   gripper cam: "<<shot<<'\n';auto bad=held_bad(camera,name,shot,steps,i);if(!bad.empty()){std::cout<<"   -> "<<bad<<": letting it go and stopping before the clock\n";node.wait(node.send(command(TC::GRIPPER,false,0,0,{},0)));for(std::size_t j=i+1;j<steps.size();++j)if(steps[j].name.find("pick: ascend")!=std::string::npos){auto q=path_arg(steps[j].arg).back();node.wait(node.send(command(TC::MANIPULATOR,q,0,0,step_i_gains(""),0)));break;}return false;}if(!grasped_at_close){std::cout<<"   -> the gripper reports no grasp: not carrying on to the clock\n";return false;}}
        if(camera.learn_ref && grasped_at_close)learn_reference(camera);
      }
    }
  }
  return true;
}
void go_home(Runner &node,CameraContext &camera,const Pose &start,bool lying) {
  std::this_thread::sleep_for(500ms);auto status=node.status();
  if(status && status->fault){std::cout<<"control FAULT (\""<<status->reason<<"\"): the board latched a fault and accepts nothing until it reboots.\n";node.stop();return;}
  std::cout<<"\nreturn to the start pose step by step? Enter = yes, n = no: "<<std::flush;if(input_line()=="n"){node.stop();return;}
  auto joints=node.joints();if(!joints){node.stop();return;}std::map<std::string,double> map;for(std::size_t i=0;i<std::min(joints->name.size(),joints->position.size());++i)map[joints->name[i]]=joints->position[i];
  Pose now;for(const auto &name:arm_names())now.push_back(map.at(name));double slide=std::clamp(now[0],-.043,.0627);auto raw=plan_pose()(20);Path rise{Pose(raw.begin()+1,raw.end())};
  if(lying){auto g=YAML::LoadFile((here()/"plan_lying.yaml").string())["puck_grasp_urdf"].as<Pose>();JointMap joint_map;auto names=arm_names();for(std::size_t i=0;i<5;++i)joint_map[names[i]]=now[i];Eigen::Vector3d tcp=chain().fk(joint_map).block<3,1>(0,3);if(tcp[1]<(g[1]+ASSUME_LIDAR_FRONT_Y)/2.){double h0=std::max(tcp[2]-g[2],0.)+number("STAGE_SEG")/1000.;Pose heights;for(double h=h0;h<=number("LYING_HIGH")+1e-6;h+=number("STAGE_SEG")/1000.)heights.push_back(h);auto up=lying_line(heights);if(up.empty())throw std::runtime_error("no recovery pose on lying line");auto path=tcp_path(up.back(),rise[0],static_cast<int>(number("LYING_SWING")));up.insert(up.end(),path.begin(),path.end());rise=up;}}
  Path full;for(const auto &r:rise){Pose q{slide};q.insert(q.end(),r.begin(),r.end());full.push_back(q);}Pose mid{start[0]};mid.insert(mid.end(),rise.back().begin(),rise.back().end());
  Steps steps={{"open gripper (drops a held puck)",TC::GRIPPER,false},{"rise above the puck height, LM stays",TC::MANIPULATOR,full},{"LM back to the start position",TC::MANIPULATOR,mid},{"arm back to the start pose",TC::MANIPULATOR,start}};
  steps=stage(with_via(std::move(steps),start,false),now);if(!run_steps(node,camera,steps)){std::cout<<"return stopped; sending STOP + hold\n";node.stop();}
}
int execute_main(const Options &opts) {
  configure(opts); interrupted.store(false);
  if(options.has("--planning")&&options.has("--lie")&&!options.has("--box"))throw std::runtime_error("--planning --lie requires --box NAME");
  auto steps=generate();
  if(options.has("--goto") || options.has("--lie") || options.has("--find") || options.has("--collect") || options.has("--standcal") || options.has("--stand") || options.has("--gripzero") || options.has("--gripcal") || options.has("--bigbro") || options.has("--dice") || options.has("--unload") || options.has("--lying")) steps=stage(std::move(steps));
  show(steps);
  if(options.has("--dry-run"))return 0;
  rclcpp::init(0,nullptr,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);std::signal(SIGINT,on_signal);std::signal(SIGTERM,on_signal);
  auto node=std::make_shared<Runner>(options.has("--planning"));rclcpp::executors::SingleThreadedExecutor spin;spin.add_node(node);
  std::thread thread([&]{spin.spin();});
  auto cleanup=[&]{if(interrupted.load())node->stop();spin.cancel();if(thread.joinable())thread.join();rclcpp::shutdown();};
  try {
    auto until=std::chrono::steady_clock::now()+3s;while(!node->status()&&std::chrono::steady_clock::now()<until&&!interrupted.load())std::this_thread::sleep_for(100ms);
    auto status=node->status();if(!status){std::cout<<"no /control/status: is control_master running?\n";cleanup();return 1;}
    std::this_thread::sleep_for(1s);status=node->status();std::cout<<"control: ready="<<status->ready<<" fault="<<status->fault<<" held="<<status->held<<" reason=\""<<status->reason<<"\"\n";
    if(!status->ready||status->fault){std::cout<<"control is not ready; aborting\n";cleanup();return 1;}
    auto joints=node->joints();if(!joints){std::cout<<"no /control/arm_joints; aborting\n";cleanup();return 1;}
    std::map<std::string,double> current;for(std::size_t i=0;i<std::min(joints->name.size(),joints->position.size());++i)current[joints->name[i]]=joints->position[i];
    Pose now;for(const auto &name:arm_names()){if(!current.count(name))throw std::runtime_error("missing arm joint "+name);now.push_back(current[name]);}
    double slide=now[0];auto start=nominal_pose();std::cout<<"LM now: slide "<<slide<<" m (nominal pose "<<start[0]<<")\n";
    if(slide<-.045||slide>.064){std::cout<<"LM position is outside the calibrated range (wrong start position?). Run checkarm; aborting\n";cleanup();return 1;}
    slide=std::clamp(slide,-.043,.0627);std::vector<std::string> outside;
    for(std::size_t i=1;i<5;++i){auto [lo,hi]=chain().limits(arm_names()[i]);if(now[i]<lo||now[i]>hi)outside.push_back(arm_names()[i]);}
    bool go=options.has("--goto");double lm_tol=(!go||options.has("--lm-tol-mm"))?number("LM_START_TOL"):1.;double joint_tol=(!go||!outside.empty()?90.:400.)*M_PI/180.;
    for(std::size_t i=0;i<5;++i)if(std::abs(now[i]-start[i])>(i==0?lm_tol:joint_tol)){std::cout<<"!! too far from the nominal pose ("<<arm_names()[i]<<"): move it closer by hand; aborting\n";cleanup();return 1;}
    CameraContext camera;camera.node=node.get();
    if(options.has("--find")&&!options.has("--dry-run"))camera.run_dir=here()/"runs"/("find_"+[] {std::time_t t=std::time(nullptr);std::tm local{};localtime_r(&t,&local);char b[32];std::strftime(b,sizeof(b),"%m%d_%H%M%S",&local);return std::string(b);}());
    bool collect=options.has("--collect");if(collect){auto target=redock_for_reach(camera,survey_pick(camera));C["POLICY_TARGET"]=target.record();if(target.posture=="lying"){int cell=std::abs(C["ROW_AHEAD_MM"]["far"].get<double>()-number("UPPER_AHEAD_BIAS_MM")-target.ahead)<=std::abs(C["ROW_AHEAD_MM"]["middle"].get<double>()-number("UPPER_AHEAD_BIAS_MM")-target.ahead)?2:5;options.set("--cell",std::to_string(cell));if(!options.has("--pick"))options.args.push_back("--pick");if(!options.has("--lie"))options.args.push_back("--lie");}}
    bool unload=options.has("--unload")||options.has("--stand")||options.has("--lie")||options.has("--standcal")||options.has("--find")||collect||options.has("--dice")||options.has("--bigbro")||options.has("--gripcal")||options.has("--gripzero");
    steps=generate(via_pose()[0],start);
    if(!go){if(!unload)steps=with_via(std::move(steps),start);else {auto via=via_pose();bool atvia=true;for(std::size_t i=0;i<5;++i)if(std::abs(now[i]-via[i])>=(i==0?.001:.5*M_PI/180.))atvia=false;if(atvia){std::cout<<"already at the via pose: the move out to it is skipped\n";steps.erase(std::remove_if(steps.begin(),steps.end(),[](const Step &s){return starts(s.name,"sis2 1:")||starts(s.name,"dice 0:")||starts(s.name,"bigbro 0:")||starts(s.name,"stand 0: to");}),steps.end());}}}
    else if(options.value("--goto")=="nominal") {if(!close_but_shoulder(now,start))steps.insert(steps.begin(),wrist_first(now,start,"the nominal pose"));}
    else if(options.value("--goto")!="cell") {steps=out_to_via(now);if(options.value("--goto")=="basic"){auto via=via_pose();if(!close_but_shoulder(via,start))steps.push_back(wrist_first(via,start,"the nominal pose"));steps.push_back({"to the nominal pose",TC::MANIPULATOR,start});}}
    if(unload||go||options.has("--lying"))steps=stage(std::move(steps),now);
    smooth_steps(steps,now,start);show(steps);
    bool automatic=options.has("--auto")||(std::find(options.args.begin(),options.args.end(),"choi")!=options.args.end() || std::find(options.args.begin(),options.args.end(),"auto")!=options.args.end());
    if(automatic){std::cout<<"\nAUTO (choi): every step runs without asking. Keep a hand on the 12 V power; Ctrl+C = STOP + hold.\n";if(!go)for(int i=3;i>0&&!interrupted.load();--i){std::cout<<"  starting in "<<i<<" s ...\n";std::this_thread::sleep_for(1s);}}
    else std::cout<<"\nKeep a hand on the 12 V power. Enter = run step, q + Enter = quit.\n\n";
    auto others=static_cast<long>(node->count_publishers("/planning/hold"))-1;if(others>0){std::cout<<"!! another program publishes /planning/hold: nothing sent\n";cleanup();return 1;}
    auto free_deadline=std::chrono::steady_clock::now()+5s;std::optional<std::chrono::steady_clock::time_point> free_since;bool ready=false;
    while(std::chrono::steady_clock::now()<free_deadline&&!interrupted.load()) {auto state=node->planning_state();if(node->planned()&&state&&*state=="Run.AwaitRequest"){ready=true;break;}auto s=node->status();if(s&&!s->held){if(!free_since)free_since=std::chrono::steady_clock::now();if(std::chrono::steady_clock::now()-*free_since>=500ms){ready=true;break;}}else free_since.reset();std::this_thread::sleep_for(50ms);}
    if(!ready){std::cout<<"control stays held (hold heartbeat / a stop not finished): nothing sent\n";cleanup();return 1;}
    if(options.has("--lie")&&!collect&&node->observation_subscribers("uppercam"))try{camera_run(node.get(),{(home()/"craft/tools/realsense_check/build/upper_puck").string()},30);}catch(const std::exception &e){std::cout<<"   upper-cam display context unavailable: "<<e.what()<<'\n';}
    bool ok=run_steps(*node,camera,steps);if(!ok||interrupted.load()){if(!interrupted.load()&&!go&&!unload)go_home(*node,camera,start,options.has("--lying"));else {std::cout<<"sending STOP + hold\n";node->stop();}std::this_thread::sleep_for(500ms);cleanup();return interrupted.load()?130:1;}
    node->publish_observation("uppercam",craft_perception::msg::PuckObservationArray());node->publish_observation("d435",craft_perception::msg::PuckObservationArray());std::cout<<"done: control keeps holding the last pose.\n";cleanup();return 0;
  } catch(...) {node->stop();std::this_thread::sleep_for(200ms);cleanup();if(interrupted.load())return 130;throw;}
}
}
