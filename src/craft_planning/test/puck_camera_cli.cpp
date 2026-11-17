#include "camera.hpp"
#include <iostream>
#include <string>
using namespace puck::execute;
using puck::Pose;
int main(int argc,char **argv) {
 try {
  if(argc!=2)throw std::runtime_error("expected scenario");
  Options opts;opts.args={"--find","--cell","5","--pick","--dry-run"};configure(opts);
  CameraContext ctx;std::string output;std::getline(std::cin,output,'\0');
  ctx.capture=[&](const std::vector<std::string>&,double){return CameraResult{0,output,{}};};
  const std::string mode=argv[1];
  if(mode=="process") {
   auto data=Json::parse(output);
   try {auto p=camera_run(nullptr,data.at("argv").get<std::vector<std::string>>(),data.at("timeout").get<double>());std::cout<<"RESULT "<<Json({{"returncode",p.returncode},{"out",p.out},{"err",p.err}}).dump()<<'\n';}
   catch(const std::exception &e) {std::cout<<"RESULT "<<Json({{"error",e.what()}}).dump()<<'\n';}
   return 0;
  }
  CameraStepResult r;Steps steps;
  Pose q=wait_at(112.5,dice_lm("centre").first);
  if(mode=="stand_yaw") {
   steps={{"wait",TC::MANIPULATOR,Path{q}},{"camera",STANDYAW,nullptr},{"stand pick: descend",TC::MANIPULATOR,Path{q}}};r=stand_yaw_step(ctx,steps,1);
  } else if(mode=="lie_yaw") {
   steps={{"wait",TC::MANIPULATOR,Path{q}},{"camera",LIEYAW,nullptr},{"lie pick: descend",TC::MANIPULATOR,Path{q}}};r=lie_yaw_step(ctx,steps,1,{{"cell",5},{"roll0",lie_grasp(5)[4]}});
  } else if(mode=="upper") {
   steps={{"upper",STANDFIND,{{"what","upper"}}},{"standfind 2: wait",TC::MANIPULATOR,Path{q}}};r=find_step(ctx,steps,0,steps[0].arg);
  } else if(mode=="grip") {
   ctx.find={{"cls","red_prism"},{"ahead",112.5},{"lat",0.},{"lm",q[0]}};
   steps={{"grip",STANDFIND,{{"what","grip"}}},{"standfind 4: approach",TC::MANIPULATOR,Path{q}},{"stand pick: descend",TC::MANIPULATOR,Path{q}},{"stand pick: up to 75 mm",TC::MANIPULATOR,Path{q}}};r=find_step(ctx,steps,0,steps[0].arg);
  } else if(mode=="photo") {
   steps={{"photo",STANDFIND,{{"what","photo"},{"tag","grasp_open"}}}};r=find_step(ctx,steps,0,steps[0].arg);
  } else if(mode=="standcal_solve") {
   ctx.cal_shots={{"ref",{{"centre",{10.,20.,60.}},{"yaw",2.},{"dlm",0.},{"dahead",0.}}}, {"lm+10",{{"centre",{20.,20.,60.}},{"yaw",2.},{"dlm",10.},{"dahead",0.}}}, {"lm-10",{{"centre",{0.,20.,60.}},{"yaw",2.},{"dlm",-10.},{"dahead",0.}}}, {"ahead+10",{{"centre",{10.,30.,60.}},{"yaw",2.},{"dlm",0.},{"dahead",10.}}}, {"ahead-10",{{"centre",{10.,10.,60.}},{"yaw",2.},{"dlm",0.},{"dahead",-10.}}}};
   r=standcal_step(ctx,{{"what","solve"},{"cell",5},{"row","middle"},{"ahead",112.5},{"lm",q[0]}});
  } else if(mode=="standcal_grip") {
   r=standcal_step(ctx,{{"what","grip"},{"tag","ref"},{"dlm",0.},{"dahead",0.}});
  } else if(mode=="grip3") {
   ctx.find={{"ahead2",112.5},{"lm2",q[0]},{"roll",q[4]}};steps={{"final",STANDFIND,{{"what","grip3"}}}};r=find_step(ctx,steps,0,steps[0].arg);
  } else throw std::runtime_error("unknown scenario");
  Json st=Json::array();for(const auto &x:steps)st.push_back({{"name",x.name},{"kind",x.kind},{"arg",x.arg}});
  std::cout<<"RESULT "<<Json({{"ok",r.first},{"reason",r.second},{"steps",st},{"find",ctx.find},{"shots",ctx.cal_shots}}).dump()<<'\n';return 0;
 } catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
