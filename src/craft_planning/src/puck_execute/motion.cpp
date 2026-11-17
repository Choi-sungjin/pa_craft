#include "motion.hpp"
#include <urdf_parser/urdf_parser.h>
#include <yaml-cpp/yaml.h>
#include <coal/collision.h>
#include <coal/BVH/BVH_model.h>
#include <coal/mesh_loader/loader.h>
#include <coal/shape/geometric_shapes.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>

namespace puck::motion {
namespace {
bool starts(const std::string &s,const std::string &prefix) { return s.rfind(prefix,0)==0; }
struct Box {
  Eigen::Vector3d lo,hi,half; Eigen::Matrix3d axes;
  coal::CollisionGeometryPtr_t geometry; coal::Transform3s transform;
  Box(const Eigen::Vector3d &low,const Eigen::Vector3d &high):lo(low),hi(high),half((high-low)/2.),axes(Eigen::Matrix3d::Identity()) {}
};
struct Visual { std::string name; Eigen::Matrix4d origin; Box box; };
Eigen::Matrix4d origin(const urdf::Pose &p) {
  Eigen::Matrix4d t=Eigen::Matrix4d::Identity();
  t.block<3,1>(0,3)=Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  t.block<3,3>(0,0)=Eigen::Quaterniond(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z).toRotationMatrix();
  return t;
}
Box transformed(const Eigen::Matrix4d &t,const Box &b) {
  Box r{Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()),Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity())};
  for(int i=0;i<8;++i) { Eigen::Vector4d p((i&1)?b.hi.x():b.lo.x(),(i&2)?b.hi.y():b.lo.y(),(i&4)?b.hi.z():b.lo.z(),1.);const Eigen::Vector3d x=(t*p).head<3>();r.lo=r.lo.cwiseMin(x);r.hi=r.hi.cwiseMax(x); }
  r.half=(b.hi-b.lo)/2.;r.axes=t.block<3,3>(0,0);r.geometry=b.geometry;r.transform=coal::Transform3s(r.axes,t.block<3,1>(0,3));return r;
}
bool overlap(const Box &a,const Box &b) {
  if(!(a.lo.array()<=b.hi.array()).all()||!(b.lo.array()<=a.hi.array()).all())return false;
  const Eigen::Matrix3d R=a.axes.transpose()*b.axes, absR=R.cwiseAbs().array()+1e-12;
  const Eigen::Vector3d t=a.axes.transpose()*((b.lo+b.hi-a.lo-a.hi)/2.);
  for(int i=0;i<3;++i)if(std::abs(t[i])>a.half[i]+absR.row(i).dot(b.half))return false;
  for(int j=0;j<3;++j)if(std::abs(t.dot(R.col(j)))>b.half[j]+a.half.dot(absR.col(j)))return false;
  for(int i=0;i<3;++i)for(int j=0;j<3;++j){int i1=(i+1)%3,i2=(i+2)%3,j1=(j+1)%3,j2=(j+2)%3;
    if(std::abs(t[i2]*R(i1,j)-t[i1]*R(i2,j))>a.half[i1]*absR(i2,j)+a.half[i2]*absR(i1,j)+b.half[j1]*absR(i,j2)+b.half[j2]*absR(i,j1))return false;}
  if(!a.geometry||!b.geometry)throw std::runtime_error("missing collision geometry");
  coal::CollisionRequest request;coal::CollisionResult result;
  return coal::collide(a.geometry.get(),a.transform,b.geometry.get(),b.transform,request,result)>0;
}
Box axis_box(const Eigen::Vector3d &lo,const Eigen::Vector3d &hi) {
  Box b(lo,hi);b.geometry=std::make_shared<coal::Box>(hi-lo);b.transform=coal::Transform3s(Eigen::Vector3d((lo+hi)/2.));return b;
}
Box read_stl(const fs::path &path,const Eigen::Vector3d &scale) {
  std::ifstream f(path,std::ios::binary);if(!f)throw std::runtime_error("cannot read mesh: "+path.string());
  char head[80];uint32_t n=0;f.read(head,80);f.read(reinterpret_cast<char*>(&n),4);
  auto size=fs::file_size(path);if(!f||!n||size!=84ULL+50ULL*n)throw std::runtime_error("mesh is not binary STL: "+path.string());
  Box b(Eigen::Vector3d::Constant(INFINITY),Eigen::Vector3d::Constant(-INFINITY));
  for(uint32_t i=0;i<n;++i){char buf[50];f.read(buf,50);
    for(int v=0;v<3;++v){Eigen::Vector3d point;
      for(int j=0;j<3;++j){float x;std::memcpy(&x,buf+12+v*12+j*4,4);if(!std::isfinite(x))throw std::runtime_error("nonfinite mesh");point[j]=x*scale[j];}
      b.lo=b.lo.cwiseMin(point);b.hi=b.hi.cwiseMax(point);
    }
  }
  return b;
}

std::string motor_bad(const Pose &q) {
  static const auto p=YAML::LoadFile(control_yaml().string())["control_master"]["ros__parameters"];
  const auto offsets=p["arm_joint_offsets"].as<std::vector<double>>(), directions=p["arm_joint_directions"].as<std::vector<double>>();
  const auto lows=p["arm_min_angle"].as<std::vector<double>>(), highs=p["arm_max_angle"].as<std::vector<double>>();
  if(offsets.size()!=4||directions.size()!=4||lows.size()!=4||highs.size()!=4)throw std::runtime_error("invalid motor configuration");
  for(size_t i=0;i<4;++i){const double v=offsets[i]+directions[i]*q[i+1];if(!(lows[i]<=v&&v<=highs[i]))return Chain::NAMES[i+1];}
  const double scale=p["lm_meters_per_rad"].as<double>();if(!std::isfinite(scale)||scale==0.)throw std::runtime_error("invalid LM configuration");
  const double lm=p["lm_motor_offset"].as<double>()+p["lm_motor_direction"].as<double>()*q[0]/scale;
  if(!(p["lm_min_angle"].as<double>()<=lm&&lm<=p["lm_max_angle"].as<double>()))return "LM";
  return {};
}
struct Model {
  urdf::ModelInterfaceSharedPtr urdf;
  std::vector<Visual> visuals;
  const Box box=axis_box({.024,-.171,0.},{.232,.034,.070});
  Model() {
    urdf=urdf::parseURDFFile(urdf_path().string());if(!urdf)throw std::runtime_error("cannot read URDF");
    coal::MeshLoader loader;
    for(const auto &[name,link]:urdf->links_) for(const auto &v:link->visual_array) {
      if(!v||!v->geometry||v->geometry->type!=urdf::Geometry::MESH)throw std::runtime_error("unsupported visual: "+name);
      const auto mesh=std::dynamic_pointer_cast<urdf::Mesh>(v->geometry);if(!mesh)throw std::runtime_error("invalid mesh: "+name);
      Eigen::Vector3d scale(mesh->scale.x,mesh->scale.y,mesh->scale.z);if((scale.array()<=0).any())throw std::runtime_error("invalid mesh scale");
      auto path=urdf_path().parent_path().parent_path()/"meshes"/fs::path(mesh->filename).filename();
      auto bounds=read_stl(path,scale);bounds.geometry=loader.load(path.string(),scale);
      if(!bounds.geometry)throw std::runtime_error("failed to load collision mesh: "+name);
      visuals.push_back({name,origin(v->origin),bounds});
    }
  }
  std::map<std::string,Eigen::Matrix4d> transforms(const Pose &q,double jaw,double carousel=0.) const {
    std::map<std::string,double> values;for(size_t i=0;i<5;++i)values[Chain::NAMES[i]]=q[i];values["jaw_upper_joint"]=jaw;values["jaw_lower_joint"]=jaw;values["carousel_joint"]=carousel;
    std::map<std::string,Eigen::Matrix4d> out{{"base_link",Eigen::Matrix4d::Identity()}};
    std::vector<urdf::JointSharedPtr> left;for(const auto &kv:urdf->joints_)left.push_back(kv.second);
    while(!left.empty()) {
      std::vector<urdf::JointSharedPtr> next;
      for(const auto &j:left) {
        if(!out.count(j->parent_link_name)){next.push_back(j);continue;}
        Eigen::Matrix4d t=out.at(j->parent_link_name)*origin(j->parent_to_joint_origin_transform);
        if(j->type!=urdf::Joint::FIXED) {
          Eigen::Vector3d axis(j->axis.x,j->axis.y,j->axis.z);if(axis.norm()==0.)throw std::runtime_error("invalid joint axis");axis.normalize();double value=values[j->name];
          Eigen::Matrix4d motion=Eigen::Matrix4d::Identity();
          if(j->type==urdf::Joint::PRISMATIC)motion.block<3,1>(0,3)=axis*value;
          else if(j->type==urdf::Joint::REVOLUTE||j->type==urdf::Joint::CONTINUOUS)motion.block<3,3>(0,0)=Eigen::AngleAxisd(value,axis).toRotationMatrix();
          else throw std::runtime_error("unsupported joint");
          t=t*motion;
        }
        out[j->child_link_name]=t;
      }
      if(next.size()==left.size())throw std::runtime_error("unconnected URDF");
      left=std::move(next);
    }
    return out;
  }
  std::string check(const Pose &q,bool held,bool box_known) const {
    if(!box_known)return "box position unknown after drive";
    for(size_t i=0;i<5;++i){auto lim=chain().limits(Chain::NAMES[i]);if(q[i]<lim.first||q[i]>lim.second)return "joint limit: "+Chain::NAMES[i];}
    const auto bad=motor_bad(q);if(!bad.empty())return "motor limit: "+bad;
    auto base=transforms(q,0.);std::vector<std::pair<std::string,Box>> fixed;
    for(const auto &v:visuals)if(starts(v.name,"base_link"))fixed.emplace_back(v.name,transformed(base.at(v.name)*v.origin,v.box));
    // Unknown carousel angle: twelve discrete modeled rotations, not a continuous sweep.
    for(int i=0;i<12;++i){auto clock=transforms(q,0.,i*2.*M_PI/12.);bool found=false;
      for(const auto &v:visuals)if(starts(v.name,"carousel_link")){found=true;fixed.emplace_back(v.name,transformed(clock.at(v.name)*v.origin,v.box));}
      if(!found)return "carousel geometry missing";
    }
    for(double jaw:{-.021,.01}) {
      auto t=transforms(q,jaw);std::vector<std::pair<std::string,Box>> moving;
      for(const auto &v:visuals)if(starts(v.name,"shoulder_link")||starts(v.name,"elbow_link")||starts(v.name,"wrist_pitch_link")||starts(v.name,"tool_roll_link")||starts(v.name,"jaw_upper_link")||starts(v.name,"jaw_lower_link"))moving.emplace_back(v.name,transformed(t.at(v.name)*v.origin,v.box));
      if(moving.empty())return "arm geometry missing";
      if(held){Eigen::Vector3d tcp=t.at("tool_tcp").block<3,1>(0,3);moving.emplace_back("held puck envelope",axis_box(tcp-Eigen::Vector3d::Constant(.06),tcp+Eigen::Vector3d::Constant(.06)));}
      for(const auto &[name,b]:moving) {
        if(b.lo.z()<0.)return name+" below floor";
        if(overlap(b,box))return name+" intersects box";
        for(const auto &[obj,obstacle]:fixed){if(starts(name,"shoulder_link")&&starts(obj,"base_link"))continue;if(overlap(b,obstacle))return name+" intersects "+obj;}
      }
      for(const auto &[shoulder,a]:moving)if(starts(shoulder,"shoulder_link"))
        for(const auto &[other,b]:moving)if((starts(other,"tool_roll_link")||starts(other,"jaw_upper_link")||starts(other,"jaw_lower_link"))&&overlap(a,b))return shoulder+" intersects "+other;
    }
    return {};
  }
};
}
bool candidate(const std::string &name) {
  for(const char *p:{"standfind 2:","standfind 4:","stand pick: descend","stand pick: up","stand pick: approach","lie ","rise above the puck height, LM stays","LM back to the start position","arm back to the start pose","via pose before the nominal pose","wrist_pitch to"})if(starts(name,p))return true;
  for(int i=2;i<=9;++i)if(starts(name,"stand load "+std::to_string(i)+":"))return true;
  return false;
}
Prepared prepare_motion(const std::string &name,const std::optional<Pose> &start,const Pose &target,double seconds,double tolerance,bool held,bool box_known) {
  Prepared original{{Move{target,seconds,tolerance,false}}, {}};
  if(!candidate(name))return original;
  if(!std::isfinite(SHOULDER_EARLY_FRACTION)||SHOULDER_EARLY_FRACTION<0.||SHOULDER_EARLY_FRACTION>1.)throw std::invalid_argument("invalid shoulder early fraction");
  if(!start||start->size()!=5||(target.size()!=5&&target.size()!=6))throw std::invalid_argument("missing or invalid joint feedback/target");
  for(size_t i=0;i<5;++i)if(!std::isfinite((*start)[i]))throw std::invalid_argument("nonfinite joint feedback/target");
  for(double v:target)if(!std::isfinite(v))throw std::invalid_argument("nonfinite joint feedback/target");
  for(size_t i=0;i<5;++i){auto lim=chain().limits(Chain::NAMES[i]);if(target[i]<lim.first||target[i]>lim.second){
    // Lying-puck moves (taught regrasp poses past the URDF limits, user 2026-11-16): no URDF limit, the move goes
    // out as taught in one piece (control/firmware still enforce the motor limits).
    if(starts(name,"lie")){original.reason="lying puck: URDF limit of "+Chain::NAMES[i]+" not applied, one move";return original;}
    throw std::invalid_argument("target joint limit: "+Chain::NAMES[i]);}}
  const auto bad=motor_bad(target);if(!bad.empty())throw std::invalid_argument("target motor limit: "+bad);
  bool other=false;for(size_t i:{0,2,3,4})other=other||std::abs(target[i]-(*start)[i])>=1e-10;
  if(std::abs(target[1]-(*start)[1])<1e-10||!other){original.reason="single-axis group";return original;}
  Pose mid=target;mid[1]=(*start)[1]+SHOULDER_EARLY_FRACTION*(target[1]-(*start)[1]);
  try {
    static const Model model;
    for(const auto &segment:std::array<std::pair<Pose,Pose>,2>{{{*start,mid},{mid,target}}}) {
      const auto &a=segment.first,&b=segment.second;double ratio=std::abs(b[0]-a[0])/.002;
      for(size_t i=1;i<5;++i)ratio=std::max(ratio,std::abs(b[i]-a[i])/(2.*M_PI/180.));
      int count=std::max(1,static_cast<int>(std::ceil(ratio)));
      if(count>2000){original.reason="sampling limit";return original;}
      for(int k=0;k<=count;++k) {Pose q(5);for(size_t i=0;i<5;++i)q[i]=a[i]+(b[i]-a[i])*k/count;auto why=model.check(q,held,box_known);if(!why.empty()){original.reason=why;return original;}}
    }
  } catch(const std::exception &e) {original.reason="clearance model unavailable: "+std::string(e.what());return original;}
  // Fold-first segment 1 only clears the way; segment 2 brings every joint to the target. 1.5 deg here, not 0.5:
  // the last 0.5 deg took seconds of creeping (integ14 08:5x bag).
  const double intermediate_tol=std::max(tolerance,1.5*M_PI/180.);
  return {{{mid,seconds,intermediate_tol,true},{target,seconds,tolerance,false}}, {}};
}
}
