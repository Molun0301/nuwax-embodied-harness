#pragma once
#include <author_command_check.h>
#include <stdexcept>

namespace scan_planner {

// 一次重叠事件跨任务退休保留；只允许离开初始实体的同一侧，不能清图或穿墙。
class AuthorOverlapEscape {
  struct Contact { GridMap::BodyCell cell; Eigen::Vector2d axis; double depth; };
  bool active_=false,latched_=false;
  int64_t began_=0,last_stamp_=0;
  unsigned ticks_=0;
  double traveled_=0.;
  double admitted_=0.;
  double distance_limit_=0.,time_limit_=0.;
  static constexpr double escape_speed_=.04;
  // 三格误差带覆盖定位偏差与占据量化；只用于恢复准入，不改变碰撞外包。
  static double penetrationLimit(const ScanConfig& c) {return 3.*c.resolution;}
  std::string failure_;
  Eigen::Vector3d origin_=Eigen::Vector3d::Zero(),last_=Eigen::Vector3d::Zero();
  std::vector<Contact> contacts_;
  std::set<GridMap::BodyCell> initial_;

  static double radius(const ScanConfig& c) {
    return std::hypot(c.body_length/2.+std::abs(c.body_center_x),c.body_width/2.+std::abs(c.body_center_y));
  }
  static double depth(const Contact& contact,const ScanConfig& c,const Eigen::Vector3d& p,double yaw) {
    using namespace command_check_detail;
    const Eigen::Vector2d x=world(Eigen::Vector2d::UnitX(),yaw),y=world(Eigen::Vector2d::UnitY(),yaw);
    const Eigen::Vector2d center=p.head<2>()+world({c.body_center_x,c.body_center_y},yaw);
    const Eigen::Vector2d voxel((contact.cell[0]+.5)*c.resolution,(contact.cell[1]+.5)*c.resolution);
    return c.body_length/2.*std::abs(contact.axis.dot(x))+c.body_width/2.*std::abs(contact.axis.dot(y))
        +c.resolution/2.*contact.axis.cwiseAbs().sum()+contact.axis.dot(voxel-center);
  }

  bool advance(GridMap& map,const ScanConfig& c,command_check_detail::State& state,
      const Eigen::Vector2d& target,double angular,command_check_detail::Clock::time_point deadline,
      bool& exhausted,double& path_bound) const {
    using namespace command_check_detail;
    const auto next=toward(state.linear,target,c.max_acceleration*c.sample_dt);
    const double next_w=std::clamp(angular,state.angular-c.max_yaw_acceleration*c.sample_dt,
        state.angular+c.max_yaw_acceleration*c.sample_dt);
    const Eigen::Vector2d mean=(state.linear+next)*.5;
    const double mean_w=(state.angular+next_w)*.5;
    const double turn=std::max(std::abs(state.angular),std::abs(next_w));
    const double speed=std::max(state.linear.norm(),next.norm());
    path_bound+=(speed+radius(c)*turn)*c.sample_dt;
    if(admitted_+path_bound>distance_limit_)return false;
    for(const auto& contact:contacts_) {
      const Eigen::Vector2d voxel((contact.cell[0]+.5)*c.resolution,(contact.cell[1]+.5)*c.resolution);
      const Eigen::Vector2d local=body(voxel-state.position.head<2>(),state.yaw)
          -Eigen::Vector2d(c.body_center_x,c.body_center_y);
      const double distance=(voxel-state.position.head<2>()).norm()+c.resolution/std::sqrt(2.)+speed*c.sample_dt;
      const double cell_projection=c.resolution/2.*(std::abs(std::cos(state.yaw))+std::abs(std::sin(state.yaw)));
      const double movement=(speed+distance*turn)*c.sample_dt;
      for(int axis=0;axis<2;++axis)for(double sign:{-1.,1.}) {
        const double half=(axis==0?c.body_length:c.body_width)/2.;
        // 格内可能切割实体的机体半平面只可向内收缩：对格内每一点，
        // gap=sign*R(-yaw)*(point-reference)-offset-half 的导数下界非负。
        // 原来位于机体外的格内点因此始终在外，禁止同格内侧向新增扫掠。
        if(sign*local[axis]+cell_projection-half>=-movement
            &&std::min(-sign*state.linear[axis],-sign*next[axis])-distance*turn<0.)return false;
      }
      // 固定轴上的有向分离量导数下界；包含平移转轴和机体全部角点的转动。
      // 连续下界非负保证穿透不会先增后减，也不能从原实体另一侧退出。
      const double outward=std::min(contact.axis.dot(world(state.linear,state.yaw)),
          contact.axis.dot(world(next,state.yaw)))-speed*turn*c.sample_dt-radius(c)*turn;
      if(outward < 0.)return false;
    }
    if(!sweepFree(map,state,mean,mean_w,radius(c),0.,c.sample_dt,deadline,exhausted,&initial_))return false;
    auto end=motionAt(state,mean,mean_w,c.sample_dt);end.linear=next;end.angular=next_w;
    if(traveled_+(end.position-last_).norm()>distance_limit_)return false;
    state=end;return true;
  }

 public:
  bool active() const { return active_; }
  void observeClear(const Eigen::Vector3d& p,double resolution) {
    active_=false;
    // 离开至少两格才允许另一事件；新目标不能重置仍在原处的预算或故障锁。
    if(latched_&&(p-origin_).head<2>().norm()>2.*resolution)latched_=false;
  }

  void plan(GridMap& map,const ScanConfig& c,const Eigen::Vector3d& p,double yaw,
      const double* velocity,double angular,int64_t stamp,ScanOutput& output) {
    // 缺测尚不能开始恢复证明，保留原输入等待语义，不宣告几何恢复失败。
    if(!velocity||!std::isfinite(angular))
      throw std::runtime_error("Current body pose is occupied; escape requires measured velocity and yaw rate");
    const bool issued=latched_;
    try {
      planImpl(map,c,p,yaw,velocity,angular,stamp,output);
      // 只有通过整拍和制动证明并输出动作，才消耗跨目标事件。
      latched_=true;
    } catch(const std::runtime_error& error) {
      const std::string reason=error.what();
      if(!issued) *this=AuthorOverlapEscape{};
      else {active_=false;if(failure_.empty())failure_=reason;}
      throw std::runtime_error("BODY_OVERLAP_RECOVERY_FAILED: "+reason);
    }
  }

 private:
  void planImpl(GridMap& map,const ScanConfig& c,const Eigen::Vector3d& p,double yaw,
      const double* velocity,double angular,int64_t stamp,ScanOutput& output) {
    using namespace command_check_detail;
    if(!active_) {
      if(latched_)throw std::runtime_error(failure_.empty()
          ? "Current body pose is occupied; escape episode is already exhausted or completed"
          : failure_);
      origin_=last_=p;began_=last_stamp_=stamp;ticks_=0;traveled_=admitted_=0.;failure_.clear();
      contacts_.clear();initial_.clear();
      std::vector<GridMap::BodyCell> cells;
      if(map.bodyContacts(p,yaw,0.,&cells)!=1||cells.empty())
        throw std::runtime_error("Current body pose is occupied or outside the local map; no eligible contacts");
      for(const auto& cell:cells) {
        if(initial_.count(cell))continue;
        Contact selected{cell,Eigen::Vector2d::Zero(),std::numeric_limits<double>::infinity()};
        const Eigen::Vector2d center=p.head<2>()+world({c.body_center_x,c.body_center_y},yaw);
        const Eigen::Vector2d delta=center-Eigen::Vector2d((cell[0]+.5)*c.resolution,(cell[1]+.5)*c.resolution);
        for(auto axis:std::array<Eigen::Vector2d,4>{Eigen::Vector2d::UnitX(),Eigen::Vector2d::UnitY(),
            world(Eigen::Vector2d::UnitX(),yaw),world(Eigen::Vector2d::UnitY(),yaw)}) {
          if(axis.dot(delta)<0.)axis=-axis;
          const Contact candidate{cell,axis,0.};const double penetration=depth(candidate,c,p,yaw);
          if(penetration<selected.depth)selected={cell,axis,penetration};
        }
        if(!(selected.depth>=0.&&selected.depth<=penetrationLimit(c)))
          throw std::runtime_error("Current body pose is occupied; escape penetration exceeds localization and mapping allowance");
        contacts_.push_back(selected);initial_.insert(cell);
      }
      // 深重叠需要超过一格的退出行程；按初始最大穿透的两倍计入斜向退出，
      // 时间由同一行程和既有低速推出。浅接触保留原一格、两秒未确认预算。
      distance_limit_=c.resolution;
      for(const auto& contact:contacts_)distance_limit_=std::max(distance_limit_,2.*contact.depth);
      time_limit_=std::max(2.,distance_limit_/escape_speed_+c.sample_dt);
      active_=true;
    }
    if(stamp<last_stamp_){active_=false;throw std::runtime_error("Current body pose is occupied; escape source clock regressed");}
    bool progress_observed=false;
    for(const auto& contact:contacts_) {
      const double now=depth(contact,c,p,yaw);
      if(now>contact.depth+1e-10) {
        active_=false;
        throw std::runtime_error("Current body pose is occupied; measured escape penetration increased");
      }
      if(now<contact.depth)progress_observed=true;
    }
    // 只有新的实测采样证实退出进展，才结清未确认运动预算；目标切换不结清。
    if(stamp>last_stamp_&&progress_observed) {
      began_=stamp;ticks_=0;traveled_=admitted_=0.;
    }
    if(stamp>last_stamp_){traveled_+=(p-last_).norm();++ticks_;last_=p;last_stamp_=stamp;}
    if(stamp-began_>time_limit_*1e9||ticks_>static_cast<unsigned>(std::ceil(time_limit_/c.sample_dt))
        ||traveled_>=distance_limit_||admitted_>=distance_limit_) {
      active_=false;
      throw std::runtime_error("Current body pose is occupied; escape time or distance budget exhausted");
    }
    const State actual{p,yaw,body(Eigen::Map<const Eigen::Vector3d>(velocity).head<2>(),yaw),angular};
    const auto deadline=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(c.sample_dt));
    std::vector<Eigen::Vector2d> requests;Eigen::Vector2d sum=Eigen::Vector2d::Zero();
    for(const auto& contact:contacts_)sum+=contact.axis;
    if(sum.norm()>0.)requests.push_back(body(sum.normalized()*escape_speed_,yaw));
    for(const auto& contact:contacts_)requests.push_back(body(contact.axis*escape_speed_,yaw));
    // 机体系精确轴候选避免坐标往返舍入使严格单调条件失去本可执行的零切向量。
    for(int axis=0;axis<2;++axis)for(double sign:{-1.,1.})
      requests.emplace_back(axis==0?sign*escape_speed_:0.,axis==1?sign*escape_speed_:0.);
    for(int i=0;i<16;++i)requests.push_back(body(world(Eigen::Vector2d::UnitX(),i*2.*M_PI/16.)*escape_speed_,yaw));
    bool exhausted=false;
    for(const auto& requested:requests) {
      if(Clock::now()>=deadline)break;
      const auto command=bounded(requested,0.,actual,c,false);
      State state=actual;
      double path_bound=0.;
      std::vector<State> samples{actual};
      if(!advance(map,c,state,command.linear,command.angular,deadline,exhausted,path_bound))continue;
      const double issued_bound=path_bound;
      samples.push_back(state);
      bool safe=true;
      while(!state.linear.isZero(0.)||state.angular!=0.) {
        if(!advance(map,c,state,Eigen::Vector2d::Zero(),0.,deadline,exhausted,path_bound)){safe=false;break;}
        samples.push_back(state);
      }
      if(!safe)continue;
      bool progress=true;
      for(const auto& contact:contacts_)
        if(depth(contact,c,state.position,state.yaw)>=depth(contact,c,p,yaw))progress=false;
      if(!progress)continue;
      output.command_forward=command.linear.x();output.command_lateral=command.linear.y();
      output.command_yaw_rate=command.angular;output.command_duration=c.sample_dt;
      output.duration=(samples.size()-1)*c.sample_dt;output.sample_count=samples.size();
      output.samples=static_cast<ScanSample*>(std::calloc(samples.size(),sizeof(ScanSample)));
      if(!output.samples)throw std::bad_alloc();
      for(size_t i=0;i<samples.size();++i) {
        auto& out=output.samples[i];out.time=i*c.sample_dt;out.yaw=samples[i].yaw;out.yaw_rate=samples[i].angular;
        const auto v=world(samples[i].linear,samples[i].yaw);
        for(int j=0;j<3;++j){out.position[j]=samples[i].position[j];out.velocity[j]=j<2?v[j]:0.;}
        if(i>0)for(int j=0;j<3;++j)
          out.acceleration[j]=(out.velocity[j]-output.samples[i-1].velocity[j])/c.sample_dt;
      }
      output.motion_boundary_sources[0]=0;output.motion_boundary_sources[1]=2;output.motion_boundary_sources[2]=0;
      // 每次发出的整拍最大运动量都记入未确认窗口；重复来源时间不能绕过位移上限。
      admitted_+=issued_bound;
      for(auto& contact:contacts_)contact.depth=depth(contact,c,p,yaw);
      return;
    }
    throw std::runtime_error(exhausted?"Current body pose is occupied; escape proof computation exhausted"
        :"Current body pose is occupied; no monotone collision-free escape and braking sweep");
  }
};
}
