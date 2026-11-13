#pragma once
#include <author_command_check.h>
#include <stdexcept>
#include <optional>

namespace scan_planner {

// 同一事件保留退出方向和运动记账；连续半秒来源时间无重叠或新目标才结束事件。
class AuthorOverlapEscape {
  struct Contact { GridMap::BodyCell cell; Eigen::Vector2d axis; double depth; double remaining=0.; bool body_axis=false; bool directed=false; };
  bool active_=false,latched_=false,recontact_=false,waiting_for_stop_=false;
  std::optional<int64_t> clear_since_;
  std::optional<int64_t> decision_stamp_;
  size_t decision_periods_=1,decision_calls_=0;
  std::vector<ScanSample> decision_samples_;
  ScanOutput decision_output_{};
  int64_t began_=0,last_stamp_=0;
  unsigned ticks_=0;
  double traveled_=0.;
  double admitted_=0.;
  double distance_limit_=0.,time_limit_=0.;
  static double escapeSpeed(const ScanConfig& c) {return std::max(.04,c.min_executable_linear_mps);}
  // 三格误差带覆盖定位偏差与占据量化；只用于恢复准入，不改变碰撞外包。
  static double penetrationLimit(const ScanConfig& c) {return 3.*c.resolution;}
  std::string failure_;
  Eigen::Vector3d last_=Eigen::Vector3d::Zero();
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
    const Eigen::Vector2d axis=contact.body_axis?world(contact.axis,yaw):contact.axis;
    return c.body_length/2.*std::abs(axis.dot(x))+c.body_width/2.*std::abs(axis.dot(y))
        +c.resolution/2.*axis.cwiseAbs().sum()+axis.dot(voxel-center);
  }

  // 四条分离轴、两个有向侧面共同给出真实平面穿透；已分离时取零。
  static std::array<Eigen::Vector2d,4> axes(double yaw) {
    using namespace command_check_detail;
    return {Eigen::Vector2d::UnitX(),Eigen::Vector2d::UnitY(),
        world(Eigen::Vector2d::UnitX(),yaw),world(Eigen::Vector2d::UnitY(),yaw)};
  }
  static double penetration(const Contact& contact,const ScanConfig& c,const Eigen::Vector3d& p,double yaw) {
    double result=std::numeric_limits<double>::infinity();
    for(const auto& axis:axes(yaw))for(double sign:{-1.,1.})
      result=std::min(result,depth({contact.cell,sign*axis,0.},c,p,yaw));
    return std::max(0.,result);
  }

  // 每段运动沿用执行器的恒定平均机体系速度。象限内各有向分离量为
  // d(t)=d(0)+a*sin(wt)/w+b*(1-cos(wt))/w；其导数为 a*cos(wt)+b*sin(wt)。
  // 按分离量交点、零点和导数零点分段，逐段检查下包络，证明整段穿透非增。
  static bool monotone(const Contact& contact,const ScanConfig& c,
      const command_check_detail::State& start,const Eigen::Vector2d& mean,double w,double duration) {
    using namespace command_check_detail;
    const Eigen::Vector2d voxel((contact.cell[0]+.5)*c.resolution,(contact.cell[1]+.5)*c.resolution);
    const double tolerance=64.*std::numeric_limits<double>::epsilon()*
        (1.+mean.norm()+std::abs(w)*(radius(c)+(voxel-start.position.head<2>()).norm()));
    if(w==0.) {
      double best=std::numeric_limits<double>::infinity(),rate=0.;
      for(const auto& axis:axes(start.yaw))for(double sign:{-1.,1.}) {
        const double d=depth({contact.cell,sign*axis,0.},c,start.position,start.yaw);
        const double r=-sign*axis.dot(world(mean,start.yaw));
        if(d<best){best=d;rate=r;}else if(d==best)rate=std::min(rate,r);
      }
      // 固定朝向下穿透是仿射函数的下包络，导数只会下降；允许切向平台段。
      return rate<=tolerance||best+rate*duration<0.;
    }
    std::vector<double> cuts{0.,duration};
    const double finish=start.yaw+w*duration;
    for(int k=static_cast<int>(std::floor(std::min(start.yaw,finish)/(M_PI/2.)))+1;
        k*M_PI/2.<std::max(start.yaw,finish);++k)cuts.push_back((k*M_PI/2.-start.yaw)/w);
    std::sort(cuts.begin(),cuts.end());
    for(size_t part=1;part<cuts.size();++part) {
      const auto state=motionAt(start,mean,w,cuts[part-1]);
      const double span=cuts[part]-cuts[part-1],theta=state.yaw;
      const double co=std::cos(theta),si=std::sin(theta),mid=theta+w*span/2.;
      const double sc=std::cos(mid)>=0.?1.:-1.,ss=std::sin(mid)>=0.?1.:-1.;
      const Eigen::Vector2d offset(c.body_center_x,c.body_center_y);
      const Eigen::Vector2d center_velocity=mean+w*Eigen::Vector2d(-offset.y(),offset.x());
      const Eigen::Vector2d local=body(state.position.head<2>()-voxel,theta);
      const Eigen::Vector2d local_rate=mean-w*Eigen::Vector2d(-local.y(),local.x());
      struct Curve {double d,a,b;};std::array<Curve,8> curves;size_t index=0;
      for(int axis=0;axis<4;++axis)for(double sign:{-1.,1.}) {
        const auto direction=axes(theta)[axis];
        double a,b;
        if(axis<2) {
          const double hx=(axis==0?c.body_length:c.body_width)/2.;
          const double hy=(axis==0?c.body_width:c.body_length)/2.;
          a=w*(-hx*sc*si+hy*ss*co);b=w*(-hx*sc*co-hy*ss*si);
          a-=sign*world(center_velocity,theta)[axis];
          b-=sign*world(Eigen::Vector2d(-center_velocity.y(),center_velocity.x()),theta)[axis];
        } else {
          const int j=axis-2;
          a=c.resolution/2.*w*(-sc*si+ss*co)-sign*local_rate[j];
          b=c.resolution/2.*w*(-sc*co-ss*si)
              +sign*Eigen::Vector2d(-local_rate.y(),local_rate.x())[j];
        }
        curves[index++]={depth({contact.cell,sign*direction,0.},c,state.position,theta),a,b};
      }
      std::vector<double> events{0.,span};
      const auto roots=[&](double A,double B,double C) {
        const double amplitude=std::hypot(A,B);if(amplitude==0.||std::abs(C)>amplitude)return;
        const double phase=std::atan2(B,A),angle=std::acos(std::clamp(-C/amplitude,-1.,1.));
        for(double root:{phase-angle,phase+angle}) {
          const double low=std::min(0.,w*span),high=std::max(0.,w*span);
          for(int k=static_cast<int>(std::ceil((low-root)/(2.*M_PI)));root+k*2.*M_PI<high;++k) {
            const double t=(root+k*2.*M_PI)/w;if(t>0.&&t<span)events.push_back(t);
          }
        }
      };
      for(size_t i=0;i<curves.size();++i) {
        const auto& v=curves[i];roots(v.a,v.b,0.);
        roots(-v.b,v.a,w*v.d+v.b);
        for(size_t j=0;j<i;++j) {
          const auto& u=curves[j];roots(u.b-v.b,v.a-u.a,w*(v.d-u.d)+v.b-u.b);
        }
      }
      std::sort(events.begin(),events.end());
      for(size_t i=1;i<events.size();++i) {
        const double t=(events[i-1]+events[i])/2.,angle=w*t;
        double best=std::numeric_limits<double>::infinity(),rate=0.;
        for(const auto& v:curves) {
          const double d=v.d+v.a*std::sin(angle)/w+v.b*2.*std::pow(std::sin(angle/2.),2)/w;
          const double r=v.a*std::cos(angle)+v.b*std::sin(angle);
          if(d<best){best=d;rate=r;}else if(d==best)rate=std::min(rate,r);
        }
        if(best>0.&&rate>tolerance)return false;
      }
    }
    return true;
  }

  // 实测瞬时速度不能先向内再靠本拍平均位移抵消；在切向平台处允许数值零。
  static bool startsOutward(const Contact& contact,const ScanConfig& c,
      const command_check_detail::State& state,double allowance=0.) {
    using namespace command_check_detail;
    const double co=std::cos(state.yaw),si=std::sin(state.yaw),w=state.angular;
    const auto abs_rate=[](double value,double rate) {
      return std::abs(value)<64.*std::numeric_limits<double>::epsilon()?std::abs(rate):std::copysign(1.,value)*rate;
    };
    const double cell_rate=c.resolution/2.*(abs_rate(co,-w*si)+abs_rate(si,w*co));
    const Eigen::Vector2d voxel((contact.cell[0]+.5)*c.resolution,(contact.cell[1]+.5)*c.resolution);
    const Eigen::Vector2d local=body(state.position.head<2>()-voxel,state.yaw);
    const Eigen::Vector2d local_rate=state.linear-w*Eigen::Vector2d(-local.y(),local.x());
    const Eigen::Vector2d center_velocity=world(state.linear+w*Eigen::Vector2d(-c.body_center_y,c.body_center_x),state.yaw);
    double best=std::numeric_limits<double>::infinity(),rate=0.;
    const auto directions=axes(state.yaw);
    for(int i=0;i<4;++i)for(double sign:{-1.,1.}) {
      const double d=depth({contact.cell,sign*directions[i],0.},c,state.position,state.yaw);
      const double r=i<2?(i==0
          ?c.body_length/2.*abs_rate(co,-w*si)+c.body_width/2.*abs_rate(si,w*co)
          :c.body_length/2.*abs_rate(si,w*co)+c.body_width/2.*abs_rate(co,-w*si))-sign*center_velocity[i]
          :cell_rate-sign*local_rate[i-2];
      if(d<best){best=d;rate=r;}else if(d==best)rate=std::min(rate,r);
    }
    return best<0.||rate<=allowance+64.*std::numeric_limits<double>::epsilon()*
        (1.+state.linear.norm()+std::abs(w)*(radius(c)+local.norm()));
  }

  bool advance(GridMap& map,const ScanConfig& c,command_check_detail::State& state,
      const Eigen::Vector2d& target,double angular,command_check_detail::Clock::time_point deadline,
      bool& exhausted,double& path_bound,bool& budget_exhausted,bool measured_stopped=false) const {
    using namespace command_check_detail;
    const auto next=toward(state.linear,target,c.max_acceleration*c.sample_dt);
    const double next_w=std::clamp(angular,state.angular-c.max_yaw_acceleration*c.sample_dt,
        state.angular+c.max_yaw_acceleration*c.sample_dt);
    const Eigen::Vector2d mean=(state.linear+next)*.5;
    const double mean_w=(state.angular+next_w)*.5;
    const double turn=std::max(std::abs(state.angular),std::abs(next_w));
    const double speed=std::max(state.linear.norm(),next.norm());
    path_bound+=(speed+radius(c)*turn)*c.sample_dt;
    if(admitted_+path_bound>distance_limit_){budget_exhausted=true;return false;}
    // 停稳实测只将起步导数按零速证明；连续运动及制动扫掠仍使用原实测边界。
    State start=state;
    if(measured_stopped){start.linear.setZero();start.angular=0.;}
    for(const auto& contact:contacts_) {
      if(!startsOutward(contact,c,start)||!monotone(contact,c,state,mean,mean_w,c.sample_dt))return false;
    }
    if(!sweepFree(map,state,mean,mean_w,radius(c),0.,c.sample_dt,deadline,exhausted,&initial_))return false;
    auto end=motionAt(state,mean,mean_w,c.sample_dt);end.linear=next;end.angular=next_w;
    if((end.position-last_).norm()>distance_limit_){budget_exhausted=true;return false;}
    state=end;return true;
  }

 public:
  bool active() const { return active_; }
  bool hasDecision(int64_t stamp) const {return decision_stamp_&&*decision_stamp_==stamp;}
  void observeOverlap() {
    if(clear_since_){recontact_=true;clear_since_.reset();}
  }
  void observeClear(const Eigen::Vector3d& p,int64_t stamp) {
    if(!active_)return;
    if(!clear_since_||stamp<last_stamp_)clear_since_=stamp;
    if(stamp-*clear_since_>=500000000){*this=AuthorOverlapEscape{};return;}
    // 暂时清除保留原事件，以确认进展点为原点计算净位移。
    if(stamp>last_stamp_){traveled_=(p-last_).norm();++ticks_;last_stamp_=stamp;}
  }

  void plan(GridMap& map,const ScanConfig& c,const Eigen::Vector3d& p,double yaw,
      const double* velocity,double angular,int64_t stamp,const double previous[3],ScanOutput& output) {
    if(hasDecision(stamp)) {
      // 同源调用在已证明的K拍内沿用首拍命令，超过后停车；不重新准入或记账。
      output=decision_output_;
      if(++decision_calls_>decision_periods_)output.command_forward=output.command_lateral=output.command_yaw_rate=0.;
      output.samples=static_cast<ScanSample*>(std::calloc(decision_samples_.size(),sizeof(ScanSample)));
      std::copy(decision_samples_.begin(),decision_samples_.end(),output.samples);return;
    }
    // 缺测尚不能开始恢复证明，保留原输入等待语义，不宣告几何恢复失败。
    if(!velocity||!std::isfinite(angular))
      throw std::runtime_error("Current body pose is occupied; escape requires measured velocity and yaw rate");
    decision_periods_=decision_stamp_?static_cast<size_t>(std::max(1.,std::ceil((stamp-*decision_stamp_)*1e-9/c.sample_dt))):1;
    decision_calls_=1;decision_stamp_=stamp;decision_samples_.clear();decision_output_={};
    try {
      planImpl(map,c,p,yaw,velocity,angular,stamp,previous,recontact_,output);
      recontact_=false;
      // 只有输出非零动作，才开始事件运动预算。
      latched_|=output.command_forward!=0.||output.command_lateral!=0.||output.command_yaw_rate!=0.;
      decision_output_=output;decision_output_.samples=nullptr;
      decision_samples_.assign(output.samples,output.samples+output.sample_count);
    } catch(const std::runtime_error& error) {
      throw std::runtime_error("BODY_OVERLAP_RECOVERY_FAILED: "+std::string(error.what()));
    }
  }

 private:
  void planImpl(GridMap& map,const ScanConfig& c,const Eigen::Vector3d& p,double yaw,
      const double* velocity,double angular,int64_t stamp,const double previous[3],bool recontact,ScanOutput& output) {
    using namespace command_check_detail;
    const double escape_speed=escapeSpeed(c);
    // 每个新来源先按当前接触检查三格上限，优先于旧事件故障和动作后增深。
    std::vector<GridMap::BodyCell> cells;
    if(map.bodyContacts(p,yaw,0.,&cells)!=1||cells.empty())
      throw std::runtime_error("Current body pose is occupied or outside the local map; no eligible contacts");
    std::vector<Contact> current;
    for(const auto& cell:cells) {
      Contact selected{cell,Eigen::Vector2d::Zero(),std::numeric_limits<double>::infinity()};
      const Eigen::Vector2d center=p.head<2>()+world({c.body_center_x,c.body_center_y},yaw);
      const Eigen::Vector2d delta=center-Eigen::Vector2d((cell[0]+.5)*c.resolution,(cell[1]+.5)*c.resolution);
      for(auto axis:std::array<Eigen::Vector2d,4>{Eigen::Vector2d::UnitX(),Eigen::Vector2d::UnitY(),
          world(Eigen::Vector2d::UnitX(),yaw),world(Eigen::Vector2d::UnitY(),yaw)}) {
        if(axis.dot(delta)<0.)axis=-axis;
        const Contact candidate{cell,axis,0.};const double penetration=depth(candidate,c,p,yaw);
        if(penetration<selected.depth)selected={cell,axis,penetration};
      }
      if(!(selected.depth>=0.&&selected.depth<=penetrationLimit(c))) {
        waiting_for_stop_|=latched_;
        throw std::runtime_error("Current body pose is occupied; escape penetration exceeds localization and mapping allowance");
      }
      selected.remaining=selected.depth;current.push_back(selected);
    }
    if(!failure_.empty())throw std::runtime_error(failure_);
    const bool starting=!latched_;
    ScanState measured{};std::copy(velocity,velocity+3,measured.velocity);measured.yaw_rate=angular;
    measured.measured_velocity=measured.measured_yaw_rate=1;
    const bool stopped=nuwax_scan_motion_stopped(&measured);
    const State actual{p,yaw,body(Eigen::Map<const Eigen::Vector3d>(velocity).head<2>(),yaw),angular};
    const bool rebuilding=waiting_for_stop_&&stopped;
    if(starting) {
      last_=p;began_=last_stamp_=stamp;ticks_=0;traveled_=admitted_=0.;failure_.clear();
      contacts_.clear();initial_.clear();
    }
    if(rebuilding){contacts_.clear();initial_.clear();}
    if(starting||recontact||rebuilding) {
      // 当前接触已经逐成员通过三格准入；重建退出方向时不重置事件预算。
      for(const auto& contact:current) {
        if(initial_.count(contact.cell))continue;
        contacts_.push_back(contact);initial_.insert(contact.cell);
      }
      if(rebuilding)waiting_for_stop_=false;
    }
    if(starting) {
      // 深重叠需要超过一格的退出行程；按初始最大穿透的两倍计入斜向退出，
      // 时间由同一行程和既有低速推出。浅接触保留原一格、两秒未确认预算。
      distance_limit_=c.resolution;
      for(const auto& contact:contacts_)distance_limit_=std::max(distance_limit_,2.*contact.depth);
      time_limit_=std::max(2.,distance_limit_/escape_speed+c.sample_dt);
    }
    if(stamp<last_stamp_)throw std::runtime_error("Current body pose is occupied; escape source clock regressed");
    const auto check_budget=[&]() {
      if(latched_&&(stamp-began_>time_limit_*1e9||ticks_>static_cast<unsigned>(std::ceil(time_limit_/c.sample_dt))
          ||traveled_>=distance_limit_||admitted_>=distance_limit_)) {
        failure_="Current body pose is occupied; escape time or distance budget exhausted";
        throw std::runtime_error(failure_);
      }
    };
    bool progress_observed=false;
    if(waiting_for_stop_) {
      if(stamp>last_stamp_){traveled_=(p-last_).norm();++ticks_;last_stamp_=stamp;}
      check_budget();
      throw std::runtime_error("Current body pose is occupied; measured escape penetration increased; waiting for measured stop");
    }
    for(auto& contact:contacts_) {
      const double now=penetration(contact,c,p,yaw);
      const double remaining=depth(contact,c,p,yaw);
      if(now>penetrationLimit(c)) {
        waiting_for_stop_|=latched_;
        throw std::runtime_error("Current body pose is occupied; escape penetration exceeds localization and mapping allowance");
      }
      const Eigen::Vector2d voxel((contact.cell[0]+.5)*c.resolution,(contact.cell[1]+.5)*c.resolution);
      const double stopped_rate=.02+.02*(voxel-p.head<2>()).norm();
      if(latched_&&!rebuilding&&!startsOutward(contact,c,actual,stopped_rate)) {
        waiting_for_stop_=true;
        if(stamp>last_stamp_){traveled_=(p-last_).norm();++ticks_;last_stamp_=stamp;}
        check_budget();
        throw std::runtime_error("Current body pose is occupied; measured escape penetration increased");
      }
      // 动作前只更新基准；动作后未证实向内的数值增深也不触发等待停车。
      if(!latched_){contact.depth=now;contact.remaining=remaining;continue;}
      if(!rebuilding&&(now<contact.depth-1e-10||remaining<contact.remaining-1e-10))progress_observed=true;
      contact.depth=now;contact.remaining=remaining;
    }
    // 只有新的实测采样证实退出进展，才结清未确认运动预算。
    if(stamp>last_stamp_&&progress_observed) {
      began_=stamp;ticks_=0;traveled_=admitted_=0.;last_=p;
    }
    if(latched_&&stamp>last_stamp_){traveled_=(p-last_).norm();++ticks_;last_stamp_=stamp;}
    check_budget();
    const auto deadline=Clock::now()+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(c.sample_dt));
    std::vector<Eigen::Vector2d> requests;Eigen::Vector2d sum=Eigen::Vector2d::Zero();
    for(const auto& contact:contacts_)sum+=contact.body_axis?world(contact.axis,yaw):contact.axis;
    if(sum.norm()>0.)requests.push_back(body(sum.normalized()*escape_speed,yaw));
    for(const auto& contact:contacts_)requests.push_back(body((contact.body_axis?world(contact.axis,yaw):contact.axis)*escape_speed,yaw));
    // 机体系精确轴候选避免坐标往返舍入使严格单调条件失去本可执行的零切向量。
    for(int axis=0;axis<2;++axis)for(double sign:{-1.,1.})
      requests.emplace_back(axis==0?sign*escape_speed:0.,axis==1?sign*escape_speed:0.);
    for(int i=0;i<16;++i)requests.push_back(body(world(Eigen::Vector2d::UnitX(),i*2.*M_PI/16.)*escape_speed,yaw));
    // 先平移，再原地正反转向，最后小幅平移转向组合；请求仍按平台命令盒和实测加速度限幅。
    std::vector<std::pair<Eigen::Vector2d,double>> candidates;
    for(const auto& request:requests)candidates.emplace_back(request,0.);
    const double turn_speed=std::max(escape_speed/radius(c),c.min_executable_angular_rps);
    for(double sign:{-1.,1.})candidates.emplace_back(Eigen::Vector2d::Zero(),sign*turn_speed);
    for(double fraction:{.5,1.})for(double sign:{-1.,1.})for(const auto& request:requests)
      candidates.emplace_back(request*.5,sign*std::max(fraction*turn_speed,c.min_executable_angular_rps));
    bool exhausted=false,budget_exhausted=false;
    // 恢复允许既有低速倒退，普通导航的后退选项不限制退出证明。
    ScanConfig escape_config=c;escape_config.max_reverse_velocity=escape_speed;
    // 命令加速以上拍下发值为基准；运动与扫掠仍从实测状态开始。
    const State command_base{p,yaw,{previous[0],previous[1]},previous[2]};
    for(const auto& requested:candidates) {
      if(Clock::now()>=deadline){exhausted=true;break;}
      const auto command=bounded(requested.first,requested.second,command_base,escape_config,false);
      State state=actual;
      double path_bound=0.;
      std::vector<State> samples{actual};
      bool safe=true;
      for(size_t tick=0;tick<decision_periods_;++tick) {
        if(!advance(map,c,state,command.linear,command.angular,deadline,exhausted,path_bound,budget_exhausted,tick==0&&stopped)){safe=false;break;}
        samples.push_back(state);
      }
      if(!safe)continue;
      const double issued_bound=path_bound;
      while(!state.linear.isZero(0.)||state.angular!=0.) {
        if(!advance(map,c,state,Eigen::Vector2d::Zero(),0.,deadline,exhausted,path_bound,budget_exhausted)){safe=false;break;}
        samples.push_back(state);
      }
      if(!safe)continue;
      bool progress=false,same_side=true;
      auto exits=contacts_;
      for(auto& contact:exits) {
        double after=depth(contact,c,state.position,state.yaw);
        if(!contact.directed) {
          // 法向不可执行时，选取同侧可达的侧面；最小穿透可先保持不变，退出距离必须下降。
          double best=std::numeric_limits<double>::infinity();
          const auto directions=axes(yaw);
          for(int i=0;i<4;++i)for(double sign:{-1.,1.}) {
            const auto& axis=directions[i];
            const Eigen::Vector2d delta=p.head<2>()+world({c.body_center_x,c.body_center_y},yaw)
                -Eigen::Vector2d((contact.cell[0]+.5)*c.resolution,(contact.cell[1]+.5)*c.resolution);
            if(sign*axis.dot(delta)<0.)continue;
            const Eigen::Vector2d local=i==2?Eigen::Vector2d::UnitX():Eigen::Vector2d::UnitY();
            const Contact option{contact.cell,sign*(i>=2?local:axis),0.,0.,i>=2};
            const double from=depth(option,c,p,yaw),to=depth(option,c,state.position,state.yaw);
            if(from-to>1e-10&&from/(from-to)<best) {
              best=from/(from-to);contact.axis=option.axis;contact.body_axis=option.body_axis;after=to;
            }
          }
        }
        const double from=depth(contact,c,p,yaw);
        if(std::max(0.,after)>std::max(0.,from)+1e-10)same_side=false;
        if(after<from-1e-10)progress=true;
        contact.directed=true;
      }
      if(!progress||!same_side)continue;
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
      // 首次非零输出起计；同源输出由缓存复用，只计一次未确认整拍运动。
      if(latched_||output.command_forward!=0.||output.command_lateral!=0.||output.command_yaw_rate!=0.) {
        if(!latched_){began_=last_stamp_=stamp;last_=p;active_=true;}
        admitted_+=issued_bound;
      }
      contacts_=std::move(exits);
      for(auto& contact:contacts_) {
        contact.depth=penetration(contact,c,p,yaw);
        contact.remaining=depth(contact,c,p,yaw);
      }
      return;
    }
    const std::string reason=budget_exhausted?"Current body pose is occupied; escape time or distance budget exhausted"
        :exhausted?"Current body pose is occupied; escape proof computation exhausted"
        :"Current body pose is occupied; no safe escape candidate: no monotone collision-free escape and braking sweep";
    if(latched_) {
      if(budget_exhausted)failure_=reason;
      else waiting_for_stop_=true;
    }
    throw std::runtime_error(reason);
  }
};
}
