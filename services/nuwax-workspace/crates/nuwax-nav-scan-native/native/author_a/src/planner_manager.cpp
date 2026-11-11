#include <author_host.h>
#include <author_diagnostics.h>
#include <trajectory_heading.h>
// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace scan_planner
{
  namespace
  {
    // 非均匀三次样条：局部延长、端部线性重拟合、逐间隔收紧；其余几何固定。
    bool retimeLocalKnots(Eigen::MatrixXd& controls, Eigen::VectorXd& knots, double seed_dt,
        const Eigen::Vector3d& start, const Eigen::Vector3d& velocity, const Eigen::Vector3d& acceleration,
        double max_velocity, double max_acceleration, double sample_velocity, double sample_acceleration, const std::vector<int>& transition_points, std::string& reason)
    {
      const int count=controls.cols(), spans=count-3;
      const Eigen::MatrixXd original_controls=controls;
      if(authorTraceEnabled()) {
        auto out=authorTraceStream();out<<"{\"event\":\"local_timing_input\",\"points\":";
        traceMatrix(out,controls);out<<",\"transition_indices\":[";
        for(std::size_t j=0;j<transition_points.size();++j){if(j)out<<',';out<<transition_points[j];}
        out<<"]}\n";
      }
      if(velocity.head<2>().norm()>max_velocity||acceleration.head<2>().norm()>max_acceleration) {
        reason="真实起点水平速度或加速度超过设备上限";return false;
      }
      std::vector<double> lengths(count-1),speeds(count,max_velocity),travel(count-1);
      for(int i=0;i<count-1;++i)lengths[i]=(controls.col(i+1)-controls.col(i)).head<2>().norm();
      // 初始化时间由内部几何决定；旧拟合端点的微小差分不作为新的时间尺度。
      lengths[0]=lengths[1]=0.;
      lengths[2]=(controls.col(3)-start).head<2>().norm();
      speeds[0]=speeds[1]=speeds[2]=velocity.head<2>().norm();
      const double terminal=(controls.col(count-1)-controls.col(count-3)).head<2>().norm()/(2.*seed_dt);
      speeds[count-1]=speeds[count-2]=speeds[count-3]=std::min(max_velocity,terminal);
      for(int i=1;i<count;++i)speeds[i]=std::min(speeds[i],std::sqrt(speeds[i-1]*speeds[i-1]+2.*max_acceleration*lengths[i-1]));
      for(int i=count-2;i>=0;--i)speeds[i]=std::min(speeds[i],std::sqrt(speeds[i+1]*speeds[i+1]+2.*max_acceleration*lengths[i]));
      for(int i=0;i<count-1;++i)travel[i]=lengths[i]>0.
          ?(speeds[i]+speeds[i+1]>0.?2.*lengths[i]/(speeds[i]+speeds[i+1]):2.*std::sqrt(lengths[i]/max_acceleration))
          :std::numeric_limits<double>::infinity();
      travel[0]=travel[1]=std::numeric_limits<double>::infinity();
      travel[count-3]=travel[count-2]=std::numeric_limits<double>::infinity();
      Eigen::VectorXd intervals(spans);
      for(int i=0;i<spans;++i) {
        double candidate=0.;int available=0;
        for(int j=i;j<=i+2;++j)if(std::isfinite(travel[j])){candidate+=travel[j];++available;}
        candidate=available?candidate/available:seed_dt;
        intervals[i]=std::isfinite(candidate)?candidate:seed_dt;
      }
      // 非零真实起点的控制点依赖时距，从原工作时距起步；零导数不改变位置。
      if(!velocity.isZero(0.)||!acceleration.isZero(0.))intervals[0]=intervals[1]=seed_dt;
      intervals[spans-2]=intervals[spans-1]=std::max(intervals[spans-2],intervals[spans-1]);
      const double inward=1.-1e-8;
      // V0、V1为真实起点速度加减a*h/2，直接解两侧范数的二次不等式。
      auto boundary_span=[&](const Eigen::VectorXd& v,const Eigen::VectorXd& a,double limit) {
        const double slack=limit*limit-v.squaredNorm();
        if(slack<0.)return 0.;
        if(a.squaredNorm()==0.)return std::numeric_limits<double>::infinity();
        const double projection=std::abs(v.dot(a));
        return slack>0.?2.*slack/(projection+std::sqrt(projection*projection+a.squaredNorm()*slack)):0.;
      };
      const double boundary_max=std::min(boundary_span(velocity.head<2>(),acceleration.head<2>(),max_velocity),
          boundary_span(velocity,acceleration,sample_velocity));
      if(boundary_max==0.){reason="真实起点导数与严格速度控制点证书不存在正首间隔";return false;}
      intervals[0]=std::min(intervals[0],boundary_max/2.);
      auto boundary_speed_ok=[&](double h){return h<boundary_max;};
      auto rebuild=[&]() {
        // 首两个物理间隔可独立变化；外延结点沿首间隔延伸。
        knots.resize(count+4);knots[3]=0.;
        for(int i=2;i>=0;--i)knots[i]=knots[i+1]-intervals[0];
        for(int i=0;i<spans;++i)knots[i+4]=knots[i+3]+intervals[i];
        for(int i=count+1;i<count+4;++i)knots[i]=knots[i-1]+intervals[spans-1];
        controls=original_controls;
        const double a=intervals[0],b=intervals[1];
        controls.col(0)=start-velocity*a+acceleration*(a*a/3.);
        controls.col(1)=start-acceleration*(a*a/6.);
        controls.col(2)=controls.col(1)+(velocity+acceleration*(a/2.))*((2.*a+b)/3.);
        // 首边界贴上限时，绝对坐标舍入可能使二阶差分向外跨一个浮点间隔。
        // 只把该舍入差向可行侧构造；最终仍由原严格证书检查，真实边界误差另行记录。
        const Eigen::Vector3d boundary_v0=3.*(controls.col(1)-controls.col(0))/(knots[4]-knots[1]);
        const Eigen::Vector3d boundary_v1=3.*(controls.col(2)-controls.col(1))/(knots[5]-knots[2]);
        const Eigen::Vector3d boundary_a=2.*(boundary_v1-boundary_v0)/(knots[4]-knots[2]);
        const double boundary_scale=std::max(boundary_a.head<2>().norm()/max_acceleration,boundary_a.norm()/sample_acceleration);
        if(boundary_scale>1.) {
          const Eigen::Vector3d difference=boundary_a*(1.-1./boundary_scale)*(a*a);
          controls.col(0)-=difference;
          Eigen::Index axis;boundary_a.cwiseAbs().maxCoeff(&axis);
          controls(axis,0)=std::nextafter(controls(axis,0),boundary_a[axis]>0.?
              -std::numeric_limits<double>::infinity():std::numeric_limits<double>::infinity());
        }
        // 与原参数化相同的线性最小二乘，仅解端部过渡列；固定列移至右端。
        // 拟合真实二阶导数的连续变化，消除旧均匀时距留下的端部导数突变。
        if(!transition_points.empty()) {
          const int unknowns=transition_points.size();
          Eigen::MatrixXd weights=Eigen::MatrixXd::Zero(count-3,unknowns);
          Eigen::MatrixXd values=Eigen::MatrixXd::Zero(count-3,3);
          int row=0;
          for(int i=0;i<count-3;++i) {
            Eigen::Vector4d coefficient=Eigen::Vector4d::Zero();
            for(int side=0;side<2;++side) {
              const int j=i+side;
              const double factor=(side?1.:-1.)*2./(knots[j+4]-knots[j+2]);
              coefficient[side]+=factor*3./(knots[j+4]-knots[j+1]);
              coefficient[side+1]-=factor*(3./(knots[j+4]-knots[j+1])+3./(knots[j+5]-knots[j+2]));
              coefficient[side+2]+=factor*3./(knots[j+5]-knots[j+2]);
            }
            bool active=false;
            for(int j=0;j<unknowns;++j)if(transition_points[j]>=i&&transition_points[j]<=i+3)active=true;
            if(!active)continue;
            const double scale=1./std::sqrt(knots[i+4]-knots[i+3]);
            for(int j=0;j<4;++j) {
              auto found=std::find(transition_points.begin(),transition_points.end(),i+j);
              if(found==transition_points.end())values.row(row)-=scale*coefficient[j]*controls.col(i+j).transpose();
              else weights(row,found-transition_points.begin())=scale*coefficient[j];
            }
            ++row;
          }
          const Eigen::MatrixXd fitted=weights.topRows(row).colPivHouseholderQr().solve(values.topRows(row));
          for(int j=0;j<unknowns;++j)controls.col(transition_points[j])=fitted.row(j).transpose();
        }
      };
      auto derivatives=[&](Eigen::MatrixXd& velocities,Eigen::MatrixXd& accelerations) {
        velocities.resize(3,count-1);accelerations.resize(3,count-2);
        for(int i=0;i<count-1;++i)
          velocities.col(i)=3.*(controls.col(i+1)-controls.col(i))/(knots[i+4]-knots[i+1]);
        for(int i=0;i<count-2;++i)
          accelerations.col(i)=2.*(velocities.col(i+1)-velocities.col(i))/(knots[i+4]-knots[i+2]);
      };
      auto extend=[&](int interval,double addition) {
        const double rounding=std::numeric_limits<double>::epsilon()*(count+4)*std::max(1.,knots[count]);
        intervals[interval]=std::nextafter(intervals[interval]+std::max(addition,rounding),std::numeric_limits<double>::infinity());
        // 末端的两个间隔相等，保留固定末三点原有的终点位置。
        if(interval>=spans-2)intervals[spans-2]=intervals[spans-1]=std::max(intervals[spans-2],intervals[spans-1]);
      };
      Eigen::MatrixXd velocities,accelerations;
      for(int pass=0;pass<64;++pass) {
        rebuild();derivatives(velocities,accelerations);
        bool changed=false;
        for(int i=0;i<count-1;++i) {
          rebuild();derivatives(velocities,accelerations);
          const double limit=i<2?max_velocity:max_velocity*inward;
          const double magnitude=std::max(velocities.col(i).head<2>().norm(),velocities.col(i).norm()*limit/(sample_velocity*inward));
          if(magnitude<=limit)continue;
          if(i<2) {reason="真实起点导数在当前首间隔下不能满足严格速度界";return false;}
          const double denominator=knots[i+4]-knots[i+1];
          const double addition=denominator*(magnitude/(limit*inward)-1.);
          const int first=std::max(2,i-2),last=std::min(spans-1,i);
          for(int j=first;j<=last;++j)extend(j,addition/(last-first+1));
          changed=true;
        }
        for(int offset=1;offset<count-2;++offset) {
          const int i=pass%2?count-2-offset:offset;
          rebuild();derivatives(velocities,accelerations);
          const bool spatial=accelerations.col(i).norm()/sample_acceleration
              >accelerations.col(i).head<2>().norm()/max_acceleration;
          const double local_limit=(spatial?sample_acceleration:max_acceleration)*inward;
          const double magnitude=spatial?accelerations.col(i).norm():accelerations.col(i).head<2>().norm();
          if(magnitude<=local_limit)continue;
          // 在该真实二阶导数的支撑内选择最有效的局部增时方向。
          int selected=-1;double slope=0.,best_gain=0.;
          const Eigen::VectorXd saved=intervals;
          for(int j=std::max(0,i-2);j<=std::min(spans-1,i+1);++j) {
            const double delta=intervals[j]*1e-4;
            if(j==0&&!boundary_speed_ok(intervals[0]+delta))continue;
            extend(j,delta);rebuild();Eigen::MatrixXd v,a;derivatives(v,a);
            const double gradient=((spatial?a.col(i).norm():a.col(i).head<2>().norm())-magnitude)/delta;
            intervals=saved;
            const double gain=gradient/(j>=spans-2?2.:1.);
            if(gain<best_gain){best_gain=gain;slope=gradient;selected=j;}
          }
          rebuild();
          if(selected<0) {reason="真实起点衔接没有可局部延长的可行方向";return false;}
          const double maximum=intervals[selected]*.15;
          const double addition=std::min(maximum,(magnitude-local_limit*inward)/(-slope));
          if(selected==0&&!boundary_speed_ok(intervals[0]+addition)) {
            reason="真实起点速度与加速度限制首间隔，无法延长衔接";return false;
          }
          extend(selected,addition);changed=true;
        }
        if(!changed) {
          // 延长得到可行解后，逐个间隔收紧；只保留仍满足原导数约束的缩短。
          auto feasible=[&]() {
            rebuild();derivatives(velocities,accelerations);
            return velocities.topRows(2).colwise().norm().maxCoeff()<=max_velocity*inward
                && accelerations.rightCols(count-3).topRows(2).colwise().norm().maxCoeff()<=max_acceleration*inward
                && velocities.colwise().norm().maxCoeff()<=sample_velocity*inward
                && accelerations.rightCols(count-3).colwise().norm().maxCoeff()<=sample_acceleration*inward;
          };
          for(int sweep=0;sweep<3;++sweep)for(int offset=0;offset<spans-1;++offset) {
            const int j=sweep%2?spans-2-offset:offset;
            const Eigen::VectorXd saved=intervals;
            double low=.5,high=1.;
            for(int step=0;step<14;++step) {
              const double fraction=(low+high)/2.;intervals=saved;
              intervals[j]*=fraction;
              if(j==spans-2)intervals[spans-1]=intervals[j];
              if(feasible())high=fraction;else low=fraction;
            }
            intervals=saved;intervals[j]*=high;
            if(j==spans-2)intervals[spans-1]=intervals[j];
          }
          rebuild();
          if(authorTraceEnabled()) {
            auto out=authorTraceStream();out<<"{\"event\":\"timing_transition_points\",\"indices\":[";
            for(std::size_t j=0;j<transition_points.size();++j){if(j)out<<',';out<<transition_points[j];}
            out<<"],\"max_displacement\":";
            double maximum=0.;for(int i:transition_points)maximum=std::max(maximum,(controls.col(i)-original_controls.col(i)).norm());
            out<<maximum<<"}\n";
          }
          int first_fixed=3;
          while(std::find(transition_points.begin(),transition_points.end(),first_fixed)!=transition_points.end())++first_fixed;
          const Eigen::Vector2d original=(original_controls.col(first_fixed)-start).head<2>();
          UniformBspline curve(controls,3,seed_dt);curve.setKnot(knots);
          auto v=curve.getDerivative(),a=v.getDerivative();
          // 控制多边形回折不等于真实曲线回头；逐段求二次速度投影的真实最小值。
          if((controls.col(3)-controls.col(2)).head<2>().dot(original)<0.)
          for(int j=0;j<std::min(first_fixed,spans);++j) {
            const double left=knots[j+3],right=knots[j+4],middle=(left+right)/2.;
            auto projection=[&](double t){return v.evaluateDeBoorT(t).head<2>().dot(original);};
            const double p=projection(left),q=projection(right),m=projection(middle);
            const double quadratic=2.*(p+q-2.*m),linear=q-p-quadratic;
            double minimum=std::min(p,q);
            if(quadratic>0.) {
              const double fraction=-linear/(2.*quadratic);
              if(fraction>0.&&fraction<1.)minimum=std::min(minimum,projection(left+fraction*(right-left)));
            }
            if(minimum<0.){reason="真实起点与固定控制多边形的衔接曲线会回头";return false;}
          }
          if(v.getControlPoint().topRows(2).colwise().norm().maxCoeff()>max_velocity
              ||a.getControlPoint().topRows(2).colwise().norm().maxCoeff()>max_acceleration) {
            if(authorTraceEnabled()) {
              auto out=authorTraceStream();out<<"{\"event\":\"local_timing_certificate_failed\",\"velocities\":";
              traceMatrix(out,v.getControlPoint());out<<",\"accelerations\":";traceMatrix(out,a.getControlPoint());out<<"}\n";
            }
            reason="局部时距的最终严格控制点证书未通过";return false;
          }
          return true;
        }
      }
      if(authorTraceEnabled()) {
        rebuild();derivatives(velocities,accelerations);
        auto out=authorTraceStream();out<<"{\"event\":\"local_timing_unconverged\",\"intervals\":";
        traceMatrix(out,intervals);out<<",\"velocities\":";traceMatrix(out,velocities);
        out<<",\"accelerations\":";traceMatrix(out,accelerations);out<<"}\n";
      }
      reason="真实起点与固定控制多边形的局部定时未收敛";
      return false;
    }

    void buildReferenceControls(RouteHeightReference& route, const Eigen::Vector3d& start,
        Eigen::Vector3d& target, int count, double resolution, double max_velocity, double max_acceleration,
        GridMap& map, Eigen::MatrixXd& controls, std::vector<double>& control_arcs, double max_yaw_rate, std::vector<bool>& anchor_points)
    {
      struct Anchor { Eigen::Vector3d point; double arc; };
      std::vector<Anchor> input{{start, route.begin_arc}};
      for (std::size_t i = 0; i < route.points.size(); ++i)
        if (route.arcs[i] > route.begin_arc && route.arcs[i] < route.end_arc)
          input.push_back({route.points[i], route.arcs[i]});
      input.push_back({target, route.end_arc});
      // 仅简化初始化参考，不修改原点云或完整参考，硬碰撞仍按原实体判定。
      std::vector<char> keep(input.size(), 0);
      keep.front() = keep.back() = 1;
      std::vector<std::pair<std::size_t, std::size_t>> pending{{0,input.size()-1}};
      while (!pending.empty()) {
        const auto [first, last] = pending.back(); pending.pop_back();
        const Eigen::Vector3d delta = input[last].point - input[first].point;
        const double length2 = delta.squaredNorm();
        double maximum = 0.75 * resolution * resolution;
        std::size_t farthest = first;
        for (std::size_t i = first + 1; i < last; ++i) {
          const double fraction = length2 > 0.
              ? std::clamp((input[i].point - input[first].point).dot(delta) / length2, 0., 1.) : 0.;
          const double distance = (input[i].point - input[first].point - fraction * delta).squaredNorm();
          if (distance > maximum) { maximum = distance; farthest = i; }
        }
        // 亚体素转折也可能提供必要净空。简化捷径若与当前实体相交，保留最偏离
        // 捷径的原点再分段；空旷段仍沿原容量简化。这里只选初值，完整曲线与命令
        // 继续经过原碰撞/动态/连续制动门禁，采样未命中不构成运动许可。
        if (farthest == first && last > first+1) {
          const double yaw=std::atan2(delta.y(),delta.x());
          const int steps=std::max(1,static_cast<int>(std::ceil(delta.norm()/(resolution*.25))));
          const double margin=preferredClearance(resolution);
          double reference_clearance=margin;
          // 宽处保留目标余量；已知窄段沿原硬可行规则简化，由最小净距软代价选择可留余量。
          for(int j=0;j<=steps;++j) {
            const double arc=input[first].arc+double(j)/steps*(input[last].arc-input[first].arc);
            const Eigen::Vector3d before=route.positionAtArc(std::max(input[first].arc,arc-resolution*.25));
            const Eigen::Vector3d after=route.positionAtArc(std::min(input[last].arc,arc+resolution*.25));
            const double heading=std::atan2(after.y()-before.y(),after.x()-before.x());
            if(map.bodyFieldClearance(route.positionAtArc(arc),heading)<margin) {
              reference_clearance=0.;
              break;
            }
          }
          for (int j=0;j<=steps;++j) {
            const double fraction=double(j)/steps;
            Eigen::Vector3d point=input[first].point+fraction*delta;
            const double arc=input[first].arc+fraction*(input[last].arc-input[first].arc);
            point.z()=route.heightAtArc(arc)
                +(1.-fraction)*(input[first].point.z()-route.heightAtArc(input[first].arc))
                +fraction*(input[last].point.z()-route.heightAtArc(input[last].arc));
            if (map.getInflateOccupancy(point,yaw)==0
                && map.bodyFieldClearance(point,yaw)>=reference_clearance)continue;
            double deviation=-1.;
            for (std::size_t i=first+1;i<last;++i) {
              const double along=length2>0.?std::clamp((input[i].point-input[first].point).dot(delta)/length2,0.,1.):0.;
              const double distance=(input[i].point-input[first].point-along*delta).squaredNorm();
              if (distance>deviation){deviation=distance;farthest=i;}
            }
            break;
          }
        }
        if (farthest != first) {
          keep[farthest] = 1;
          pending.emplace_back(first, farthest); pending.emplace_back(farthest, last);
        }
      }
      std::vector<Anchor> anchors;
      for (std::size_t i = 0; i < input.size(); ++i) if (keep[i]) anchors.push_back(input[i]);
      if (3 * anchors.size() > static_cast<std::size_t>(count)) {
        // 容量只覆盖本次连续参考前缀；保留必要转折，不跨障碍删点。
        anchors.resize(count / 3);
        // 末端落在最后一段内部，避免拐角处的出段速度成为入段边界。
        anchors.back().arc = (anchors[anchors.size() - 2].arc + anchors.back().arc) / 2.;
        anchors.back().point = route.positionAtArc(anchors.back().arc);
        target = anchors.back().point;
        route.end_arc = anchors.back().arc;
        for (std::size_t i = 1; i < route.timed_arcs.size(); ++i) {
          if (route.end_arc > route.timed_arcs[i]) continue;
          const double fraction = (route.end_arc - route.timed_arcs[i - 1])
              / (route.timed_arcs[i] - route.timed_arcs[i - 1]);
          route.end_time = route.times[i - 1] + fraction * (route.times[i] - route.times[i - 1]);
          break;
        }
      }
      std::vector<int> repeats(anchors.size(), 3);
      std::vector<double> lengths(anchors.size() - 1), speeds(anchors.size(), 0.);
      for (std::size_t i = 0; i < lengths.size(); ++i)
        lengths[i] = (anchors[i + 1].point - anchors[i].point).norm();
      for (std::size_t i = 1; i + 1 < anchors.size(); ++i) {
        const Eigen::Vector2d incoming = (anchors[i].point - anchors[i - 1].point).head<2>();
        const Eigen::Vector2d outgoing = (anchors[i + 1].point - anchors[i].point).head<2>();
        const double planar_length=incoming.norm()*outgoing.norm();
        const double angle=planar_length>0.?std::acos(std::clamp(incoming.dot(outgoing)/planar_length,-1.,1.)):0.;
        if(angle<=M_PI/2.) {
          const double blend=std::min({lengths[i-1]/2.,lengths[i]/2.,max_velocity*max_velocity/(2.*max_acceleration)});
          const double radius=angle>0.?blend/std::tan(angle/2.):std::numeric_limits<double>::infinity();
          repeats[i]=1;
          speeds[i]=std::min({max_velocity,max_yaw_rate*radius,std::sqrt(max_acceleration*radius)});
        }
      }

      // 连续通过点的速度同时受前后停车距离约束。
      for (std::size_t i = 1; i < speeds.size(); ++i)
        speeds[i] = std::min(speeds[i], std::sqrt(speeds[i - 1] * speeds[i - 1] + 2. * max_acceleration * lengths[i - 1]));
      for (std::size_t i = speeds.size() - 1; i > 0; --i)
        speeds[i - 1] = std::min(speeds[i - 1], std::sqrt(speeds[i] * speeds[i] + 2. * max_acceleration * lengths[i - 1]));
      std::vector<int> interior(lengths.size(), 0);
      std::vector<double> times(lengths.size()), peaks(lengths.size());
      for (std::size_t i = 0; i < lengths.size(); ++i) {
        peaks[i] = std::min(max_velocity, std::sqrt(lengths[i] * max_acceleration
            + (speeds[i] * speeds[i] + speeds[i + 1] * speeds[i + 1]) / 2.));
        times[i] = lengths[i] > 0. ? lengths[i] / peaks[i]
            + ((peaks[i] - speeds[i]) * (peaks[i] - speeds[i])
               + (peaks[i] - speeds[i + 1]) * (peaks[i] - speeds[i + 1])) / (2. * max_acceleration * peaks[i]) : 0.;
      }
      // 容量缩短仍按每锚点三槽计算；省下的小转角槽用于各段内部采样。
      const int anchor_slots = std::accumulate(repeats.begin(), repeats.end(), 0);
      for (int left = count - anchor_slots; left > 0; --left) {
        std::size_t selected = 0;
        for (std::size_t i = 1; i < lengths.size(); ++i)
          if (times[i] / (interior[i] + 1) > times[selected] / (interior[selected] + 1)) selected = i;
        ++interior[selected];
      }
      controls.resize(3, count); control_arcs.clear(); anchor_points.assign(count,false);
      int column = 0;
      for (std::size_t i = 0; i < anchors.size(); ++i) {
        for (int repeat = 0; repeat < repeats[i]; ++repeat) {
          anchor_points[column]=true;
          controls.col(column++) = anchors[i].point; control_arcs.push_back(anchors[i].arc);
        }
        if (i + 1 == anchors.size()) break;
        for (int j = 1; j <= interior[i]; ++j) {
          const double t = times[i] * j / (interior[i] + 1);
          const double ramp = (peaks[i] - speeds[i]) / max_acceleration;
          const double brake = (peaks[i] - speeds[i + 1]) / max_acceleration;
          const double remaining = times[i] - t;
          const double distance = t < ramp ? speeds[i] * t + .5 * max_acceleration * t * t
              : remaining < brake ? lengths[i] - speeds[i + 1] * remaining - .5 * max_acceleration * remaining * remaining
              : speeds[i] * ramp + .5 * max_acceleration * ramp * ramp + peaks[i] * (t - ramp);
          const double fraction = lengths[i] > 0. ? distance / lengths[i] : 0.;
          const double arc = anchors[i].arc + fraction * (anchors[i + 1].arc - anchors[i].arc);
          Eigen::Vector3d point = anchors[i].point + fraction * (anchors[i + 1].point - anchors[i].point);
          // 实测端点可能偏离参考高程；连续吸收这一边界差，不能在首个内部点突然跳回旧高度。
          const double from_offset = anchors[i].point.z() - route.heightAtArc(anchors[i].arc);
          const double to_offset = anchors[i + 1].point.z() - route.heightAtArc(anchors[i + 1].arc);
          point.z() = route.heightAtArc(arc) + (1. - fraction) * from_offset + fraction * to_offset;
          controls.col(column++) = point; control_arcs.push_back(arc);
        }
      }
    }

    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cerr << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis, GridMap::Ptr map)
  {
    refreshParameters(nh);

    local_data_.traj_id_ = 0;
    grid_map_ = std::move(map);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // Host lifecycle only: reuse original parameter reads without reallocating map or search pool.
  void SCANPlannerManager::refreshParameters(ros::NodeHandle &nh)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/max_yaw_rate", pp_.max_yaw_rate_, -1.0);
    nh.param("optimization/vel_tolerance", pp_.vel_tolerance_, 1.0);
    nh.param("optimization/acc_tolerance", pp_.acc_tolerance_, 1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizon_, 5.0);

    if (bspline_optimizer_rebound_) bspline_optimizer_rebound_->setParam(nh);
  }

  void SCANPlannerManager::retireExecution()
  {
    route_height_.clear();
    if(bspline_optimizer_rebound_) {
      bspline_optimizer_rebound_->bind_route_segment_={};
      if(bspline_optimizer_rebound_->a_star_)bspline_optimizer_rebound_->a_star_->route_height_query_={};
    }
    local_data_ = LocalTrajData{};
    local_data_.start_pos_.setZero();
    global_data_.global_traj_ = PolynomialTraj{};
    global_data_.local_traj_.clear();
    global_data_.global_duration_ = 0.;
    global_data_.global_start_time_ = ros::Time{};
    global_data_.local_start_time_ = global_data_.local_end_time_ = -1.;
    global_data_.time_increase_ = global_data_.last_time_inc_ = global_data_.last_progress_time_ = 0.;
    continuous_failures_count_ = 0;
    flag_first_call = true;
    flag_force_polynomial = false;
    // Deliberately keep the original A* pool and rounds; each original search advances its round.
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj,
                                        double start_yaw)
  {

    const double resolution = grid_map_->getResolution();
    bspline_optimizer_rebound_->setInitialClearance(std::min(2.*preferredClearance(resolution),
        grid_map_->bodyFieldClearance(start_pt,start_yaw)));
    bool reference_detour = false;
    if (route_height_.ready()) {
      const Eigen::Vector2d chord = (local_target_pt - start_pt).head<2>();
      for (size_t i = 0; i < route_height_.points.size(); ++i) {
        if (route_height_.arcs[i] <= route_height_.begin_arc || route_height_.arcs[i] >= route_height_.end_arc) continue;
        const Eigen::Vector2d offset = (route_height_.points[i] - start_pt).head<2>();
        const double fraction = chord.squaredNorm() > 0.
            ? std::clamp(offset.dot(chord) / chord.squaredNorm(), 0., 1.) : 0.;
        if ((offset - fraction * chord).squaredNorm() > resolution * resolution) {
          reference_detour = true;
          break;
        }
      }
    }
    bool reanchor = false;
    if (local_data_.traj_id_ > 0) {
      const double t = std::clamp((ros::Time::now() - local_data_.start_time_).toSec(), 0., local_data_.duration_);
      reanchor = (local_data_.position_traj_.evaluateDeBoorT(t) - start_pt).norm() > resolution;
    }
    // 可分辨的绕行不能退化为首尾初值；机体落后时也不能继续旧计划位置。
    // 普通直线及亚体素过渡仍保留原初始化，仅改变当前窗口确需重建的几何来源。
    const bool original_retry_seed = !flag_first_call && flag_polyInit && local_data_.traj_id_ == 0
        && continuous_failures_count_ == 1;
    const bool route_time_seed = (reference_detour || reanchor || original_retry_seed) && route_height_.ready()
        && route_height_.end_time > route_height_.begin_time;
    const bool ordered_xyz_fallback = !route_time_seed && !flag_first_call && flag_polyInit && !flag_randomPolyTraj;
    // ROS node-local lifetime becomes native session lifetime.
    std::cerr << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    std::cerr.precision(3);
    std::cerr << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"rebound_input\",\"attempt\":"<<count-1<<",\"time\":"<<ros::Time::now().toSec()<<",\"poly_init\":"<<flag_polyInit<<",\"random_init\":"<<flag_randomPolyTraj<<",\"ordered_xyz_fallback\":"<<(ordered_xyz_fallback && route_height_.ready());
      out<<",\"route_time_seed\":"<<route_time_seed;
      out<<",\"start_p\":";traceMatrix(out,start_pt);out<<",\"start_v\":";traceMatrix(out,start_vel);out<<",\"start_a\":";traceMatrix(out,start_acc);
      out<<",\"target_p\":";traceMatrix(out,local_target_pt);out<<",\"target_v\":";traceMatrix(out,local_target_vel);out<<",\"route_height_ready\":"<<route_height_.ready()<<",\"route_begin_arc\":"<<route_height_.begin_arc<<",\"route_end_arc\":"<<route_height_.end_arc<<"}\n";
    }
    // 到达容差由原业务目标负责；非零短程仍须生成受约束轨迹，不能固定截断在20厘米。
    if ((start_pt - local_target_pt).squaredNorm() == 0.)
    {
      std::cerr << "Zero translation to local target" << endl;
      continuous_failures_count_++;
      return false;
    }

    ros::WallTime t_start = ros::WallTime::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    std::vector<double> timed_seed_arcs;
    Eigen::MatrixXd ctrl_pts;
    std::vector<double> seed_control_arcs;
    std::vector<bool> seed_anchor_points;
    // Initialization flags are members of this persistent manager.
    bool flag_regenerate = false;
    if (route_time_seed)
    {
      flag_first_call = false;
      flag_force_polynomial = false;
      // 使用既有完整参考的时标，不把拐角重试再次压成起终点直线。
      double span = route_height_.end_time - route_height_.begin_time;
      const int capacity = std::max(7, static_cast<int>(std::floor(pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3)));
      const double detail_dt = std::min(ts, grid_map_->getResolution() / pp_.max_vel_);
      const int intervals = std::clamp(static_cast<int>(std::ceil(span / detail_dt)), 6, capacity - 1);
      const double original_end_arc = route_height_.end_arc;
      buildReferenceControls(route_height_, start_pt, local_target_pt, intervals + 3,
          resolution, pp_.max_vel_, pp_.max_acc_, *grid_map_, ctrl_pts, seed_control_arcs, pp_.max_yaw_rate_, seed_anchor_points);
      if (route_height_.end_arc < original_end_arc) {
        span = route_height_.end_time - route_height_.begin_time;
        global_data_.last_progress_time_ = route_height_.end_time;
        // 与原局部目标相同的制动距离及速度边界，位置和时标取缩短后的参考。
        local_target_vel = (route_height_.points.back() - local_target_pt).norm()
                < pp_.max_vel_ * pp_.max_vel_ / (2. * pp_.max_acc_)
            ? Eigen::Vector3d::Zero().eval() : route_height_.velocityAtTime(route_height_.end_time);
        if (local_target_vel.norm() > pp_.max_vel_)
          local_target_vel = local_target_vel.normalized() * pp_.max_vel_;
      }
      ts = span / intervals;
      for (int i = 0; i <= intervals; ++i) {
        const double arc = route_height_.arcAtTime(route_height_.begin_time + i * ts);
        timed_seed_arcs.push_back(arc);
        point_set.push_back(route_height_.positionAtArc(arc));
      }
      point_set.front() = start_pt;
      point_set.back() = local_target_pt;
      start_end_derivatives = {start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero()};
    }
    else do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        // 先生成原五次曲线的几何采样；静止短程在拟合后按实际三次样条重新定时。
        const double short_time = std::max(3. * dist / pp_.max_vel_, sqrt(8. * dist / pp_.max_acc_));
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? short_time : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        // 等间隔覆盖真实末端，不能把截断采样位置与目标的零速度混在同一拟合边界。
        const int sample_intervals=static_cast<int>(std::ceil(time/ts));
        ts=time/sample_intervals;point_set.clear();
        for(int i=0;i<=sample_intervals;++i)point_set.push_back(gl_traj.evaluate(i*ts));
        t=time;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        // 实际机体未到目标时，旧曲线仍可能已经到期；空历史必须按原分支重建初值。
        if (!std::isfinite(t_cur) || t_cur < 0. || !(t_cur < local_data_.duration_)) {
          flag_force_polynomial = flag_regenerate = true;
          continue;
        }
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        if (pseudo_arc_length.size() < 2 || !std::isfinite(pseudo_arc_length.back())
            || !(pseudo_arc_length.back() > 0.)) {
          flag_force_polynomial = flag_regenerate = true;
          continue;
        }
        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            // 沿用原历史种子容量；在扩张向量前判定，避免坏历史耗尽内存或阻塞退出。
            if (point_set.size() >= pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) {
              flag_force_polynomial = flag_regenerate = true;
              break;
            }
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          if (flag_regenerate) break;
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help
        if (flag_regenerate) continue;

        // 点距为满足原最少点数而缩小时，时距同步缩小，保留原计划速度而非拉长短尾段。
        ts *= cps_dist / pp_.ctrl_pt_dist;
        start_end_derivatives.push_back(start_vel);
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(start_acc);
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    std::vector<double> height_sample_arcs;
    if(route_height_.ready()) {
      if (route_time_seed) {
        height_sample_arcs = std::move(timed_seed_arcs);
      } else {
      height_sample_arcs.assign(point_set.size(),0.);
      for(size_t i=1;i<point_set.size();++i)
        height_sample_arcs[i]=height_sample_arcs[i-1]+(point_set[i].head<2>()-point_set[i-1].head<2>()).norm();
      const double length=height_sample_arcs.back();
      for(size_t i=0;i<point_set.size();++i){
        const double fraction=length>0.?height_sample_arcs[i]/length:double(i)/double(point_set.size()-1);
        height_sample_arcs[i]=route_height_.begin_arc+fraction*(route_height_.end_arc-route_height_.begin_arc);
        if(ordered_xyz_fallback && i>0 && i+1<point_set.size())
          point_set[i]=route_height_.positionAtArc(height_sample_arcs[i]);
        else
          point_set[i].z()=route_height_.heightAtArc(height_sample_arcs[i]);
        const double from_offset = start_pt.z() - route_height_.heightAtArc(route_height_.begin_arc);
        const double to_offset = local_target_pt.z() - route_height_.heightAtArc(route_height_.end_arc);
        point_set[i].z() += (1. - fraction) * from_offset + fraction * to_offset;
      }
      }
      // Retain requested boundary positions and the original LS derivatives.
      point_set.front().z()=start_pt.z();point_set.back().z()=local_target_pt.z();
    } else {
      applyLinearZReference(point_set, start_pt(2), local_target_pt(2));
      bspline_optimizer_rebound_->a_star_->route_height_query_={};
      bspline_optimizer_rebound_->bind_route_segment_={};
    }

    if (!(ts > 0.) || !std::isfinite(ts)
        || !std::all_of(point_set.begin(), point_set.end(), [](const auto& p){return p.allFinite();})
        || !std::all_of(start_end_derivatives.begin(), start_end_derivatives.end(), [](const auto& p){return p.allFinite();})) {
      ROS_WARN("局部轨迹初值包含无效数值，等待重新规划。");
      ++continuous_failures_count_;
      return false;
    }
    if (route_time_seed) {
      // 控制多边形保持原折线的单调段，避免插值在窄通道拐角产生反向摆动。
      const int size = static_cast<int>(point_set.size()) + 2;
      const double square_dt = ts * ts;
      // 三次样条端点的解析位置/速度/加速度约束，使用原实际运动边界。
      ctrl_pts.col(0) = start_pt - start_vel * ts + start_acc * (square_dt / 3.);
      ctrl_pts.col(1) = start_pt - start_acc * (square_dt / 6.);
      ctrl_pts.col(2) = start_pt + start_vel * ts + start_acc * (square_dt / 3.);
      ctrl_pts.col(size - 3) = local_target_pt - local_target_vel * ts;
      ctrl_pts.col(size - 2) = local_target_pt;
      ctrl_pts.col(size - 1) = local_target_pt + local_target_vel * ts;
    } else {
      UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    }

    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"initial_control_points\",\"attempt\":"<<count-1<<",\"ts\":"<<ts<<",\"points\":";traceMatrix(out,ctrl_pts);out<<"}\n";
    }
    vector<vector<Eigen::Vector3d>> a_star_paths;
    // 路线锚点和重复停车点固定；每端最多三个相邻的导数过渡点参与拟合。
    std::vector<int> transition_points;
    for(int side=0;side<2;++side)for(int offset=0;offset<3;++offset) {
      const int i=side?ctrl_pts.cols()-4-offset:3+offset;
      if(i<3||i>=ctrl_pts.cols()-3)break;
      if(route_time_seed&&seed_anchor_points[i])break;
      if(std::find(transition_points.begin(),transition_points.end(),i)==transition_points.end())transition_points.push_back(i);
    }
    const Eigen::MatrixXd reference_controls=ctrl_pts;
    UniformBspline reference_curve(reference_controls,3,ts);
    const bool reference_free=route_time_seed&&trajectoryCollisionFree(reference_curve);
    // 初值和优化结果统一在优化后定时，避免端部定时成功使曲折初值跳过几何优化。
    std::string timing_reason;
    if(!height_sample_arcs.empty()) {
      // Cubic Greville index j corresponds to sample j-1; boundary extensions
      // keep the endpoint arc. This mapping stays fixed throughout the solve.
      std::vector<double> cp_arcs = seed_control_arcs;
      if (cp_arcs.empty()) {
        cp_arcs.resize(ctrl_pts.cols());
        for(int j=0;j<ctrl_pts.cols();++j)
          cp_arcs[j]=height_sample_arcs[std::clamp(j-1,0,int(height_sample_arcs.size())-1)];
      }
      bspline_optimizer_rebound_->bind_route_segment_=[this,cp_arcs](int first,int last){
        const double a=cp_arcs.at(first),b=cp_arcs.at(last);
        bspline_optimizer_rebound_->a_star_->route_height_query_=[this,a,b](double ratio){
          return route_height_.heightAtArc(a+ratio*(b-a));
        };
      };
    }
    bspline_optimizer_rebound_->fixed_stop_points_.clear();
    if(route_time_seed)for(int i=3;i+2<ctrl_pts.cols()-3;++i) {
      if(seed_anchor_points[i]&&seed_anchor_points[i+1]&&seed_anchor_points[i+2]
          &&ctrl_pts.col(i)==ctrl_pts.col(i+1)&&ctrl_pts.col(i)==ctrl_pts.col(i+2)) {
        for(int j=0;j<3;++j)bspline_optimizer_rebound_->fixed_stop_points_.push_back(i+j);
        i+=2;
      }
    }
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::WallTime::now() - t_start;

    // Visualization counter has session scope.
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = ros::WallTime::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"rebound_result\",\"attempt\":"<<count-1<<",\"success\":"<<flag_step_1_success<<",\"ts\":"<<ts<<",\"points\":";traceMatrix(out,ctrl_pts);out<<",\"search_paths\":[";
      for(size_t i=0;i<a_star_paths.size();++i){if(i)out<<',';out<<'[';for(size_t j=0;j<a_star_paths[i].size();++j){if(j)out<<',';traceMatrix(out,a_star_paths[i][j]);}out<<']';}out<<"]}\n";
    }
    std::cerr << "first_optimize_step_success=" << flag_step_1_success << endl;
    // 保留原有无碰撞参考回退；数值状态失败但几何已无碰撞时保留排斥后的结果。
    if(!flag_step_1_success&&reference_free) {
      UniformBspline optimized_geometry(ctrl_pts,3,ts);
      if(!trajectoryCollisionFree(optimized_geometry))ctrl_pts=reference_controls;
    }
    t_opt = ros::WallTime::now() - t_start;
    t_start = ros::WallTime::now();

    // 所有来源和优化结果共用局部结点定时；只重拟合明确允许的端部过渡点。
    Eigen::MatrixXd final_controls=ctrl_pts;
    Eigen::VectorXd final_knots;
    bool flag_step_2_success=retimeLocalKnots(final_controls,final_knots,ts,start_pt,start_vel,start_acc,
        pp_.max_vel_,pp_.max_acc_,pp_.max_vel_+pp_.vel_tolerance_,pp_.max_acc_+pp_.acc_tolerance_,transition_points,timing_reason);
    UniformBspline pos(final_controls,3,ts);
    if(flag_step_2_success)pos.setKnot(final_knots);
    else ROS_WARN_STREAM("局部结点定时失败："<<timing_reason);

    bool accepted = flag_step_2_success && checkDynamicFeasibility(pos)
        && trajectoryCollisionFree(pos) && strictDynamicScale(pos) <= 1.;
    if (!accepted)
    {
      std::fprintf(stderr,"\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = ros::WallTime::now() - t_start;

    // save planned results
    updateTrajInfo(pos, ros::Time::now());

    std::cerr << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    // The original FSM passes the first waypoint separately as start_pos.
    const std::vector<double>& point_arcs=route_height_.waypoint_arcs;
    const bool have_height=point_arcs.size()==waypoints.size()+1;

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    std::vector<double> inter_arcs;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      if(have_height)inter_arcs.push_back(point_arcs[i]);
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
          if(have_height)inter_arcs.push_back(point_arcs[i]+double(j)/id_num*(point_arcs[i+1]-point_arcs[i]));
        }
      }
    }

    inter_points.push_back(points.back());
    if(have_height)inter_arcs.push_back(point_arcs.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   std::cerr << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    if(have_height){
      route_height_.times.assign(1,0.);route_height_.timed_arcs=inter_arcs;
      for(int i=0;i<time.size();++i)route_height_.times.push_back(route_height_.times.back()+time(i));
    }
    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
    if(authorTraceEnabled()) {
      auto out=authorTraceStream();out<<"{\"event\":\"published_spline\",\"attempt\":"<<count-1<<",\"time\":"<<time_now.toSec()<<",\"duration\":"<<local_data_.duration_<<",\"control_points\":";
      traceMatrix(out,local_data_.position_traj_.getControlPoint());out<<",\"knots\":";traceMatrix(out,local_data_.position_traj_.getKnot());out<<"}\n";
    }
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: velocity limit exceeded at t="
                        << tc << ", |v|=" << vel.norm() << " > " << vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: acceleration limit exceeded at t="
                        << tc << ", |a|=" << acc.norm() << " > " << acc_limit);
        return false;
      }
    }

    return true;
  }

  bool SCANPlannerManager::trajectoryCollisionFree(UniformBspline& trajectory)
  {
    const double duration = trajectory.getTimeSum();
    auto velocity=trajectory.getDerivative();
    const double dt = std::min(.01, grid_map_->getResolution() / (4. * pp_.max_vel_));
    const int samples = std::max(1, static_cast<int>(std::ceil(duration / dt)));
    for (int i = 0; i <= samples; ++i) {
      const double time = duration * i / samples;
      const Eigen::Vector3d point = trajectory.evaluateDeBoorT(time);
      const double yaw = trajectoryHeading(velocity,duration,time,dt,0.);
      if (grid_map_->getInflateOccupancy(point, yaw) != 0) return false;
    }
    return true;
  }

  // 已有余量或确实窄到无法留足余量时可直交参考，其余情况进入原样条优化。
  bool SCANPlannerManager::trajectoryHasPreferredClearance(UniformBspline& trajectory)
  {
    const double duration=trajectory.getTimeSum();
    auto velocity=trajectory.getDerivative();
    const double dt=std::min(.01,grid_map_->getResolution()/(4.*pp_.max_vel_));
    const int samples=std::max(1,static_cast<int>(std::ceil(duration/dt)));
    for(int i=0;i<=samples;++i) {
      const double time=duration*i/samples;
      const double yaw=trajectoryHeading(velocity,duration,time,dt,0.);
      if(!grid_map_->bodyHasPreferredClearance(trajectory.evaluateDeBoorT(time),yaw))return false;
    }
    return true;
  }

  double SCANPlannerManager::strictDynamicScale(UniformBspline& trajectory)
  {
    auto velocity = trajectory.getDerivative();
    auto acceleration = velocity.getDerivative();
    // 水平导数样条位于其控制多边形凸包内，上界覆盖整个时间段的平面运动。
    return std::max(velocity.getControlPoint().topRows(2).colwise().norm().maxCoeff() / pp_.max_vel_,
        std::sqrt(acceleration.getControlPoint().topRows(2).colwise().norm().maxCoeff() / pp_.max_acc_));
  }

} // namespace scan_planner
