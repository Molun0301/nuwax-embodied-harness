// 独立几何基准：暴力枚举已知障碍列，凸多边形分离轴与点到边距离。
#include <author_parameters.h>
#include <business_constraint_mask.h>
#include <surface_obstacle_layer.h>
#include <plan_env/grid_map.h>
#include <cassert>
#include <iostream>
using Polygon = std::array<Eigen::Vector2d,4>;

double pointEdge(const Eigen::Vector2d& point,const Eigen::Vector2d& a,const Eigen::Vector2d& b) {
  const auto edge=(b-a).eval();
  const double t=std::clamp((point-a).dot(edge)/edge.squaredNorm(),0.,1.);
  return (point-a-t*edge).norm();
}

double polygonDistance(const Polygon& a,const Polygon& b) {
  bool separated=false;
  for(const auto* polygon:{&a,&b})for(int i=0;i<4;++i) {
    const auto edge=(*polygon)[(i+1)%4]-(*polygon)[i];
    const Eigen::Vector2d axis(-edge.y(),edge.x());
    double al=INFINITY,ah=-INFINITY,bl=INFINITY,bh=-INFINITY;
    for(const auto& p:a){al=std::min(al,p.dot(axis));ah=std::max(ah,p.dot(axis));}
    for(const auto& p:b){bl=std::min(bl,p.dot(axis));bh=std::max(bh,p.dot(axis));}
    separated|=ah<bl||bh<al;
  }
  if(!separated)return 0.;
  double distance=INFINITY;
  for(int i=0;i<4;++i)for(int j=0;j<4;++j) {
    distance=std::min(distance,pointEdge(a[i],b[j],b[(j+1)%4]));
    distance=std::min(distance,pointEdge(b[i],a[j],a[(j+1)%4]));
  }
  return distance;
}

int main() {
  ScanConfig c{};
  c.resolution=.1;c.body_length=.6;c.body_width=.32;c.body_height=.55;
  c.body_to_ground=.4;c.body_z_min=-.4;c.body_z_max=.15;
  c.body_center_x=.03;c.body_center_y=-.015;
  c.max_step_up=.15;c.max_step_down=.15;c.planning_horizon=3.;
  const Eigen::Vector3d body(-.15,.05,.4);
  auto parameters=scan_planner::authorParameters(c);
  parameters.setParam("grid_map/sliding_map_size_x",2.4);
  parameters.setParam("grid_map/sliding_map_size_y",2.);
  GridMap owner;owner.initMap(parameters);
  const std::vector<Eigen::Vector3d> lidar{{-.55,-.15,.25},{.65,.35,.45},{.35,-.35,.15},{-.85,-.45,.14}};
  owner.importStaticOccupied({lidar[0],lidar[1],lidar[2]},body);
  // 最后一项是已识别的真实支撑成员：格顶越过碰撞带，但实测最高点仍低于下沿。
  for(int frame=0;frame<3;++frame) {
    assert(owner.ingestLidarFrame(lidar,body,Eigen::Quaterniond::Identity(),body,{3}));
    owner.updateOccupancyCallback({});
  }
  assert(owner.getOccupancy(lidar.back())==1);
  const auto& mp=owner.mappingParameters();
  const int xmin=mp.map_bound_min_idx_.x(),xmax=mp.map_bound_max_idx_.x();
  const int ymin=mp.map_bound_min_idx_.y(),ymax=mp.map_bound_max_idx_.y();
  std::vector<double> ground;
  for(int x=xmin;x<=xmax;++x)for(int y=ymin;y<=ymax;++y) {
    ground.insert(ground.end(),{(x+.5)*c.resolution,(y+.5)*c.resolution,x>=0?.1:0.});
    if(x==2&&y==2)ground.insert(ground.end(),{.25,.25,1.1});
  }
  const std::vector<double> surface_obstacles{-.05,.55,.35,.45,-.25,.4,.55,.65,.1};
  SurfaceObstacleLayer surface(c,surface_obstacles.data(),3,ground.data(),ground.size()/3);
  const double left=(xmin+.75)*c.resolution,right=(xmax+.25)*c.resolution;
  const double bottom=(ymin+.75)*c.resolution,top=(ymax+.25)*c.resolution;
  BusinessConstraintMask business(c.resolution,c.body_length,c.body_width,c.body_center_x,c.body_center_y,
      {{left,bottom},{right,bottom},{right,top},{left,top}},{{1,-1}});
  const auto snapshot=owner.collisionSnapshot(c,&surface,&business);
  GridMap query;query.bindCollisionSnapshot(snapshot);
  // 障碍名单取自合成几何本身；不调用被测分类器生成预期结果。
  std::set<std::pair<int,int>> obstacles{{-6,-2},{6,3},{-1,5},{4,-3},{1,-1}};
  for(int x=xmin;x<=xmax;++x)for(int y=ymin;y<=ymax;++y)
    if(x==xmin||x==xmax||y==ymin||y==ymax)obstacles.emplace(x,y);
  double max_field_error=0.;
  for(int x=xmin;x<=xmax;++x)for(int y=ymin;y<=ymax;++y) {
    double nearest=INFINITY;
    for(const auto& obstacle:obstacles)
      nearest=std::min(nearest,std::hypot(double(x-obstacle.first),double(y-obstacle.second))*c.resolution);
    const double expected=std::max(0.,nearest-c.resolution/2.);
    const double actual=query.fieldDistance((x+.5)*c.resolution,(y+.5)*c.resolution);
    max_field_error=std::max(max_field_error,std::abs(actual-expected));
    if(std::abs(actual-expected)>1e-9)std::cerr<<"逐格不符 "<<x<<","<<y<<" actual="<<actual<<" expected="<<expected<<"\n";
    assert(std::abs(actual-expected)<=1e-9);
  }
  double max_body_error=0.;int poses=0;
  for(const Eigen::Vector2d position:{Eigen::Vector2d(-.6,.25),Eigen::Vector2d(-.3,0.),Eigen::Vector2d(.25,.25),Eigen::Vector2d(.45,-.45)})
    for(double yaw:{0.,.37,.9,1.57}) {
      const double cs=std::cos(yaw),sn=std::sin(yaw);
      Polygon rectangle;int i=0;
      for(const auto& corner:Polygon{Eigen::Vector2d(-.3,-.16),Eigen::Vector2d(.3,-.16),Eigen::Vector2d(.3,.16),Eigen::Vector2d(-.3,.16)}) {
        const Eigen::Vector2d q=corner+Eigen::Vector2d(c.body_center_x,c.body_center_y);
        rectangle[i++]=position+Eigen::Vector2d(cs*q.x()-sn*q.y(),sn*q.x()+cs*q.y());
      }
      double exact=INFINITY;
      for(const auto& [x,y]:obstacles) {
        const double lo_x=x*c.resolution,lo_y=y*c.resolution;
        Polygon square{Eigen::Vector2d(lo_x,lo_y),Eigen::Vector2d(lo_x+c.resolution,lo_y),
            Eigen::Vector2d(lo_x+c.resolution,lo_y+c.resolution),Eigen::Vector2d(lo_x,lo_y+c.resolution)};
        exact=std::min(exact,polygonDistance(rectangle,square));
      }
      const double actual=query.bodyFieldClearance(Eigen::Vector3d(position.x(),position.y(),.4),yaw);
      max_body_error=std::max(max_body_error,std::abs(actual-exact));++poses;
      assert(std::abs(actual-exact)<=c.resolution);
    }
  assert(query.fieldDistance(-100.,.05)==query.fieldDistance((xmin+.5)*c.resolution,.05));
  // 发布后的表不随拥有方的后续雷达修改而变化。
  const double old=query.fieldDistance(-.55,-.15);
  owner.repopulateStaticOccupied({},body);
  assert(query.fieldDistance(-.55,-.15)==old);
  GridMap empty;empty.bindCollisionSnapshot(owner.collisionSnapshot(c,nullptr,nullptr));
  assert(empty.fieldDistance(0.,0.)>1000.);
  std::cout<<"格数="<<(xmax-xmin+1)*(ymax-ymin+1)<<" 姿态数="<<poses
      <<" 逐格最大误差="<<max_field_error<<" 矩形最大误差="<<max_body_error<<"\n";
}
