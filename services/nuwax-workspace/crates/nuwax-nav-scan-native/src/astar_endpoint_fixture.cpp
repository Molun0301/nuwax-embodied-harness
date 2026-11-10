// 复用 N6-24a 端点夹具：真实地图初始化、业务掩码和原 A* 调用。
#include <author_parameters.h>
#include <business_constraint_mask.h>
#include <path_searching/dyn_a_star.h>
#include <chrono>
#include <iostream>

int main() {
  ScanConfig c{};
  c.resolution=.05; c.body_length=.679; c.body_width=.328; c.body_height=.419;
  c.body_to_ground=.405; c.max_velocity=1.; c.max_acceleration=.5;
  c.max_yaw_rate=1.; c.planning_horizon=5.; c.sample_dt=.05;
  auto map=std::make_shared<GridMap>();
  auto params=scan_planner::authorParameters(c);
  map->initMap(params);
  map->importStaticOccupied({}, Eigen::Vector3d::Zero());
  auto mask=std::make_shared<BusinessConstraintMask>(c.resolution,c.body_length,c.body_width,
      0.,0.,std::vector<Eigen::Vector2d>{{-1,-3},{5,-3},{5,3},{-1,3}},
      std::vector<Eigen::Vector2i>{});
  map->setBusinessConstraintQuery([mask](const Eigen::Vector3d& p,double yaw){return mask->rejects(p,yaw);});
  AStar astar;
  astar.initGridMap(map,Eigen::Vector3i(100,100,100));
  const auto check=[&](const char* name, double start, double end, bool expected) {
    const auto before=std::chrono::steady_clock::now();
    const bool found=astar.AstarSearch(.05,{start,0.,0.},{end,0.,0.},0.,0.) == ASTAR_RET::SUCCESS;
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-before).count();
    std::cout<<name<<": found="<<found<<", seconds="<<elapsed<<std::endl;
    return found == expected && elapsed < 1.;
  };
  if (!check("N6-24a业务外终点",0.,-2.,false)
      || !check("业务外起点",-2.,0.,false)
      || !check("范围内自由端点",0.,1.,true)) return 1;
  // 从域内占据点开始调整，分别逼近两端边界，覆盖循环内的越界退出。
  map->setSurfaceCollisionQuery([](const Eigen::Vector3d&,double,double,
      std::vector<GridMap::BodyCell>*,const std::set<GridMap::BodyCell>*,bool){return true;});
  if (!check("起点射线越业务边界",0.,1.,false)) return 2;
  map->setSurfaceCollisionQuery([](const Eigen::Vector3d& p,double,double,
      std::vector<GridMap::BodyCell>*,const std::set<GridMap::BodyCell>*,bool){return p.x() >= 1.;});
  if (!check("终点射线越业务边界",0.,1.,false)) return 3;
  // 撤掉业务边界后，附加占据仍会掩盖原占据查询的-1；物理范围必须终止调整。
  map->setBusinessConstraintQuery({});
  if (!check("终点射线越物理地图",0.,1.,false)) return 4;
  map->setSurfaceCollisionQuery([](const Eigen::Vector3d&,double,double,
      std::vector<GridMap::BodyCell>*,const std::set<GridMap::BodyCell>*,bool){return true;});
  if (!check("起点射线越物理地图",0.,1.,false)) return 5;
}
