// 短程真实规划初值与原动力学检查；只由测试编译，不加入生产原生库。
#include <author_parameters.h>
#include <author_command_check.h>
#include <route_height_reference.h>
#include <bspline_opt/bspline_optimizer.h>
#include <plan_manage/plan_container.hpp>
// 直接调用原检查，测试权限不扩展到生产接口。
#define private public
#include <plan_manage/planner_manager.h>
#undef private
#include <cassert>
#include <fstream>
#include <iostream>

ScanConfig config() {
  ScanConfig c{};
  c.resolution = .1; c.body_length = .6; c.body_width = .32; c.body_height = .3;
  c.body_to_ground = .4; c.body_z_min = -.15; c.body_z_max = .15;
  c.max_velocity = .5; c.max_acceleration = .5; c.max_yaw_rate = .55;
  c.planning_horizon = 3.5; c.sample_dt = .05;
  c.max_step_up = .2; c.max_step_down = .2; c.max_slope = .6; c.max_yaw_acceleration = 1.;
  return c;
}

int main(int argc, char** argv) {
  if (argc == 2) {
    std::ifstream input(argv[1]);
    int count; double dt; input >> count >> dt;
    Eigen::MatrixXd controls(3, count);
    for (int i = 0; i < count; ++i) for (int j = 0; j < 3; ++j) input >> controls(j, i);
    scan_planner::UniformBspline seed(controls, 3, dt);
    seed.setPhysicalLimits(.5, .5, 0.);
    scan_planner::SCANPlannerManager manager;
    manager.pp_.max_vel_ = manager.pp_.max_acc_ = .5;
    manager.pp_.vel_tolerance_ = manager.pp_.acc_tolerance_ = 0.;
    double ratio;
    assert(manager.checkDynamicFeasibility(seed));
    assert(seed.checkFeasibility(ratio, false));
    assert(manager.strictDynamicScale(seed) <= 1.);
    return 0;
  }
  auto c = config();
  auto params = scan_planner::authorParameters(c);
  for (double distance : {.03, .4}) {
    auto empty = std::make_shared<GridMap>(); empty->initMap(params);
    const Eigen::Vector3d start(0., 0., .5);
    empty->importStaticOccupied({}, start);
    scan_planner::SCANPlannerManager short_plan;
    short_plan.initPlanModules(params, std::make_shared<scan_planner::PlanningVisualization>(params), empty);
    assert(short_plan.reboundReplan(start, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
        start + Eigen::Vector3d(distance, 0., 0.), Eigen::Vector3d::Zero(), true, false, 0.));
  }
}
