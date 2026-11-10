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
  // 执行中带真实非零速度、加速度重新锚定；不能因存在历史而跳过初值定时。
  auto empty = std::make_shared<GridMap>(); empty->initMap(params);
  const Eigen::Vector3d start(0., 0., .5), velocity(.12, 0., 0.), acceleration(.1, 0., .05);
  empty->importStaticOccupied({}, start);
  scan_planner::SCANPlannerManager moving;
  moving.initPlanModules(params, std::make_shared<scan_planner::PlanningVisualization>(params), empty);
  moving.route_height_.set({start, Eigen::Vector3d(1., 0., .5), Eigen::Vector3d(1., 1., .5)});
  moving.route_height_.times = {0., 2., 4.};
  moving.route_height_.timed_arcs = moving.route_height_.arcs;
  moving.route_height_.window(0., 4.);
  moving.local_data_.traj_id_ = 1;
  moving.local_data_.start_time_ = ros::Time::now();
  moving.local_data_.duration_ = 100.;
  moving.local_data_.position_traj_ = scan_planner::UniformBspline(Eigen::MatrixXd::Constant(3, 7, .5), 3, .1);
  assert(moving.reboundReplan(start, velocity, acceleration, Eigen::Vector3d(1., 1., .5),
      Eigen::Vector3d::Zero(), false, false, 0.));
  auto& curve = moving.local_data_.position_traj_;
  auto v = curve.getDerivative(), a = v.getDerivative();
  assert((curve.evaluateDeBoorT(0.) - start).norm() < 1e-10);
  assert((v.evaluateDeBoorT(0.) - velocity).norm() < 1e-10);
  assert((a.evaluateDeBoorT(0.) - acceleration).norm() < 1e-10);
  assert(moving.checkDynamicFeasibility(curve));
  assert(moving.strictDynamicScale(curve) <= 1.);
  for (double t = 0.; t <= curve.getTimeSum(); t += .001) {
    assert(v.evaluateDeBoorT(t).head<2>().norm() <= c.max_velocity + 1e-10);
    assert(a.evaluateDeBoorT(t).head<2>().norm() <= c.max_acceleration + 1e-10);
  }
  std::cout << "移动重规划：非零位置/速度/加速度边界保持，原采样与严格动力学检查通过\n";
  // 地面高度起伏的竖直导数不消耗平面设备限额；三维边界仍须完整保持。
  const Eigen::Vector3d terrain_acceleration(.3, 0., .8);
  for (double vertical_velocity : {0., .6}) {
    const Eigen::Vector3d terrain_velocity(.12, 0., vertical_velocity);
    scan_planner::SCANPlannerManager terrain;
    terrain.initPlanModules(params, std::make_shared<scan_planner::PlanningVisualization>(params), empty);
    terrain.route_height_.set({start, Eigen::Vector3d(1., 0., .5), Eigen::Vector3d(1., 1., .5)});
    terrain.route_height_.times = {0., 2., 4.};
    terrain.route_height_.timed_arcs = terrain.route_height_.arcs;
    terrain.route_height_.window(0., 4.);
    assert(terrain.reboundReplan(start, terrain_velocity, terrain_acceleration, Eigen::Vector3d(1., 1., .5),
        Eigen::Vector3d::Zero(), false, false, 0.));
    auto& reference = terrain.local_data_.position_traj_;
    auto terrain_v = reference.getDerivative(), terrain_a = terrain_v.getDerivative();
    assert((reference.evaluateDeBoorT(0.) - start).norm() < 1e-10);
    assert((terrain_v.evaluateDeBoorT(0.) - terrain_velocity).norm() < 1e-10);
    assert((terrain_a.evaluateDeBoorT(0.) - terrain_acceleration).norm() < 1e-10);
    assert(terrain.checkDynamicFeasibility(reference));
    assert(terrain.strictDynamicScale(reference) <= 1.);
    // 导数样条凸包给出连续时间证书；另按1毫秒采样复核全过程。
    const double max_horizontal_v = terrain_v.getControlPoint().topRows(2).colwise().norm().maxCoeff();
    const double max_horizontal_a = terrain_a.getControlPoint().topRows(2).colwise().norm().maxCoeff();
    assert(max_horizontal_v <= c.max_velocity);
    assert(max_horizontal_a <= c.max_acceleration);
    for (double t = 0.; t <= reference.getTimeSum(); t += .001) {
      assert(terrain_v.evaluateDeBoorT(t).head<2>().norm() <= c.max_velocity);
      assert(terrain_a.evaluateDeBoorT(t).head<2>().norm() <= c.max_acceleration);
    }
    std::cout << "地面竖直导数：起点vz=" << vertical_velocity << "，az=0.8，水平加速度=0.3；时长="
        << reference.getTimeSum() << "，水平速度/加速度连续上界=" << max_horizontal_v << "/" << max_horizontal_a << "\n";
  }
}
