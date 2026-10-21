#pragma once
#include <author_host.h>
#include <bspline_opt/uniform_bspline.h>
namespace scan_planner {
class AuthorController {
public:


static constexpr double kMaxVYawLimit = 1.0;

ros::Publisher cmd_vel_pub;
ros::Publisher execution_frozen_pub;




bool receive_traj = false;
bool have_odom = false;
std::vector<UniformBspline> traj;
double traj_duration = 0.0;
int traj_id = 0;

Eigen::Vector3d odom_pos = Eigen::Vector3d::Zero();
double odom_yaw = 0.0;

double exec_time = 0.0;
ros::Time last_update_time;

double time_forward;
double heading_error_threshold;
double kp_pos;
double kp_yaw;
double max_vx;
double max_vy;
double max_vyaw;
double finish_dist;
std::string body_pose_topic;

double normalizeAngle(double angle)
{
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(max_value, value));
}

Eigen::Vector2d clampNorm(const Eigen::Vector2d &value, double max_norm)
{
  const double norm = value.norm();
  if (norm <= max_norm || norm < 1e-6)
    return value;
  return value / norm * max_norm;
}

double estimateDesiredYaw(double t_cur, const Eigen::Vector3d &pos_des)
{
  const double t_look = std::min(traj_duration, t_cur + time_forward);
  Eigen::Vector3d dir = traj[0].evaluateDeBoorT(t_look) - pos_des;

  if (dir.head<2>().squaredNorm() < 1e-4)
  {
    Eigen::Vector3d vel = traj[1].evaluateDeBoorT(t_cur);
    dir = vel;
  }

  if (dir.head<2>().squaredNorm() < 1e-4)
    return odom_yaw;

  return std::atan2(dir(1), dir(0));
}

void publishStop(double vyaw = 0.0)
{
  geometry_msgs::Twist cmd;
  cmd.angular.z = clamp(vyaw, -max_vyaw, max_vyaw);
  cmd_vel_pub.publish(cmd);
}

void publishExecutionFrozen(bool frozen)
{
  std_msgs::Bool msg;
  msg.data = frozen;
  execution_frozen_pub.publish(msg);
}

void bsplineCallback(const scan_planner::BsplineConstPtr &msg)
{
  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
  Eigen::VectorXd knots(msg->knots.size());

  for (size_t i = 0; i < msg->knots.size(); ++i)
    knots(i) = msg->knots[i];

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  traj.clear();
  traj.push_back(pos_traj);
  traj.push_back(traj[0].getDerivative());
  traj.push_back(traj[1].getDerivative());

  traj_duration = traj[0].getTimeSum();
  traj_id = msg->traj_id;
  exec_time = 0.0;
  last_update_time = ros::Time::now();
  receive_traj = true;

  ROS_WARN("[closed_loop_controller] received bspline traj_id=%d duration=%.3f", traj_id, traj_duration);
}

void odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_pos(0) = msg->pose.pose.position.x;
  odom_pos(1) = msg->pose.pose.position.y;
  odom_pos(2) = msg->pose.pose.position.z;
  const auto& q=msg->pose.pose.orientation;
  odom_yaw=std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
  have_odom = true;
}

void cmdCallback(const ros::TimerEvent &)
{
  if (!receive_traj || !have_odom)
  {
    publishExecutionFrozen(false);
    publishStop();
    return;
  }

  const ros::Time now = ros::Time::now();
  double dt = (now - last_update_time).toSec();
  if (dt < 0.0 || dt > 0.2)
    dt = 0.0;

  const double t_eval = std::min(exec_time, traj_duration);
  Eigen::Vector3d pos_des = traj[0].evaluateDeBoorT(t_eval);
  Eigen::Vector3d vel_des = traj[1].evaluateDeBoorT(t_eval);

  const double yaw_des = estimateDesiredYaw(t_eval, pos_des);
  const double yaw_err = normalizeAngle(yaw_des - odom_yaw);
  const double vyaw_cmd = clamp(kp_yaw * yaw_err, -max_vyaw, max_vyaw);

  // Explicit user exception: no fixed heading-error translation freeze.
  publishExecutionFrozen(false);
  exec_time = std::min(traj_duration, exec_time + dt);
  last_update_time = now;

  pos_des = traj[0].evaluateDeBoorT(exec_time);
  vel_des = traj[1].evaluateDeBoorT(exec_time);

  Eigen::Vector2d pos_err(pos_des(0) - odom_pos(0), pos_des(1) - odom_pos(1));
  Eigen::Vector2d vel_ff(vel_des(0), vel_des(1));
  Eigen::Vector2d vel_world = clampNorm(vel_ff + kp_pos * pos_err, std::max(max_vx, max_vy));

  const double c = std::cos(odom_yaw);
  const double s = std::sin(odom_yaw);
  geometry_msgs::Twist cmd;
  cmd.linear.x = clamp(c * vel_world(0) + s * vel_world(1), -max_vx, max_vx);
  cmd.linear.y = clamp(-s * vel_world(0) + c * vel_world(1), -max_vy, max_vy);
  cmd.angular.z = vyaw_cmd;

  if (exec_time >= traj_duration && pos_err.norm() < finish_dist)
    cmd = geometry_msgs::Twist();

  cmd_vel_pub.publish(cmd);
}

void configure(double vx,double vy,double w,double lookahead=.8,double position_gain=.8,double yaw_gain=1.5,double finish=.15){
 max_vx=vx;max_vy=vy;max_vyaw=std::min(w,kMaxVYawLimit);time_forward=lookahead;kp_pos=position_gain;kp_yaw=yaw_gain;finish_dist=finish;
}
};
}
