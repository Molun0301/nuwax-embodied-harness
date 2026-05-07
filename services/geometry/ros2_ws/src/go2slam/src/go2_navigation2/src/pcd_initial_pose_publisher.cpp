#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "go2_navigation2/pcd_initial_pose_utils.hpp"

namespace {

using PointType = pcl::PointXYZI;

Eigen::Matrix4f transformToEigen(
    const geometry_msgs::msg::TransformStamped &transform_msg) {
  const auto &translation = transform_msg.transform.translation;
  const auto &rotation = transform_msg.transform.rotation;

  Eigen::Quaternionf quaternion(
      static_cast<float>(rotation.w),
      static_cast<float>(rotation.x),
      static_cast<float>(rotation.y),
      static_cast<float>(rotation.z));
  quaternion.normalize();

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
  transform(0, 3) = static_cast<float>(translation.x);
  transform(1, 3) = static_cast<float>(translation.y);
  transform(2, 3) = static_cast<float>(translation.z);
  return transform;
}

pcl::PointCloud<PointType>::Ptr downsampleCloud(
    const pcl::PointCloud<PointType>::Ptr &input,
    const double voxel_leaf_size) {
  auto cleaned = std::make_shared<pcl::PointCloud<PointType>>();
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*input, *cleaned, indices);

  if (voxel_leaf_size <= 0.0 || cleaned->empty()) {
    return cleaned;
  }

  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(
      static_cast<float>(voxel_leaf_size),
      static_cast<float>(voxel_leaf_size),
      static_cast<float>(voxel_leaf_size));
  voxel.setInputCloud(cleaned);

  auto filtered = std::make_shared<pcl::PointCloud<PointType>>();
  voxel.filter(*filtered);
  return filtered;
}

class PcdInitialPosePublisher : public rclcpp::Node {
 public:
  PcdInitialPosePublisher()
      : Node("pcd_initial_pose_publisher"),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    pcd_path_ = declare_parameter<std::string>("pcd_path", "");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.15);
    source_voxel_leaf_size_ = declare_parameter<double>("source_voxel_leaf_size", 0.12);
    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 0.8);
    max_fitness_score_ = declare_parameter<double>("max_fitness_score", 0.8);
    max_iterations_ = declare_parameter<int>("max_iterations", 80);
    min_map_points_ = declare_parameter<int>("min_map_points", 200);
    min_source_points_ = declare_parameter<int>("min_source_points", 80);
    publish_retries_ = declare_parameter<int>("publish_retries", 8);
    publish_period_ms_ = declare_parameter<int>("publish_period_ms", 500);
    xy_covariance_ = declare_parameter<double>("xy_covariance", 0.25);
    yaw_covariance_ = declare_parameter<double>("yaw_covariance", 0.25);

    if (!loadMap()) {
      return;
    }

    initial_pose_pub_ =
        create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_, rclcpp::QoS(10));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PcdInitialPosePublisher::cloudCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "PCD initial pose enabled. map=%s points=%zu cloud=%s initialpose=%s",
        pcd_path_.c_str(),
        map_cloud_->size(),
        cloud_topic_.c_str(),
        initial_pose_topic_.c_str());
  }

 private:
  bool loadMap() {
    if (pcd_path_.empty()) {
      RCLCPP_WARN(get_logger(), "pcd_path is empty; PCD initial pose is disabled.");
      return false;
    }

    auto raw_map = std::make_shared<pcl::PointCloud<PointType>>();
    if (pcl::io::loadPCDFile<PointType>(pcd_path_, *raw_map) != 0) {
      RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", pcd_path_.c_str());
      return false;
    }

    map_cloud_ = downsampleCloud(raw_map, voxel_leaf_size_);
    if (static_cast<int>(map_cloud_->size()) < min_map_points_) {
      RCLCPP_ERROR(
          get_logger(),
          "PCD map has too few usable points after filtering: %zu < %d",
          map_cloud_->size(),
          min_map_points_);
      return false;
    }
    return true;
  }

  bool transformCloudToBase(
      const sensor_msgs::msg::PointCloud2 &msg,
      pcl::PointCloud<PointType> *cloud_in_base) {
    if (!cloud_in_base) {
      return false;
    }

    pcl::PointCloud<PointType> cloud;
    pcl::fromROSMsg(msg, cloud);
    if (cloud.empty()) {
      return false;
    }

    if (msg.header.frame_id.empty() || msg.header.frame_id == base_frame_) {
      *cloud_in_base = cloud;
      return true;
    }

    try {
      const auto transform_msg = tf_buffer_.lookupTransform(
          base_frame_, msg.header.frame_id, tf2::TimePointZero);
      pcl::transformPointCloud(cloud, *cloud_in_base, transformToEigen(transform_msg));
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Waiting for transform %s <- %s: %s",
          base_frame_.c_str(),
          msg.header.frame_id.c_str(),
          ex.what());
      return false;
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (matched_) {
      return;
    }

    pcl::PointCloud<PointType> cloud_in_base;
    if (!transformCloudToBase(*msg, &cloud_in_base)) {
      return;
    }

    auto source_cloud = downsampleCloud(
        std::make_shared<pcl::PointCloud<PointType>>(cloud_in_base),
        source_voxel_leaf_size_);
    if (static_cast<int>(source_cloud->size()) < min_source_points_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Current cloud has too few usable points after filtering: %zu < %d",
          source_cloud->size(),
          min_source_points_);
      return;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputTarget(map_cloud_);
    icp.setInputSource(source_cloud);
    icp.setMaximumIterations(max_iterations_);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);

    pcl::PointCloud<PointType> aligned;
    icp.align(aligned);
    if (!icp.hasConverged()) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "ICP did not converge for PCD initial pose.");
      return;
    }

    const double fitness = icp.getFitnessScore();
    if (!std::isfinite(fitness) || fitness > max_fitness_score_) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "ICP fitness %.3f is above threshold %.3f; not publishing initial pose.",
          fitness,
          max_fitness_score_);
      return;
    }

    pending_pose_ = go2_navigation2::makeInitialPoseMessage(
        icp.getFinalTransformation(), map_frame_, now(), xy_covariance_, yaw_covariance_);
    matched_ = true;
    remaining_publishes_ = std::max(1, publish_retries_);
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(std::max(100, publish_period_ms_)),
        std::bind(&PcdInitialPosePublisher::publishInitialPose, this));

    RCLCPP_INFO(
        get_logger(),
        "PCD ICP initial pose matched: x=%.3f y=%.3f fitness=%.3f; publishing %d times.",
        pending_pose_.pose.pose.position.x,
        pending_pose_.pose.pose.position.y,
        fitness,
        remaining_publishes_);
    publishInitialPose();
  }

  void publishInitialPose() {
    if (remaining_publishes_ <= 0) {
      if (publish_timer_) {
        publish_timer_->cancel();
      }
      return;
    }

    pending_pose_.header.stamp = now();
    initial_pose_pub_->publish(pending_pose_);
    --remaining_publishes_;
  }

  std::string pcd_path_;
  std::string cloud_topic_;
  std::string map_frame_;
  std::string base_frame_;
  std::string initial_pose_topic_;
  double voxel_leaf_size_{0.15};
  double source_voxel_leaf_size_{0.12};
  double max_correspondence_distance_{0.8};
  double max_fitness_score_{0.8};
  int max_iterations_{80};
  int min_map_points_{200};
  int min_source_points_{80};
  int publish_retries_{8};
  int publish_period_ms_{500};
  double xy_covariance_{0.25};
  double yaw_covariance_{0.25};

  pcl::PointCloud<PointType>::Ptr map_cloud_;
  bool matched_{false};
  int remaining_publishes_{0};
  geometry_msgs::msg::PoseWithCovarianceStamped pending_pose_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdInitialPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
