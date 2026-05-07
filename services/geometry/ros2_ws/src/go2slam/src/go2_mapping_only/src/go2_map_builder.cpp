#include <cerrno>
#include <csignal>
#include <cmath>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

namespace {

volatile std::sig_atomic_t g_should_exit = 0;

void SigHandle(int) {
  g_should_exit = 1;
}

using PointType = pcl::PointXYZI;

bool ensureDirectory(const std::string &path) {
  if (path.empty() || path == "/") {
    return true;
  }

  struct stat info;
  if (stat(path.c_str(), &info) == 0) {
    return S_ISDIR(info.st_mode);
  }

  const auto parent_end = path.find_last_of('/');
  if (parent_end != std::string::npos && parent_end > 0) {
    if (!ensureDirectory(path.substr(0, parent_end))) {
      return false;
    }
  }

  return mkdir(path.c_str(), 0755) == 0 || errno == EEXIST;
}

bool ensureParentDirectory(const std::string &file_path) {
  const auto parent_end = file_path.find_last_of('/');
  if (parent_end == std::string::npos || parent_end == 0) {
    return true;
  }
  return ensureDirectory(file_path.substr(0, parent_end));
}

class Go2MapBuilder : public rclcpp::Node {
 public:
  Go2MapBuilder()
      : Node("go2_map_builder"),
        map_cloud_(new pcl::PointCloud<PointType>()),
        tf_buffer_(get_clock()),
        tf_listener_(tf_buffer_) {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/utlidar/cloud_deskewed");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/utlidar/robot_odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/go2_built_map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/go2_robot_path");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.10);
    publish_every_n_scans_ = declare_parameter<int>("publish_every_n_scans", 5);
    downsample_every_n_scans_ = declare_parameter<int>("downsample_every_n_scans", 10);
    enable_icp_z_correction_ = declare_parameter<bool>("enable_icp_z_correction", true);
    odom_z_stale_epsilon_ = declare_parameter<double>("odom_z_stale_epsilon", 0.01);
    odom_z_stale_distance_ = declare_parameter<double>("odom_z_stale_distance", 0.50);
    icp_scan_voxel_leaf_size_ = declare_parameter<double>("icp_scan_voxel_leaf_size", 0.12);
    icp_max_correspondence_distance_ = declare_parameter<double>("icp_max_correspondence_distance", 0.45);
    icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 30);
    icp_max_z_step_ = declare_parameter<double>("icp_max_z_step", 0.20);
    save_path_ = declare_parameter<std::string>("save_path", "/home/huang/xxx/output/go2_built_map.pcd");

    auto cloud_qos = rclcpp::SensorDataQoS();
    auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(50)).reliable();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, cloud_qos,
        std::bind(&Go2MapBuilder::cloudCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, odom_qos,
        std::bind(&Go2MapBuilder::odomCallback, this, std::placeholders::_1));

    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(map_topic_, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

    path_msg_.header.frame_id = latest_odom_frame_id_;

    RCLCPP_INFO(get_logger(), "Listening cloud: %s", cloud_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Listening odom: %s", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "Publishing built map frame: %s", output_frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "Saving map to: %s", save_path_.c_str());
    RCLCPP_INFO(get_logger(),
                "ICP z correction: %s (z_stale_eps=%.3f, z_stale_dist=%.2f, icp_voxel=%.2f)",
                enable_icp_z_correction_ ? "enabled" : "disabled",
                odom_z_stale_epsilon_,
                odom_z_stale_distance_,
                icp_scan_voxel_leaf_size_);
  }

  ~Go2MapBuilder() override {
    saveMap();
  }

  void saveMap() {
    if (saved_ || map_cloud_->empty()) {
      return;
    }

    auto save_cloud = map_cloud_;
    pcl::PointCloud<PointType>::Ptr transformed_cloud;
    Eigen::Matrix4f output_transform;
    if (latest_odom_received_ && lookupOutputTransform(&output_transform)) {
      transformed_cloud = std::make_shared<pcl::PointCloud<PointType>>();
      pcl::transformPointCloud(*map_cloud_, *transformed_cloud, output_transform);
      save_cloud = transformed_cloud;
    } else {
      RCLCPP_WARN(
          get_logger(),
          "Saving PCD in %s frame because transform to %s is unavailable.",
          latest_odom_frame_id_.c_str(),
          output_frame_id_.c_str());
    }

    pcl::PointCloud<PointType> filtered = applyVoxelFilter(save_cloud);

    if (!ensureParentDirectory(save_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to create directory for %s", save_path_.c_str());
      saved_ = true;
      return;
    }

    if (pcl::io::savePCDFileBinary(save_path_, filtered) == 0) {
      RCLCPP_INFO(get_logger(), "Saved %zu points to %s", filtered.size(), save_path_.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "Failed to save map to %s", save_path_.c_str());
    }
    saved_ = true;
  }

 private:
  Eigen::Matrix4f transformToEigen(
      const geometry_msgs::msg::TransformStamped &transform_msg) const {
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

  bool lookupOutputTransform(Eigen::Matrix4f *transform) {
    if (!transform) {
      return false;
    }

    if (output_frame_id_ == latest_odom_frame_id_) {
      *transform = Eigen::Matrix4f::Identity();
      return true;
    }

    try {
      const auto transform_msg = tf_buffer_.lookupTransform(
          output_frame_id_, latest_odom_frame_id_, tf2::TimePointZero);
      *transform = transformToEigen(transform_msg);
      return true;
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Cannot transform accumulated cloud from %s to %s yet: %s",
          latest_odom_frame_id_.c_str(),
          output_frame_id_.c_str(),
          ex.what());
      return false;
    }
  }

  pcl::PointCloud<PointType> applyVoxelFilter(
      const pcl::PointCloud<PointType>::Ptr &input_cloud) const {
    if (voxel_leaf_size_ <= 0.0) {
      return *input_cloud;
    }

    pcl::PointCloud<PointType> filtered;
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_),
        static_cast<float>(voxel_leaf_size_));
    voxel.setInputCloud(input_cloud);
    voxel.filter(filtered);
    return filtered;
  }

  pcl::PointCloud<PointType>::Ptr downsampleForIcp(
      const pcl::PointCloud<PointType> &input_cloud) const {
    auto downsampled = std::make_shared<pcl::PointCloud<PointType>>(input_cloud);
    if (downsampled->empty() || icp_scan_voxel_leaf_size_ <= 0.0) {
      return downsampled;
    }
    pcl::VoxelGrid<PointType> voxel;
    voxel.setLeafSize(
        static_cast<float>(icp_scan_voxel_leaf_size_),
        static_cast<float>(icp_scan_voxel_leaf_size_),
        static_cast<float>(icp_scan_voxel_leaf_size_));
    voxel.setInputCloud(downsampled);
    auto filtered = std::make_shared<pcl::PointCloud<PointType>>();
    voxel.filter(*filtered);
    return filtered;
  }

  bool estimateIcpZCorrection(
      const pcl::PointCloud<PointType>::Ptr &target_cloud,
      const pcl::PointCloud<PointType>::Ptr &source_cloud,
      double *z_delta,
      double *fitness) const {
    if (!target_cloud || !source_cloud || !z_delta || target_cloud->size() < 40 || source_cloud->size() < 40) {
      return false;
    }

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputTarget(target_cloud);
    icp.setInputSource(source_cloud);
    icp.setMaximumIterations(icp_max_iterations_);
    icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);

    pcl::PointCloud<PointType> aligned;
    icp.align(aligned);
    if (!icp.hasConverged()) {
      return false;
    }

    const Eigen::Matrix4f transform = icp.getFinalTransformation();
    double step = static_cast<double>(transform(2, 3));
    if (!std::isfinite(step)) {
      return false;
    }

    if (step > icp_max_z_step_) {
      step = icp_max_z_step_;
    } else if (step < -icp_max_z_step_) {
      step = -icp_max_z_step_;
    }

    *z_delta = step;
    if (fitness) {
      *fitness = icp.getFitnessScore();
    }
    return true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<PointType> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
      return;
    }

    if (!received_cloud_) {
      received_cloud_ = true;
      RCLCPP_INFO(
          get_logger(),
          "Received first cloud on %s (frame=%s, width=%u, height=%u)",
          cloud_topic_.c_str(),
          msg->header.frame_id.c_str(),
          msg->width,
          msg->height);
    }

    pcl::PointCloud<PointType> cloud_in_map;
    if (!latest_odom_received_) {
      if (!warned_missing_odom_) {
        RCLCPP_WARN(
            get_logger(),
            "No odometry received yet on %s, skip cloud accumulation until odom is available.",
            odom_topic_.c_str());
        warned_missing_odom_ = true;
      }
      return;
    }

    if (msg->header.frame_id == latest_odom_frame_id_) {
      cloud_in_map = cloud;
    } else {
      Eigen::Isometry3f odom_transform = Eigen::Isometry3f::Identity();
      Eigen::Vector3f odom_translation = latest_odom_position_.cast<float>();
      odom_translation.z() += static_cast<float>(z_compensation_);
      odom_transform.translation() = odom_translation;
      odom_transform.linear() =
          latest_odom_orientation_.toRotationMatrix().cast<float>();
      pcl::transformPointCloud(cloud, cloud_in_map, odom_transform.matrix());
    }

    if (msg->header.frame_id == latest_odom_frame_id_ && std::abs(z_compensation_) > 1e-6) {
      for (auto &pt : cloud_in_map.points) {
        pt.z += static_cast<float>(z_compensation_);
      }
    }

    if (enable_icp_z_correction_ && odom_z_stale_ && previous_scan_for_icp_) {
      auto current_for_icp = downsampleForIcp(cloud_in_map);
      double z_delta = 0.0;
      double fitness = 0.0;
      if (estimateIcpZCorrection(previous_scan_for_icp_, current_for_icp, &z_delta, &fitness)) {
        if (std::abs(z_delta) > 1e-4) {
          z_compensation_ += z_delta;
          for (auto &pt : cloud_in_map.points) {
            pt.z += static_cast<float>(z_delta);
          }
        }
        if (scan_count_ % 20 == 0) {
          RCLCPP_INFO(get_logger(),
                      "ICP z correction applied: step=%.4f, total=%.4f, fitness=%.4f",
                      z_delta,
                      z_compensation_,
                      fitness);
        }
      }
    }

    previous_scan_for_icp_ = downsampleForIcp(cloud_in_map);

    *map_cloud_ += cloud_in_map;
    ++scan_count_;

    if (downsample_every_n_scans_ > 0 && scan_count_ % downsample_every_n_scans_ == 0) {
      pcl::PointCloud<PointType>::Ptr filtered(
          new pcl::PointCloud<PointType>(applyVoxelFilter(map_cloud_)));
      map_cloud_.swap(filtered);
    }

    if (publish_every_n_scans_ > 0 && scan_count_ % publish_every_n_scans_ == 0) {
      Eigen::Matrix4f output_transform;
      if (!lookupOutputTransform(&output_transform)) {
        return;
      }

      pcl::PointCloud<PointType> output_cloud;
      pcl::transformPointCloud(*map_cloud_, output_cloud, output_transform);

      sensor_msgs::msg::PointCloud2 map_msg;
      pcl::toROSMsg(output_cloud, map_msg);
      map_msg.header.stamp = now();
      map_msg.header.frame_id = output_frame_id_;
      map_pub_->publish(map_msg);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    latest_odom_position_ = Eigen::Vector3d(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    latest_odom_orientation_ = Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z);
    latest_odom_orientation_.normalize();
    latest_odom_frame_id_ = msg->header.frame_id.empty() ? "odom" : msg->header.frame_id;
    latest_odom_received_ = true;

    if (has_previous_odom_) {
      const Eigen::Vector3d delta = latest_odom_position_ - previous_odom_position_;
      const double delta_xy = std::hypot(delta.x(), delta.y());
      const double delta_z = std::abs(delta.z());
      if (delta_xy > 1e-3 && delta_z < odom_z_stale_epsilon_) {
        odom_z_flat_xy_distance_ += delta_xy;
        if (odom_z_flat_xy_distance_ > odom_z_stale_distance_) {
          odom_z_stale_ = true;
          if (!warned_odom_z_stale_) {
            warned_odom_z_stale_ = true;
            RCLCPP_WARN(get_logger(),
                        "Odometry z seems stale while moving in XY. Enabling ICP z compensation.");
          }
        }
      } else {
        odom_z_flat_xy_distance_ = 0.0;
        odom_z_stale_ = false;
        warned_odom_z_stale_ = false;
        z_compensation_ = 0.0;
      }
    }
    previous_odom_position_ = latest_odom_position_;
    has_previous_odom_ = true;

    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.header.stamp = now();
    pose.pose = msg->pose.pose;
    path_msg_.header.stamp = pose.header.stamp;
    path_msg_.header.frame_id = msg->header.frame_id;
    path_msg_.poses.push_back(pose);
    path_pub_->publish(path_msg_);
  }

  std::string cloud_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string path_topic_;
  std::string output_frame_id_;
  std::string save_path_;

  double voxel_leaf_size_;
  int publish_every_n_scans_;
  int downsample_every_n_scans_;
  bool enable_icp_z_correction_;
  double odom_z_stale_epsilon_;
  double odom_z_stale_distance_;
  double icp_scan_voxel_leaf_size_;
  double icp_max_correspondence_distance_;
  int icp_max_iterations_;
  double icp_max_z_step_;
  int scan_count_ = 0;
  bool saved_ = false;
  bool received_cloud_ = false;
  bool latest_odom_received_ = false;
  bool warned_missing_odom_ = false;
  bool warned_odom_z_stale_ = false;
  bool odom_z_stale_ = false;
  bool has_previous_odom_ = false;
  double odom_z_flat_xy_distance_ = 0.0;
  double z_compensation_ = 0.0;

  Eigen::Vector3d latest_odom_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d previous_odom_position_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond latest_odom_orientation_ = Eigen::Quaterniond::Identity();
  std::string latest_odom_frame_id_ = "odom";

  pcl::PointCloud<PointType>::Ptr map_cloud_;
  pcl::PointCloud<PointType>::Ptr previous_scan_for_icp_;
  nav_msgs::msg::Path path_msg_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  signal(SIGINT, SigHandle);
  signal(SIGTERM, SigHandle);

  auto node = std::make_shared<Go2MapBuilder>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  while (rclcpp::ok() && g_should_exit == 0) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  node->saveMap();
  rclcpp::shutdown();
  return 0;
}
