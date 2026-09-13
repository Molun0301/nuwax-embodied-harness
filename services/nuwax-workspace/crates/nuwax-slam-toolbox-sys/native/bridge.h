#ifndef NUWAX_SLAM_TOOLBOX_BRIDGE_H
#define NUWAX_SLAM_TOOLBOX_BRIDGE_H
#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// 该接口的坐标为米与弧度，所有数组长度均由调用者显式传入。
typedef struct { double x, y, yaw; } NwSlamPose;
typedef struct {
  uint32_t abi_version, scan_count, local_window, max_nodes;
  double angle_min, angle_increment, range_min, range_max, range_threshold;
  NwSlamPose base_from_laser;
  double resolution, minimum_travel, minimum_heading, minimum_response;
  double search_dimension, search_resolution, search_smear, max_scan_gap_s;
  uint32_t loop_closing, solver_threads, use_scan_barycenter, loop_minimum_chain;
  double link_maximum_distance, loop_search_distance;
} NwSlamConfig;
typedef struct {
  uint64_t timestamp_ns, sequence;
  NwSlamPose odom_from_base;
  const double* ranges;
  size_t count;
  uint32_t use_laser_odometry;
} NwSlamScan;
typedef struct {
  uint64_t timestamp_ns, sequence, graph_revision, optimizations;
  NwSlamPose odom_from_base, map_from_base, map_from_odom;
  double covariance[9], matching_response, local_response;
  uint32_t disposition, node_count, edge_count, valid_beams;
} NwSlamResult;
typedef struct {
  uint64_t graph_revision;
  uint32_t width, height;
  double resolution, origin_x, origin_y;
} NwSlamGrid;
// 返回0表示操作完成；扫描接纳情况见disposition：0初始化，1接纳，2运动不足，3匹配拒绝。
uint32_t nw_slam_abi_version(void);
int nw_slam_validate_config(const NwSlamConfig*, char*, size_t);
int nw_slam_create(const NwSlamConfig*, void**, char*, size_t);
void nw_slam_destroy(void*);
int nw_slam_process(void*, const NwSlamScan*, NwSlamResult*, char*, size_t);
int nw_slam_grid(void*, NwSlamGrid*, char*, size_t);
int nw_slam_copy_grid(void*, int8_t*, size_t, char*, size_t);
int nw_slam_resume(void*,char*,size_t);
int nw_slam_save(void*, const char*, char*, size_t);
// 只允许空的新实例加载；调用前保证同一进程没有另一个原生实例。
int nw_slam_load(void*, const char*, uint32_t localization, NwSlamPose seed, char*, size_t);
#ifdef __cplusplus
}
#endif
#endif
