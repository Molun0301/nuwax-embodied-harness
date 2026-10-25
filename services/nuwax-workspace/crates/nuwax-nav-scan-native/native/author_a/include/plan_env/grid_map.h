#ifndef _GRID_MAP_H
#define _GRID_MAP_H

// Ported from wuyi2121/SCAN-Planner@348e8a590a50a5a6bbab8d8c6dcfd171f009be26.
// 保留作者A的占据概率、射线及滑动缓存；真实机体查询和静态夹具由宿主适配。
#include <author_host.h>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <functional>
#include <utility>
#include <memory>
#include <queue>
#include <random>
#include <set>
#include <stdexcept>
#include <tuple>
#include <plan_env/raycast.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// voxel hashing
template <typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

// constant parameters

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3i map_bound_min_idx_, map_bound_max_idx_;
  Eigen::Vector3i map_origin_idx_;
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_z_up, obstacles_inflation_z_down;
  double double_cylinder_radius_, double_cylinder_offset_;
  double body_length_, body_width_;
  Eigen::Vector3d body_center_offset_; // Host physical reference -> body center, not A tuning.
  bool map_sliding_en_;
  double map_sliding_thresh_;
  int map_sliding_thresh_vox_;
  string frame_id_, sliding_map_frame_id_;

  /* depth camera intrinsics */
  double cx_, cy_, fx_, fy_;

  /* depth image projection filtering */
  double depth_filter_maxdist_, depth_filter_mindist_;
  int depth_filter_margin_;
  double k_depth_scaling_factor_;
  int skip_pixel_;

  /* raycasting */
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_;  // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_;                   // logit of occupancy probability
  double min_ray_length_, max_ray_length_;  // range of doing raycasting

  /* visualization and computation time display */
  double vis_height_, ground_height_;
  bool show_occ_time_;

  /* mapping sensor input */
  string sensor_type_;
  bool cloud_is_world_;
  bool need_extrinsic_;
  Eigen::Matrix4d lidar_extrinsic_;
  Eigen::Matrix4d depth_extrinsic_;

  /* active mapping */
  double unknown_flag_;
};

// intermediate mapping data for fusion

struct MappingData {
  // main map data, occupancy of each voxel and Euclidean distance

  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<int> occupancy_buffer_inflate_cnt_;
  // 原概率占据阈值的派生位图，布局按XY列分组，和原环形缓冲一同更新与清除。
  std::vector<std::uint64_t> occupancy_bits_;
  int occupancy_words_per_column_;
  vector<Eigen::Vector3i> inflate_offsets_;

  // raycast origin and sensor pose data

  Eigen::Vector3d ray_pos_;
  Eigen::Quaterniond ray_q_;
  Eigen::Vector3d sliding_map_frame_pos_;

  // depth image data

  int image_cnt_;
  // flags of map state

  bool occ_need_update_;
  bool use_cloud_update_;
  bool has_first_depth_;
  bool has_ray_pose_, has_cloud_;

  // depth image projected point cloud

  vector<Eigen::Vector3d> proj_points_;
  int proj_points_cnt;

  // flag buffers for speeding up raycasting

  vector<short> count_hit_, count_hit_and_miss_;
  vector<char> flag_traverse_, flag_rayend_;
  char raycast_num_;
  queue<Eigen::Vector3i> cache_voxel_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  // computation time

  double fuse_time_, max_fuse_time_;
  int update_num_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  enum { INVALID_IDX = -10000 };

  // occupancy map management
  void resetBuffer();
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline void setOccupancy(Eigen::Vector3d pos, double occ = 1);
  inline void setOccupied(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3d pos);
  inline int getOccupancy(Eigen::Vector3i id);
  inline int getInflateOccupancy(Eigen::Vector3d pos, double yaw);
  // 双圆柱候选查询保留前后顺序及图外-1；真实外包由getInflateOccupancy复核。
  inline int getAuthorInflateOccupancy(Eigen::Vector3d pos, double yaw);

  inline void boundIndex(Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3i& id);
  inline bool isUnknown(const Eigen::Vector3d& pos);
  inline bool isKnownFree(const Eigen::Vector3i& id);
  inline bool isKnownOccupied(const Eigen::Vector3i& id);

  void initMap(ros::NodeHandle& nh);
  using BusinessConstraintQuery = std::function<bool(const Eigen::Vector3d&, double)>;
  // Caller-owned explicit polygon/forbidden mask only, before body calibration.
  // A missing callback creates no business constraints. It cannot mutate odds.
  void setBusinessConstraintQuery(BusinessConstraintQuery query) {
    business_constraint_query_ = std::move(query);
  }

  // One atomic LiDAR observation: points, sensor pose and body position MUST belong
  // to the same registered frame/correction generation. Identity/freshness checks
  // belong to the host boundary. world points are never transformed a second time.
  // The pose is already sensor pose when need_extrinsic=false; otherwise the host
  // MUST supply the actual lidar_extrinsic matrix in initMap parameters.
  // Empty/invalid input returns false without changing a pending valid observation.
  bool ingestLidarFrame(const std::vector<Eigen::Vector3d>& points,
                        const Eigen::Vector3d& paired_sensor_origin,
                        const Eigen::Quaterniond& paired_sensor_orientation,
                        const Eigen::Vector3d& paired_body_pos);
  // Host invokes at the original 0.05-second occupancy timer cadence.
  void updateOccupancyCallback(const ros::TimerEvent& event);

  // Static truth fixtures only, NOT a LiDAR observation or a raycast surrogate.
  // Full load centers the grid at current physical body position, clears all data,
  // then calls original setOccupied for each unmodified in-map truth point.
  void importStaticOccupied(const std::vector<Eigen::Vector3d>& points,
                            const Eigen::Vector3d& current_body_pos);
  // Slides using original map logic, then repopulates from the same static truth.
  // This is explicitly fixture truth loading, not sensing or free-space inference.
  void repopulateStaticOccupied(const std::vector<Eigen::Vector3d>& points,
                                const Eigen::Vector3d& current_body_pos);
  const MappingParameters& mappingParameters() const { return mp_; }
  const MappingData& mappingData() const { return md_; }
  bool hasPendingOccupancyUpdate() const { return md_.occ_need_update_; }


  void publishMap();
  void publishMapInflate(bool all_info = false);

  void publishUnknown();
  void publishDepth();
  void publishDepthCloud();
  void publishSlidingMapBBox();
  void publishSlidingMapFrame();

  bool hasDepthObservation();
  bool odomValid();
  void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size);
  inline double getResolution();
  Eigen::Vector3d getOrigin();
  int getVoxelNum();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
  // 仅静态夹具记录上次完整占据集合；生产原帧不使用此集合。
  std::set<std::array<int, 3>> static_cells_;


  // main update process
  void raycastProcess();

  inline void inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts);
  inline int getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer);
  inline int getLocalIndex(int id, int dim) const;
  inline int toAddressLocal(const Eigen::Vector3i& id_l) const;
  inline int toAddressLocal(int x, int y, int z) const;
  int setCacheOccupancy(Eigen::Vector3d pos, int occ);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d& pt, const Eigen::Vector3d& ray_pos);
  void updateSlidingMap(const Eigen::Vector3d& center);
  void updateMapBoundaryFromIndex();
  void resetAllMapData();
  void resetCellByAddress(int addr);
  void resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask);
  void hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const;
  void applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds);
  void updateOccupiedBit(int address, bool occupied);
  void rebuildInflationOffsets();
  void updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask = nullptr);
  void updateInflationLayer(const Eigen::Vector3i& id, int delta,
                            const vector<Eigen::Vector3i>& offsets,
                            std::vector<int>& cnt_buffer,
                            std::vector<char>& flag_buffer,
                            const std::vector<char>* ignore_mask);

  ros::NodeHandle node_;
  BusinessConstraintQuery business_constraint_query_;

  //
  uniform_real_distribution<double> rand_noise_;
  normal_distribution<double> rand_noise2_;
  default_random_engine eng_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return getLocalIndex(id(0), 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(id(1), 1) * mp_.map_voxel_num_(2) + getLocalIndex(id(2), 2);
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
  return getLocalIndex(x, 0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
         getLocalIndex(y, 1) * mp_.map_voxel_num_(2) + getLocalIndex(z, 2);
}

inline int GridMap::getLocalIndex(int id, int dim) const {
  int local_id = id % mp_.map_voxel_num_(dim);
  if (local_id < 0) local_id += mp_.map_voxel_num_(dim);
  return local_id;
}

inline int GridMap::toAddressLocal(const Eigen::Vector3i& id_l) const {
  return id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id_l(1) * mp_.map_voxel_num_(2) + id_l(2);
}

inline int GridMap::toAddressLocal(int x, int y, int z) const {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_bound_max_idx_(0)), mp_.map_bound_min_idx_(0));
  id1(1) = max(min(id(1), mp_.map_bound_max_idx_(1)), mp_.map_bound_min_idx_(1));
  id1(2) = max(min(id(2), mp_.map_bound_max_idx_(2)), mp_.map_bound_min_idx_(2));
  id = id1;
}

inline bool GridMap::isUnknown(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  return md_.occupancy_buffer_[toAddress(id1)] < mp_.clamp_min_log_ - 1e-3;
}

inline bool GridMap::isUnknown(const Eigen::Vector3d& pos) {
  Eigen::Vector3i idc;
  posToIndex(pos, idc);
  return isUnknown(idc);
}

inline bool GridMap::isKnownFree(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  // return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ &&
  //     md_.occupancy_buffer_[adr] < mp_.min_occupancy_log_;
  return md_.occupancy_buffer_[adr] >= mp_.clamp_min_log_ && md_.occupancy_buffer_inflate_[adr] == 0;
}

inline bool GridMap::isKnownOccupied(const Eigen::Vector3i& id) {
  Eigen::Vector3i id1 = id;
  boundIndex(id1);
  int adr = toAddress(id1);

  return md_.occupancy_buffer_inflate_[adr] == 1;
}

inline void GridMap::setOccupied(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, mp_.clamp_max_log_);
}

inline void GridMap::setOccupancy(Eigen::Vector3d pos, double occ) {
  if (occ != 1 && occ != 0) {
    cout << "occ value error!" << endl;
    return;
  }

  if (!isInMap(pos)) return;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  applyOccupancyUpdate(id, occ > 0.5 ? mp_.clamp_max_log_ : mp_.clamp_min_log_);
}

inline int GridMap::getOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  if (business_constraint_query_ && business_constraint_query_(pos, yaw)) return 1;
  // Explicit physical calibration adapter. For the author reference convention
  // all offsets are zero and this reduces exactly to the original query.
  const double c = std::cos(yaw), s = std::sin(yaw);
  pos.x() += c * mp_.body_center_offset_.x() - s * mp_.body_center_offset_.y();
  pos.y() += s * mp_.body_center_offset_.x() + c * mp_.body_center_offset_.y();
  pos.z() += mp_.body_center_offset_.z();
  // 原膨胀缓冲已经覆盖候选；竖向精查统一在下方矩形中执行，避免重复扫描偏移体积。
  const Eigen::Vector3d heading(c, s, 0.);
  const auto candidate = [&](const Eigen::Vector3d& point) {
    if (!isInMap(point)) return -1;
    Eigen::Vector3i id;
    posToIndex(point, id);
    return static_cast<int>(md_.occupancy_buffer_inflate_[toAddress(id)]);
  };
  const int front = candidate(pos + mp_.double_cylinder_offset_ * heading);
  const int rear = candidate(pos - mp_.double_cylinder_offset_ * heading);
  if (front < 0 || rear < 0) return -1;
  if (front == 0 && rear == 0) return 0;

  // 双圆柱只提供保守候选；按真实矩形与原占据体素的四条分离轴复核。
  // 机体和格子闭边接触仍是碰撞，竖向沿原台阶碰撞带使用正体积相交。
  const double hx = mp_.body_length_ / 2., hy = mp_.body_width_ / 2.;
  const double r = mp_.resolution_, half_cell = r / 2.;
  const double ac = std::abs(c), as = std::abs(s);
  const double extent_x = ac * hx + as * hy, extent_y = as * hx + ac * hy;
  const double low = pos.z() - mp_.obstacles_inflation_z_up;
  const double high = pos.z() + mp_.obstacles_inflation_z_down;
  const int x0 = static_cast<int>(std::floor((pos.x() - extent_x) / r)) - 1;
  const int x1 = static_cast<int>(std::floor((pos.x() + extent_x) / r));
  const int y0 = static_cast<int>(std::floor((pos.y() - extent_y) / r)) - 1;
  const int y1 = static_cast<int>(std::floor((pos.y() + extent_y) / r));
  int z0 = std::max(static_cast<int>(std::floor(low / r)), mp_.map_bound_min_idx_.z());
  int z1 = std::min(static_cast<int>(std::ceil(high / r)) - 1, mp_.map_bound_max_idx_.z());
  while (z0 <= z1 && (z0 + 1.) * r <= low) ++z0;
  while (z0 <= z1 && z1 * r >= high) --z1;
  if (z0 > z1) return 0;
  const int nz = mp_.map_voxel_num_.z(), local_z = getLocalIndex(z0, 2);
  const int first_count = std::min(z1 - z0 + 1, nz - local_z);
  const int second_count = z1 - z0 + 1 - first_count;
  for (int x = std::max(x0, mp_.map_bound_min_idx_.x()); x <= std::min(x1, mp_.map_bound_max_idx_.x()); ++x)
    for (int y = std::max(y0, mp_.map_bound_min_idx_.y()); y <= std::min(y1, mp_.map_bound_max_idx_.y()); ++y)
    {
      const double dx = (x + .5) * r - pos.x(), dy = (y + .5) * r - pos.y();
      if (std::abs(dx) > extent_x + half_cell || std::abs(dy) > extent_y + half_cell
          || std::abs(c * dx + s * dy) > hx + half_cell * (ac + as)
          || std::abs(-s * dx + c * dy) > hy + half_cell * (ac + as))
        continue;
      // 位图与原概率阈值完全对应；环形Z范围仍按相同两段查询。
      const std::size_t column = getLocalIndex(x, 0) * mp_.map_voxel_num_.y() + getLocalIndex(y, 1);
      const auto* bits = md_.occupancy_bits_.data() + column * md_.occupancy_words_per_column_;
      const auto occupied = [&](int begin, int count) {
        if (count == 0) return false;
        const int first_word = begin / 64, last_word = (begin + count - 1) / 64;
        for (int word = first_word; word <= last_word; ++word) {
          std::uint64_t mask = ~std::uint64_t{0};
          if (word == first_word) mask &= ~std::uint64_t{0} << (begin % 64);
          if (word == last_word && (begin + count) % 64 != 0)
            mask &= (std::uint64_t{1} << ((begin + count) % 64)) - 1;
          if (bits[word] & mask) return true;
        }
        return false;
      };
      if (occupied(local_z, first_count) || occupied(0, second_count)) return 1;
    }
  return 0;
}

inline int GridMap::getAuthorInflateOccupancy(Eigen::Vector3d pos, double yaw) {
  Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Vector3d front = pos + mp_.double_cylinder_offset_ * heading;
  Eigen::Vector3d rear = pos - mp_.double_cylinder_offset_ * heading;

  int front_occ = getInflateOccupancyFromBuffer(front, md_.occupancy_buffer_inflate_);
  if (front_occ != 0) return front_occ;

  return getInflateOccupancyFromBuffer(rear, md_.occupancy_buffer_inflate_);
}

inline int GridMap::getInflateOccupancyFromBuffer(Eigen::Vector3d pos, const std::vector<char>& buffer) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  if (!buffer[toAddress(id)]) return 0;

  // The original ceil-based dilation is a conservative candidate index. Its
  // query-cell phase must not enlarge the calibrated vertical collision band.
  // Preserve the conservative XY candidate index and probability threshold, but resolve
  // each candidate against its original occupied half-open voxel volume.
  // Positive-volume intersection is required; no epsilon or shifted surface.
  const double low = pos.z() - mp_.obstacles_inflation_z_up;
  const double high = pos.z() + mp_.obstacles_inflation_z_down;
  for (const auto& offset : md_.inflate_offsets_) {
    const Eigen::Vector3i occupied = id - offset;
    const double cell_low = occupied.z() * mp_.resolution_;
    const double cell_high = (occupied.z() + 1.0) * mp_.resolution_;
    if (cell_high > low && cell_low < high && getOccupancy(occupied) == 1)
      return 1;
  }
  return 0;
}

inline int GridMap::getOccupancy(Eigen::Vector3i id) {
  if (!isInMap(id))
    return -1;

  return md_.occupancy_buffer_[toAddress(id)] > mp_.min_occupancy_log_ ? 1 : 0;
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < mp_.map_bound_min_idx_(0) || idx(1) < mp_.map_bound_min_idx_(1) ||
      idx(2) < mp_.map_bound_min_idx_(2)) {
    return false;
  }
  if (idx(0) > mp_.map_bound_max_idx_(0) || idx(1) > mp_.map_bound_max_idx_(1) ||
      idx(2) > mp_.map_bound_max_idx_(2)) {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor(pos(i) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_;
}

inline void GridMap::inflatePoint(const Eigen::Vector3i& pt, int inf_step_xy, int inf_step_z_up, int inf_step_z_down, vector<Eigen::Vector3i>& pts) {

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  pts.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      if (std::sqrt(x * x + y * y) > inf_step_xy)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z) {
        pts.push_back(Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z));
      }
    }
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif

