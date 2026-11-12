#pragma once

#include <Eigen/Eigen>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

// Independent NEH business constraints, not an Author A occupancy-map algorithm.
// Source semantics: existing native/include/plan_env/grid_map.h boundary polygon
// admission and Z-unbounded forbidden columns. No ground/bodyHeight/traversable,
// rectangular swept envelope, implicit boundary, occupancy buffer, or sensing.
class BusinessConstraintMask {
 public:
  BusinessConstraintMask(double resolution, double body_length, double body_width,
                         double body_center_x, double body_center_y,
                         std::vector<Eigen::Vector2d> boundary,
                         const std::vector<Eigen::Vector2i>& forbidden)
      : resolution_(resolution), body_center_x_(body_center_x), body_center_y_(body_center_y),
        offset_(body_length / 4.0), radius_(std::hypot(body_width / 2.0, offset_)),
        bucket_resolution_(std::max(resolution, 0.1)), boundary_(std::move(boundary)) {
    if (!std::isfinite(resolution) || resolution <= 0 ||
        !std::isfinite(body_length) || body_length <= 0 ||
        !std::isfinite(body_width) || body_width <= 0 ||
        !std::isfinite(body_center_x) || !std::isfinite(body_center_y))
      throw std::invalid_argument("Business mask requires actual finite body geometry and resolution");
    for (const auto& vertex : boundary_)
      if (!vertex.allFinite())
        throw std::invalid_argument("Business boundary vertices must be finite");
    for (const auto& index : forbidden) {
      const double x = (index.x() + 0.5) * resolution_;
      const double y = (index.y() + 0.5) * resolution_;
      buckets_[bucketKey(x, y)].push_back(index);
    }
  }

  bool matches(const double* boundary,size_t boundary_count,const int32_t* forbidden,size_t forbidden_count) const {
    if(boundary_.size()!=boundary_count)return false;
    for(size_t i=0;i<boundary_count;++i)
      if(boundary_[i].x()!=boundary[2*i]||boundary_[i].y()!=boundary[2*i+1])return false;
    size_t count=0;for(const auto& [_,list]:buckets_)count+=list.size();
    if(count!=forbidden_count)return false;
    for(size_t i=0;i<forbidden_count;++i) {
      const int x=forbidden[2*i],y=forbidden[2*i+1];
      const auto found=buckets_.find(bucketKey((x+.5)*resolution_,(y+.5)*resolution_));
      if(found==buckets_.end()||!std::any_of(found->second.begin(),found->second.end(),
          [x,y](const Eigen::Vector2i& p){return p.x()==x&&p.y()==y;}))return false;
    }
    return true;
  }
  bool empty() const { return boundary_.size() < 3 && buckets_.empty(); }

  bool rejects(const Eigen::Vector3d& reference, double yaw) const {
    return rejectsEnvelope(reference, yaw, 0.0);
  }

  // 距离场标记原业务列，不预先叠加机身外包。
  bool rejectsColumn(int x,int y) const {
    const Eigen::Vector2d center((x+.5)*resolution_,(y+.5)*resolution_);
    if(boundary_.size()>=3 && !insideBoundary(center))return true;
    const auto found=buckets_.find(bucketKey(center.x(),center.y()));
    return found!=buckets_.end() && std::any_of(found->second.begin(),found->second.end(),
        [x,y](const Eigen::Vector2i& cell){return cell.x()==x&&cell.y()==y;});
  }

  // 半径padding包住整段机体点的位移，业务边界与禁行格同样不能在采样间隙穿越。
  bool rejectsEnvelope(const Eigen::Vector3d& reference, double yaw, double padding) const {
    Eigen::Vector2d heading(std::cos(yaw), std::sin(yaw));
    Eigen::Vector2d center = reference.head<2>() + Eigen::Vector2d(
        body_center_x_*heading.x()-body_center_y_*heading.y(),
        body_center_x_*heading.y()+body_center_y_*heading.x());
    for (const auto& p : std::array<Eigen::Vector2d,2>{
        center+offset_*heading,center-offset_*heading}) {
      if (boundary_.size() >= 3) {
        if (!insideBoundary(p)) return true;
        for (size_t i=0,j=boundary_.size()-1;i<boundary_.size();j=i++) {
          const auto d=(boundary_[i]-boundary_[j]).eval();
          const double t=d.squaredNorm()>0.?std::clamp((p-boundary_[j]).dot(d)/d.squaredNorm(),0.,1.):0.;
          if ((p-(boundary_[j]+t*d)).norm() < radius_ + padding) return true;
        }
      }
    }
    return intersectsForbidden(Eigen::Vector3d(center.x()+offset_*heading.x(),center.y()+offset_*heading.y(),0.),padding)
        || intersectsForbidden(Eigen::Vector3d(center.x()-offset_*heading.x(),center.y()-offset_*heading.y(),0.),padding);
  }

 private:
  struct Key {
    int x, y;
    bool operator==(const Key& other) const { return x == other.x && y == other.y; }
  };
  struct Hash {
    size_t operator()(const Key& k) const {
      size_t s = std::hash<int>{}(k.x);
      s ^= std::hash<int>{}(k.y) + 0x9e3779b9 + (s << 6) + (s >> 2);
      return s;
    }
  };

  Key bucketKey(double x, double y) const {
    return {static_cast<int>(std::floor(x / bucket_resolution_)),
            static_cast<int>(std::floor(y / bucket_resolution_))};
  }

  bool intersectsForbidden(const Eigen::Vector3d& p, double padding=0.) const {
    if (buckets_.empty()) return false;
    const double half_voxel = resolution_ / 2.0;
    // Same broad-phase extent as NEH's original anyBucket. Buckets only select
    // candidates; the original disk-to-cell-square predicate below decides.
    const double radius=radius_+padding;
    const Key low = bucketKey(p.x() - radius - half_voxel, p.y() - radius - half_voxel);
    const Key high = bucketKey(p.x() + radius + half_voxel, p.y() + radius + half_voxel);
    for (int x = low.x; x <= high.x; ++x) for (int y = low.y; y <= high.y; ++y) {
      const auto found = buckets_.find({x, y});
      if (found == buckets_.end()) continue;
      for (const auto& c : found->second) {
        const double dx = std::max(0.0, std::abs((c.x() + 0.5) * resolution_ - p.x()) - half_voxel);
        const double dy = std::max(0.0, std::abs((c.y() + 0.5) * resolution_ - p.y()) - half_voxel);
        if (dx * dx + dy * dy <= radius * radius) return true;
      }
    }
    return false;
  }

  bool insideBoundary(const Eigen::Vector2d& p) const {
    bool inside = false;
    for (size_t i = 0, j = boundary_.size() - 1; i < boundary_.size(); j = i++) {
      const auto& a = boundary_[i]; const auto& b = boundary_[j]; const Eigen::Vector2d d = b - a;
      const double t = d.squaredNorm() > 0 ? std::clamp((p - a).dot(d) / d.squaredNorm(), 0.0, 1.0) : 0.0;
      if ((p - (a + t * d)).norm() < 1e-8) return true;
      if ((a.y() > p.y()) != (b.y() > p.y()) &&
          p.x() < (b.x() - a.x()) * (p.y() - a.y()) / (b.y() - a.y()) + a.x()) inside = !inside;
    }
    return inside;
  }

  double resolution_, body_center_x_, body_center_y_, offset_, radius_, bucket_resolution_;
  std::vector<Eigen::Vector2d> boundary_;
  std::unordered_map<Key, std::vector<Eigen::Vector2i>, Hash> buckets_;
};
