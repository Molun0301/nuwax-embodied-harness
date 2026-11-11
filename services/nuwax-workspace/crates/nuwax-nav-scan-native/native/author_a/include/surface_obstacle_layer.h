#pragma once
#include <scan_native.h>
#include <body_collision_cell.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <set>
#include <vector>

// 当前完整表面的已知实体，独立于注册雷达概率图；不生成射线、自由格或来源时效。
class SurfaceObstacleLayer {
  struct Band { double low, high; };
  ScanConfig config_;
  std::vector<double> obstacles_, ground_;
  std::unordered_map<std::uint64_t,std::vector<Band>> columns_;
  static std::uint64_t key(int x,int y) {
    return (std::uint64_t(std::uint32_t(x))<<32)|std::uint32_t(y);
  }
 public:
  SurfaceObstacleLayer(const ScanConfig& config,const double* obstacles,size_t count,
      const double* ground,size_t ground_count):config_(config) {
    if(count)obstacles_.assign(obstacles,obstacles+3*count);
    if(ground_count)ground_.assign(ground,ground+3*ground_count);
    const double r=config.resolution;
    for(size_t i=0;i<count;++i) {
      const auto* p=obstacles+3*i;
      columns_[key(int(std::floor(p[0]/r)),int(std::floor(p[1]/r)))].push_back({p[2]-r/2.,p[2]+r/2.});
    }
    // 地面保留测得的表面高度；高于当前碰撞带底面的桌面不能因其是某层支撑就消失。
    for(size_t i=0;i<ground_count;++i) {
      const auto* p=ground+3*i;
      columns_[key(int(std::floor(p[0]/r)),int(std::floor(p[1]/r)))].push_back({p[2],p[2]});
    }
  }
  bool matches(const double* obstacles,size_t count,const double* ground,size_t ground_count)const {
    return obstacles_.size()==3*count&&ground_.size()==3*ground_count
        &&(!count||std::equal(obstacles_.begin(),obstacles_.end(),obstacles))
        &&(!ground_count||std::equal(ground_.begin(),ground_.end(),ground));
  }
  bool intersects(const Eigen::Vector3d& reference,double yaw,double padding=0.)const {
    return contacts(reference,yaw,padding,nullptr,nullptr);
  }
  const std::vector<double>& groundPoints() const { return ground_; }
  // 硬接触和净空场的列分类共用高度带及开闭边界。
  std::pair<double,double> heightBand(double reference_z) const {
    return {reference_z+std::max(config_.max_step_up,config_.resolution)-config_.body_to_ground,
            reference_z+config_.body_z_max};
  }
  bool columnContact(int x,int y,double reference_z) const {
    const auto found=columns_.find(key(x,y));
    if(found==columns_.end())return false;
    const auto [low,high]=heightBand(reference_z);
    for(const auto& band:found->second)
      if(band.low==band.high ? band.low>=low&&band.low<high : band.high>low&&band.low<high)return true;
    return false;
  }
  bool contacts(const Eigen::Vector3d& reference,double yaw,double padding,
      std::vector<BodyCollisionCell>* output,const std::set<BodyCollisionCell>* initial,bool planar=false)const {
    if(columns_.empty())return false;
    bool hit=false;
    const auto& cfg=config_;const double r=cfg.resolution,c=std::cos(yaw),s=std::sin(yaw);
    const double ac=std::abs(c),as=std::abs(s);
    const double x=reference.x()+c*cfg.body_center_x-s*cfg.body_center_y;
    const double y=reference.y()+s*cfg.body_center_x+c*cfg.body_center_y;
    const double hx=cfg.body_length/2.+padding,hy=cfg.body_width/2.+padding;
    const double ex=ac*hx+as*hy,ey=as*hx+ac*hy;
    // 与原作者适配器使用相同的机体碰撞带，保持其地面/台阶语义。
    const auto [low,high]=heightBand(reference.z());
    for(int ix=int(std::floor((x-ex)/r))-1;ix<=int(std::floor((x+ex)/r));++ix)
      for(int iy=int(std::floor((y-ey)/r))-1;iy<=int(std::floor((y+ey)/r));++iy) {
        const auto found=columns_.find(key(ix,iy));if(found==columns_.end())continue;
        const double dx=(ix+.5)*r-x,dy=(iy+.5)*r-y;
        if(std::abs(dx)>ex+r/2.||std::abs(dy)>ey+r/2.
            ||std::abs(c*dx+s*dy)>hx+r/2.*(ac+as)||std::abs(-s*dx+c*dy)>hy+r/2.*(ac+as))continue;
        for(const auto& band:found->second) {
          if(!(band.low==band.high ? band.low>=low&&band.low<high : band.high>low&&band.low<high))continue;
          const BodyCollisionCell cell{ix,iy,0,true,band.low,band.high};
          if(initial&&initial->count(cell))continue;
          if(!output)return true;
          output->push_back(cell);hit=true;
          // 软净距只依赖平面列；恢复证明仍保留每一个真实高度带。
          if(planar)break;
        }
      }
    return hit;
  }
};
