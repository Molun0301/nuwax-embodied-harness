#pragma once
#include <scan_native.h>
#include <body_collision_cell.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <set>
#include <memory>
#include <vector>

// 当前完整表面的已知实体，独立于注册雷达概率图；不生成射线、自由格或来源时效。
class SurfaceObstacleLayer {
  struct Band { double low, high; };
  ScanConfig config_;
  using Columns = std::unordered_map<std::uint64_t,std::vector<Band>>;
  struct Obstacles { std::vector<double> points; Columns columns; };
  std::shared_ptr<const Obstacles> obstacles_;
  std::vector<double> ground_;
  Columns ground_columns_;
  static std::uint64_t key(int x,int y) {
    return (std::uint64_t(std::uint32_t(x))<<32)|std::uint32_t(y);
  }
 public:
  SurfaceObstacleLayer(const ScanConfig& config,const double* obstacles,size_t count,
      const double* ground,size_t ground_count):config_(config) {
    auto obstacles_data=std::make_shared<Obstacles>();
    if(count)obstacles_data->points.assign(obstacles,obstacles+3*count);
    const double r=config.resolution;
    for(size_t i=0;i<count;++i) {
      const auto* p=obstacles+3*i;
      obstacles_data->columns[key(int(std::floor(p[0]/r)),int(std::floor(p[1]/r)))].push_back({p[2]-r/2.,p[2]+r/2.});
    }
    obstacles_=std::move(obstacles_data);
    setGround(ground,ground_count);
  }
  // 位姿相关的地面视图只重建地面列，实体点与索引继续共享。
  SurfaceObstacleLayer(const SurfaceObstacleLayer& source,const double* ground,size_t count)
      :config_(source.config_),obstacles_(source.obstacles_) {setGround(ground,count);}
 private:
  void setGround(const double* ground,size_t count) {
    if(count)ground_.assign(ground,ground+3*count);
    for(size_t i=0;i<count;++i) {
      const auto* p=ground+3*i;
      ground_columns_[key(int(std::floor(p[0]/config_.resolution)),int(std::floor(p[1]/config_.resolution)))].push_back({p[2],p[2]});
    }
  }
 public:
  bool matches(const double* obstacles,size_t count,const double* ground,size_t ground_count)const {
    return obstacles_->points.size()==3*count&&ground_.size()==3*ground_count
        &&(!count||std::equal(obstacles_->points.begin(),obstacles_->points.end(),obstacles))
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
    const auto [low,high]=heightBand(reference_z);
    for(const auto* columns:{&obstacles_->columns,&ground_columns_}) {
      const auto found=columns->find(key(x,y));
      if(found==columns->end())continue;
      for(const auto& band:found->second)
        if(band.low==band.high ? band.low>=low&&band.low<high : band.high>low&&band.low<high)return true;
    }
    return false;
  }
  bool contacts(const Eigen::Vector3d& reference,double yaw,double padding,
      std::vector<BodyCollisionCell>* output,const std::set<BodyCollisionCell>* initial,bool planar=false)const {
    if(obstacles_->columns.empty()&&ground_columns_.empty())return false;
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
        const double dx=(ix+.5)*r-x,dy=(iy+.5)*r-y;
        if(std::abs(dx)>ex+r/2.||std::abs(dy)>ey+r/2.
            ||std::abs(c*dx+s*dy)>hx+r/2.*(ac+as)||std::abs(-s*dx+c*dy)>hy+r/2.*(ac+as))continue;
        bool column_hit=false;
        for(const auto* columns:{&obstacles_->columns,&ground_columns_}) {
          const auto found=columns->find(key(ix,iy));if(found==columns->end())continue;
          for(const auto& band:found->second) {
          if(!(band.low==band.high ? band.low>=low&&band.low<high : band.high>low&&band.low<high))continue;
          const BodyCollisionCell cell{ix,iy,0,true,band.low,band.high};
          if(initial&&initial->count(cell))continue;
          recordCollision((ix+.5)*r,(iy+.5)*r,band.low,band.high,SurfaceContact);
          if(!output)return true;
          output->push_back(cell);hit=true;
          // 软净距只依赖平面列；恢复证明仍保留每一个真实高度带。
          if(planar){column_hit=true;break;}
          }
          if(column_hit)break;
        }
      }
    return hit;
  }
};
