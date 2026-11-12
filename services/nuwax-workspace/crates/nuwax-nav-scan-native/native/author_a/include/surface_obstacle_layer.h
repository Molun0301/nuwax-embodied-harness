#pragma once
#include <scan_native.h>
#include <body_collision_cell.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <array>
#include <memory>
#include <cstring>
#include <set>
#include <vector>

// 当前完整表面的已知实体，独立于注册雷达概率图；不生成射线、自由格或来源时效。
class SurfaceObstacleLayer {
 public:
  struct Band { double low, high; };
  struct Column { std::vector<double> obstacles, ground; std::vector<Band> bands; };
 private:
  using Key = std::array<int, 2>;
  using Chunk = std::map<Key, std::shared_ptr<const Column>>;
  ScanConfig config_;
  std::map<Key, std::shared_ptr<const Chunk>> chunks_;
  static Key chunk(Key key) {
    return {int(std::floor(key[0] / 32.)), int(std::floor(key[1] / 32.))};
  }
  const Column* column(int x, int y) const {
    const auto c = chunks_.find(chunk({x, y}));
    if(c == chunks_.end()) return nullptr;
    const auto entry = c->second->find({x, y});
    return entry == c->second->end() ? nullptr : entry->second.get();
  }
  std::shared_ptr<const Column> makeColumn(const double* obstacles,size_t count,
      const double* ground,size_t ground_count) const {
    auto result=std::make_shared<Column>();
    if(count)result->obstacles.assign(obstacles,obstacles+3*count);
    if(ground_count)result->ground.assign(ground,ground+3*ground_count);
    const double r=config_.resolution;
    for(size_t i=0;i<count;++i)result->bands.push_back({obstacles[3*i+2]-r/2.,obstacles[3*i+2]+r/2.});
    for(size_t i=0;i<ground_count;++i)result->bands.push_back({ground[3*i+2],ground[3*i+2]});
    return result;
  }
 public:
  SurfaceObstacleLayer(const ScanConfig& config,const double* obstacles,size_t count,
      const double* ground,size_t ground_count):config_(config) {
    std::map<Key,Column> columns;
    auto append=[&](const double* points,size_t count,bool is_ground){
      for(size_t i=0;i<count;++i) {
        const auto* p=points+3*i;
        auto& c=columns[{int(std::floor(p[0]/config.resolution)),int(std::floor(p[1]/config.resolution))}];
        auto& list=is_ground?c.ground:c.obstacles;list.insert(list.end(),p,p+3);
      }
    };
    append(obstacles,count,false);append(ground,ground_count,true);
    std::map<Key,std::shared_ptr<Chunk>> grouped;
    for(const auto& [key,c]:columns) {
      auto& part=grouped[chunk(key)];if(!part)part=std::make_shared<Chunk>();
      (*part)[key]=makeColumn(c.obstacles.data(),c.obstacles.size()/3,c.ground.data(),c.ground.size()/3);
    }
    for(auto& [key,c]:grouped)chunks_[key]=std::move(c);
  }
  SurfaceObstacleLayer(const SurfaceObstacleLayer& previous,const ScanSurfaceColumn* columns,size_t count)
      :config_(previous.config_),chunks_(previous.chunks_) {
    std::map<Key,std::shared_ptr<Chunk>> changed;
    for(size_t i=0;i<count;++i) {
      const auto& c=columns[i];const Key key{c.x,c.y},part_key=chunk(key);
      const auto* old=column(c.x,c.y);
      if(old&&old->obstacles.size()==c.obstacle_count*3&&old->ground.size()==c.ground_count*3
          &&(!c.obstacle_count||std::memcmp(old->obstacles.data(),c.obstacles,c.obstacle_count*3*sizeof(double))==0)
          &&(!c.ground_count||std::memcmp(old->ground.data(),c.ground,c.ground_count*3*sizeof(double))==0))continue;
      if(!old&&!c.obstacle_count&&!c.ground_count)continue;
      auto& part=changed[part_key];
      if(!part) {auto found=chunks_.find(part_key);part=found==chunks_.end()
          ?std::make_shared<Chunk>():std::make_shared<Chunk>(*found->second);}
      if(!c.obstacle_count&&!c.ground_count)part->erase(key);
      else (*part)[key]=makeColumn(c.obstacles,c.obstacle_count,c.ground,c.ground_count);
    }
    for(auto& [key,c]:changed) {if(c->empty())chunks_.erase(key);else chunks_[key]=std::move(c);}
  }
  template<class F> void eachColumn(F&& function) const {
    for(const auto& [_,chunk]:chunks_)for(const auto& [xy,column]:*chunk)function(xy[0],xy[1],*column);
  }
  bool sameColumns(const SurfaceObstacleLayer& other,int x0,int x1,int y0,int y1) const {
    const auto first=chunk({x0,y0}),last=chunk({x1,y1});
    for(int bx=first[0];bx<=last[0];++bx)for(int by=first[1];by<=last[1];++by) {
      const auto a=chunks_.find({bx,by}),b=other.chunks_.find({bx,by});
      if(a==chunks_.end()&&b==other.chunks_.end())continue;
      if(a!=chunks_.end()&&b!=other.chunks_.end()&&a->second==b->second)continue;
      for(int x=std::max(x0,bx*32);x<=std::min(x1,bx*32+31);++x)
        for(int y=std::max(y0,by*32);y<=std::min(y1,by*32+31);++y) {
          const auto* one=column(x,y);const auto* two=other.column(x,y);
          if(one==two)continue;
          if(!one||!two) {if((one&&!one->bands.empty())||(two&&!two->bands.empty()))return false;continue;}
          if(one->bands.size()!=two->bands.size()||!std::equal(one->bands.begin(),one->bands.end(),two->bands.begin(),
              [](const Band& a,const Band& b){return a.low==b.low&&a.high==b.high;}))return false;
        }
    }
    return true;
  }
  bool intersects(const Eigen::Vector3d& reference,double yaw,double padding=0.)const {
    return contacts(reference,yaw,padding,nullptr,nullptr);
  }
  // 硬接触和净空场的列分类共用高度带及开闭边界。
  std::pair<double,double> heightBand(double reference_z) const {
    return {reference_z+std::max(config_.max_step_up,config_.resolution)-config_.body_to_ground,
            reference_z+config_.body_z_max};
  }
  bool columnContact(int x,int y,double reference_z) const {
    const auto* found=column(x,y);
    if(!found)return false;
    const auto [low,high]=heightBand(reference_z);
    for(const auto& band:found->bands)
      if(band.low==band.high ? band.low>=low&&band.low<high : band.high>low&&band.low<high)return true;
    return false;
  }
  bool contacts(const Eigen::Vector3d& reference,double yaw,double padding,
      std::vector<BodyCollisionCell>* output,const std::set<BodyCollisionCell>* initial,bool planar=false)const {
    if(chunks_.empty())return false;
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
        const auto* found=column(ix,iy);if(!found)continue;
        const double dx=(ix+.5)*r-x,dy=(iy+.5)*r-y;
        if(std::abs(dx)>ex+r/2.||std::abs(dy)>ey+r/2.
            ||std::abs(c*dx+s*dy)>hx+r/2.*(ac+as)||std::abs(-s*dx+c*dy)>hy+r/2.*(ac+as))continue;
        for(const auto& band:found->bands) {
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
