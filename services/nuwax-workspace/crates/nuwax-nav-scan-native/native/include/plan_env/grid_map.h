#pragma once
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <unordered_map>
#include <vector>
#include <scan_native.h>
#include <scan_native_log.h>

// 上游双圆柱碰撞保留为粗查；精细实测体素按列索引，避免逐点物化三维膨胀。
class GridMap {
 public:
  using Ptr=std::shared_ptr<GridMap>;
  struct Key {int x,y,z;bool operator==(const Key& o) const{return x==o.x&&y==o.y&&z==o.z;}};
  struct Hash {size_t operator()(const Key& k) const {
    size_t s=std::hash<int>{}(k.x);
    s^=std::hash<int>{}(k.y)+0x9e3779b9+(s<<6)+(s>>2);
    s^=std::hash<int>{}(k.z)+0x9e3779b9+(s<<6)+(s>>2);return s;
  }};
  explicit GridMap(const ScanConfig& c,const double* obstacles,size_t obstacle_count,const double* ground,size_t ground_count,
      const double* boundary,size_t boundary_count,const int32_t* forbidden,size_t forbidden_count):config_(c),body_reference_offset_(c.body_to_ground) {
    // 候选桶降低碰撞查询成本；搜索和实体碰撞保留输入的精细体素尺寸。
    bucket_resolution_=std::max(config_.resolution,0.1);
    offset_=c.body_length/4.0;radius_=std::hypot(c.body_width/2.0,offset_);
    for(size_t i=0;i<boundary_count;++i) boundary_.emplace_back(boundary[2*i],boundary[2*i+1]);
    for(size_t i=0;i<obstacle_count;++i) {
      Key k=key(Eigen::Map<const Eigen::Vector3d>(obstacles+3*i));column(k.x,k.y).occupied.push_back(k.z);
    }
    for(size_t i=0;i<ground_count;++i) {
      Key k=key(Eigen::Map<const Eigen::Vector3d>(ground+3*i));column(k.x,k.y).ground.push_back(ground[3*i+2]);
    }
    for(size_t i=0;i<forbidden_count;++i) column(forbidden[2*i],forbidden[2*i+1]).forbidden=true;
    for(auto& entry:columns_) {
      Column& c=entry.second;
      std::sort(c.occupied.begin(),c.occupied.end());c.occupied.erase(std::unique(c.occupied.begin(),c.occupied.end()),c.occupied.end());
      std::sort(c.ground.begin(),c.ground.end());c.ground.erase(std::unique(c.ground.begin(),c.ground.end()),c.ground.end());
      const double x=(c.x+0.5)*config_.resolution,y=(c.y+0.5)*config_.resolution;
      Bucket& bucket=buckets_[bucketKey(x,y)];
      if(c.forbidden||!c.occupied.empty()) bucket.obstacles.push_back(&c);
      bucket.has_forbidden=bucket.has_forbidden||c.forbidden;
      if(!c.occupied.empty()) {
        bucket.min_obstacle=std::min(bucket.min_obstacle,c.occupied.front()*config_.resolution);
        bucket.max_obstacle=std::max(bucket.max_obstacle,(c.occupied.back()+1)*config_.resolution);
      }
      if(!c.ground.empty()) {
        bucket.ground.push_back(&c);
        bucket.min_ground=std::min(bucket.min_ground,c.ground.front());
        bucket.max_ground=std::max(bucket.max_ground,c.ground.back());
      }
    }
  }
  void setBodyReference(const Eigen::Vector3d& actual,double yaw=0.0) {
    initial_yaw_=yaw;
    double ground;
    if(measuredGround(actual,ground)) {
      const double delta=ground-(actual.z()-config_.body_to_ground);
      if(ground<=actual.z()+config_.body_z_min || (delta<=config_.max_step_up+1e-8&&delta>=-config_.max_step_down-1e-8))
        body_reference_offset_=actual.z()-ground;
    }
  }
  double initialYaw() const {return initial_yaw_;}
  double getResolution() const{return config_.resolution;}
  double getSearchResolution() const{return config_.resolution;}
  int getInflateOccupancy(Eigen::Vector3d pos,double yaw) const {
    if(!bodyInsideBoundary(pos,yaw)||groundCollision(pos,yaw)) return 1;
    const Eigen::Vector3d actual=pos;
    pos+=Eigen::Vector3d(config_.body_center_x*std::cos(yaw)-config_.body_center_y*std::sin(yaw),
        config_.body_center_x*std::sin(yaw)+config_.body_center_y*std::cos(yaw),(config_.body_z_min+config_.body_z_max)/2.0);
    const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.0);
    const Eigen::Vector3d front=pos+offset_*heading,rear=pos-offset_*heading;
    if(!getInflateOccupancyFromBuffer(front)&&!getInflateOccupancyFromBuffer(rear)) return 0;
    // 圆柱覆盖矩形会多出圆弧，必须由真实旋转包络与精细体素相交来裁决。
    return actualBodyCollision(actual,yaw)?1:0;
  }
  bool measuredGround(const Eigen::Vector3d& p,double& height) const {
    const Key k=key(p);auto it=columns_.find({k.x,k.y,0});
    if(it==columns_.end()||it->second.ground.empty()) return false;
    const auto& ground=it->second.ground;const double expected=p.z()-body_reference_offset_;
    auto found=std::lower_bound(ground.begin(),ground.end(),expected);
    if(found==ground.end()) height=ground.back();
    else if(found==ground.begin()) height=*found;
    else height=(expected-*(found-1)<*found-expected)?*(found-1):*found;
    return true;
  }
  double bodyHeight(const Eigen::Vector3d& p) const {
    double h;if(!measuredGround(p,h)) return p.z();
    const double delta=h+body_reference_offset_-p.z();
    const double step=delta>=0?config_.max_step_up:config_.max_step_down;
    // 同列只有高箱顶时不能把参考机身瞬移到箱顶；按原地形能力选择当前可接续层。
    if(std::abs(delta)>step+std::tan(config_.max_slope)*config_.resolution+1e-8) return p.z();
    return h+body_reference_offset_;
  }
  bool traversable(const Eigen::Vector3d& from,const Eigen::Vector3d& to) const {
    double a,b;if(!measuredGround(from,a)||!measuredGround(to,b)) return true;
    const double dz=b-a,step=dz>=0?config_.max_step_up:config_.max_step_down;
    return std::abs(dz)<=step+1e-8||std::atan2(std::abs(dz),(to.head<2>()-from.head<2>()).norm())<=config_.max_slope+1e-8;
  }
 private:
  struct Column {int x=0,y=0;bool forbidden=false;std::vector<int> occupied;std::vector<double> ground;};
  struct Bucket {
    std::vector<const Column*> obstacles,ground;
    double min_ground=std::numeric_limits<double>::infinity(),max_ground=-std::numeric_limits<double>::infinity();
    double min_obstacle=std::numeric_limits<double>::infinity(),max_obstacle=-std::numeric_limits<double>::infinity();
    bool has_forbidden=false;
  };
  Column& column(int x,int y) {auto& c=columns_[{x,y,0}];c.x=x;c.y=y;return c;}
  Key key(const Eigen::Vector3d& p) const{return {static_cast<int>(std::floor(p.x()/config_.resolution)),static_cast<int>(std::floor(p.y()/config_.resolution)),static_cast<int>(std::floor(p.z()/config_.resolution))};}
  Key bucketKey(double x,double y) const{return {static_cast<int>(std::floor(x/bucket_resolution_)),static_cast<int>(std::floor(y/bucket_resolution_)),0};}
  template<typename Predicate> bool anyBucket(double low_x,double high_x,double low_y,double high_y,Predicate predicate) const {
    const double half=config_.resolution/2.0;
    const Key low=bucketKey(low_x-half,low_y-half),high=bucketKey(high_x+half,high_y+half);
    for(int x=low.x;x<=high.x;++x) for(int y=low.y;y<=high.y;++y) {
      auto it=buckets_.find({x,y,0});if(it!=buckets_.end()&&predicate(it->second)) return true;
    }
    return false;
  }
  bool verticalOccupied(const Column& column,double low,double high) const {
    if(column.forbidden) return true;
    auto it=std::lower_bound(column.occupied.begin(),column.occupied.end(),static_cast<int>(std::floor(low/config_.resolution)));
    return it!=column.occupied.end()&&*it*config_.resolution<high&&(*it+1)*config_.resolution>low;
  }
  bool getInflateOccupancyFromBuffer(const Eigen::Vector3d& p) const {
    const double half_height=(config_.body_z_max-config_.body_z_min)/2.0,half_voxel=config_.resolution/2.0;
    return anyBucket(p.x()-radius_,p.x()+radius_,p.y()-radius_,p.y()+radius_,[&](const Bucket& b) {
      if(!b.has_forbidden&&(b.max_obstacle<=p.z()-half_height||b.min_obstacle>=p.z()+half_height)) return false;
      for(const Column* c:b.obstacles) {
        const double dx=std::max(0.0,std::abs((c->x+0.5)*config_.resolution-p.x())-half_voxel);
        const double dy=std::max(0.0,std::abs((c->y+0.5)*config_.resolution-p.y())-half_voxel);
        if(dx*dx+dy*dy<=radius_*radius_&&verticalOccupied(*c,p.z()-half_height,p.z()+half_height)) return true;
      }
      return false;
    });
  }
  bool rectangleIntersectsCell(const Eigen::Vector3d& p,double yaw,int x,int y) const {
    const double cs=std::cos(yaw),sn=std::sin(yaw),half=config_.resolution/2.0,hx=config_.body_length/2.0,hy=config_.body_width/2.0;
    const double cx=p.x()+cs*config_.body_center_x-sn*config_.body_center_y,cy=p.y()+sn*config_.body_center_x+cs*config_.body_center_y;
    const double dx=(x+0.5)*config_.resolution-cx,dy=(y+0.5)*config_.resolution-cy,voxel=half*(std::abs(cs)+std::abs(sn));
    return std::abs(dx*cs+dy*sn)<hx+voxel&&std::abs(-dx*sn+dy*cs)<hy+voxel&&
        std::abs(dx)<hx*std::abs(cs)+hy*std::abs(sn)+half&&std::abs(dy)<hx*std::abs(sn)+hy*std::abs(cs)+half;
  }
  template<typename Predicate> bool anyBodyBucket(const Eigen::Vector3d& p,double yaw,Predicate predicate) const {
    const double cs=std::cos(yaw),sn=std::sin(yaw);
    const double cx=p.x()+cs*config_.body_center_x-sn*config_.body_center_y,cy=p.y()+sn*config_.body_center_x+cs*config_.body_center_y;
    const double ex=std::abs(cs)*config_.body_length/2.0+std::abs(sn)*config_.body_width/2.0;
    const double ey=std::abs(sn)*config_.body_length/2.0+std::abs(cs)*config_.body_width/2.0;
    return anyBucket(cx-ex,cx+ex,cy-ey,cy+ey,predicate);
  }
  bool actualBodyCollision(const Eigen::Vector3d& p,double yaw) const {
    return anyBodyBucket(p,yaw,[&](const Bucket& b) {
      if(!b.has_forbidden&&(b.max_obstacle<=p.z()+config_.body_z_min||b.min_obstacle>=p.z()+config_.body_z_max)) return false;
      for(const Column* c:b.obstacles) if(rectangleIntersectsCell(p,yaw,c->x,c->y)&&bodyIntersectsOccupiedColumn(*c,p)) return true;
      return false;
    });
  }
  bool bodyIntersectsOccupiedColumn(const Column& column,const Eigen::Vector3d& p) const {
    if(column.forbidden) return true;
    const double low=p.z()+config_.body_z_min,high=p.z()+config_.body_z_max;
    int first_z=static_cast<int>(std::floor(low/config_.resolution));
    if(!column.ground.empty()) {
      const double expected=p.z()-body_reference_offset_;
      auto surface=std::lower_bound(column.ground.begin(),column.ground.end(),expected);
      const double height=surface==column.ground.end()?column.ground.back():
          surface==column.ground.begin()?*surface:
          expected-*(surface-1)<*surface-expected?*(surface-1):*surface;
      if(height>=expected-config_.max_step_down-1e-8&&height<=expected+config_.max_step_up+1e-8) {
        // 同列已确认的支撑表面所属体素及其下方实体，允许真实足端接触。
        // 表面高度仍是观测代表值；这里只统一占据格与支撑格的分类，不推高地面或删除占据。
        first_z=std::max(first_z,static_cast<int>(std::floor(height/config_.resolution))+1);
      }
    }
    const auto occupied=std::lower_bound(column.occupied.begin(),column.occupied.end(),first_z);
    return occupied!=column.occupied.end()&&*occupied*config_.resolution<high&&(*occupied+1)*config_.resolution>low;
  }
  bool groundCollision(const Eigen::Vector3d& p,double yaw) const {
    const double expected=p.z()-body_reference_offset_;
    return anyBodyBucket(p,yaw,[&](const Bucket& b) {
      // 平地/缓坡整桶均为可走支撑时，省去逐毫米格遍历；多层和低顶仍逐格裁决。
      if((b.min_ground>=expected-config_.max_step_down-1e-8&&b.max_ground<=expected+config_.max_step_up+1e-8)||
          b.max_ground<=p.z()+config_.body_z_min+1e-8||b.min_ground>p.z()+config_.body_z_max+1e-8) return false;
      for(const Column* c:b.ground) {
        if(!rectangleIntersectsCell(p,yaw,c->x,c->y)) continue;
        for(double h:c->ground) {
          if(h>=expected-config_.max_step_down-1e-8&&h<=expected+config_.max_step_up+1e-8) continue;
          if(h>p.z()+config_.body_z_min+1e-8&&h<=p.z()+config_.body_z_max+1e-8) return true;
        }
      }
      return false;
    });
  }
  bool insideBoundary(const Eigen::Vector2d& p) const {
    bool inside=false;
    for(size_t i=0,j=boundary_.size()-1;i<boundary_.size();j=i++) {
      const auto& a=boundary_[i];const auto& b=boundary_[j];const Eigen::Vector2d d=b-a;
      const double t=d.squaredNorm()>0?std::clamp((p-a).dot(d)/d.squaredNorm(),0.0,1.0):0.0;
      if((p-(a+t*d)).norm()<1e-8) return true;
      if((a.y()>p.y())!=(b.y()>p.y())&&p.x()<(b.x()-a.x())*(p.y()-a.y())/(b.y()-a.y())+a.x()) inside=!inside;
    }
    return inside;
  }
  bool bodyInsideBoundary(const Eigen::Vector3d& p,double yaw) const {
    if(boundary_.size()<3) return true;
    const double cs=std::cos(yaw),sn=std::sin(yaw),hx=config_.body_length/2.0,hy=config_.body_width/2.0;
    std::vector<Eigen::Vector2d> corners;
    for(const auto& v:std::vector<Eigen::Vector2d>{{-hx,-hy},{hx,-hy},{hx,hy},{-hx,hy}}) {
      const double x=v.x()+config_.body_center_x,y=v.y()+config_.body_center_y;
      corners.emplace_back(p.x()+cs*x-sn*y,p.y()+sn*x+cs*y);
      if(!insideBoundary(corners.back())) return false;
    }
    auto cross=[](const Eigen::Vector2d& a,const Eigen::Vector2d& b){return a.x()*b.y()-a.y()*b.x();};
    for(size_t i=0;i<4;++i) for(size_t j=0;j<boundary_.size();++j) {
      const Eigen::Vector2d a=corners[i],r=corners[(i+1)%4]-a,b=boundary_[j],s=boundary_[(j+1)%boundary_.size()]-b;
      const double denominator=cross(r,s);if(std::abs(denominator)<1e-12) continue;
      const double t=cross(b-a,s)/denominator,u=cross(b-a,r)/denominator;
      if(t>1e-8&&t<1.0-1e-8&&u>1e-8&&u<1.0-1e-8) return false;
    }
    return true;
  }
  ScanConfig config_;
  double offset_,radius_,bucket_resolution_,body_reference_offset_;
  double initial_yaw_=0.0;
  std::vector<Eigen::Vector2d> boundary_;
  std::unordered_map<Key,Column,Hash> columns_;
  std::unordered_map<Key,Bucket,Hash> buckets_;
};
