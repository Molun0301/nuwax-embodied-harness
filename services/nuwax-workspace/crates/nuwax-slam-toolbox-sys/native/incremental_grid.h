#pragma once
#include <karto_sdk/Karto.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <tuple>
#include <vector>

namespace karto {
// 复用原Karto射线计数和占据判定，只撤销/重画量化端点发生变化的束。
// 原点可以连续变化；每次仍按当前精确原点重新量化。尺寸变化完整回退。
class IncrementalOccupancyGrid {
 public:
  IncrementalOccupancyGrid() = default;
  OccupancyGrid* update(const LocalizedRangeScanVector& scans,double resolution,
                        kt_int32u minimum_pass=2,double threshold=0.1) {
    changed_rays_=0;full_rebuild_=false;
    if(scans.empty()) {grid_.reset();rays_.clear();keys_.clear();return nullptr;}
    kt_int32s width,height;Vector2<kt_double> offset;
    OccupancyGrid::ComputeDimensions(scans,resolution,width,height,offset);
    if(width<=0 || height<=0 || std::uint64_t(width)*std::uint64_t(height)>16*1024*1024)
      throw std::runtime_error("二维增量栅格尺寸无效或超过预算");
    if(!grid_ || !cache_ready_ || grid_->GetWidth()!=width || grid_->GetHeight()!=height || grid_->GetResolution()!=resolution) {
      grid_=std::make_unique<OccupancyGrid>(width,height,offset,resolution);
      grid_->m_pCellPassCnt->Resize(width,height);grid_->m_pCellHitsCnt->Resize(width,height);
      rays_.clear();keys_.clear();full_rebuild_=true;cache_ready_=true;
    }
    const auto previous_offset=grid_->GetCoordinateConverter()->GetOffset();
    const bool origin_changed=previous_offset.GetX()!=offset.GetX() || previous_offset.GetY()!=offset.GetY();
    grid_->SetMinPassThrough(minimum_pass);grid_->SetOccupancyThreshold(threshold);
    grid_->GetCoordinateConverter()->SetOffset(offset);
    grid_->m_pCellPassCnt->GetCoordinateConverter()->SetOffset(offset);
    grid_->m_pCellHitsCnt->GetCoordinateConverter()->SetOffset(offset);
    for(std::size_t i=scans.size();i<rays_.size();++i) for(const auto& ray:rays_[i]) {apply(ray,false);++changed_rays_;}
    rays_.resize(scans.size());keys_.resize(scans.size());
    for(std::size_t i=0;i<scans.size();++i) {
      const auto pose=scans[i]->GetSensorPose();
      const CacheKey key{scans[i],scans[i]->GetRangeReadings(),scans[i]->GetNumberOfRangeReadings(),pose};
      // 原始扫描几何在本次图生命周期内不可变，稳定节点无需逐束重复量化。
      if(!origin_changed && keys_[i]==key)continue;
      auto next=quantize(scans[i]);auto& previous=rays_[i];
      const auto shared=std::min(previous.size(),next.size());
      for(std::size_t j=0;j<shared;++j) if(!(previous[j]==next[j])) {apply(previous[j],false);apply(next[j],true);++changed_rays_;}
      for(std::size_t j=shared;j<previous.size();++j) {apply(previous[j],false);++changed_rays_;}
      for(std::size_t j=shared;j<next.size();++j) {apply(next[j],true);++changed_rays_;}
      previous=std::move(next);keys_[i]=key;
    }
    grid_->Update();return grid_.get();
  }
  std::size_t changed_rays() const {return changed_rays_;}
  bool full_rebuild() const {return full_rebuild_;}
  std::size_t cached_rays() const {std::size_t count=0;for(const auto& rays:rays_)count+=rays.size();return count;}
 private:
  struct CacheKey {
    const LocalizedRangeScan* scan=nullptr;const double* ranges=nullptr;std::size_t count=0;Pose2 pose;
    bool operator==(const CacheKey& other)const {
      return scan==other.scan && ranges==other.ranges && count==other.count && pose.GetX()==other.pose.GetX() && pose.GetY()==other.pose.GetY() && pose.GetHeading()==other.pose.GetHeading();
    }
  };
  struct Ray {
    kt_int32s x0=0,y0=0,x1=0,y1=0;bool hit=false,active=false;
    bool operator==(const Ray& other)const {
      return std::tie(x0,y0,x1,y1,hit,active)==std::tie(other.x0,other.y0,other.x1,other.y1,other.hit,other.active);
    }
  };
  std::vector<Ray> quantize(LocalizedRangeScan* scan) const {
    if(!scan)return {};
    auto* laser=scan->GetLaserRangeFinder();const double threshold=laser->GetRangeThreshold();
    const auto position=scan->GetSensorPose().GetPosition();const auto& points=scan->GetPointReadings(false);
    if(points.size()!=scan->GetNumberOfRangeReadings())throw std::runtime_error("二维扫描端点与距离数量不一致");
    const auto from=grid_->m_pCellPassCnt->WorldToGrid(position);
    std::vector<Ray> result(points.size());
    for(std::size_t i=0;i<points.size();++i) {
      const double range=scan->GetRangeReadings()[i];
      if(range<=laser->GetMinimumRange() || range>=laser->GetMaximumRange() || std::isnan(range))continue;
      auto point=points[i];
      if(range>=threshold) {
        const double ratio=threshold/range;
        point.SetX(position.GetX()+ratio*(point.GetX()-position.GetX()));
        point.SetY(position.GetY()+ratio*(point.GetY()-position.GetY()));
      }
      const auto to=grid_->m_pCellPassCnt->WorldToGrid(point);
      result[i]={from.GetX(),from.GetY(),to.GetX(),to.GetY(),range<(threshold-KT_TOLERANCE),true};
    }
    return result;
  }
  class Undo final:public Functor {
   public:
    explicit Undo(kt_int32u* values):values_(values){}
    void operator()(kt_int32u index) override {
      // TraceLine已增加一次；再减二，净撤销原来的一次穿越。
      if(values_[index]<2)throw std::runtime_error("二维射线穿越计数撤销下溢");
      values_[index]-=2;
    }
   private:kt_int32u* values_;
  };
  void apply(const Ray& ray,bool add) {
    if(!ray.active)return;
    auto* passes=grid_->m_pCellPassCnt;auto* hits=grid_->m_pCellHitsCnt;
    Undo undo(passes->GetDataPointer());
    passes->TraceLine(ray.x0,ray.y0,ray.x1,ray.y1,add?nullptr:&undo);
    const Vector2<kt_int32s> end(ray.x1,ray.y1);
    if(ray.hit && passes->IsValidGridIndex(end)) {
      const auto index=passes->GridIndex(end,false);auto& pass=passes->GetDataPointer()[index];auto& hit=hits->GetDataPointer()[index];
      if(add){++pass;++hit;}
      else {if(pass==0||hit==0)throw std::runtime_error("二维命中计数撤销下溢");--pass;--hit;}
    }
  }
  std::vector<CacheKey> keys_;
  std::unique_ptr<OccupancyGrid> grid_;
  std::vector<std::vector<Ray>> rays_;
  std::size_t changed_rays_=0;
  bool full_rebuild_=false,cache_ready_=false;
};
}
