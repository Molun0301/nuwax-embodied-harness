#pragma once
#include "bridge.h"
#include "incremental_grid.h"
#include <condition_variable>
#include <mutex>
#include <optional>
#include <thread>
#include <unordered_map>

// 只传不可变扫描几何和位姿副本，背景线程从不访问主线程的可变Karto图。
struct NuwaxGridGeometry {std::vector<double> ranges,angles;};
struct NuwaxGridNode {const void* id;std::shared_ptr<const NuwaxGridGeometry> geometry;karto::Pose2 pose;};
struct NuwaxGridJob {uint64_t revision,epoch;std::vector<NuwaxGridNode> nodes;};
struct NuwaxGridResult {uint64_t epoch;NwSlamGrid metadata{};std::vector<int8_t> cells;std::string error;};
class NuwaxAsyncGrid {
 public:
  NuwaxAsyncGrid(karto::Name sensor,double resolution):sensor_(std::move(sensor)),resolution_(resolution),thread_([this]{run();}){}
  ~NuwaxAsyncGrid(){
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;pending_.reset();}
    wake_.notify_one();thread_.join();
  }
  void submit(uint64_t revision,std::vector<NuwaxGridNode> nodes){
    {std::lock_guard<std::mutex> lock(mutex_);pending_=NuwaxGridJob{revision,epoch_,std::move(nodes)};ready_.reset();}
    wake_.notify_one();
  }
  std::optional<NuwaxGridResult> poll(){
    std::lock_guard<std::mutex> lock(mutex_);auto result=std::move(ready_);ready_.reset();return result;
  }
  void cancel(){std::lock_guard<std::mutex> lock(mutex_);++epoch_;pending_.reset();ready_.reset();}
 private:
  struct Shadow {std::shared_ptr<const NuwaxGridGeometry> geometry;std::unique_ptr<karto::LocalizedRangeScan> scan;};
  NuwaxGridResult build(const NuwaxGridJob& job){
    NuwaxGridResult result;result.epoch=job.epoch;
    try {
      karto::LocalizedRangeScanVector scans;scans.reserve(job.nodes.size());
      for(const auto& node:job.nodes){
        auto& shadow=shadows_[node.id];
        if(shadow.geometry!=node.geometry){
          shadow.geometry=node.geometry;
          shadow.scan=std::make_unique<karto::LocalizedRangeScan>(sensor_,node.geometry->ranges);
          shadow.scan->SetBeamAngles(node.geometry->angles);
        }
        const auto& previous=shadow.scan->GetCorrectedPose();
        if(previous.GetX()!=node.pose.GetX()||previous.GetY()!=node.pose.GetY()||previous.GetHeading()!=node.pose.GetHeading())
          shadow.scan->SetCorrectedPose(node.pose);
        scans.push_back(shadow.scan.get());
      }
      auto* map=cache_.update(scans,resolution_,2,0.1);
      if(!map)throw std::runtime_error("尚无可构图扫描");
      auto origin=map->GetCoordinateConverter()->GetOffset();
      result.metadata={job.revision,static_cast<uint32_t>(map->GetWidth()),static_cast<uint32_t>(map->GetHeight()),resolution_,origin.GetX(),origin.GetY()};
      result.cells.resize(size_t(map->GetWidth())*map->GetHeight());
      for(uint32_t y=0;y<result.metadata.height;++y)for(uint32_t x=0;x<result.metadata.width;++x){
        auto value=*map->GetDataPointer(karto::Vector2<kt_int32s>(x,y));
        result.cells[size_t(y)*result.metadata.width+x]=value==karto::GridStates_Occupied?100:value==karto::GridStates_Free?0:-1;
      }
    }catch(const std::exception& error){result.error=error.what();}
    return result;
  }
  void run(){
    for(;;){
      NuwaxGridJob job;
      {std::unique_lock<std::mutex> lock(mutex_);wake_.wait(lock,[&]{return stopping_||pending_.has_value();});
       if(stopping_)return;job=std::move(*pending_);pending_.reset();}
      auto result=build(job);
      {std::lock_guard<std::mutex> lock(mutex_);if(!stopping_&&result.epoch==epoch_)ready_=std::move(result);}
    }
  }
  karto::Name sensor_;double resolution_;
  std::mutex mutex_;std::condition_variable wake_;bool stopping_=false;uint64_t epoch_=0;
  std::optional<NuwaxGridJob> pending_;std::optional<NuwaxGridResult> ready_;
  std::unordered_map<const void*,Shadow> shadows_;karto::IncrementalOccupancyGrid cache_;
  std::thread thread_;
};
