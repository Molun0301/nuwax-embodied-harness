#include "bridge.h"
#include "solver.h"
#include "incremental_grid.h"
#include "async_grid.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <vector>
#include <tbb/global_control.h>
#include <Eigen/Eigenvalues>

namespace {
std::atomic<bool> occupied{false};
constexpr size_t max_beams=32768, max_grid_cells=16*1024*1024;
struct InputError: std::runtime_error { using std::runtime_error::runtime_error; };
void require(bool ok,const char* message) { if(!ok) throw InputError(message); }
bool finite(NwSlamPose p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.yaw); }
karto::Pose2 pose(NwSlamPose p) { return {p.x,p.y,p.yaw}; }
NwSlamPose pose(karto::Pose2 p) { return {p.GetX(),p.GetY(),p.GetHeading()}; }
NwSlamPose compose(NwSlamPose a,NwSlamPose b) {
  const double c=std::cos(a.yaw),s=std::sin(a.yaw);
  return {a.x+c*b.x-s*b.y,a.y+s*b.x+c*b.y,std::remainder(a.yaw+b.yaw,2*M_PI)};
}
NwSlamPose inverse(NwSlamPose a) {
  const double c=std::cos(a.yaw),s=std::sin(a.yaw);
  return {-c*a.x-s*a.y,s*a.x-c*a.y,-a.yaw};
}
void config_valid(const NwSlamConfig& c) {
  require(c.abi_version==1,"不支持的二维算法接口版本");
  require(c.scan_count>=16 && c.scan_count<=max_beams,"扫描束数超出范围");
  require(c.local_window>=2 && c.local_window<=128 && c.max_nodes>=2 && c.max_nodes<=100000,"扫描窗口或图容量超出范围");
  const double values[]={c.angle_min,c.angle_increment,c.range_min,c.range_max,c.range_threshold,
    c.resolution,c.minimum_travel,c.minimum_heading,c.minimum_response,c.search_dimension,
    c.search_resolution,c.search_smear,c.max_scan_gap_s,c.link_maximum_distance,c.loop_search_distance};
  for(double v:values) require(std::isfinite(v),"配置含非有限数值");
  require(finite(c.base_from_laser),"雷达外参无效");
  require(c.angle_increment>0 && c.angle_increment*(c.scan_count-1)<=2*M_PI+1e-5,"扫描角度跨度非法");
  require(c.range_min>=0 && c.range_max>c.range_min && c.range_max<=1000 && c.range_threshold>c.range_min && c.range_threshold<=c.range_max,"有效量程非法");
  require(c.resolution>=0.02 && c.resolution<=1,"栅格分辨率超出范围");
  require(c.minimum_travel>=0 && c.minimum_heading>=0 && c.minimum_heading<=M_PI,"关键帧门限非法");
  require(c.minimum_response>0 && c.minimum_response<=1,"扫描匹配门限非法");
  require(c.search_dimension>=0.1 && c.search_dimension<=5 && c.search_resolution>=0.005 && c.search_resolution<=0.1 && c.search_smear>=0.005 && c.search_smear<=0.2,"扫描搜索配置超出范围");
  const double correlation_side=(c.search_dimension+2*c.range_threshold)/c.search_resolution+4;
  require(correlation_side*correlation_side<=16*1024*1024,"扫描相关网格超过内存预算");
  const double cells=c.search_dimension/c.search_resolution;
  require(std::abs(cells-std::round(cells))<1e-6,"搜索宽度必须为搜索分辨率的整数倍");
  require(c.use_scan_barycenter<=1&&c.loop_minimum_chain>=2&&c.loop_minimum_chain<=100&&c.link_maximum_distance>0&&c.link_maximum_distance<=30&&c.loop_search_distance>0&&c.loop_search_distance<=30,"图连接或回环范围非法");
  require(c.max_scan_gap_s>0 && c.max_scan_gap_s<=5 && c.solver_threads>=1 && c.solver_threads<=16 && c.loop_closing<=1,"时间或线程配置非法");
}
struct Context {
  NwSlamConfig config;
  tbb::global_control threads;
  NuwaxCeresSolver solver;
  std::unique_ptr<karto::Dataset> dataset;
  std::unique_ptr<karto::Mapper> mapper;
  std::unique_ptr<karto::Mapper> local_parameters;
  std::unique_ptr<karto::ScanMatcher> local_matcher;
  std::deque<std::unique_ptr<karto::LocalizedRangeScan>> local_scans;
  // 两份独立Boost归档中的图扫描与Dataset扫描不是同一指针，分别回收。
  std::vector<karto::LocalizedRangeScan*> loaded_graph_scans;
  karto::LaserRangeFinder* laser=nullptr;
  uint64_t timestamp=0,sequence=0,revision=0,local_timestamp=0,local_interval=0;
  NwSlamPose local_pose{},local_delta{},map_from_odom{},reprocessing{},seed{};
  bool poisoned=false,localization=false,near_seed=false,has_source=false,resume_pending=false;
  int odometry_mode=-1;
  std::vector<int8_t> grid;
  karto::IncrementalOccupancyGrid grid_cache;
  std::unique_ptr<NuwaxAsyncGrid> async_grid;
  std::unordered_map<const void*,std::shared_ptr<const NuwaxGridGeometry>> grid_geometry;
  std::vector<NuwaxGridNode> capture_grid() {
    auto scans=localization?loaded_graph_scans:mapper->GetAllProcessedScans();
    std::vector<NuwaxGridNode> nodes;nodes.reserve(scans.size());
    for(auto* scan:scans){
      auto& geometry=grid_geometry[scan];
      if(!geometry){
        auto data=std::make_shared<NuwaxGridGeometry>();
        data->ranges=scan->GetRangeReadingsVector();data->angles=scan->GetBeamAngles();geometry=std::move(data);
      }
      nodes.push_back({scan,geometry,scan->GetCorrectedPose()});
    }
    return nodes;
  }
  explicit Context(NwSlamConfig c):config(c),threads(tbb::global_control::max_allowed_parallelism,c.solver_threads),solver(c.solver_threads) {
    dataset=std::make_unique<karto::Dataset>();
    mapper=std::make_unique<karto::Mapper>(); configure(*mapper);
    mapper->SetScanSolver(&solver);
    laser=karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom,karto::Name("nuwax_planar_laser"));
    laser->SetOffsetPose(pose(c.base_from_laser));
    laser->SetMinimumRange(c.range_min); laser->SetMaximumRange(c.range_max);
    laser->SetMinimumAngle(c.angle_min); laser->SetMaximumAngle(c.angle_min+(c.scan_count-1)*c.angle_increment);
    laser->SetAngularResolution(c.angle_increment); laser->SetRangeThreshold(c.range_threshold);
    dataset->Add(laser,true);
    local_parameters=std::make_unique<karto::Mapper>(); configure(*local_parameters);

  }
  void configure(karto::Mapper& m) {
    m.setParamUseScanBarycenter(config.use_scan_barycenter!=0);
    m.setParamLoopMatchMinimumChainSize(config.loop_minimum_chain);
    m.setParamLinkScanMaximumDistance(config.link_maximum_distance);
    m.setParamLoopSearchMaximumDistance(config.loop_search_distance);
    m.setParamMinimumTravelDistance(config.minimum_travel);
    m.setParamMinimumTravelHeading(config.minimum_heading);
    m.setParamMinimumTimeInterval(0.5);
    m.setParamScanBufferSize(config.local_window);
    m.setParamScanBufferMaximumScanDistance(10.0);
    m.setParamCorrelationSearchSpaceDimension(config.search_dimension);
    m.setParamCorrelationSearchSpaceResolution(config.search_resolution);
    m.setParamCorrelationSearchSpaceSmearDeviation(config.search_smear);
    m.setParamDoLoopClosing(config.loop_closing!=0);
    m.setMinimumScanResponse(config.minimum_response);
  }
  ~Context() {
    // 构图影子对象仍引用只读雷达配置，先等待其退出再释放传感器。
    async_grid.reset();
    local_scans.clear(); local_matcher.reset(); local_parameters.reset();
    if(mapper && mapper->GetMapperSensorManager()) mapper->ClearLocalizationBuffer();
    mapper.reset();
    for(auto* scan:loaded_graph_scans) delete scan;
    dataset.reset();
  }
  size_t nodes() const {
    size_t count=0;
    if(mapper->GetGraph())for(const auto& group:mapper->GetGraph()->GetVertices())count+=group.second.size();
    return count;
  }
  bool raw_scan_input=false;
  std::unique_ptr<karto::LocalizedRangeScan> make_scan(const std::vector<double>& ranges,const std::vector<double>& angles,NwSlamPose p,uint64_t stamp) {
    auto scan=std::make_unique<karto::LocalizedRangeScan>(laser->GetName(),ranges);
    scan->SetBeamAngles(angles);
    scan->SetOdometricPose(pose(p)); scan->SetCorrectedPose(pose(p));
    scan->SetTime(static_cast<double>(stamp)*1e-9); return scan;
  }
  bool local_odometry(const std::vector<double>& ranges,const std::vector<double>& angles,uint64_t stamp,NwSlamPose& output,double& response) {
    if(!local_matcher)local_matcher.reset(karto::ScanMatcher::Create(local_parameters.get(),config.search_dimension,config.search_resolution,config.search_smear,config.range_threshold));
    auto scan=make_scan(ranges,angles,local_pose,stamp);
    response=-1;
    if(!local_scans.empty()) {
      require(stamp>local_timestamp && (resume_pending || (stamp-local_timestamp)*1e-9<=config.max_scan_gap_s),"激光里程计连续扫描缺测超限，需要重建来源");
      // 预测只作有界搜索初值，不能作为已测得运动返回。
      const double ratio=resume_pending?0.0:local_interval?std::min(2.0,double(stamp-local_timestamp)/double(local_interval)):1.0;
      NwSlamPose predicted_delta{local_delta.x*ratio,local_delta.y*ratio,local_delta.yaw*ratio};
      auto predicted=compose(local_pose,predicted_delta);
      scan->SetCorrectedPose(pose(predicted));
      karto::LocalizedRangeScanVector base;
      for(const auto& item:local_scans) base.push_back(item.get());
      karto::Pose2 best; karto::Matrix3 covariance;
      response=local_matcher->MatchScan(scan.get(),base,best,covariance,false,true);
      if(!std::isfinite(response)||response<config.minimum_response) return false;
      // 响应高不等于运动可观测；长直走廊等场景不能用零增量续期。
      for(int i=0;i<3;++i)for(int j=0;j<3;++j)
        require(std::isfinite(covariance(i,j)),"激光里程计协方差非有限");
      const double xx=covariance(0,0),yy=covariance(1,1),xy=(covariance(0,1)+covariance(1,0))*0.5;
      const double spread=std::hypot(xx-yy,2*xy);
      const double largest=(xx+yy+spread)*0.5,smallest=(xx+yy-spread)*0.5;
      require(smallest>=-1e-8 && covariance(2,2)>=0,"激光里程计协方差非法");
      // 由相邻有效回波的线法向构建三自由度信息矩阵，避免有限搜索网格制造虚假的窄协方差。
      double squared_range=0.0;size_t observed=0;
      for(double range:ranges)if(std::isfinite(range)&&range>config.range_min&&range<=config.range_threshold) {squared_range+=range*range;++observed;}
      const double scale=std::sqrt(squared_range/std::max<size_t>(1,observed));
      Eigen::Matrix3d information=Eigen::Matrix3d::Zero();
      Eigen::Vector2d previous;double previous_angle=0;size_t segments=0;bool has_previous=false;
      for(size_t index=0;index<ranges.size();++index) {
        const double range=ranges[index];
        if(!std::isfinite(range)||range<=config.range_min||range>config.range_threshold)continue;
        const double angle=angles.empty()?config.angle_min+index*config.angle_increment:angles[index];
        const Eigen::Vector2d point(range*std::cos(angle),range*std::sin(angle));
        const double angular_gap=std::abs(std::remainder(angle-previous_angle,2*M_PI));
        if(has_previous && angular_gap<=0.25) {
          const Eigen::Vector2d tangent=point-previous;const double length=tangent.norm();
          if(length>1e-6 && length<std::max(0.5,scale*angular_gap*3)) {
            const Eigen::Vector2d normal(-tangent.y()/length,tangent.x()/length),middle=(point+previous)*0.5;
            const Eigen::Vector3d row(normal.x(),normal.y(),(middle.x()*normal.y()-middle.y()*normal.x())/std::max(0.1,scale));
            information+=row*row.transpose();++segments;
          }
        }
        previous=point;previous_angle=angle;has_previous=true;
      }
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(information);
      const double information_ratio=eigen.info()==Eigen::Success?eigen.eigenvalues()[0]/std::max(1e-12,eigen.eigenvalues()[2]):0.0;
      if(segments<12 || information_ratio<0.002 || covariance(2,2)>0.03)
        throw InputError("激光里程计几何退化：最小信息比例="+std::to_string(information_ratio)+",平移最大方差="+std::to_string(largest));
      scan->SetSensorPose(best);
      auto matched=pose(scan->GetCorrectedPose());
      auto delta=compose(inverse(local_pose),matched);
      if(!finite(matched) || std::hypot(delta.x,delta.y)>config.search_dimension || std::abs(delta.yaw)>0.5) return false;
      local_delta=delta; local_pose=matched; resume_pending=false;
    }
    output=local_pose; local_interval=local_timestamp?stamp-local_timestamp:0;local_timestamp=stamp;
    local_scans.push_back(std::move(scan));
    while(local_scans.size()>config.local_window) local_scans.pop_front();
    return true;
  }
};
void error_text(char* dst,size_t capacity,const char* message) {
  if(!dst||!capacity)return;
  const size_t n=std::min(capacity-1,std::strlen(message));
  std::memcpy(dst,message,n); dst[n]=0;
}
template<class F> int protect(void* ptr,char* error,size_t capacity,F operation) {
  auto* ctx=static_cast<Context*>(ptr);
  if(!ctx) { error_text(error,capacity,"二维算法上下文为空"); return 1; }
  try {
    require(!ctx->poisoned,"二维算法上下文已失败，必须由拥有方回收重建");
    operation(*ctx); return 0;
  } catch(const InputError& e) { error_text(error,capacity,e.what());return 1;
  } catch(const std::exception& e) { ctx->poisoned=true;error_text(error,capacity,e.what());return 2;
  } catch(...) { ctx->poisoned=true;error_text(error,capacity,"未知原生算法异常");return 2; }
}
}
extern "C" uint32_t nw_slam_abi_version() { return 1; }
extern "C" int nw_slam_validate_config(const NwSlamConfig* config,char* error,size_t capacity) {
  try {require(config,"二维算法配置为空");config_valid(*config);return 0;}
  catch(const std::exception& e){error_text(error,capacity,e.what());return 1;}
  catch(...){error_text(error,capacity,"未知配置校验错误");return 2;}
}
extern "C" int nw_slam_create(const NwSlamConfig* config,void** out,char* error,size_t capacity) {
  if(out)*out=nullptr;
  bool claimed=false;
  try {
    require(config&&out,"二维算法配置或输出为空"); config_valid(*config);
    bool expected=false; require(occupied.compare_exchange_strong(expected,true),"每个工作进程只允许一个Karto上下文"); claimed=true;
    *out=new Context(*config); return 0;
  } catch(const std::exception& e) { if(claimed)occupied=false;error_text(error,capacity,e.what());return 1;
  } catch(...) {if(claimed)occupied=false;error_text(error,capacity,"二维算法创建失败");return 2;}
}
extern "C" void nw_slam_destroy(void* ctx) {
  if(!ctx)return;
  delete static_cast<Context*>(ctx); occupied=false;
}
extern "C" int nw_slam_set_scan_input_mode(void* ptr,uint32_t raw_mode,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(raw_mode<=1&&!c.has_source&&c.nodes()==0,"扫描模式必须在空实例初始化时选择");
    c.raw_scan_input=raw_mode!=0;
  });
}
static int process_scan(void* ptr,const NwSlamScan* in,const double* input_angles,size_t angle_count,NwSlamResult* out,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(in&&out&&in->ranges,"二维扫描为空");
    require(in->count>=16&&in->count<=max_beams&&in->timestamp_ns>0&&in->use_laser_odometry<=1,"二维扫描尺寸或时间非法");
    require(c.raw_scan_input == (input_angles!=nullptr),"输入角度布局与会话模式不一致");
    require(input_angles?angle_count==in->count:in->count==c.config.scan_count,"角度与距离数量不一致");
    std::vector<double> angles;
    if(input_angles) {
      angles.assign(input_angles,input_angles+angle_count);
      for(double angle:angles)require(std::isfinite(angle)&&std::abs(angle)<=M_PI*2+1e-5,"原始角度非有限或超出一圈");
    }
    require(!c.has_source || (in->timestamp_ns>c.timestamp && in->sequence>c.sequence),"二维扫描源时间或序号未递增");
    require(c.odometry_mode<0 || c.odometry_mode==static_cast<int>(in->use_laser_odometry),"运行中不能隐式更换里程计来源");
    require(in->use_laser_odometry || finite(in->odom_from_base),"设备里程计位姿无效");
    std::vector<double> ranges(in->ranges,in->ranges+in->count);
    unsigned valid=0;
    for(double& r:ranges) {
      if(std::isfinite(r)&&r>c.config.range_min&&r<c.config.range_max)++valid;
      else r=std::numeric_limits<double>::quiet_NaN();
    }
    require(valid>=std::max<size_t>(16,in->count/10),"有效激光束不足");
    require(c.localization || c.nodes()<c.config.max_nodes,"二维图节点容量耗尽，需要完成当前地图");
    c.has_source=true;c.timestamp=in->timestamp_ns;c.sequence=in->sequence;c.odometry_mode=in->use_laser_odometry;
    *out={};out->timestamp_ns=in->timestamp_ns;out->sequence=in->sequence;out->valid_beams=valid;
    out->matching_response=-1;out->local_response=-1;
    NwSlamPose odom=in->odom_from_base;
    auto fill=[&]{
      out->odom_from_base=odom; out->map_from_odom=c.map_from_odom;
      out->map_from_base=compose(c.map_from_odom,odom);
      out->graph_revision=c.revision;out->node_count=c.nodes();
      out->edge_count=c.mapper->GetGraph()?c.mapper->GetGraph()->GetEdges().size():0;
      out->optimizations=c.solver.optimizations();
    };
    if(in->use_laser_odometry && !c.local_odometry(ranges,angles,in->timestamp_ns,odom,out->local_response)) {
      odom=c.local_pose;out->disposition=3;fill();return;
    }
    const bool first=c.nodes()==0;
    auto scan=c.make_scan(ranges,angles,compose(c.reprocessing,odom),in->timestamp_ns);
    karto::Matrix3 covariance; covariance.SetToIdentity();
    bool accepted;
    if(c.near_seed) {
      scan->SetOdometricPose(pose(c.seed));scan->SetCorrectedPose(pose(c.seed));
      accepted=c.mapper->ProcessAgainstNodesNearBy(scan.get(),c.localization,&covariance);
    } else if(c.localization) accepted=c.mapper->ProcessLocalization(scan.get(),&covariance);
    else accepted=c.mapper->Process(scan.get(),&covariance);
    out->matching_response=c.mapper->lastScanResponse();
    if(accepted) {
      const auto corrected=pose(scan->GetCorrectedPose());
      if(!finite(corrected))throw std::runtime_error("扫描校正结果非有限");
      c.map_from_odom=compose(corrected,inverse(odom));
      if(c.near_seed) {c.reprocessing=c.map_from_odom;c.near_seed=false;}
      if(!c.localization)c.dataset->Add(scan.get());
      scan.release();
      if(!c.localization)++c.revision;out->disposition=first?0:1;
      for(int i=0;i<3;++i)for(int j=0;j<3;++j)out->covariance[i*3+j]=covariance(i,j);
    } else out->disposition=out->matching_response<0 && !c.near_seed?2:3;
    fill();
  });
}
extern "C" int nw_slam_process(void* ptr,const NwSlamScan* in,NwSlamResult* out,char* error,size_t capacity) {
  return process_scan(ptr,in,nullptr,0,out,error,capacity);
}
extern "C" int nw_slam_process_raw(void* ptr,const NwSlamScan* in,const double* angles,size_t angle_count,NwSlamResult* out,char* error,size_t capacity) {
  if(!angles){error_text(error,capacity,"原始角度数组为空");return 1;}
  return process_scan(ptr,in,angles,angle_count,out,error,capacity);
}
extern "C" int nw_slam_grid(void* ptr,NwSlamGrid* out,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(out,"栅格输出为空");require(c.nodes()>0,"尚无可构图扫描");
    auto scans=c.localization?c.loaded_graph_scans:c.mapper->GetAllProcessedScans();
    // 实际尺寸由同一增量构图器检查，不再按传感器中心外扩重复估算。
    auto* map=c.grid_cache.update(scans,c.config.resolution,2,0.1);
    require(bool(map),"栅格生成失败");
    auto origin=map->GetCoordinateConverter()->GetOffset();
    *out={c.revision,static_cast<uint32_t>(map->GetWidth()),static_cast<uint32_t>(map->GetHeight()),map->GetResolution(),origin.GetX(),origin.GetY()};
    const size_t count=size_t(out->width)*out->height;require(count<=max_grid_cells,"栅格载荷超过容量");
    c.grid.resize(count);
    for(uint32_t y=0;y<out->height;++y)for(uint32_t x=0;x<out->width;++x) {
      auto value=*map->GetDataPointer(karto::Vector2<kt_int32s>(x,y));
      c.grid[size_t(y)*out->width+x]=value==karto::GridStates_Occupied?100:value==karto::GridStates_Free?0:-1;
    }
  });
}
extern "C" int nw_slam_begin_grid(void* ptr,uint64_t revision,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(revision==c.revision&&c.nodes()>0,"异步栅格修订与当前图不一致");
    if(!c.async_grid)c.async_grid=std::make_unique<NuwaxAsyncGrid>(c.laser->GetName(),c.config.resolution);
    c.async_grid->submit(revision,c.capture_grid());
  });
}
extern "C" int nw_slam_poll_grid(void* ptr,NwSlamGrid* output,uint32_t* ready,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(output&&ready,"异步栅格输出为空");*ready=0;
    if(!c.async_grid)return;
    auto result=c.async_grid->poll();if(!result)return;
    if(!result->error.empty())throw std::runtime_error(result->error);
    *output=result->metadata;c.grid=std::move(result->cells);*ready=1;
  });
}
extern "C" int nw_slam_cancel_grid(void* ptr,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[](Context& c){if(c.async_grid)c.async_grid->cancel();});
}
extern "C" int nw_slam_copy_grid(void* ptr,int8_t* out,size_t count,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(out&&!c.grid.empty()&&count==c.grid.size(),"栅格缓冲区尺寸不匹配");std::copy(c.grid.begin(),c.grid.end(),out);
  });
}
extern "C" int nw_slam_save(void* ptr,const char* prefix,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(prefix&&*prefix&&!c.localization&&c.nodes()>0,"仅有有效图的建图模式可以保存");
    std::string graph=std::string(prefix)+".posegraph",data=std::string(prefix)+".data";
    require(!std::filesystem::exists(graph)&&!std::filesystem::exists(data),"图文件已存在，需要新的资产版本目录");
    try {c.mapper->SaveToFile(graph);c.dataset->SaveToFile(data);}
    catch(...) {std::error_code ec;std::filesystem::remove(graph,ec);std::filesystem::remove(data,ec);throw;}
  });
}
extern "C" int nw_slam_load(void* ptr,const char* prefix,uint32_t localization,NwSlamPose seed,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[&](Context& c){
    require(prefix&&*prefix&&localization<=1&&finite(seed)&&!c.has_source&&c.nodes()==0,"图加载必须使用空实例和有效初值");
    const std::string graph=std::string(prefix)+".posegraph",data=std::string(prefix)+".data";
    require(std::filesystem::is_regular_file(graph)&&std::filesystem::is_regular_file(data),"完整图数据缺失");
    require(std::filesystem::file_size(graph)<=512*1024*1024 && std::filesystem::file_size(data)<=512*1024*1024,"图数据超过加载预算");
    c.mapper.reset();c.dataset.reset();c.solver.Reset();
    c.mapper=std::make_unique<karto::Mapper>(); c.dataset=std::make_unique<karto::Dataset>();
    c.mapper->LoadFromFile(graph);
    c.loaded_graph_scans=c.mapper->GetAllProcessedScans();
    c.dataset->LoadFromFile(data);
    if(c.dataset->GetLasers().size()!=1)throw std::runtime_error("首版要求图中只有一个雷达");
    c.laser=dynamic_cast<karto::LaserRangeFinder*>(c.dataset->GetLasers().front());
    if(!c.laser)throw std::runtime_error("图中激光配置无效");
    auto equal=[](double a,double b){return std::abs(a-b)<1e-8;};
    auto offset=pose(c.laser->GetOffsetPose());
    if(!equal(c.laser->GetMinimumAngle(),c.config.angle_min)||!equal(c.laser->GetAngularResolution(),c.config.angle_increment)||
       !equal(c.laser->GetMaximumAngle(),c.config.angle_min+(c.config.scan_count-1)*c.config.angle_increment)||
       !equal(c.laser->GetMinimumRange(),c.config.range_min)||!equal(c.laser->GetMaximumRange(),c.config.range_max)||
       !equal(c.laser->GetRangeThreshold(),c.config.range_threshold)||
       !equal(offset.x,c.config.base_from_laser.x)||!equal(offset.y,c.config.base_from_laser.y)||!equal(offset.yaw,c.config.base_from_laser.yaw))
      throw std::runtime_error("图数据与当前雷达几何或外参不兼容");
    karto::SensorManager::GetInstance()->RegisterSensor(c.laser);
    if(c.loaded_graph_scans.empty()||c.loaded_graph_scans.size()>c.config.max_nodes)throw std::runtime_error("图节点为空或超过容量");
    for(auto* scan:c.loaded_graph_scans) {
      const auto count=scan->GetNumberOfRangeReadings();
      if(count<16||count>max_beams||c.raw_scan_input!=!scan->GetBeamAngles().empty()||!c.laser->Validate(scan))
        throw std::runtime_error("图扫描角度布局与输入模式不一致");
    }
    c.configure(*c.mapper);c.mapper->SetScanSolver(&c.solver);
    for(const auto& group:c.mapper->GetGraph()->GetVertices())for(const auto& item:group.second)c.solver.AddNode(item.second);
    for(auto* edge:c.mapper->GetGraph()->GetEdges())c.solver.AddConstraint(edge);
    // 加载保持已保存的优化姿态；下一次真实回环再运行求解器。
    c.localization=localization;c.near_seed=true;c.seed=seed;c.revision=1;
  });
}

extern "C" int nw_slam_resume(void* ptr,char* error,size_t capacity) {
  return protect(ptr,error,capacity,[](Context& c){
    // 暂停后的首个观测仍须匹配保留窗口，只取消跨长间隔的速度外推。
    c.resume_pending=true;
  });
}
