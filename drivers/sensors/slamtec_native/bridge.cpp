#include "sl_lidar.h"
#include "sl_lidar_driver.h"
#include <cmath>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
struct Device {
  std::unique_ptr<sl::IChannel> channel;
  std::unique_ptr<sl::ILidarDriver> driver;
  bool motor_started=false;
  std::vector<sl_lidar_response_measurement_node_hq_t> nodes;
  ~Device() {
    if(driver){driver->stop();if(motor_started)driver->setMotorSpeed(0);driver->disconnect();}
  }
};
void checked(sl_result code,const char* step){if(SL_IS_FAIL(code))throw std::runtime_error(std::string(step)+"，设备错误码"+std::to_string(code));}
void error_text(char* buffer,size_t size,const char* message){if(buffer&&size)std::snprintf(buffer,size,"%s",message);}
}
extern "C" {
uint32_t nw_slamtec_abi_version() { return 1; }
struct NuwaxSlamtecMode {double sample_period_us,range_max_m;uint32_t model,firmware;};
struct NuwaxSlamtecPoint {float angle_rad,range_m;uint8_t quality;};
int nw_slamtec_open(const char* port,uint32_t baud,int32_t mode_id,void** output,NuwaxSlamtecMode* mode,char* error,size_t capacity) {
  if(output)*output=nullptr;
  try {
    if(!port||!*port||!output||!mode||baud==0)throw std::runtime_error("雷达连接参数无效");
    auto device=std::make_unique<Device>();
    auto channel=sl::createSerialPortChannel(port,baud);checked(channel.err,"创建雷达串口");device->channel.reset(*channel);
    auto driver=sl::createLidarDriver();checked(driver.err,"创建雷达驱动");device->driver.reset(*driver);
    checked(device->driver->connect(device->channel.get()),"连接雷达");
    sl_lidar_response_device_info_t info{};checked(device->driver->getDeviceInfo(info,1000),"读取雷达身份");
    sl_lidar_response_device_health_t health{};checked(device->driver->getHealth(health,1000),"读取雷达健康");
    if(health.status==SL_LIDAR_STATUS_ERROR)throw std::runtime_error("雷达处于设备错误状态");
    sl::LidarScanMode selected{};
    checked(device->driver->setMotorSpeed(),"启动雷达电机");device->motor_started=true;
    if(mode_id<0) checked(device->driver->startScan(false,true,0,&selected),"启动典型扫描模式");
    else {
      std::vector<sl::LidarScanMode> modes;checked(device->driver->getAllSupportedScanModes(modes,1000),"读取扫描模式");
      bool found=false;for(const auto& entry:modes)if(entry.id==mode_id)found=true;
      if(!found)throw std::runtime_error("设备不支持配置的扫描模式");
      checked(device->driver->startScanExpress(false,static_cast<sl_u16>(mode_id),0,&selected,1000),"启动指定扫描模式");
    }
    if(!std::isfinite(selected.us_per_sample)||selected.us_per_sample<=0||!std::isfinite(selected.max_distance)||selected.max_distance<=0)
      throw std::runtime_error("雷达返回了无效采样周期或量程");
    *mode={selected.us_per_sample,selected.max_distance,info.model,info.firmware_version};*output=device.release();return 0;
  } catch(const std::exception& e){error_text(error,capacity,e.what());return -1;}
  catch(...){error_text(error,capacity,"未知雷达连接异常");return -1;}
}
int nw_slamtec_read(void* handle,NuwaxSlamtecPoint* output,size_t* count,uint64_t* timestamp_us,char* error,size_t capacity) {
  try {
    if(!handle||!output||!count||!timestamp_us||*count==0||*count>32768)throw std::runtime_error("雷达读取缓冲区无效");
    auto& device=*static_cast<Device*>(handle);
    auto& nodes=device.nodes;
    nodes.resize(*count);
    sl_u64 stamp=0;size_t n=nodes.size();
    auto result=device.driver->grabScanDataHqWithTimeStamp(nodes.data(),n,stamp,100);
    if(result==SL_RESULT_OPERATION_TIMEOUT){*count=0;return 1;}
    checked(result,"读取完整雷达扫描");
    if(n>nodes.size())throw std::runtime_error("雷达返回扫描超过缓冲区");
    for(size_t i=0;i<n;++i){
      // 保留设备采集顺序；在规范适配边界归一坐标方向和等角网格。
      output[i]={static_cast<float>(nodes[i].angle_z_q14*M_PI/32768.0),nodes[i].dist_mm_q2/4000.0f,
        static_cast<uint8_t>(nodes[i].quality>>SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT)};
    }
    *count=n;*timestamp_us=stamp;return 0;
  } catch(const std::exception& e){error_text(error,capacity,e.what());return -1;}
  catch(...){error_text(error,capacity,"未知雷达读取异常");return -1;}
}
void nw_slamtec_close(void* handle){delete static_cast<Device*>(handle);}
}
