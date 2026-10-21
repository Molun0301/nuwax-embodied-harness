#pragma once
#include <Eigen/Eigen>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <plan_env/grid_map.h>
// Optional observation only. Disabled runs perform no diagnostic map queries
// or output. Diagnostic timings are never acceptance/performance evidence.
namespace scan_planner {
inline bool authorTraceEnabled(){const char* p=std::getenv("NEH_SCAN_A_TRACE");return p&&*p;}
inline std::ofstream authorTraceStream(){
 const char* path=std::getenv("NEH_SCAN_A_TRACE");
 std::ofstream out;if(path&&*path){out.open(path,std::ios::app);out<<std::setprecision(17);}return out;
}
template<class Derived> inline void traceMatrix(std::ostream& out,const Eigen::MatrixBase<Derived>& value){
 out<<'[';for(Eigen::Index col=0;col<value.cols();++col){if(col)out<<',';out<<'[';
 for(Eigen::Index row=0;row<value.rows();++row){if(row)out<<',';if(std::isfinite(value(row,col)))out<<value(row,col);else out<<"null";}out<<']';}out<<']';
}
// Observes original buffers without changing cache, odds, or query semantics.
inline void traceMapProbe(std::ostream& out, GridMap& map,
                          const Eigen::Vector3d& point, double yaw) {
 out << "{\"point\":"; traceMatrix(out, point);
 out << ",\"raw_occupancy\":" << map.getOccupancy(point)
     << ",\"body_inflated_occupancy\":" << map.getInflateOccupancy(point, yaw);
 const auto cell = [&](const Eigen::Vector3d& p) {
  out << "{\"point\":"; traceMatrix(out,p);
  if (!map.isInMap(p)) {out << ",\"in_map\":false}"; return;}
  Eigen::Vector3i index; map.posToIndex(p,index); const int address=map.toAddress(index);
  const auto& data=map.mappingData();
  out << ",\"in_map\":true,\"index\":"; traceMatrix(out,index);
  out << ",\"log_odds\":" << data.occupancy_buffer_[address]
      << ",\"inflated_cell\":" << int(data.occupancy_buffer_inflate_[address]) << '}';
 };
 out << ",\"raw_cell\":"; cell(point);
 const auto& parameters=map.mappingParameters();
 const double c=std::cos(yaw),s=std::sin(yaw);
 Eigen::Vector3d center=point;
 center.x()+=c*parameters.body_center_offset_.x()-s*parameters.body_center_offset_.y();
 center.y()+=s*parameters.body_center_offset_.x()+c*parameters.body_center_offset_.y();
 center.z()+=parameters.body_center_offset_.z();
 const Eigen::Vector3d heading(c,s,0.);
 out << ",\"front_cell\":"; cell(center+parameters.double_cylinder_offset_*heading);
 out << ",\"rear_cell\":"; cell(center-parameters.double_cylinder_offset_*heading);
 out << ",\"occupancy_log_threshold\":" << parameters.min_occupancy_log_ << '}';
}

}
