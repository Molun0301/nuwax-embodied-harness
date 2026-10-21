#pragma once
// Transport-only, dependency-free equivalents for author A's ROS values.
// No ROS node, network or spinner exists. The owner supplies a single clock.
#include <Eigen/Eigen>
#include <algorithm>
#include <any>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <scan_native_log.h>
namespace ros {
inline thread_local double host_time_seconds=0.0;
struct Duration { double value=0; Duration()=default; explicit Duration(double x):value(x){} double toSec()const{return value;} void sleep()const{} };
struct Time { double value=0; Time()=default; explicit Time(double x):value(x){} static Time now(){return Time(host_time_seconds);} double toSec()const{return value;} Time& operator+=(Duration d){value+=d.value;return *this;} };
inline Duration operator+(Duration a,Duration b){return Duration(a.value+b.value);}
inline Duration operator-(Time a,Time b){return Duration(a.value-b.value);} inline Time operator+(Time t,Duration d){return Time(t.value+d.value);} inline Time operator-(Time t,Duration d){return Time(t.value-d.value);}
struct WallTime { double value=0; static WallTime now(){return {std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count()};} };
inline Duration operator-(WallTime a,WallTime b){return Duration(a.value-b.value);}
struct TimerEvent{};
class NodeHandle {
 public:
 std::map<std::string,std::any> values;
 template<class T> void setParam(const std::string& name,T value){values[name]=value;}
 template<class T> bool getParam(const std::string& name,T& value)const {auto i=values.find(name);if(i==values.end())return false;value=std::any_cast<T>(i->second);return true;}
 template<class T> void param(const std::string& name,T& value,const T& fallback)const {if(!getParam(name,value))value=fallback;}
};
struct Publisher {std::function<void(const std::any&)> sink; template<class T> void publish(const T& v)const{if(sink)sink(v);}};
}
namespace std_msgs {struct Header {ros::Time stamp;std::string frame_id;};struct Bool{bool data=false;};using BoolConstPtr=std::shared_ptr<const Bool>;}
namespace geometry_msgs {struct Point{double x=0,y=0,z=0;};using Vector3=Point;struct Quaternion{double x=0,y=0,z=0,w=1;};struct Pose{Point position;Quaternion orientation;};struct PoseStamped{std_msgs::Header header;Pose pose;};using PoseStampedConstPtr=std::shared_ptr<const PoseStamped>;struct Twist{Vector3 linear,angular;};}
namespace nav_msgs {struct Path{std_msgs::Header header;std::vector<geometry_msgs::PoseStamped> poses;};using PathPtr=std::shared_ptr<Path>;using PathConstPtr=std::shared_ptr<const Path>;struct Odometry{std_msgs::Header header;struct{geometry_msgs::Pose pose;}pose;struct{geometry_msgs::Twist twist;}twist;};using OdometryConstPtr=std::shared_ptr<const Odometry>;}
namespace scan_planner {
struct Bspline{int order=3,traj_id=0;ros::Time start_time;std::vector<geometry_msgs::Point> pos_pts;std::vector<double> knots;};using BsplineConstPtr=std::shared_ptr<const Bspline>;
struct DataDisp{std_msgs::Header header;};
class PlanningVisualization {public:using Ptr=std::shared_ptr<PlanningVisualization>;explicit PlanningVisualization(const ros::NodeHandle&){};
#define AUTHOR_DIAGNOSTIC_SINK(name) template<class... T>void name(const T&...){}
AUTHOR_DIAGNOSTIC_SINK(displayGoalPoint) AUTHOR_DIAGNOSTIC_SINK(displayGlobalPathList) AUTHOR_DIAGNOSTIC_SINK(displayInitPathList) AUTHOR_DIAGNOSTIC_SINK(displayAStarList) AUTHOR_DIAGNOSTIC_SINK(displayOptimalTraj) AUTHOR_DIAGNOSTIC_SINK(displayOptimalList)
#undef AUTHOR_DIAGNOSTIC_SINK
};
}
#define ROS_WARN(...) scanNativeDiagnostic(__VA_ARGS__)
#define ROS_ERROR(...) scanNativeDiagnostic(__VA_ARGS__)
#define ROS_INFO(...) ((void)0)
#define ROS_WARN_THROTTLE(period,...) scanNativeDiagnostic(__VA_ARGS__)
#define ROS_ERROR_STREAM(value) do{std::ostringstream s;s<<value;scan_native_diagnostic=s.str();}while(0)

#define ROS_WARN_STREAM(value) ROS_ERROR_STREAM(value)
