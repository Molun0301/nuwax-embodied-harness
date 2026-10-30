// PCT上游搜索的无ROS数据适配；只由有效地面层构建可搜索节点。
#include "a_star/a_star_search.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

extern "C" {
struct PctConfig {
  double resolution, point_resolution, robot_height, body_to_ground;
  double max_step, max_step_down, max_slope, footprint_radius, body_z_min, body_z_max, search_margin;
  const double* footprint; size_t footprint_len;
  const double* boundary; size_t boundary_len;
};
struct PctCell { int32_t x, y; double ground, ceiling; };
struct PctBand { int32_t x, y; double low, high; };
struct PctResult { double* points; size_t length; double elapsed_ms; size_t expanded; int32_t kind; };
struct PctCost { double cost, endpoint[3], yaw, elapsed_ms; size_t expanded; };
struct PctSearchStats {
  size_t checks, hits, source_searches, rejected;
  size_t source_expanded, exact_queries, exact_searches, exact_expanded;
  double source_elapsed_ms, exact_elapsed_ms;
};
using CancelFn = bool (*)(const void*);
}

namespace {
using XY = std::pair<int,int>;
using Point = std::array<double, 2>;
using Edge = std::array<double, 6>;
struct EdgeHash {
  size_t operator()(const Edge& edge) const {
    size_t hash=0;
    for(double value:edge)hash^=std::hash<double>{}(value)+0x9e3779b9+(hash<<6)+(hash>>2);
    return hash;
  }
};
struct SourceLabel {
  double ground, cost, weighted=INFINITY, length=INFINITY;
};
// 同一冻结几何内的纯转移判定。键包含两端真实坐标，保留格内偏移、方向与层高。
struct SharedTransitions {
  std::unordered_map<Edge,bool,EdgeHash> edges;
  PctSearchStats stats{};
  std::map<XY,std::vector<double>> nodes,reachable;
  std::map<XY,std::vector<SourceLabel>> labels;
  XY source_xy{};
  Edge source{};
  std::array<double,3> requested_source{};
  bool complete=false;
};
struct Column { std::vector<PctCell> surfaces; std::vector<PctBand> occupied; };
bool inside(const std::vector<Point>& polygon, double x, double y) {
  if (polygon.empty()) return true;
  bool hit = false;
  for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
    const auto a = polygon[j], b = polygon[i];
    const double cross = (x-a[0])*(b[1]-a[1])-(y-a[1])*(b[0]-a[0]);
    if (std::abs(cross)<1e-9 && x>=std::min(a[0],b[0])-1e-9 && x<=std::max(a[0],b[0])+1e-9 &&
        y>=std::min(a[1],b[1])-1e-9 && y<=std::max(a[1],b[1])+1e-9) return true;
    if (((a[1]>y)!=(b[1]>y)) && x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]) hit=!hit;
  }
  return hit;
}
// 分离轴检测足迹多边形与占据格方形是否相交，格内偏移不丢失。
bool intersects(const std::vector<Point>& polygon, double x0, double y0, double q) {
  const Point box[] = {{x0,y0},{x0+q,y0},{x0+q,y0+q},{x0,y0+q}};
  auto separated = [&](double ax, double ay) {
    double pmin=INFINITY,pmax=-INFINITY,bmin=INFINITY,bmax=-INFINITY;
    for (auto p: polygon) { double d=p[0]*ax+p[1]*ay; pmin=std::min(pmin,d);pmax=std::max(pmax,d); }
    for (auto p: box) { double d=p[0]*ax+p[1]*ay;bmin=std::min(bmin,d);bmax=std::max(bmax,d); }
    return pmax<=bmin+1e-9 || bmax<=pmin+1e-9;
  };
  if (separated(1,0)||separated(0,1)) return false;
  for (size_t i=0;i<polygon.size();++i) {
    const auto a=polygon[i],b=polygon[(i+1)%polygon.size()];
    if (separated(a[1]-b[1],b[0]-a[0])) return false;
  }
  return true;
}
// 保留地面边缘的连续软势场，影响距离由真实包络和期望余量共同确定。
void add_clearance_cost(Eigen::MatrixXd& cost,const std::vector<unsigned char>& measured,
                        int layers,int nx,int ny,double resolution,double influence) {
  const int width=nx+2,height=ny+2;
  std::vector<double> distances(static_cast<size_t>(width)*height);
  std::vector<int> vertices(width);
  std::vector<double> boundaries(width+1);
  for(int layer=0;layer<layers;++layer) {
    auto valid=[&](int x,int y) {
      return x>0&&x<=nx&&y>0&&y<=ny&&
          measured[(static_cast<size_t>(layer)*ny+y-1)*nx+x-1]&&
          cost(layer*ny+y-1,x-1)<=20.0;
    };
    for(int x=0;x<width;++x) {
      int distance=0;
      for(int y=0;y<height;++y) {
        distance=valid(x,y)?distance+1:0;
        distances[static_cast<size_t>(y)*width+x]=distance;
      }
      distance=0;
      for(int y=height-1;y>=0;--y) {
        distance=valid(x,y)?distance+1:0;
        auto& value=distances[static_cast<size_t>(y)*width+x];
        value=std::min(value,static_cast<double>(distance));value*=value;
      }
    }
    for(int y=1;y<=ny;++y) {
      const double* row=distances.data()+static_cast<size_t>(y)*width;
      int envelope=0;vertices[0]=0;boundaries[0]=-INFINITY;boundaries[1]=INFINITY;
      for(int x=1;x<width;++x) {
        auto crossing=[&] {
          const int old=vertices[envelope];
          return ((row[x]+double(x)*x)-(row[old]+double(old)*old))/(2.0*(x-old));
        };
        double intersection=crossing();
        while(intersection<=boundaries[envelope]){--envelope;intersection=crossing();}
        ++envelope;vertices[envelope]=x;boundaries[envelope]=intersection;
        boundaries[envelope+1]=INFINITY;
      }
      envelope=0;
      for(int x=1;x<=nx;++x) {
        while(boundaries[envelope+1]<x)++envelope;
        if(!valid(x,y))continue;
        const double dx=x-vertices[envelope];
        const double clearance=std::sqrt(dx*dx+row[vertices[envelope]])*resolution;
        const double soft_cost=20.0*std::clamp(1.0-clearance/influence,0.0,1.0);
        // 上游将软代价封顶于原可通行阈值，窄通道保持连通。
        auto& value=cost(layer*ny+y-1,x-1);
        value=std::max(value,std::min(soft_cost,20.0));
      }
    }
  }
}
struct Planner {
  PctConfig cfg;
  std::vector<Point> footprint,boundary;
  double clearance_radius=0.;
  std::map<XY,Column> columns;
  // 粗桶只筛候选，桶内仍保留每个原分辨率占据区间和原足迹判定。
  std::map<XY,std::vector<PctBand>> collision_buckets;
  int bucket_cells=1;
  Astar astar;
  SharedTransitions* shared=nullptr;
  explicit Planner(const PctConfig& c):cfg(c) {
    for(size_t i=0;i<c.footprint_len;++i) footprint.push_back({c.footprint[2*i],c.footprint[2*i+1]});
    for(size_t i=0;i<c.boundary_len;++i) boundary.push_back({c.boundary[2*i],c.boundary[2*i+1]});
    if (footprint.empty()) {
      for (int i=0;i<16;++i) {
        const double a=i*2*3.14159265358979323846/16;
        footprint.push_back({c.footprint_radius*std::cos(a),c.footprint_radius*std::sin(a)});
      }
    }
    for(auto p:footprint)clearance_radius=std::max(clearance_radius,std::hypot(p[0],p[1]));
  }
  int bucket(int cell) const {
    const int whole=cell/bucket_cells;
    return whole-(cell<0&&cell%bucket_cells!=0);
  }
  void build_collision_index() {
    bucket_cells=std::max(1,static_cast<int>(std::ceil(0.25/cfg.resolution)));
    for(const auto& column:columns)for(auto band:column.second.occupied)
      collision_buckets[{bucket(band.x),bucket(band.y)}].push_back(band);
    for(auto& block:collision_buckets)std::sort(block.second.begin(),block.second.end(),
        [](const PctBand& a,const PctBand& b){return a.high<b.high;});
  }
  bool collision(double x, double y, double ground, double heading) const {
    if (!inside(boundary,x,y)) return true;
    const double co=std::cos(heading),si=std::sin(heading),q=cfg.resolution;
    std::vector<Point> shape;shape.reserve(footprint.size());
    double minx=INFINITY,miny=INFINITY,maxx=-INFINITY,maxy=-INFINITY;
    for (auto p:footprint) {
      Point v{x+co*p[0]-si*p[1],y+si*p[0]+co*p[1]};
      if(!inside(boundary,v[0],v[1])) return true;
      minx=std::min(minx,v[0]);maxx=std::max(maxx,v[0]);
      miny=std::min(miny,v[1]);maxy=std::max(maxy,v[1]);shape.push_back(v);
    }
    // 踏面以下为支撑/可跨台阶；机身垂向界来自同一真实外形。
    const double bottom=std::max(ground+cfg.max_step,ground+cfg.body_to_ground+cfg.body_z_min);
    const double top=ground+cfg.body_to_ground+cfg.body_z_max;
    const int ix0=static_cast<int>(std::floor(minx/q)),ix1=static_cast<int>(std::floor(maxx/q));
    const int iy0=static_cast<int>(std::floor(miny/q)),iy1=static_cast<int>(std::floor(maxy/q));
    for(int bx=bucket(ix0);bx<=bucket(ix1);++bx)for(int by=bucket(iy0);by<=bucket(iy1);++by) {
      auto block=collision_buckets.find({bx,by});if(block==collision_buckets.end())continue;
      auto candidate=std::upper_bound(block->second.begin(),block->second.end(),bottom+1e-8,
          [](double low,const PctBand& band){return low<band.high;});
      for(;candidate!=block->second.end();++candidate) {
        const auto& b=*candidate;
        if(b.x<ix0||b.x>ix1||b.y<iy0||b.y>iy1||b.low>=top-1e-8)continue;
        bool support=false;
        if(b.high-b.low<=cfg.point_resolution*1.5) {
          const auto& col=columns.at({b.x,b.y});
          for(auto surface:col.surfaces) {
            if(surface.ground>=b.low-1e-8&&surface.ground<=b.high+1e-8&&
               std::abs(surface.ground-ground)<=cfg.point_resolution*1.5) {support=true;break;}
          }
        }
        // 真实薄地面带允许足端接触，较高实体和其它层仍检查原细列。
        if(!support&&((maxx==minx&&maxy==miny)||intersects(shape,b.x*q,b.y*q,q)))return true;
      }
    }
    return false;
  }
  // 全姿态外包圆生成连续软代价，避免无朝向状态的二维搜索靠瞬时转向降低代价。
  double clearance_cost(const Edge& edge) const {
    const double q=cfg.resolution,margin=std::max(0.05,q);
    const double radius=clearance_radius,range=2.*margin,reach=radius+range;
    double clearance=range;
    for(int i=0;i<=2;++i) {
      const double t=i*.5,ground=edge[2]+t*(edge[5]-edge[2]);
      const double x=edge[0]+t*(edge[3]-edge[0]),y=edge[1]+t*(edge[4]-edge[1]);
      const double bottom=std::max(ground+cfg.max_step,ground+cfg.body_to_ground+cfg.body_z_min);
      const double top=ground+cfg.body_to_ground+cfg.body_z_max;
      for(int bx=bucket(int(std::floor((x-reach)/q)));bx<=bucket(int(std::floor((x+reach)/q)));++bx)
        for(int by=bucket(int(std::floor((y-reach)/q)));by<=bucket(int(std::floor((y+reach)/q)));++by) {
          auto block=collision_buckets.find({bx,by});if(block==collision_buckets.end())continue;
          auto band=std::upper_bound(block->second.begin(),block->second.end(),bottom+1e-8,
              [](double low,const PctBand& b){return low<b.high;});
          for(;band!=block->second.end();++band) {
            if(band->low>=top-1e-8)continue;
            bool support=false;
            if(band->high-band->low<=cfg.point_resolution*1.5) {
              for(auto surface:columns.at({band->x,band->y}).surfaces)
                if(surface.ground>=band->low-1e-8&&surface.ground<=band->high+1e-8&&
                    std::abs(surface.ground-ground)<=cfg.point_resolution*1.5){support=true;break;}
            }
            if(support)continue;
            const double dx=std::max(0.,std::abs((band->x+.5)*q-x)-q/2.);
            const double dy=std::max(0.,std::abs((band->y+.5)*q-y)-q/2.);
            clearance=std::min(clearance,std::hypot(dx,dy)-radius);
          }
        }
    }
    // 包络在窄门处可有负净距，软代价仍连续；原真实多边形硬判定保留通行能力。
    return 20.*(1.-clearance/range)*std::hypot(edge[3]-edge[0],edge[4]-edge[1])/q;
  }
  bool transition(const Edge& edge,XY from,XY to) {
    const double ax=edge[0],ay=edge[1],bx=edge[3],by=edge[4];
    if(shared) {
      auto found=shared->edges.find(edge);
      if(found!=shared->edges.end()){++shared->stats.hits;return found->second;}
    }
    auto check=[&] {
    const double h=std::hypot(bx-ax,by-ay),dh=std::abs(edge[5]-edge[2]);
    const double step=edge[5]>=edge[2]?cfg.max_step:cfg.max_step_down;
    if(dh>std::max(step,std::tan(cfg.max_slope)*h)+1e-8)return false;
    if(dh>std::tan(cfg.max_slope)*h+1e-8) {
      // 连续实测斜面不能被逐格误解释成台阶；真正高度跳变两侧的踏面仍用阶高。
      const int ex=to.first-from.first,ey=to.second-from.second;
      const double signed_dh=edge[5]-edge[2],tolerance=cfg.point_resolution*0.5+1e-8;
      auto measured_height=[&](int x,int y,double expected) {
        auto col=columns.find({x,y});
        if(col==columns.end())return false;
        return std::any_of(col->second.surfaces.begin(),col->second.surfaces.end(),
            [&](const PctCell& cell){return std::abs(cell.ground-expected)<=tolerance;});
      };
      if(measured_height(from.first,from.second,edge[2])&&measured_height(to.first,to.second,edge[5])&&
         measured_height(from.first-ex,from.second-ey,edge[2]-signed_dh)&&
         measured_height(to.first+ex,to.second+ey,edge[5]+signed_dh))return false;
    }
    const double yaw=std::atan2(by-ay,bx-ax);
    // 两端与中点检查覆盖对角薄墙，保持真实足迹而非外包圆膨胀。
    for(int i=0;i<=2;++i){double t=i*0.5;
      if(collision(ax+(bx-ax)*t,ay+(by-ay)*t,edge[2]+(edge[5]-edge[2])*t,yaw))return false;}
    return true;
    };
    const bool allowed=check();
    if(shared){++shared->stats.checks;shared->edges.emplace(edge,allowed);}
    return allowed;
  }
  void add_points(const double* points,size_t count) {
    std::map<XY,std::vector<double>> heights;
    const double q=cfg.resolution,sq=cfg.point_resolution;
    for(size_t i=0;i<count;++i) heights[{static_cast<int>(std::floor(points[3*i]/q)),
        static_cast<int>(std::floor(points[3*i+1]/q))}].push_back(points[3*i+2]);
    for(auto& entry:heights) {
      auto& zs=entry.second;std::sort(zs.begin(),zs.end());
      auto& column=columns[entry.first];
      size_t first=0;
      for(size_t end=1;end<=zs.size();++end) {
        if(end<zs.size()&&zs[end]-zs[end-1]<=sq*1.5) continue;
        const double low=zs[first],high=zs[end-1];
        // 薄测量带取中位数；连续实体的暴露顶取最高采样带中位数，不取体素上界。
        size_t surface_first=first;
        if(high-low>sq*1.5) surface_first=std::lower_bound(zs.begin()+first,zs.begin()+end,high-sq*0.5)-zs.begin();
        const double ground=zs[surface_first+(end-surface_first)/2];
        column.surfaces.push_back({entry.first.first,entry.first.second,ground,INFINITY});
        column.occupied.push_back({entry.first.first,entry.first.second,low,high});
        first=end;
      }
      for(size_t i=0;i+1<column.surfaces.size();++i) column.surfaces[i].ceiling=column.occupied[i+1].low;
    }
  }
  bool plan_region(const double* start,const double* goal,double margin,PctResult& result,
                   CancelFn cancel,const void* user,bool progressive=false,bool allow_partial=false,bool connectivity=false) {
    astar.Reset();
    const double q=cfg.resolution,sg=start[2]-cfg.body_to_ground,gg=goal[2]-cfg.body_to_ground;
    int minx=static_cast<int>(std::floor((std::min(start[0],goal[0])-margin)/q));
    int maxx=static_cast<int>(std::floor((std::max(start[0],goal[0])+margin)/q));
    int miny=static_cast<int>(std::floor((std::min(start[1],goal[1])-margin)/q));
    int maxy=static_cast<int>(std::floor((std::max(start[1],goal[1])+margin)/q));
    if(!boundary.empty()) {
      double bx0=INFINITY,by0=INFINITY,bx1=-INFINITY,by1=-INFINITY;
      for(auto p:boundary){bx0=std::min(bx0,p[0]);bx1=std::max(bx1,p[0]);by0=std::min(by0,p[1]);by1=std::max(by1,p[1]);}
      minx=std::max(minx,static_cast<int>(std::floor(bx0/q)));maxx=std::min(maxx,static_cast<int>(std::floor(bx1/q)));
      miny=std::max(miny,static_cast<int>(std::floor(by0/q)));maxy=std::min(maxy,static_cast<int>(std::floor(by1/q)));
    }
    // 请求矩形外的空哨兵不可能成为路径；保留实际地面与原端点，避免按远处单点向四周分配整块空地图。
    int known_minx=maxx,known_maxx=minx,known_miny=maxy,known_maxy=miny;
    bool has_ground=false;
    for(const auto& entry:columns) {
      const auto key=entry.first;
      if(entry.second.surfaces.empty()||key.first<minx||key.first>maxx||key.second<miny||key.second>maxy)continue;
      has_ground=true;
      known_minx=std::min(known_minx,key.first);known_maxx=std::max(known_maxx,key.first);
      known_miny=std::min(known_miny,key.second);known_maxy=std::max(known_maxy,key.second);
    }
    if(!has_ground)return false;
    known_minx=std::min({known_minx,static_cast<int>(std::floor(start[0]/q)),static_cast<int>(std::floor(goal[0]/q))});
    known_maxx=std::max({known_maxx,static_cast<int>(std::floor(start[0]/q)),static_cast<int>(std::floor(goal[0]/q))});
    known_miny=std::min({known_miny,static_cast<int>(std::floor(start[1]/q)),static_cast<int>(std::floor(goal[1]/q))});
    known_maxy=std::max({known_maxy,static_cast<int>(std::floor(start[1]/q)),static_cast<int>(std::floor(goal[1]/q))});
    minx=std::max(minx,known_minx);maxx=std::min(maxx,known_maxx);
    miny=std::max(miny,known_miny);maxy=std::min(maxy,known_maxy);
    const int nx=maxx-minx+1,ny=maxy-miny+1;
    int sx=static_cast<int>(std::floor(start[0]/q))-minx,sy=static_cast<int>(std::floor(start[1]/q))-miny;
    int gx=static_cast<int>(std::floor(goal[0]/q))-minx,gy=static_cast<int>(std::floor(goal[1]/q))-miny;
    if(nx<=0||ny<=0||sx<0||sx>=nx||sy<0||sy>=ny||gx<0||gx>=nx||gy<0||gy>=ny)return false;
    double low=std::min(sg,gg),high=std::max(sg,gg);
    for(const auto& entry:columns) {
      const auto key=entry.first;if(key.first<minx||key.first>maxx||key.second<miny||key.second>maxy)continue;
      for(auto s:entry.second.surfaces){low=std::min(low,s.ground);high=std::max(high,s.ground);}
    }
    // 每层至少容纳一个可站立净空；真正的实测高度仍逐格保存，层号不是高度值。
    const double spacing=std::max({cfg.robot_height,q,std::max(cfg.max_step,cfg.max_step_down)*2+1e-6});
    low=sg+std::floor((low-sg)/spacing)*spacing;
    const int layers=std::max(1,static_cast<int>(std::ceil((high-low)/spacing))+1);
    // 原PCT没有有效地面高度的格子不参与地表连通；不再用起终点补造参考面。
    Eigen::MatrixXd heights=Eigen::MatrixXd::Constant(layers*ny,nx,-100.0);
    Eigen::MatrixXd cost=Eigen::MatrixXd::Constant(layers*ny,nx,100.0);
    Eigen::MatrixXd elevation=Eigen::MatrixXd::Zero(layers*ny,nx);
    std::vector<unsigned char> measured(static_cast<size_t>(layers)*nx*ny,0);
    auto key_index=[&](int l,int y,int x){return (static_cast<size_t>(l)*ny+y)*nx+x;};
    const double dx=goal[0]-start[0],dy=goal[1]-start[1];
    auto continuous_grade_sq=[&](XY key,double height) {
      double grade_sq=0;
      const double tolerance=cfg.point_resolution*0.5+1e-8;
      auto nearest=[&](XY neighbor)->const PctCell* {
        auto column=columns.find(neighbor);
        if(column==columns.end()||column->second.surfaces.empty())return nullptr;
        const auto& surfaces=column->second.surfaces;
        return &*std::min_element(surfaces.begin(),surfaces.end(),[&](const PctCell& a,const PctCell& b) {
          return std::abs(a.ground-height)<std::abs(b.ground-height);
        });
      };
      for(int axis=0;axis<2;++axis) {
        const int coordinate=axis==0?key.first:key.second;
        if(coordinate==std::numeric_limits<int>::min()||coordinate==std::numeric_limits<int>::max())continue;
        XY minus=key,plus=key;
        (axis==0?minus.first:minus.second)--;
        (axis==0?plus.first:plus.second)++;
        const auto* lower=nearest(minus);const auto* upper=nearest(plus);
        if(!lower||!upper)continue;
        const double before=height-lower->ground,after=upper->ground-height;
        // 双侧同向且高度增量相符才计连续坡面；阶边一侧为踏面时仍由原阶高判定。
        if(before*after>0&&std::abs(before-after)<=tolerance) {
          const double grade=std::min(std::abs(before),std::abs(after))/q;
          grade_sq+=grade*grade;
        }
      }
      return grade_sq;
    };
    for(const auto& entry:columns){
      if(cancel&&cancel(user))return false;
      const int x=entry.first.first-minx,y=entry.first.second-miny;
      if(x<0||x>=nx||y<0||y>=ny)continue;
      if(!inside(boundary,(entry.first.first+0.5)*q,(entry.first.second+0.5)*q))continue;
      for(auto s:entry.second.surfaces){
        const int l=std::clamp(static_cast<int>(std::llround((s.ground-low)/spacing)),0,layers-1);
        const auto index=key_index(l,y,x);
        if(measured[index]) {
          // 一个净空层只选择最接近层参考高度的实测面；相距一个可站立空间的地面进入不同层。
          if(std::abs(heights(l*ny+y,x)-(low+l*spacing))<=std::abs(s.ground-(low+l*spacing)))continue;
        }
        measured[index]=1;heights(l*ny+y,x)=s.ground;
        // 完整平面梯度与行进方向无关，不能通过斜向走格绕过连续坡度上限。
        const bool steep=continuous_grade_sq(entry.first,s.ground)>std::pow(std::tan(cfg.max_slope),2)+1e-8;
        cost(l*ny+y,x)=s.ceiling-s.ground+1e-8<cfg.robot_height||steep?100:0;
      }
    }
    // 地面边缘距离按格中心测量，额外一格补偿格内偏移；实体带按真实格方形计算。
    add_clearance_cost(cost,measured,layers,nx,ny,q,clearance_radius+std::max(.05,q)+q);
    // 原planner_node的snap_search_radius_cells=10；用三维距离绑定附近有效实测层。
    // 搜索半径只用于端点选择，不会在两个已知岛之间补地面。
    auto snap_endpoint=[&](int& x,int& y,double z,const double* requested) {
      int layer=-1;
      const int cx=x,cy=y;double best=INFINITY;
      for(int yy=std::max(0,cy-10);yy<=std::min(ny-1,cy+10);++yy)
        for(int xx=std::max(0,cx-10);xx<=std::min(nx-1,cx+10);++xx)
          for(int l=0;l<layers;++l) {
            if(!measured[key_index(l,yy,xx)]||cost(l*ny+yy,xx)>20)continue;
            const double wx=(xx+minx+0.5)*q,wy=(yy+miny+0.5)*q;
            const double score=std::hypot(std::hypot(wx-requested[0],wy-requested[1]),heights(l*ny+yy,xx)-z);
            if(score<best){best=score;x=xx;y=yy;layer=l;}
          }
      return layer;
    };
    const int original_sx=sx,original_sy=sy,original_gx=gx,original_gy=gy;
    const int sl=snap_endpoint(sx,sy,sg,start);
    int gl=snap_endpoint(gx,gy,gg,goal);
    if(sl<0 || (gl<0 && !progressive))return false;
    // 完整路线必须绑定原目标所在地面格，端点搜索半径不能把远处缺测目标当成已到达。
    if(progressive && (gl<0 || goal[0]<(gx+minx)*q || goal[0]>(gx+minx+1)*q ||
        goal[1]<(gy+miny)*q || goal[1]>(gy+miny+1)*q)) {
      gx=original_gx;gy=original_gy;gl=-1;
    }
    const double start_x=sx==original_sx&&sy==original_sy?start[0]:(sx+minx+0.5)*q;
    const double start_y=sx==original_sx&&sy==original_sy?start[1]:(sy+miny+0.5)*q;
    const double goal_x=gx==original_gx&&gy==original_gy?goal[0]:(gx+minx+0.5)*q;
    const double goal_y=gx==original_gx&&gy==original_gy?goal[1]:(gy+miny+0.5)*q;

    if(!connectivity && sx==gx&&sy==gy&&sl==gl) {
      const double yaw=std::atan2(dy,dx);
      if(collision(start_x,start_y,heights(sl*ny+sy,sx),yaw)||
         collision(goal_x,goal_y,heights(gl*ny+gy,gx),yaw))return false;
    }
    if(shared && !progressive) {
      const Edge source{start_x,start_y,heights(sl*ny+sy,sx),0,0,0};
      if(connectivity) {
        shared->source=source;
        shared->source_xy={sx+minx,sy+miny};
        for(int l=0;l<layers;++l)for(int y=0;y<ny;++y)for(int x=0;x<nx;++x)
          if(measured[key_index(l,y,x)] && cost(l*ny+y,x)<=20) {
            shared->nodes[{x+minx,y+miny}].push_back(heights(l*ny+y,x));
            shared->labels[{x+minx,y+miny}].push_back({heights(l*ny+y,x),cost(l*ny+y,x)});
          }
        gl=-1;
      } else if(shared->complete && shared->source==source && !(sx==gx&&sy==gy&&sl==gl)) {
        // 全图必须覆盖本窗口的每个真实节点；层分组不同则沿原查询，不能错误排除。
        bool compatible=true;
        for(int l=0;l<layers && compatible;++l)for(int y=0;y<ny && compatible;++y)for(int x=0;x<nx;++x) {
          if(!measured[key_index(l,y,x)] || cost(l*ny+y,x)>20)continue;
          auto found=shared->nodes.find({x+minx,y+miny});
          if(found==shared->nodes.end() || std::find(found->second.begin(),found->second.end(),heights(l*ny+y,x))==found->second.end()) {compatible=false;break;}
        }
        bool possible=!compatible;
        const double limit=std::max({cfg.max_step,cfg.max_step_down,std::tan(cfg.max_slope)*q*std::sqrt(2.0)})+1e-8;
        for(int y=gy-1;y<=gy+1 && !possible;++y)for(int x=gx-1;x<=gx+1 && !possible;++x) {
          if(x==gx && y==gy)continue;
          const XY xy{x+minx,y+miny};
          auto found=shared->reachable.find(xy);if(found==shared->reachable.end())continue;
          for(double ground:found->second) {
            if(std::abs(ground-heights(gl*ny+gy,gx))>limit)continue;
            const bool first=x==sx && y==sy && ground==source[2];
            if(transition({first?start_x:(xy.first+0.5)*q,first?start_y:(xy.second+0.5)*q,ground,
                           goal_x,goal_y,heights(gl*ny+gy,gx)},xy,{gx+minx,gy+miny})) {possible=true;break;}
          }
        }
        if(!possible){++shared->stats.rejected;return false;}
      }
    }
    astar.Init(20,layers,q,1,cost,heights,elevation);
    astar.SetHeightLimit(std::max({cfg.max_step,cfg.max_step_down,std::tan(cfg.max_slope)*q*std::sqrt(2.0)})+1e-8);
    astar.SetCancellation([&]{return cancel&&cancel(user);});
    astar.SetApproachScore(progressive ? std::function<double(const Node*)>([&](const Node* node) -> double {
      const double x=(node->idx[2]+minx+0.5)*q,y=(node->idx[1]+miny+0.5)*q;
      // 非起点节点已通过入边的完整机体检查；起点也必须确实容纳机体。
      if(node->parent==nullptr && collision(start_x,start_y,node->height,std::atan2(dy,dx)))return INFINITY;
      return std::hypot(std::hypot(x-goal[0],y-goal[1]),node->height+cfg.body_to_ground-goal[2]);
    }) : std::function<double(const Node*)>());
    astar.SetTransition([&](const Node* a,const Node* b){
      const bool first=a->layer==sl&&a->idx[2]==sx&&a->idx[1]==sy;
      const bool last=b->layer==gl&&b->idx[2]==gx&&b->idx[1]==gy;
      const double ax=first?start_x:(a->idx[2]+minx+0.5)*q,ay=first?start_y:(a->idx[1]+miny+0.5)*q;
      const double bx=last?goal_x:(b->idx[2]+minx+0.5)*q,by=last?goal_y:(b->idx[1]+miny+0.5)*q;
      return transition({ax,ay,a->height,bx,by,b->height},
                        {a->idx[2]+minx,a->idx[1]+miny},{b->idx[2]+minx,b->idx[1]+miny});
    });
    astar.SetTraversalCost([&](const Node* a,const Node* b){
      const bool first=a->layer==sl&&a->idx[2]==sx&&a->idx[1]==sy;
      const bool last=b->layer==gl&&b->idx[2]==gx&&b->idx[1]==gy;
      return clearance_cost({first?start_x:(a->idx[2]+minx+.5)*q,
          first?start_y:(a->idx[1]+miny+.5)*q,a->height,
          last?goal_x:(b->idx[2]+minx+.5)*q,last?goal_y:(b->idx[1]+miny+.5)*q,b->height});
    });
    if(connectivity) {
      ++shared->stats.source_searches;
      astar.Search({sl,sx,sy},{-1,0,0},[&](const Node* node) {
        const XY xy{node->idx[2]+minx,node->idx[1]+miny};
        shared->reachable[xy].push_back(node->height);
        double length=0;
        if(const auto* parent=node->parent) {
          const XY pxy{parent->idx[2]+minx,parent->idx[1]+miny};
          const auto& previous=shared->labels.at(pxy);
          const auto& label=*std::find_if(previous.begin(),previous.end(),[&](const SourceLabel& v){return v.ground==parent->height;});
          const double ax=parent->parent?(pxy.first+0.5)*q:start_x;
          const double ay=parent->parent?(pxy.second+0.5)*q:start_y;
          length=label.length+std::hypot(std::hypot((xy.first+0.5)*q-ax,(xy.second+0.5)*q-ay),node->height-parent->height);
        }
        auto& labels=shared->labels.at(xy);
        auto& label=*std::find_if(labels.begin(),labels.end(),[&](const SourceLabel& v){return v.ground==node->height;});
        label.weighted=node->g;label.length=length;
      });
      shared->complete=!astar.Cancelled() && !(cancel&&cancel(user));
      shared->stats.source_expanded+=astar.Expanded();
      return false;
    }
    const bool found=astar.Search({sl,sx,sy},{gl,gx,gy});result.expanded+=astar.Expanded();
    if(shared){++shared->stats.exact_searches;shared->stats.exact_expanded+=astar.Expanded();}
    if(!found && (!allow_partial || astar.Cancelled()))return false;
    const auto path=astar.GetPathPoints();
    if(path.empty())return false;
    result.kind=found?0:1;
    result.length=path.size();result.points=new double[3*path.size()];
    for(size_t i=0;i<path.size();++i){
      result.points[3*i]=(path[i].x+minx+0.5)*q;
      result.points[3*i+1]=(path[i].y+miny+0.5)*q;
      result.points[3*i+2]=path[i].height+cfg.body_to_ground;
    }
    // 已有端点格保留调用方XY；缺测端点使用原规则选择的实测格心。
    result.points[0]=start_x;result.points[1]=start_y;
    if(found){result.points[3*(path.size()-1)]=goal_x;result.points[3*(path.size()-1)+1]=goal_y;}
    return true;
  }
};
void error(char* buffer,size_t cap,const char* message){if(cap){std::strncpy(buffer,message,cap-1);buffer[cap-1]=0;}}
}
extern "C" void* pct_create(const PctConfig* config,const PctCell* cells,size_t ncells,
                             const PctBand* bands,size_t nbands,const double* points,size_t npoints,
                             char* err,size_t cap){
  try {
    auto p=std::make_unique<Planner>(*config);
    for(size_t i=0;i<ncells;++i)p->columns[{cells[i].x,cells[i].y}].surfaces.push_back(cells[i]);
    for(size_t i=0;i<nbands;++i)p->columns[{bands[i].x,bands[i].y}].occupied.push_back(bands[i]);
    if(npoints)p->add_points(points,npoints);
    p->build_collision_index();
    return p.release();
  }catch(const std::exception& e){error(err,cap,e.what());return nullptr;}
}
extern "C" void pct_destroy(void* planner){delete static_cast<Planner*>(planner);}
extern "C" void pct_result_free(PctResult* result){delete[] result->points;result->points=nullptr;}
extern "C" int pct_plan(void* planner,const double* start,const double* goal,PctResult* result,
                         CancelFn cancel,const void* user,bool progressive,char* err,size_t cap){
  const auto begin=std::chrono::steady_clock::now();
  try {
    auto& p=*static_cast<Planner*>(planner);
    double margin=std::max(p.cfg.search_margin,p.cfg.resolution*2),full=margin;
    // 先搜索请求附近，失败后按实际已知地图范围扩展；边界约束仍由同一内核检查。
    for(const auto& entry:p.columns){
      const double x=entry.first.first*p.cfg.resolution,y=entry.first.second*p.cfg.resolution;
      full=std::max({full,std::min(std::abs(x-start[0]),std::abs(x-goal[0]))+margin,
                         std::min(std::abs(y-start[1]),std::abs(y-goal[1]))+margin});
    }
    for(auto point:p.boundary) {
      full=std::max({full,std::min(std::abs(point[0]-start[0]),std::abs(point[0]-goal[0]))+margin,
                         std::min(std::abs(point[1]-start[1]),std::abs(point[1]-goal[1]))+margin});
    }
    for(;;){
      if(cancel&&cancel(user)){error(err,cap,"PCT规划已取消");return 2;}
      if(p.plan_region(start,goal,margin,*result,cancel,user,progressive,progressive && margin>=full)){
        result->elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return 0;
      }
      if(p.astar.Cancelled()||(cancel&&cancel(user))){error(err,cap,"PCT规划已取消");return 2;}
      if(margin>=full)break;
      margin=std::min(full,margin*2);
    }
    error(err,cap,"PCT未找到满足已知占据与机体尺寸的路径");return 1;
  }catch(const std::exception& e){error(err,cap,e.what());return 3;}
}
namespace {
struct MultiGoalSearch {
  Planner& planner;
  SharedTransitions transitions;
  explicit MultiGoalSearch(Planner& p):planner(p) {}
  bool prepare_source(const double* start,CancelFn cancel,const void* user) {
    const std::array<double,3> requested{start[0],start[1],start[2]};
    if(transitions.complete && transitions.requested_source==requested)return true;
    transitions.complete=false;transitions.nodes.clear();transitions.reachable.clear();transitions.labels.clear();
    transitions.requested_source=requested;
    double full=std::max(planner.cfg.search_margin,planner.cfg.resolution*2);
    for(const auto& entry:planner.columns)full=std::max({full,
        std::abs(entry.first.first*planner.cfg.resolution-start[0])+planner.cfg.resolution*2,
        std::abs(entry.first.second*planner.cfg.resolution-start[1])+planner.cfg.resolution*2});
    PctResult unused{};
    const auto began=std::chrono::steady_clock::now();
    planner.plan_region(start,start,full,unused,cancel,user,false,false,true);
    transitions.stats.source_elapsed_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    return transitions.complete;
  }
};
}
extern "C" void* pct_multi_goal_create(void* planner,char* err,size_t cap) {
  try{return new MultiGoalSearch(*static_cast<Planner*>(planner));}
  catch(const std::exception& e){error(err,cap,e.what());return nullptr;}
}
extern "C" void pct_multi_goal_destroy(void* search){delete static_cast<MultiGoalSearch*>(search);}
extern "C" PctSearchStats pct_multi_goal_stats(const void* search){
  return static_cast<const MultiGoalSearch*>(search)->transitions.stats;
}
extern "C" int pct_multi_goal_cost(void* search,const double* start,const double* goal,PctCost* cost,
                                    CancelFn cancel,const void* user,char* err,size_t cap) {
  try {
  auto& s=*static_cast<MultiGoalSearch*>(search);
  // 缓存仅在本次同步查询绑定；普通导航与逐步逼近没有共享搜索状态。
  struct Restore {Planner& p;~Restore(){p.shared=nullptr;}} restore{s.planner};
  s.planner.shared=&s.transitions;
  PctResult route{};
  s.prepare_source(start,cancel,user);
  if(cancel&&cancel(user)){error(err,cap,"PCT规划已取消");return 2;}
  ++s.transitions.stats.exact_queries;
  const auto began=std::chrono::steady_clock::now();
  const int status=pct_plan(&s.planner,start,goal,&route,cancel,user,false,err,cap);
  s.transitions.stats.exact_elapsed_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
  if(status==0) {
    cost->cost=0;cost->yaw=0;cost->elapsed_ms=route.elapsed_ms;cost->expanded=route.expanded;
    for(size_t i=1;i<route.length;++i) {
      const double* a=route.points+3*(i-1);const double* b=route.points+3*i;
      cost->cost+=std::hypot(std::hypot(b[0]-a[0],b[1]-a[1]),b[2]-a[2]);
      cost->yaw=std::atan2(b[1]-a[1],b[0]-a[0]);
    }
    std::copy(route.points+3*(route.length-1),route.points+3*route.length,cost->endpoint);
  }
  delete[] route.points;
  return status;
  }catch(const std::exception& e){error(err,cap,e.what());return 3;}
}
// 前沿排序沿同一加权单源父树计实际几何长度；每个目标只复核附近真实入边。
// 完整/逼近导航仍使用原目标专属A*，不调用本入口。
extern "C" int pct_multi_goal_rank_cost(void* search,const double* start,const double* goal,PctCost* out,
                                        CancelFn cancel,const void* user,char* err,size_t cap) {
  const auto began=std::chrono::steady_clock::now();
  try {
    auto& s=*static_cast<MultiGoalSearch*>(search);
    auto& p=s.planner;auto& t=s.transitions;
    struct Restore {Planner& p;~Restore(){p.shared=nullptr;}} restore{p};p.shared=&t;
    if(cancel&&cancel(user)){error(err,cap,"PCT规划已取消");return 2;}
    if(!s.prepare_source(start,cancel,user)) {
      error(err,cap,cancel&&cancel(user)?"PCT规划已取消":"PCT没有可搜索的实测起点");return 1;
    }
    const double q=p.cfg.resolution;
    const XY requested{static_cast<int>(std::floor(goal[0]/q)),static_cast<int>(std::floor(goal[1]/q))};
    XY target{};const SourceLabel* label=nullptr;double best=INFINITY;
    // 与原完整查询一致的十格、三维距离端点吸附；只从本轮全图实测节点选取。
    for(int y=requested.second-10;y<=requested.second+10;++y)for(int x=requested.first-10;x<=requested.first+10;++x) {
      auto found=t.labels.find({x,y});if(found==t.labels.end())continue;
      for(const auto& candidate:found->second) {
        const double d=std::hypot(std::hypot((x+0.5)*q-goal[0],(y+0.5)*q-goal[1]),candidate.ground+p.cfg.body_to_ground-goal[2]);
        if(d<best){best=d;target={x,y};label=&candidate;}
      }
    }
    auto unavailable=[&] {++t.stats.rejected;error(err,cap,"PCT未找到满足已知占据与机体尺寸的路径");return 1;};
    if(!label)return unavailable();
    const double gx=target==requested?goal[0]:(target.first+0.5)*q;
    const double gy=target==requested?goal[1]:(target.second+0.5)*q;
    if(target==t.source_xy && label->ground==t.source[2]) {
      const double yaw=std::atan2(gy-t.source[1],gx-t.source[0]);
      if(!p.transition({t.source[0],t.source[1],label->ground,gx,gy,label->ground},t.source_xy,target))return unavailable();
      out->cost=std::hypot(gx-t.source[0],gy-t.source[1]);out->yaw=yaw;
    } else {
      double weight=INFINITY;
      for(int y=target.second-1;y<=target.second+1;++y)for(int x=target.first-1;x<=target.first+1;++x) {
        if(x==target.first && y==target.second)continue;
        auto found=t.labels.find({x,y});if(found==t.labels.end())continue;
        for(const auto& previous:found->second) {
          if(!std::isfinite(previous.length))continue;
          const double height_limit=std::max({p.cfg.max_step,p.cfg.max_step_down,std::tan(p.cfg.max_slope)*q*std::sqrt(2.0)})+1e-8;
          if(std::abs(label->ground-previous.ground)>height_limit)continue;
          const bool first=XY{x,y}==t.source_xy && previous.ground==t.source[2];
          const double ax=first?t.source[0]:(x+0.5)*q,ay=first?t.source[1]:(y+0.5)*q;
          if(!p.transition({ax,ay,previous.ground,gx,gy,label->ground},{x,y},target))continue;
          const double dz=(label->ground-previous.ground)/q;
          const double soft=label->cost+p.clearance_cost({ax,ay,previous.ground,gx,gy,label->ground});
          const double g=previous.weighted+std::sqrt(dz*dz+(x-target.first)*(x-target.first)+(y-target.second)*(y-target.second))+soft;
          if(g<weight) {
            weight=g;out->cost=previous.length+std::hypot(std::hypot(gx-ax,gy-ay),label->ground-previous.ground);
            out->yaw=std::atan2(gy-ay,gx-ax);
          }
        }
      }
      if(!std::isfinite(weight))return unavailable();
    }
    out->endpoint[0]=gx;out->endpoint[1]=gy;out->endpoint[2]=label->ground+p.cfg.body_to_ground;
    out->expanded=0;out->elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    return 0;
  }catch(const std::exception& e){error(err,cap,e.what());return 3;}
}
extern "C" bool pct_single_source_cancellation_regression() {
  Astar a;Eigen::MatrixXd cost=Eigen::MatrixXd::Zero(10,10),height=cost,ele=cost;
  a.Init(20,1,0.1,1,cost,height,ele);
  size_t visited=0,callbacks=0;
  a.SetCancellation([&]{return ++callbacks==8;});
  a.Search({0,0,0},{-1,0,0},[&](const Node*){++visited;});
  if(!a.Cancelled() || visited!=7 || a.Expanded()!=7)return false;
  a.SetCancellation({});
  return a.Search({0,0,0},{0,9,9}) && !a.Cancelled();
}
// 直接复用同一上游对象验证失败残留清理，不通过重建对象掩盖错误。
extern "C" bool pct_retry_regression(){
  Astar a;Eigen::MatrixXd cost=Eigen::MatrixXd::Zero(3,5),height=cost,ele=cost;
  cost.col(2).setConstant(100);a.Init(35,1,0.1,1,cost,height,ele);
  if(a.Search({0,0,1},{0,4,1}))return false;
  return a.Search({0,0,1},{0,1,1})&&a.GetPathPoints().size()==2;
}
// 生产全局与局部适配读取同一份原始点分类，不各自构造不同的地面层。
extern "C" void pct_geometry_sizes(const void* planner,size_t* ncells,size_t* nbands){
  *ncells=0;*nbands=0;
  for(const auto& entry:static_cast<const Planner*>(planner)->columns){
    *ncells+=entry.second.surfaces.size();*nbands+=entry.second.occupied.size();
  }
}
extern "C" void pct_geometry_copy(const void* planner,PctCell* cells,PctBand* bands){
  for(const auto& entry:static_cast<const Planner*>(planner)->columns){
    for(auto s:entry.second.surfaces)*cells++=s;
    for(auto b:entry.second.occupied)*bands++=b;
  }
}
