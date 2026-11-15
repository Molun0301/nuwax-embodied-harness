// PCT上游搜索的无ROS数据适配；只由有效地面层构建可搜索节点。
#include "a_star/a_star_search.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <queue>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

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
  size_t checks, hits, source_searches;
  size_t source_expanded, exact_queries, exact_searches, exact_expanded;
  double source_elapsed_ms, exact_elapsed_ms;
};
using CancelFn = bool (*)(const void*);
}

namespace {
using XY = std::pair<int,int>;
struct XYHash {
  size_t operator()(const XY& xy) const {
    return std::hash<uint64_t>{}((uint64_t(uint32_t(xy.first))<<32)|uint32_t(xy.second));
  }
};
using Point = std::array<double, 2>;
using Edge = std::array<double, 6>;
struct EdgeHash {
  size_t operator()(const Edge& edge) const {
    size_t hash=0;
    for(double value:edge)hash^=std::hash<double>{}(value)+0x9e3779b9+(hash<<6)+(hash>>2);
    return hash;
  }
};
// 同一冻结几何内的纯转移判定。键包含两端真实坐标，保留格内偏移、方向与层高。
struct SharedTransitions {
  std::unordered_map<Edge,bool,EdgeHash> edges;
  PctSearchStats stats{};
};
struct Column {
  std::vector<PctCell> surfaces;
  std::vector<PctBand> occupied;
  size_t static_surfaces=0,static_bands=0;
};
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
  std::unordered_map<XY,Column,XYHash> columns;
  std::unordered_set<XY,XYHash> overlay_columns;
  std::unordered_set<XY,XYHash> dirty_buckets;
  // 请求矩阵保留容量，搜索只覆盖本轮所用区域。
  Eigen::MatrixXd heights,cost,elevation;
  std::vector<unsigned char> measured;
  // 粗桶只筛候选，桶内仍保留每个原分辨率占据区间和原足迹判定。
  std::unordered_map<XY,std::vector<PctBand>,XYHash> collision_buckets;
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
    for(auto& column:columns) {
      column.second.static_surfaces=column.second.surfaces.size();
      column.second.static_bands=column.second.occupied.size();
    }
    for(const auto& column:columns)for(auto band:column.second.occupied)
      collision_buckets[{bucket(band.x),bucket(band.y)}].push_back(band);
    for(auto& block:collision_buckets)std::sort(block.second.begin(),block.second.end(),
        [](const PctBand& a,const PctBand& b){return a.high<b.high;});
  }
  void dirty_column(XY key) { dirty_buckets.insert({bucket(key.first),bucket(key.second)}); }
  void refresh_buckets() {
    for(auto key:dirty_buckets) {
      auto& bands=collision_buckets[key];bands.clear();
      for(int x=key.first*bucket_cells;x<(key.first+1)*bucket_cells;++x)
        for(int y=key.second*bucket_cells;y<(key.second+1)*bucket_cells;++y) {
          auto column=columns.find({x,y});
          if(column!=columns.end())bands.insert(bands.end(),column->second.occupied.begin(),column->second.occupied.end());
        }
      std::sort(bands.begin(),bands.end(),[](const PctBand& a,const PctBand& b){return a.high<b.high;});
      if(bands.empty())collision_buckets.erase(key);
    }
    dirty_buckets.clear();
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
  // 完整平面梯度与行进方向无关：两侧最近实测面高度差同向且相符才计连续坡面。
  double grade_sq(XY key,double height) const {
    const double q=cfg.resolution;
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
                   CancelFn cancel,const void* user,bool progressive=false,bool allow_partial=false) {
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
    heights.resize(layers*ny,nx);heights.setConstant(-100.0);
    cost.resize(layers*ny,nx);cost.setConstant(100.0);
    elevation.resize(layers*ny,nx);elevation.setZero();
    measured.assign(static_cast<size_t>(layers)*nx*ny,0);
    auto key_index=[&](int l,int y,int x){return (static_cast<size_t>(l)*ny+y)*nx+x;};
    const double dx=goal[0]-start[0],dy=goal[1]-start[1];
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
        const bool steep=grade_sq(entry.first,s.ground)>std::pow(std::tan(cfg.max_slope),2)+1e-8;
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

    if(sx==gx&&sy==gy&&sl==gl) {
      const double yaw=std::atan2(dy,dx);
      if(collision(start_x,start_y,heights(sl*ny+sy,sx),yaw)||
         collision(goal_x,goal_y,heights(gl*ny+gy,gx),yaw))return false;
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
extern "C" void pct_replace_columns(void* planner,const int32_t* keys,size_t count,
    const PctCell* cells,size_t ncells,const PctBand* bands,size_t nbands) {
  auto& p=*static_cast<Planner*>(planner);
  for(size_t i=0;i<count;++i) {
    XY key{keys[2*i],keys[2*i+1]};auto& column=p.columns[key];
    column.surfaces.clear();column.occupied.clear();column.static_surfaces=0;column.static_bands=0;
    p.dirty_column(key);
  }
  for(size_t i=0;i<ncells;++i)p.columns[{cells[i].x,cells[i].y}].surfaces.push_back(cells[i]);
  for(size_t i=0;i<nbands;++i)p.columns[{bands[i].x,bands[i].y}].occupied.push_back(bands[i]);
  for(size_t i=0;i<count;++i) {
    XY key{keys[2*i],keys[2*i+1]};auto& column=p.columns.at(key);
    column.static_surfaces=column.surfaces.size();column.static_bands=column.occupied.size();
    if(column.surfaces.empty()&&column.occupied.empty())p.columns.erase(key);
  }
}
extern "C" void pct_replace_overlay(void* planner,const PctCell* cells,size_t ncells,
    const PctBand* bands,size_t nbands) {
  auto& p=*static_cast<Planner*>(planner);
  for(auto key:p.overlay_columns) {
    auto found=p.columns.find(key);
    if(found==p.columns.end())continue;
    auto& column=found->second;
    column.surfaces.resize(column.static_surfaces);column.occupied.resize(column.static_bands);
    if(column.surfaces.empty()&&column.occupied.empty())p.columns.erase(found);
    p.dirty_column(key);
  }
  p.overlay_columns.clear();
  for(size_t i=0;i<ncells;++i) {
    XY key{cells[i].x,cells[i].y};p.columns[key].surfaces.push_back(cells[i]);p.overlay_columns.insert(key);p.dirty_column(key);
  }
  for(size_t i=0;i<nbands;++i) {
    XY key{bands[i].x,bands[i].y};p.columns[key].occupied.push_back(bands[i]);p.overlay_columns.insert(key);p.dirty_column(key);
  }
  p.refresh_buckets();
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
// 从一个起点按需展开的单源搜索：只在展开到某格时由原生列计算该格的节点，不建全图矩阵，
// 展开量只取决于到达目标所需的范围。地面层按起点地面高度分组（层参考高度为起点地面加整数倍层距）；
// 节点代价（低净空、连续坡度、地面边缘软代价）、转移判定与入边代价的定义与目标窗口搜索相同。
struct SourceSearch {
  struct Layer { int k; double ground, ceiling; bool valid=false; double soft=-1; };
  struct Key {
    int x,y,k;
    bool operator==(const Key& other) const {return x==other.x&&y==other.y&&k==other.k;}
  };
  struct KeyHash {
    size_t operator()(const Key& key) const {
      return std::hash<uint64_t>{}((uint64_t(uint32_t(key.x))<<32)|uint32_t(key.y))^
          (std::hash<int>{}(key.k)*0x9e3779b97f4a7c15ull);
    }
  };
  // 已定值节点的加权代价、沿父链的几何长度，以及最后一条入边的起点（供目标端点计长度与朝向）。
  struct State {
    double g=INFINITY,length=0,ground=0;
    double from_x=0,from_y=0,from_ground=0,from_length=0;
    bool from_source=true,closed=false;
  };
  struct Entry { double g; uint64_t sequence; Key key; };
  struct Later {
    bool operator()(const Entry& a,const Entry& b) const {return a.g==b.g?a.sequence>b.sequence:a.g>b.g;}
  };
  struct Offset { int dx,dy; double distance; };

  Planner& p;
  double q,sg,spacing,influence,height_limit;
  std::vector<Offset> offsets;
  std::unordered_map<XY,std::vector<Layer>,XYHash> columns;
  std::unordered_map<Key,State,KeyHash> states;
  std::priority_queue<Entry,std::vector<Entry>,Later> open;
  uint64_t sequence=0;
  Key start{};
  double start_x=0,start_y=0;
  std::unordered_map<Key,std::vector<size_t>,KeyHash> waiting;
  std::vector<std::array<double,2>> targets;
  std::deque<std::pair<size_t,Key>> ready;
  size_t expanded=0;

  SourceSearch(Planner& planner,double source_ground):p(planner),q(planner.cfg.resolution),sg(source_ground) {
    spacing=std::max({p.cfg.robot_height,q,std::max(p.cfg.max_step,p.cfg.max_step_down)*2+1e-6});
    influence=p.clearance_radius+std::max(.05,q)+q;
    height_limit=std::max({p.cfg.max_step,p.cfg.max_step_down,std::tan(p.cfg.max_slope)*q*std::sqrt(2.0)})+1e-8;
    const int reach=static_cast<int>(std::ceil(influence/q));
    for(int dy=-reach;dy<=reach;++dy)for(int dx=-reach;dx<=reach;++dx) {
      const double distance=std::hypot(dx,dy);
      if((dx||dy) && distance*q<influence)offsets.push_back({dx,dy,distance});
    }
    std::stable_sort(offsets.begin(),offsets.end(),[](const Offset& a,const Offset& b){return a.distance<b.distance;});
  }
  // 一列的节点：每层取最接近层参考高度的实测面；边界外与缺测列没有节点。
  std::vector<Layer>& column(XY key) {
    auto found=columns.find(key);
    if(found!=columns.end())return found->second;
    std::vector<Layer> layers;
    auto source=p.columns.find(key);
    if(source!=p.columns.end() && inside(p.boundary,(key.first+0.5)*q,(key.second+0.5)*q)) {
      for(const auto& surface:source->second.surfaces) {
        const int k=static_cast<int>(std::llround((surface.ground-sg)/spacing));
        const double reference=sg+k*spacing;
        auto same=std::find_if(layers.begin(),layers.end(),[&](const Layer& layer){return layer.k==k;});
        if(same==layers.end())layers.push_back({k,surface.ground,surface.ceiling});
        else if(std::abs(same->ground-reference)>std::abs(surface.ground-reference)) {
          same->ground=surface.ground;same->ceiling=surface.ceiling;
        }
      }
      std::sort(layers.begin(),layers.end(),[](const Layer& a,const Layer& b){return a.k<b.k;});
      for(auto& layer:layers) {
        const bool steep=p.grade_sq(key,layer.ground)>std::pow(std::tan(p.cfg.max_slope),2)+1e-8;
        layer.valid=!(layer.ceiling-layer.ground+1e-8<p.cfg.robot_height||steep);
      }
    }
    return columns.emplace(key,std::move(layers)).first->second;
  }
  Layer* node(int x,int y,int k) {
    for(auto& layer:column({x,y}))if(layer.k==k)return &layer;
    return nullptr;
  }
  // 地面边缘软代价：到同层最近无效格（缺测、低净空或过陡）的格心距离，取值与全图距离变换相同。
  double soft(int x,int y,Layer& layer) {
    if(layer.soft>=0)return layer.soft;
    double distance=INFINITY;
    for(const auto& offset:offsets) {
      const Layer* other=node(x+offset.dx,y+offset.dy,layer.k);
      if(!other||!other->valid){distance=offset.distance;break;}
    }
    layer.soft=std::isfinite(distance)?20.0*std::clamp(1.0-distance*q/influence,0.0,1.0):0.0;
    return layer.soft;
  }
  double x_of(const Key& key) const {return key==start?start_x:(key.x+0.5)*q;}
  double y_of(const Key& key) const {return key==start?start_y:(key.y+0.5)*q;}
  // 起点按原完整查询的十格、三维距离吸附到附近有效节点，吸附到别的格时用格心；
  // 目标按平面位置所在格与地面高度所在层对应到节点，没有有效节点即不可达。
  bool begin(const double* requested,const double* goals,size_t count,unsigned char* reachable) {
    const XY origin{static_cast<int>(std::floor(requested[0]/q)),static_cast<int>(std::floor(requested[1]/q))};
    double best=INFINITY;bool found=false;
    for(int y=origin.second-10;y<=origin.second+10;++y)for(int x=origin.first-10;x<=origin.first+10;++x)
      for(const auto& layer:column({x,y})) {
        if(!layer.valid)continue;
        const double score=std::hypot(std::hypot((x+0.5)*q-requested[0],(y+0.5)*q-requested[1]),layer.ground-sg);
        if(score<best){best=score;start={x,y,layer.k};found=true;}
      }
    for(size_t i=0;i<count;++i)reachable[i]=0;
    if(!found)return false;
    const bool same=start.x==origin.first&&start.y==origin.second;
    start_x=same?requested[0]:(start.x+0.5)*q;start_y=same?requested[1]:(start.y+0.5)*q;
    auto& source=states[start];
    source.g=0;source.ground=node(start.x,start.y,start.k)->ground;
    open.push({0,sequence++,start});
    for(size_t i=0;i<count;++i) {
      const double* goal=goals+3*i;
      targets.push_back({goal[0],goal[1]});
      const Key key{static_cast<int>(std::floor(goal[0]/q)),static_cast<int>(std::floor(goal[1]/q)),
          static_cast<int>(std::llround((goal[2]-p.cfg.body_to_ground-sg)/spacing))};
      const Layer* layer=node(key.x,key.y,key.k);
      if(!layer||!layer->valid)continue;
      reachable[i]=1;
      waiting[key].push_back(i);
    }
    return true;
  }
  void relax(const Key& from,const State& state) {
    // 与上游A*相同的八邻域顺序（行、列偏移），每格先查同层再按层号查其它层。
    static const int neighbors[8][2]={{-1,-1},{-1,0},{-1,1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
    const double ax=x_of(from),ay=y_of(from);
    for(const auto& offset:neighbors) {
      const int x=from.x+offset[1],y=from.y+offset[0];
      auto& layers=column({x,y});
      auto visit=[&](Layer& layer) {
        if(!layer.valid||std::abs(layer.ground-state.ground)>height_limit)return;
        const double bx=(x+0.5)*q,by=(y+0.5)*q;
        if(!p.transition({ax,ay,state.ground,bx,by,layer.ground},{from.x,from.y},{x,y}))return;
        const double dz=(layer.ground-state.ground)/q;
        const double step=soft(x,y,layer)+p.clearance_cost({ax,ay,state.ground,bx,by,layer.ground});
        const double g=state.g+std::sqrt(dz*dz+offset[0]*offset[0]+offset[1]*offset[1])+step;
        const Key key{x,y,layer.k};
        auto& next=states[key];
        if(next.closed||g>=next.g)return;
        next.g=g;next.ground=layer.ground;
        next.length=state.length+std::hypot(std::hypot(bx-ax,by-ay),layer.ground-state.ground);
        next.from_x=ax;next.from_y=ay;next.from_ground=state.ground;next.from_length=state.length;
        next.from_source=false;
        open.push({g,sequence++,key});
      };
      for(auto& layer:layers)if(layer.k==from.k)visit(layer);
      for(auto& layer:layers)if(layer.k!=from.k)visit(layer);
    }
  }
  // 展开到下一个已登记目标的节点定值。返回0为得到一个目标，1为可达范围已展开完，2为取消。
  // 目标长度与末段朝向按最后一条入边起点到目标平面位置计；起点格内的目标从起点直接计。
  int next(size_t& index,double& weighted,PctCost& out,CancelFn cancel,const void* user) {
    while(ready.empty()) {
      if(open.empty())return 1;
      if(cancel&&cancel(user))return 2;
      const Entry entry=open.top();open.pop();
      auto& state=states.at(entry.key);
      if(state.closed||entry.g!=state.g)continue;
      state.closed=true;++expanded;
      if(auto found=waiting.find(entry.key);found!=waiting.end()) {
        for(size_t i:found->second)ready.push_back({i,entry.key});
        waiting.erase(found);
      }
      relax(entry.key,state);
    }
    const auto [i,key]=ready.front();ready.pop_front();
    const auto& state=states.at(key);
    const auto& target=targets[i];
    const double ax=state.from_source?start_x:state.from_x,ay=state.from_source?start_y:state.from_y;
    const double ground=state.from_source?state.ground:state.from_ground;
    out=PctCost{};
    out.cost=(state.from_source?0.0:state.from_length)+std::hypot(std::hypot(target[0]-ax,target[1]-ay),state.ground-ground);
    out.yaw=std::atan2(target[1]-ay,target[0]-ax);
    out.endpoint[0]=target[0];out.endpoint[1]=target[1];out.endpoint[2]=state.ground+p.cfg.body_to_ground;
    index=i;weighted=state.g;
    return 0;
  }
};
struct MultiGoalSearch {
  Planner& planner;
  SharedTransitions transitions;
  std::unique_ptr<SourceSearch> source;
  explicit MultiGoalSearch(Planner& p):planner(p) {}
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
// 从起点开始一次按需展开的单源搜索并登记目标；reachable[i]为0表示该目标没有对应的有效节点。
extern "C" int pct_source_begin(void* search,const double* start,const double* goals,size_t count,
                                unsigned char* reachable,char* err,size_t cap) {
  try {
    auto& s=*static_cast<MultiGoalSearch*>(search);
    struct Restore {Planner& p;~Restore(){p.shared=nullptr;}} restore{s.planner};s.planner.shared=&s.transitions;
    const auto began=std::chrono::steady_clock::now();
    ++s.transitions.stats.source_searches;
    s.source=std::make_unique<SourceSearch>(s.planner,start[2]-s.planner.cfg.body_to_ground);
    const bool ready=s.source->begin(start,goals,count,reachable);
    s.transitions.stats.source_elapsed_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    if(!ready){s.source.reset();error(err,cap,"PCT没有可搜索的实测起点");return 1;}
    return 0;
  }catch(const std::exception& e){error(err,cap,e.what());return 3;}
}
// 继续当前单源搜索到下一个已登记目标定值：0为得到目标，1为可达范围已展开完，2为取消。
extern "C" int pct_source_next(void* search,size_t* index,double* weighted,PctCost* out,
                               CancelFn cancel,const void* user,char* err,size_t cap) {
  try {
    auto& s=*static_cast<MultiGoalSearch*>(search);
    struct Restore {Planner& p;~Restore(){p.shared=nullptr;}} restore{s.planner};s.planner.shared=&s.transitions;
    const auto began=std::chrono::steady_clock::now();
    const size_t before=s.source->expanded;
    const int status=s.source->next(*index,*weighted,*out,cancel,user);
    s.transitions.stats.source_expanded+=s.source->expanded-before;
    s.transitions.stats.source_elapsed_ms+=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    if(status==2)error(err,cap,"PCT规划已取消");
    return status;
  }catch(const std::exception& e){error(err,cap,e.what());return 3;}
}
// 平面点所在原生格的实测地面高度个数，并复制至多cap个；缺测列为零个，不补造地面。
extern "C" size_t pct_multi_goal_grounds(const void* search,double x,double y,double* out,size_t cap) {
  const auto& p=static_cast<const MultiGoalSearch*>(search)->planner;
  const double q=p.cfg.resolution;
  const auto column=p.columns.find({static_cast<int>(std::floor(x/q)),static_cast<int>(std::floor(y/q))});
  if(column==p.columns.end())return 0;
  const auto& surfaces=column->second.surfaces;
  for(size_t i=0;i<surfaces.size()&&i<cap;++i)out[i]=surfaces[i].ground;
  return surfaces.size();
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
  const auto& columns=static_cast<const Planner*>(planner)->columns;
  std::vector<XY> keys;keys.reserve(columns.size());
  for(const auto& entry:columns)keys.push_back(entry.first);
  std::sort(keys.begin(),keys.end());
  for(auto key:keys){
    const auto& entry=*columns.find(key);
    for(auto s:entry.second.surfaces)*cells++=s;
    for(auto b:entry.second.occupied)*bands++=b;
  }
}
