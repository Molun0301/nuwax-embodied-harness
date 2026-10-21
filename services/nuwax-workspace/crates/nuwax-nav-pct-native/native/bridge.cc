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

extern "C" {
struct PctConfig {
  double resolution, point_resolution, robot_height, body_to_ground;
  double max_step, max_step_down, max_slope, footprint_radius, body_z_min, body_z_max, search_margin;
  const double* footprint; size_t footprint_len;
  const double* boundary; size_t boundary_len;
};
struct PctCell { int32_t x, y; double ground, ceiling; };
struct PctBand { int32_t x, y; double low, high; };
struct PctResult { double* points; size_t length; double elapsed_ms; size_t expanded; };
using CancelFn = bool (*)(const void*);
}

namespace {
using XY = std::pair<int,int>;
using Point = std::array<double, 2>;
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
// 复用上游pct_scan_bridge/clearance_cost.py的软代价公式；距离变换改为无Python依赖的等价两遍算法。
void add_clearance_cost(Eigen::MatrixXd& cost,const std::vector<unsigned char>& measured,
                        int layers,int nx,int ny,double resolution) {
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
        const double soft_cost=20.0*std::clamp(1.0-clearance/0.5,0.0,1.0);
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
  std::map<XY,Column> columns;
  // 粗桶只筛候选，桶内仍保留每个原分辨率占据区间和原足迹判定。
  std::map<XY,std::vector<PctBand>> collision_buckets;
  int bucket_cells=1;
  Astar astar;
  explicit Planner(const PctConfig& c):cfg(c) {
    for(size_t i=0;i<c.footprint_len;++i) footprint.push_back({c.footprint[2*i],c.footprint[2*i+1]});
    for(size_t i=0;i<c.boundary_len;++i) boundary.push_back({c.boundary[2*i],c.boundary[2*i+1]});
    if (footprint.empty()) {
      for (int i=0;i<16;++i) {
        const double a=i*2*3.14159265358979323846/16;
        footprint.push_back({c.footprint_radius*std::cos(a),c.footprint_radius*std::sin(a)});
      }
    }
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
                   CancelFn cancel,const void* user) {
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
        cost(l*ny+y,x)=s.ceiling-s.ground+1e-8<cfg.robot_height?100:0;
      }
    }
    add_clearance_cost(cost,measured,layers,nx,ny,q);
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
    const int sl=snap_endpoint(sx,sy,sg,start),gl=snap_endpoint(gx,gy,gg,goal);
    if(sl<0||gl<0)return false;
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
    astar.SetTransition([&](const Node* a,const Node* b){
      const bool first=a->layer==sl&&a->idx[2]==sx&&a->idx[1]==sy;
      const bool last=b->layer==gl&&b->idx[2]==gx&&b->idx[1]==gy;
      const double ax=first?start_x:(a->idx[2]+minx+0.5)*q,ay=first?start_y:(a->idx[1]+miny+0.5)*q;
      const double bx=last?goal_x:(b->idx[2]+minx+0.5)*q,by=last?goal_y:(b->idx[1]+miny+0.5)*q;
      const double h=std::hypot(bx-ax,by-ay),dh=std::abs(b->height-a->height);
      const double step=b->height>=a->height?cfg.max_step:cfg.max_step_down;
      if(dh>std::max(step,std::tan(cfg.max_slope)*h)+1e-8)return false;
      if(dh>std::tan(cfg.max_slope)*h+1e-8) {
        // 连续实测斜面不能被逐格误解释成台阶；真正高度跳变两侧的踏面仍用阶高。
        const int ex=b->idx[2]-a->idx[2],ey=b->idx[1]-a->idx[1];
        const double signed_dh=b->height-a->height,tolerance=cfg.point_resolution*0.5+1e-8;
        auto measured_height=[&](int x,int y,double expected) {
          auto col=columns.find({x+minx,y+miny});
          if(col==columns.end())return false;
          return std::any_of(col->second.surfaces.begin(),col->second.surfaces.end(),
              [&](const PctCell& cell){return std::abs(cell.ground-expected)<=tolerance;});
        };
        if(measured_height(a->idx[2],a->idx[1],a->height)&&measured_height(b->idx[2],b->idx[1],b->height)&&
           measured_height(a->idx[2]-ex,a->idx[1]-ey,a->height-signed_dh)&&
           measured_height(b->idx[2]+ex,b->idx[1]+ey,b->height+signed_dh))return false;
      }
      const double yaw=std::atan2(by-ay,bx-ax);
      // 两端与中点检查覆盖对角薄墙，保持真实足迹而非外包圆膨胀。
      for(int i=0;i<=2;++i){double t=i*0.5;
        if(collision(ax+(bx-ax)*t,ay+(by-ay)*t,a->height+(b->height-a->height)*t,yaw))return false;}
      return true;
    });
    const bool found=astar.Search({sl,sx,sy},{gl,gx,gy});result.expanded+=astar.Expanded();
    if(!found)return false;
    const auto path=astar.GetPathPoints();
    result.length=path.size();result.points=new double[3*path.size()];
    for(size_t i=0;i<path.size();++i){
      result.points[3*i]=(path[i].x+minx+0.5)*q;
      result.points[3*i+1]=(path[i].y+miny+0.5)*q;
      result.points[3*i+2]=path[i].height+cfg.body_to_ground;
    }
    // 已有端点格保留调用方XY；缺测端点使用原规则选择的实测格心。
    result.points[0]=start_x;result.points[1]=start_y;
    result.points[3*(path.size()-1)]=goal_x;result.points[3*(path.size()-1)+1]=goal_y;
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
                         CancelFn cancel,const void* user,char* err,size_t cap){
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
      if(p.plan_region(start,goal,margin,*result,cancel,user)){
        result->elapsed_ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();return 0;
      }
      if(p.astar.Cancelled()||(cancel&&cancel(user))){error(err,cap,"PCT规划已取消");return 2;}
      if(margin>=full)break;
      margin=std::min(full,margin*2);
    }
    error(err,cap,"PCT未找到满足已知占据与机体尺寸的路径");return 1;
  }catch(const std::exception& e){error(err,cap,e.what());return 3;}
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
