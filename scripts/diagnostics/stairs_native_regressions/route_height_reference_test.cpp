#include <route_height_reference.h>
#include <cassert>
#include <iostream>
using scan_planner::RouteHeightReference;
void prepare(RouteHeightReference& h,std::vector<Eigen::Vector3d> p){h.set(p);h.times=h.arcs;h.timed_arcs=h.arcs;h.window(0,h.arcs.back());}
int main(){
 RouteHeightReference h;prepare(h,{{0,0,0},{2,0,0},{2,1,1},{0,1,1},{0,0,2},{2,0,2}});
 assert(h.heightAtArc(1)==0.);assert(h.heightAtArc(h.arcs[4]+1)==2.);double a=h.heightAtArc(h.arcs[4]+1);for(int i=0;i<20;++i)assert(h.heightAtArc(h.arcs[4]+1)==a);
 prepare(h,{{0,0,0},{1,0,1},{1,1,2}});double corner=h.arcs[1];
 for(double e:{.1,.001,.000001})assert(std::abs(h.heightAtArc(corner+e)-h.heightAtArc(corner-e))<2*e);
 prepare(h,{{0,0,0},{0,0,2}});assert(h.heightAtArc(1)==1.);
 prepare(h,{{0,0,0},{1,1,1},{0,1,2},{1,0,3},{0,0,4}});assert(h.heightAtArc(h.arcs[0]+.5*(h.arcs[1]-h.arcs[0]))==.5);assert(h.heightAtArc(h.arcs[2]+.5*(h.arcs[3]-h.arcs[2]))==2.5);
 prepare(h,{{0,0,0},{2,0,0},{2,1,1},{0,1,1}});assert(h.heightAtArc(.5)==0.);assert(h.heightAtArc(h.arcs.back()-.5)==1.);
 h.clear();assert(!h.ready());std::cout<<"same XY layers, figure eight, U turn, vertical segment, corner continuity, pure queries and retirement pass\n";
}
