//! Point-level decisions shared by map consumers. Geometry is protection, not a
//! semantic person detector. References never use a whole voxel as protection.
use ahash::{AHashMap as HashMap,AHashSet as HashSet};
use nalgebra::{Matrix3,Point3,SymmetricEigen,Vector3};
use serde::{Deserialize,Serialize};
use std::collections::VecDeque;

pub const SURFACE_M:f64=0.03;
pub const REFERENCE_M:f64=0.15;
pub const GROW_M:f64=0.12;
#[derive(Clone,Copy,Debug,PartialEq,Eq,Serialize,Deserialize)]
#[serde(rename_all="snake_case")]
pub enum PointLabel {Ground,SurfacePlane,SurfaceLine,SurfacePoint,Seed,Grown,PendingNew,FinalStatic}
impl PointLabel {pub fn dynamic(self)->bool{matches!(self,Self::Seed|Self::Grown)}}
#[derive(Clone,Debug,Serialize,Deserialize)]
pub struct FrameLabels {pub source_timestamp_ns:u64,pub labels:Vec<PointLabel>}
#[derive(Clone,Debug)]
struct Reference {point:[f64;3],first:u64,last:u64,hits:u32,ground:bool}
#[derive(Clone,Default,Debug)]
pub struct StaticReferences {points:HashMap<[i32;3],Reference>}
fn key(p:[f64;3],q:f64)->[i32;3]{p.map(|v|(v/q).floor() as i32)}
fn d2(a:[f64;3],b:[f64;3])->f64{(0..3).map(|i|(a[i]-b[i]).powi(2)).sum()}
impl StaticReferences {
    pub fn remove_where(&mut self,mut clears:impl FnMut([f64;3])->bool){self.points.retain(|_,v|v.ground||!clears(v.point));}
    pub fn protect(&self,p:[f64;3],before:Option<u64>)->Option<PointLabel>{
        let mut nearby=Vec::new();
        // Fine-grained spatial index: 11^3 lookups, independent of map history.
        let k=key(p,SURFACE_M);let radius=(REFERENCE_M/SURFACE_M).ceil() as i32;
        for x in -radius..=radius{for y in -radius..=radius{for z in -radius..=radius{
            if let Some(v)=self.points.get(&[k[0]+x,k[1]+y,k[2]+z]){
                if (v.ground||(v.hits>=5&&v.last.saturating_sub(v.first)>=2_000_000_000))
                    && before.is_none_or(|t|v.first<t)&&d2(p,v.point)<=REFERENCE_M.powi(2){nearby.push(v.point);}
            }
        }}}
        protect_from_points(p,&nearby)
    }
    pub fn observe(&mut self,points:&[Point3<f64>],labels:&[PointLabel],stamp:u64){
        let mut seen=HashSet::new();
        for (p,label) in points.iter().zip(labels){
            if label.dynamic(){continue;} let point=[p.x,p.y,p.z];let k=key(point,SURFACE_M);
            if !seen.insert(k){continue;}
            let v=self.points.entry(k).or_insert(Reference{point,first:stamp,last:0,hits:0,ground:false});
            if v.last!=stamp{v.last=stamp;v.hits=v.hits.saturating_add(1);}
            v.ground|=*label==PointLabel::Ground;
        }
    }
}
pub fn protect_from_points(p:[f64;3],nearby:&[[f64;3]])->Option<PointLabel>{
    if nearby.is_empty(){return None;}
    let nearest=nearby.iter().map(|&q|d2(p,q)).fold(f64::INFINITY,f64::min);
    if nearby.len()>=3{
        let mean=nearby.iter().fold(Vector3::zeros(),|s,&q|s+Vector3::from(q))/nearby.len() as f64;
        let cov=nearby.iter().fold(Matrix3::zeros(),|s,&q|{let d=Vector3::from(q)-mean;s+d*d.transpose()})/nearby.len() as f64;
        let e=SymmetricEigen::new(cov);let mut order=[0,1,2];order.sort_by(|&a,&b|e.eigenvalues[a].total_cmp(&e.eigenvalues[b]));
        let [a,b,c]=order;let d=Vector3::from(p)-mean;
        if e.eigenvalues[b]<=0.0001&&e.eigenvalues[c]>4.0*e.eigenvalues[b].max(1e-10){
            let direction=e.eigenvectors.column(c);let residual=(d-direction*d.dot(&direction)).norm();
            if residual<=SURFACE_M{return Some(PointLabel::SurfaceLine);}
        }else if e.eigenvalues[a]<=0.0001&&e.eigenvalues[b]>4.0*e.eigenvalues[a].max(1e-10){
            if d.dot(&e.eigenvectors.column(a)).abs()<=SURFACE_M{return Some(PointLabel::SurfacePlane);}
        }
    }
    (nearest<=SURFACE_M.powi(2)).then_some(PointLabel::SurfacePoint)
}

/// Flood through actual non-protected returns, then enforce the original cluster
/// shape/seed requirements. Rejected growth becomes pending; seeds stay dynamic.
pub fn grow(points:&[Point3<f64>],labels:&mut [PointLabel]){
    let mut cells:HashMap<[i32;3],Vec<usize>>=HashMap::new();
    for (i,p) in points.iter().enumerate(){if matches!(labels[i],PointLabel::Seed|PointLabel::PendingNew){
        cells.entry(key([p.x,p.y,p.z],GROW_M)).or_default().push(i);
    }}
    let mut visited=vec![false;points.len()];
    for start in 0..points.len(){
        if labels[start]!=PointLabel::Seed||visited[start]{continue;}
        visited[start]=true;let mut queue=VecDeque::from([start]);let mut cluster=Vec::new();let mut seeds=0;
        let mut occupied=HashSet::new();let mut low=[f64::INFINITY;3];let mut high=[f64::NEG_INFINITY;3];
        while let Some(i)=queue.pop_front(){
            cluster.push(i);seeds+=usize::from(labels[i]==PointLabel::Seed);
            let p=[points[i].x,points[i].y,points[i].z];occupied.insert(key(p,0.1));
            for a in 0..3{low[a]=low[a].min(p[a]);high[a]=high[a].max(p[a]);}
            let k=key(p,GROW_M);
            for x in -1..=1{for y in -1..=1{for z in -1..=1{
                if let Some(ids)=cells.get(&[k[0]+x,k[1]+y,k[2]+z]){for &j in ids{
                    if !visited[j]&&d2(p,[points[j].x,points[j].y,points[j].z])<=GROW_M.powi(2){visited[j]=true;queue.push_back(j);}
                }}
            }}}
        }
        if cluster.len()>=8&&occupied.len()>=3&&seeds*5>=cluster.len()&&high[0]-low[0]<=3.0&&high[1]-low[1]<=3.0&&high[2]-low[2]<=2.5{
            for i in cluster{if labels[i]==PointLabel::PendingNew{labels[i]=PointLabel::Grown;}}
        }
    }
}
#[cfg(test)]mod tests{
    use super::*;
    #[test]fn plane_line_and_point_protection_are_distinct(){
        let plane:Vec<_>=(-2..=2).flat_map(|x|(-2..=2).map(move|y|[x as f64*0.04,y as f64*0.04,0.])).collect();
        assert_eq!(protect_from_points([0.01,0.01,0.029],&plane),Some(PointLabel::SurfacePlane));
        assert_eq!(protect_from_points([0.01,0.01,0.05],&plane),None);
        let line:Vec<_>=(-3..=3).map(|i|[0.,0.,i as f64*0.04]).collect();
        assert_eq!(protect_from_points([0.029,0.,0.01],&line),Some(PointLabel::SurfaceLine));
        assert_eq!(protect_from_points([0.05,0.,0.01],&line),None);
        assert_eq!(protect_from_points([0.02,0.,0.],&[[0.;3]]),Some(PointLabel::SurfacePoint));
    }
    #[test]fn protected_rod_does_not_become_dynamic_by_cluster_membership(){
        let points:Vec<_>=(0..20).map(|i|Point3::new(0.,0.,i as f64*0.04)).collect();let mut labels=vec![PointLabel::PendingNew;20];
        for label in &mut labels[..6]{*label=PointLabel::Seed;}labels[8]=PointLabel::SurfaceLine;grow(&points,&mut labels);
        assert_eq!(labels[8],PointLabel::SurfaceLine);assert_eq!(labels[6],PointLabel::Grown);
    }
}
