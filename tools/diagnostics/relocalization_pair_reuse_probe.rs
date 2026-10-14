//! Standalone fixture template. Generated source includes actual production
//! modules; the generated candidate only reuses one index for a candidate pair.
#![allow(dead_code)]
mod nearest;
mod features;
mod ransac;
mod baseline;
mod candidate;
use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};
use baseline::*;
use ransac::evaluate_alignment;
use std::time::Instant;

#[cfg(probe_counts)] mod stats {
    use std::cell::RefCell;
    #[derive(Default, Clone, Debug)] pub struct S { pub fine_ptr:usize, pub builds:usize, pub fine_builds:usize, pub tree_ns:u128, pub fine_ns:u128, pub live:usize, pub peak:usize }
    thread_local! { static SLOTS: RefCell<S> = RefCell::new(S::default()); }
    pub fn reset() { SLOTS.with(|s| { assert_eq!(s.borrow().live,0); *s.borrow_mut()=S::default(); }); }
    pub fn fine(ptr:usize) { SLOTS.with(|s|s.borrow_mut().fine_ptr=ptr); }
    pub fn built(ptr:usize, ns:u128, bytes:usize) { SLOTS.with(|s|{let mut s=s.borrow_mut(); s.builds+=1; s.tree_ns+=ns; if ptr==s.fine_ptr {s.fine_builds+=1;s.fine_ns+=ns;} s.live+=bytes;s.peak=s.peak.max(s.live);}); }
    pub fn freed(bytes:usize) { SLOTS.with(|s|s.borrow_mut().live-=bytes); }
    pub fn get() -> S {SLOTS.with(|s|s.borrow().clone())}
}
struct RelocalizationComputeOutput { result: RelocalizationResult, correction: Option<Isometry3<f64>> }
include!("compute.rs");
fn fingerprint(r:&RelocalizationComputeOutput)->Vec<u64> {
    let mut out=vec![r.result.registration.mse.to_bits(),r.result.registration.iterations as u64,
        r.result.registration.converged as u64,r.result.feature_matches as u64,r.result.ransac_inliers as u64,r.result.fitness.to_bits(),r.correction.is_some() as u64];
    out.extend(r.result.registration.transformation.to_homogeneous().iter().map(|v|v.to_bits()));
    if let Some(c)=r.correction {out.extend(c.to_homogeneous().iter().map(|v|v.to_bits()));}
    out
}
// This is the exact existing sparse-wrong-heading source generator, independently
// copied into this diagnostic. The original regression stays unchanged.
fn base_source()->Vec<Point3<f64>> {
    let mut source=Vec::new();
    for index in 0..8 {source.push(Point3::new((index&1) as f64*0.8+0.03*index as f64,
        ((index>>1)&1) as f64*0.9+0.011*(index*index) as f64,
        ((index>>2)&1) as f64*0.7+0.009*(index*index) as f64));}
    for index in 0..120 {source.push(Point3::new(5.0+(index%11) as f64*0.43,
        -3.0+((index*7)%19) as f64*0.31,0.2+((index*13)%23) as f64*0.19));}
    source
}
fn input(n:usize, kind:&str)->(Vec<Point3<f64>>,Vec<Point3<f64>>,Isometry3<f64>) {
    let base=base_source();
    let source=(0..n).map(|i| { let mut p=base[i%base.len()];
        // Additional disjoint copies extend the scene without raising per-frame
        // limits; global features/scoring remain the untouched production code.
        let tile=i/base.len(); p.x+=((tile%16)*16) as f64; p.y+=((tile/16)*16) as f64; p
    }).collect::<Vec<_>>();
    let truth=if kind=="seed" {Isometry3::from_parts(Translation3::new(0.03,-0.02,0.01),UnitQuaternion::from_euler_angles(0.0,0.0,0.01))}
        else {Isometry3::from_parts(Translation3::new(30.0,20.0,0.0),UnitQuaternion::from_euler_angles(0.0,0.0,37_f64.to_radians()))};
    let mut target=source.iter().map(|p|truth*p).collect::<Vec<_>>();
    if kind=="wrong" {let distractor=Isometry3::from_parts(Translation3::new(0.1,-0.05,0.02),UnitQuaternion::from_euler_angles(0.0,0.0,-17_f64.to_radians()));target.extend(source[..8].iter().map(|p|distractor*p));}
    (source,target,truth)
}
fn main() {
    let args=std::env::args().collect::<Vec<_>>();
    let n=args.get(1).unwrap().parse::<usize>().unwrap();let kind=args.get(2).unwrap();
    let samples=args.get(3).map(|s|s.parse::<usize>().unwrap()).unwrap_or(7);
    let reverse=args.get(4).is_some_and(|s|s=="BA");
    let (source,target,truth)=input(n,kind);
    let registration_seed=if kind=="seed" && n>128 {truth}else{Isometry3::identity()};
    let mut expected=None;
    for sample in 0..samples+2 { for reuse in if reverse ^ (sample%2==1) {[true,false]}else{[false,true]} {
        #[cfg(probe_counts)] stats::reset();
        let began=Instant::now();
        let result=run_relocalization_compute(reuse,std::hint::black_box(&source),std::hint::black_box(&target),Isometry3::identity(),registration_seed);
        let elapsed=began.elapsed().as_nanos();
        let fp=fingerprint(&result);if let Some(ref e)=expected {assert_eq!(&fp,e);}else{expected=Some(fp);}
        if kind=="seed" {assert_eq!(result.result.feature_matches,0);assert!(result.correction.is_some());}
        if kind=="wrong" && n==128 {let c=result.correction.expect("existing full-match fixture");assert!(result.result.feature_matches>0);assert_eq!(result.result.fitness,1.0);assert!((c.rotation.inverse()*truth.rotation).angle()<1e-6);assert!((c.translation.vector-truth.translation.vector).norm()<1e-6);}
        if sample>=2 {
            #[cfg(probe_counts)] {let s=stats::get();assert_eq!(s.live,0);println!("{{\"n\":{n},\"target\":{},\"kind\":\"{kind}\",\"reuse\":{reuse},\"sample\":{},\"elapsed_ns\":{elapsed},\"builds\":{},\"fine_builds\":{},\"tree_ns\":{},\"fine_ns\":{},\"index_peak_payload_bytes\":{},\"matches\":{},\"fitness\":{},\"converged\":{}}}",target.len(),sample-2,s.builds,s.fine_builds,s.tree_ns,s.fine_ns,s.peak,result.result.feature_matches,result.result.fitness,result.result.registration.converged);}
            #[cfg(not(probe_counts))] println!("{{\"n\":{n},\"target\":{},\"kind\":\"{kind}\",\"reuse\":{reuse},\"sample\":{},\"elapsed_ns\":{elapsed},\"matches\":{},\"fitness\":{},\"converged\":{}}}",target.len(),sample-2,result.result.feature_matches,result.result.fitness,result.result.registration.converged);
        }
    }}
}
