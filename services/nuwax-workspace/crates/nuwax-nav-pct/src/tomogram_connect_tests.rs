//! Frozen cardinal-lookup trial oracle. No transport, controller or motion.
use super::*;
use std::{hint::black_box, time::{Duration, Instant}};

trait FrozenConnect { fn connect(&mut self) -> Result<(), String>; }
// Exact original connect body, frozen before the production trial.
impl FrozenConnect for Tomogram {
    fn connect(&mut self) -> Result<(), String> {
        for id in 0..self.cells.len() {
            checkpoint()?;
            let [x, y] = self.cells[id].column;
            let mut neighbors = Vec::with_capacity(8);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if dx == 0 && dy == 0 {
                        continue;
                    }
                    let distance = self.resolution * f64::from(dx * dx + dy * dy).sqrt();
                    let Some(next) = self.neighbor_in(id, [x + dx, y + dy], distance) else {
                        continue;
                    };
                    if dx != 0 && dy != 0 {
                        // 对角连接不能穿过两面相接的墙角或落差。
                        let Some(a) = self.neighbor_in(id, [x + dx, y], self.resolution) else {
                            continue;
                        };
                        let Some(b) = self.neighbor_in(id, [x, y + dy], self.resolution) else {
                            continue;
                        };
                        if !self.compatible(a, next, self.resolution)
                            || !self.compatible(b, next, self.resolution)
                        {
                            continue;
                        }
                    }
                    neighbors.push(next);
                }
            }
            self.cells[id].neighbors = neighbors;
        }
        checkpoint()?;
        Ok(())
    }
}

fn copy_map(m: &Tomogram) -> Tomogram {
    Tomogram { resolution:m.resolution, cells:m.cells.clone(), columns:m.columns.clone(),
        known_columns:m.known_columns.clone(), step_up:m.step_up, step_down:m.step_down,
        slope:m.slope, body_height:m.body_height, half_width:m.half_width,
        clearance_radius:m.clearance_radius }
}
fn assert_exact(a: &Tomogram, b: &Tomogram) {
    let parameters=|m:&Tomogram| [m.resolution,m.step_up,m.step_down,m.slope,m.body_height,
        m.half_width,m.clearance_radius].map(f64::to_bits);
    assert_eq!(parameters(a),parameters(b));
    assert_eq!(a.columns,b.columns); assert_eq!(a.known_columns,b.known_columns);
    assert_eq!(a.cells.len(),b.cells.len());
    for (x,y) in a.cells.iter().zip(&b.cells) {
        let bits=|c:&Cell| [c.ground,c.ground_interval[0],c.ground_interval[1],c.ceiling,
            c.clearance,c.support_clearance,c.cost].map(f64::to_bits);
        assert_eq!((x.column,x.layer,bits(x),&x.neighbors),
            (y.column,y.layer,bits(y),&y.neighbors));
    }
}
fn pair(map:&Tomogram)->(Tomogram,Tomogram) {
    let mut old=copy_map(map); let mut new=copy_map(map);
    for c in &mut old.cells { c.neighbors.clear(); }
    for c in &mut new.cells { c.neighbors.clear(); }
    <Tomogram as FrozenConnect>::connect(&mut old).unwrap();
    new.connect().unwrap(); old.compute_costs().unwrap(); new.compute_costs().unwrap();
    assert_exact(&old,&new); (old,new)
}
fn assert_route(a:&Tomogram,b:&Tomogram,start:[f64;3],goal:[f64;3],boundary:&[[f64;2]]) {
    let old=a.plan(start,goal,0.01,boundary,None);
    let new=b.plan(start,goal,0.01,boundary,None);
    match (old,new) {
        (Ok(a),Ok(b))=>{
            assert_eq!(a.points.iter().map(|p|p.map(f64::to_bits)).collect::<Vec<_>>(),
                b.points.iter().map(|p|p.map(f64::to_bits)).collect::<Vec<_>>());
            assert_eq!((a.stats.expanded,a.stats.cells),(b.stats.expanded,b.stats.cells));
        },
        (Err(a),Err(b))=>assert_eq!(a,b),
        _=>panic!("route success changed"),
    }
}
fn mixed_columns()->Vec<([i32;2],Vec<[f64;2]>)> {
    let mut columns=Vec::new();
    for x in 0..80 { for y in 0..64 {
        if (52..54).contains(&x) && !(42..52).contains(&y) {continue;}
        let ground=f64::from((x/10+y/8)%4)*0.015;
        let mut bands=if x==27 && !(20..30).contains(&y) {vec![[-0.1,2.0]]}
            else {vec![[ground-0.1,ground]]};
        if (8..24).contains(&x) && (8..24).contains(&y) {bands.extend([[1.4,1.5],[1.45,1.55]]);}
        columns.push(([x,y],bands));
    }} columns
}
fn mixed()->Tomogram {
    Tomogram::from_columns(0.1,mixed_columns(),0.5,0.2,0.15,0.15,20.0).unwrap()
}
#[test]
fn cardinal_reuse_preserves_mixed_graph_and_routes() {
    let (old,new)=pair(&mixed());
    assert_eq!((new.cells.len(),new.known_columns.len()),(5268,5012));
    assert_eq!(new.cells.iter().map(|c|c.neighbors.len()).sum::<usize>(),40120);
    for (start,goal) in [([0.45,0.65,0.0],[7.45,5.85,0.03]),
        ([0.95,0.95,1.525],[2.25,2.25,1.525]),([0.95,0.95,0.0],[2.25,2.25,1.525])] {
        assert_route(&old,&new,start,goal,&[]);
    }
    let boundary=[[0.,0.],[8.,0.],[8.,1.],[1.,1.],[1.,6.4],[0.,6.4]];
    assert_route(&old,&new,[0.45,5.85,0.03],[7.45,0.65,0.03],&boundary);
}
#[test]
fn cardinal_reuse_preserves_corner_missing_mask_layers_and_direction() {
    // Enumerate every presence pattern around a fixed origin, with missing columns
    // and explicitly known-empty columns tested independently at negative coordinates.
    for mask in 0u16..256 { for known_empty in [false,true] {
        let mut columns=vec![([-2,-2],vec![[-0.1,0.0]])];let mut bit=0;
        for dx in -1..=1 {for dy in -1..=1 {
            if dx==0 && dy==0 {continue;}
            if mask&(1<<bit)!=0 {columns.push(([-2+dx,-2+dy],vec![[-0.1,0.0]]));}
            else if known_empty {columns.push(([-2+dx,-2+dy],vec![]));}
            bit+=1;
        }}
        let source=Tomogram::from_columns(0.1,columns,0.3,0.1,0.1,0.1,20.0).unwrap();
        let (old,new)=pair(&source);
        assert_route(&old,&new,[-0.15,-0.15,0.0],[-0.05,-0.05,0.0],&[]);
    }}
    // A nearest-height tie in two separated layers, plus directional step and ceiling cases.
    for (bands,body,up,down) in [
        (vec![[-0.6,-0.5],[0.4,0.5]],0.1,0.6,0.6),
        (vec![[0.1,0.2]],0.1,0.25,0.05),
        (vec![[-0.1,0.0],[0.15,0.25]],0.2,0.2,0.2),
    ] {
        let columns=(-1..=1).flat_map(|x|(-1..=1).map({let bands=bands.clone();move|y|
            ([x,y],if x==0 && y==0 {vec![[-0.1,0.0]]}else{bands.clone()})}));
        let source=Tomogram::from_columns(0.1,columns,body,0.1,up,down,10.0).unwrap();
        let (old,new)=pair(&source);
        assert_route(&old,&new,[0.05,0.05,0.0],[0.15,0.05,0.2],&[]);
        assert_route(&old,&new,[0.15,0.05,0.2],[0.05,0.05,0.0],&[]);
    }
}
#[test]
fn cardinal_reuse_keeps_tie_identity_and_shared_ceiling_rejection() {
    let tie=Tomogram::from_columns(0.1,vec![
        ([0,0],vec![[-0.1,0.0]]),
        ([-1,0],vec![[-0.6,-0.5],[0.4,0.5]]),
        ([0,-1],vec![[-0.6,-0.5],[0.4,0.5]]),
        ([-1,-1],vec![[0.4,0.5]]),
    ],0.1,0.1,0.6,0.6,10.0).unwrap();
    let (old,new)=pair(&tie);
    let origin=new.cell_at([0.05,0.05,0.0]).unwrap();
    let flank=new.neighbor_in(origin,[-1,0],new.resolution).unwrap();
    assert_eq!(new.cells[flank].ground,-0.5,"exact tie keeps first layer");
    let diagonal=new.neighbor_in(origin,[-1,-1],new.resolution*2.0_f64.sqrt()).unwrap();
    assert!(new.compatible(origin,diagonal,new.resolution*2.0_f64.sqrt()));
    assert!(!new.compatible(flank,diagonal,new.resolution));
    assert!(!new.cells[origin].neighbors.contains(&diagonal),"alternative tie layer must not open corner");
    assert_route(&old,&new,[0.05,0.05,0.0],[-0.05,-0.05,0.5],&[]);

    let ceiling=Tomogram::from_columns(0.1,vec![
        ([0,0],vec![[0.1,0.2],[0.5,0.6]]),
        ([1,0],vec![[-0.1,0.0],[0.4,0.5]]),
    ],0.3,0.1,0.2,0.3,10.0).unwrap();
    let (_,new)=pair(&ceiling);
    let a=new.cell_at([0.05,0.05,0.2]).unwrap();
    let b=new.cell_at([0.15,0.05,0.0]).unwrap();
    assert!(new.cells[a].ceiling-new.cells[a].ground+1e-6>=new.body_height);
    assert!(new.cells[b].ceiling-new.cells[b].ground+1e-6>=new.body_height);
    assert!((new.cells[b].ground-new.cells[a].ground).abs()<=new.step_down);
    assert!(new.cells[a].ceiling.min(new.cells[b].ceiling)
        -new.cells[a].ground.max(new.cells[b].ground)+1e-6<new.body_height);
    assert!(!new.compatible(a,b,new.resolution));
    assert!(!new.cells[a].neighbors.contains(&b));
}
#[test]
fn cardinal_reuse_preserves_cancellation_checkpoints_and_partial_state() {
    let map=Tomogram::from_columns(0.1,(0..4).flat_map(|x|(0..4)
        .map(move|y|([x,y],vec![[-0.1,0.0]]))),0.3,0.1,0.1,0.1,20.0).unwrap();
    for count in 0..=map.cells.len()+1 {
        let mut old=copy_map(&map);let mut new=copy_map(&map);
        for c in &mut old.cells {c.neighbors.clear();}
        for c in &mut new.cells {c.neighbors.clear();}
        let a=crate::cancellation::after_checkpoints(count,||<Tomogram as FrozenConnect>::connect(&mut old));
        let b=crate::cancellation::after_checkpoints(count,||new.connect());
        assert_eq!(a,b,"checkpoint {count}");assert_exact(&old,&new);
    }
}
#[test]
#[ignore="manual frozen cardinal connect trial; no transport or motion"]
fn cardinal_connect_matched_benchmark() {
    const WARMUPS:usize=4;const PAIRS:usize=32;const CONSTRUCT:usize=16;
    let columns=mixed_columns();let map=mixed();let (mut old,mut new)=pair(&map);
    let reverse=std::env::var_os("NEH_CONNECT_REVERSE").is_some();
    let mut old_elapsed=Duration::ZERO;let mut new_elapsed=Duration::ZERO;
    for i in 0..WARMUPS+PAIRS {
        for legacy in if (i%2==0)^reverse {[true,false]}else{[false,true]} {
            let began=Instant::now();
            if legacy {black_box(<Tomogram as FrozenConnect>::connect(black_box(&mut old))).unwrap();}
            else {black_box(Tomogram::connect(black_box(&mut new))).unwrap();}
            let elapsed=began.elapsed();
            if i>=WARMUPS {if legacy {old_elapsed+=elapsed;}else{new_elapsed+=elapsed;}}
        }
    }
    assert_exact(&old,&new);
    let mut construction=Duration::ZERO;
    for i in 0..WARMUPS+CONSTRUCT {
        let began=Instant::now();
        let built=Tomogram::from_columns(0.1,black_box(columns.clone()),0.5,0.2,0.15,0.15,20.0).unwrap();
        black_box(&built);let elapsed=began.elapsed();
        if i>=WARMUPS {construction+=elapsed;}
        assert_exact(&map,&built);drop(built);
    }
    println!("cardinal_connect cells={} edges={} warmups={} pairs={} construct_reps={} reference_ns={} current_ns={} construction_ns={}",
        map.cells.len(),map.cells.iter().map(|c|c.neighbors.len()).sum::<usize>(),WARMUPS,PAIRS,CONSTRUCT,
        old_elapsed.as_nanos(),new_elapsed.as_nanos(),construction.as_nanos());
}
