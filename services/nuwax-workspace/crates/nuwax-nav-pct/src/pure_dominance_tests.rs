//! Shared-kernel bookkeeping qualification. Analytical predicates here are
//! deterministic test fixtures, never a production purity escape hatch.
use super::*;

fn outcome(value:&Result<SearchOutput,PlanningFailure>)->String {
    match value {
        Err(error)=>format!("{error:?}"),
        Ok(value)=>format!("{:?}|{:?}|{:?}|{:?}",bits(&value.route.points),
            (value.route.stats.expanded,value.route.stats.cells),
            value.trace.as_ref().unwrap().connections.iter().map(|r|(r.point_span,
                r.evaluated_from.map(f64::to_bits),r.evaluated_to.map(f64::to_bits),r.steps,r.changes.clone())).collect::<Vec<_>>(),
            value.trace.as_ref().unwrap().changes),
    }
}
fn pair(map:&Tomogram,start:[f64;3],goal:[f64;3],boundary:&[[f64;2]],radius:Option<f64>,
    penalties:&[crate::RoutePenalty],accepts:&dyn Fn([f64;3],[f64;3])->bool,
    limits:SearchLimits,cancel:Option<usize>)->(Result<SearchOutput,PlanningFailure>,Work) {
    let run=|early:bool| {
        let calls=RefCell::new(Vec::new());
        let accept=|a:[f64;3],b:[f64;3]| {let ok=accepts(a,b);calls.borrow_mut().push((bits(&[a,b]),ok));ok};
        let call=||if early {map.plan_bounded_impl_in::<true,true>(start,goal,0.,boundary,None,radius,
            penalties,&accept,limits,SearchGeometry::Ground{unanchored_z:None,target:None,initial:None})}
            else {map.plan_bounded_impl_in::<true,false>(start,goal,0.,boundary,None,radius,
            penalties,&accept,limits,SearchGeometry::Ground{unanchored_z:None,target:None,initial:None})};
        let result=observe(||match cancel {Some(n)=>crate::cancellation::after_checkpoints(n,call),None=>call()});
        (result,calls.into_inner())
    };
    let ((old,old_work),old_calls)=run(false);let ((new,new_work),new_calls)=run(true);
    assert_eq!(outcome(&old),outcome(&new));
    assert_eq!(old_work.events,new_work.events,"same checkpoints, reference walks, costs, parent updates and stale pops");
    assert_eq!(old_work.parents,new_work.parents);
    assert_eq!(old_work.stale_parents,new_work.stale_parents);
    assert_eq!(old_work.best,new_work.best);
    // Remaining deterministic admissions preserve their original relative order.
    let mut tail=old_calls.as_slice();for call in &new_calls {
        let i=tail.iter().position(|old|old==call).expect("new admission must be an ordered old admission");tail=&tail[i+1..];
    }
    eprintln!("P1_BOOKKEEP old_calls={} new_calls={} events={} parents={} outcome={}",
        old_calls.len(),new_calls.len(),new_work.events.len(),new_work.parents.len(),outcome(&new));
    (new,new_work)
}

#[test]
fn p1_03_lower_g_reentry_keeps_current_parent_trace_and_stale_order() {
    // Exact original B1-07 fixture; only Ground XY interpretation differs,
    // while every height is zero so all arrival/g operations have equal values.
    let map=Tomogram::from_columns(0.1,(-2..=8).flat_map(|x|(-3..=5)
        .map(move|y|([x,y],vec![[-0.1,0.]]))),0.4,0.,0.1,0.1,20.).unwrap();
    let penalties=[crate::RoutePenalty{center:[0.15,0.05,0.],radius:0.02,weight:0.01}];
    let accepts=|a:[f64;3],b:[f64;3]|if a[0].min(b[0])<=0.3 && a[0].max(b[0])>=0.3 {
        a[1]+(0.3-a[0])/(b[0]-a[0])*(b[1]-a[1])>0.1
    }else{true};
    let (result,work)=pair(&map,[0.05,0.05,0.],[0.55,0.05,0.],
        &[[-0.2,-0.3],[0.9,-0.3],[0.9,0.6],[-0.2,0.6]],None,&penalties,&accepts,SearchLimits::default(),None);
    assert!(result.is_ok());
    let updates:Vec<_>=work.events.iter().filter(|e|matches!(e,Event::ParentUpdate(..))).collect();
    assert_eq!(updates.len(),3);assert_eq!(work.stale_parents.len(),2);
    assert!(updates.iter().all(|e|matches!(e,Event::ParentUpdate(_,old,new,_,_) if f64::from_bits(*new)<f64::from_bits(*old))));
    // This proves an admitted state's cheaper queue reinsertion. It does not
    // assert that that state had already been expanded before its improvement.
}

#[test]
fn p1_04_same_xy_distinct_effective_references_remain_distinct() {
    let map=Tomogram::from_columns(0.1,
        [([0,0],vec![[-0.1,0.]]),([1,0],vec![[0.,0.1]])],0.4,0.,0.2,0.2,20.).unwrap();
    let penalties=[crate::RoutePenalty{center:[10.,0.,0.],radius:0.1,weight:1.}];
    let (result,work)=pair(&map,[0.05,0.05,0.],[0.25,0.25,0.],
        &[[0.,0.],[0.3,0.],[0.3,0.3],[0.,0.3]],None,&penalties,
        &|_,b|b[0]<0.2||b[1]<0.2,SearchLimits::default(),None);
    assert!(result.is_err());
    let admitted:std::collections::HashSet<_>=work.parents.iter().map(|p|p.3.last().unwrap().reference).collect();
    let mut heights=std::collections::BTreeSet::new();
    for reference in &admitted {if let Reference::Unknown{column:[0,1],height}=reference {heights.insert(*height);}}
    assert_eq!(heights,std::collections::BTreeSet::from([0.0f64.to_bits(),0.1f64.to_bits()]),
        "the actual search must admit both carried heights at the same XY");
    // Raw signed-bit/variant identity is the original AHashMap contract.
    let keys=[Reference::Known(0),Reference::Known(1),Reference::Unanchored{column:[0,1]},
        Reference::Unknown{column:[0,1],height:0.0f64.to_bits()},Reference::Unknown{column:[0,1],height:0.1f64.to_bits()}];
    let mut indexes=ahash::AHashMap::new();for (i,key) in keys.into_iter().enumerate(){assert_eq!(indexes.insert(key,i),None);}
    assert_eq!(indexes.len(),5);
}

#[test]
fn p1_05_budget_tail_radius_and_domain_reasons_match() {
    let map=map([]);let penalties=[crate::RoutePenalty{center:[10.,0.,0.],radius:0.1,weight:1.}];
    for maximum in [0,1,8,64] {
        let (r,_)=pair(&map,[0.05,0.05,0.],[0.15,0.05,0.],&[],None,&penalties,&|_,_|true,
            SearchLimits{max_states:maximum},None);
        if maximum<8 {assert!(matches!(r,Err(PlanningFailure::BudgetExhausted(_))))}else{assert!(r.is_ok())}
    }
    let (r,_)=pair(&map,[0.05,0.05,0.],[0.55,0.05,0.],&[],Some(0.2),&penalties,&|_,_|true,
        SearchLimits::default(),None);
    assert!(matches!(r,Err(PlanningFailure::SearchDomainLimited(_))));
    let corner=Tomogram::from_columns(0.1,[([0,1],vec![[0.,0.1]])],0.4,0.,0.15,0.15,20.).unwrap();
    let initial=corner.reference_start([0.01,0.09,0.]).unwrap();
    let endpoint=corner.reference_position(corner.reference_neighbor(initial,[1,1]).unwrap());
    let (r,_)=pair(&corner,[0.01,0.09,0.],[0.55,0.55,0.],&[],Some(0.05),&[],&|_,b|b==endpoint,
        SearchLimits::default(),None);assert!(matches!(r,Err(PlanningFailure::Other(_))));
    let mut cropped=map;cropped.window=Some(([0.05,0.05],0.3));
    let (r,_)=pair(&cropped,[0.05,0.05,0.],[0.55,0.05,0.],&[],None,&penalties,&|_,_|true,
        SearchLimits::default(),None);assert!(matches!(r,Err(PlanningFailure::SearchDomainLimited(_))));
}

#[test]
fn p1_06_checkpoints_and_generic_stateful_callback_order_are_preserved() {
    let map=map([]);let penalty=[crate::RoutePenalty{center:[10.,0.,0.],radius:0.1,weight:1.}];
    for n in [0,20,80] {
        let (r,_)=pair(&map,[0.05,0.05,0.],[0.55,0.05,0.],&BOUNDARY,None,&penalty,&|_,_|true,
            SearchLimits::default(),Some(n));assert!(matches!(r,Err(PlanningFailure::Other(ref m)) if m.contains("取消")));
    }
    let flag=AtomicBool::new(false);let calls=RefCell::new(Vec::new());
    let callback=|a,b| {calls.borrow_mut().push((a,b));if b==[0.05,0.05,0.] {flag.store(true,AtomicOrdering::Release);}true};
    let r=map.plan_bounded_with_segment_filter_and_limits_typed([0.05,0.05,0.],[0.45,0.05,0.],0.,
        &[[0.,0.],[0.5,0.],[0.5,0.1],[0.,0.1]],Some(&flag),None,&penalty,&callback,SearchLimits::default());
    assert_eq!(r.unwrap_err(),PlanningFailure::Other("层析规划已取消".into()));
    assert!(flag.load(AtomicOrdering::Acquire));assert!(calls.borrow().iter().any(|(a,b)|a!=b&&*b==[0.05,0.05,0.]));
}
