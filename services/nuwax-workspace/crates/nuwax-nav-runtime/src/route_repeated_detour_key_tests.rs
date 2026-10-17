// Supplemental exact-key checks. These seed the existing synthetic negative fixture;
// real PCT/Scan repeat, budget, costs, cancellation and recovery are tested separately.
#[test]
fn early_failure_cache_covers_exact_budget_prefix_and_cost_inputs() {
    for change in 0..7 {
        let (mut kernel,mut request,cancel)=fixture();
        seed_exact_failures(&mut kernel,&request);
        match change {
            0=>request.scan.search_expansion_limit+=1,
            1=>request.scan.optimization_iterations+=1,
            2=>request.scan.horizon_m=f64::from_bits(request.scan.horizon_m.to_bits()+1),
            3=>request.scan.reference_corridor_m=f64::from_bits(request.scan.reference_corridor_m.to_bits()+1),
            4=>request.pose.pose.position.x=f64::from_bits(request.pose.pose.position.x.to_bits()+1),
            5=>kernel.config.pct.report_local_blockage([1.0,1.0,0.1],0.2),
            _=>{
                request.source=Arc::new(request.source.with_recent_obstacle_points(
                    1,1_000_000_000,0,&[],[(0,[2.0,1.5,0.5])]).unwrap());
            }
        }
        let result=kernel.compute_inner(&request,&cancel);
        if let Err(failures)=result {
            assert!(failures.iter().all(|f|f.reason!="exact cached failure"),
                "actual planning semantic change {change} reused a seeded old failure");
        }
    }
}
