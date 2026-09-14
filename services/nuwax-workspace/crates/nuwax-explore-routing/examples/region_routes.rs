//! 可重复的区域调度对照与安全图查询计时，不模拟传感器或机器人运动。
use nuwax_explore_routing::{select, shortest_costs, Candidate};
use std::time::Instant;

fn main() {
    let mut old_travel = 0.0;
    let mut new_travel = 0.0;
    let mut old_latency = 0.0;
    let mut new_latency = 0.0;
    let mut latency_regressions = 0;
    for seed in 1..=128u64 {
        let mut random = seed;
        let mut next = || {
            random = random.wrapping_mul(6364136223846793005).wrapping_add(1);
            random >> 32
        };
        // 八个区域经走廊连接，既含近分支也含必须回走的深分支。
        let mut graph = vec![Vec::new(); 9];
        for node in 1..9 {
            let parent = next() as usize % node;
            // 区域沿横轴每隔四米，边长包含直线下界及额外走廊绕行。
            let cost = 4.0 * (node - parent) as f64 + (next() % 8 + 1) as f64;
            graph[parent].push((node, cost));
            graph[node].push((parent, cost));
        }
        let gains: Vec<_> = (0..9).map(|_| (next() % 6 + 1) as f64).collect();
        let matrix: Vec<_> = (0..9)
            .map(|source| {
                shortest_costs(9, source, None, |id, out| out.extend_from_slice(&graph[id]))
            })
            .collect();
        let run = |regional: bool| {
            let mut remaining: Vec<_> = (1..9).collect();
            let mut current = 0;
            let mut travelled = 0.0;
            let mut latency = 0.0;
            while !remaining.is_empty() {
                let candidates: Vec<_> = remaining
                    .iter()
                    .map(|&node| Candidate {
                        position: [4.0 * node as f64, 0.0, 0.0],
                        gain: gains[node],
                        travel_cost: matrix[current][node],
                    })
                    .collect();
                let index = if regional {
                    select(&candidates, |source, targets| {
                        targets
                            .iter()
                            .map(|&i| matrix[remaining[source]][remaining[i]])
                            .collect()
                    })
                    .unwrap()
                    .index
                } else {
                    (0..remaining.len())
                        .max_by(|&a, &b| {
                            (candidates[a].gain / (1.0 + candidates[a].travel_cost)).total_cmp(
                                &(candidates[b].gain / (1.0 + candidates[b].travel_cost)),
                            )
                        })
                        .unwrap()
                };
                let node = remaining.remove(index);
                travelled += matrix[current][node];
                latency += travelled * gains[node];
                current = node;
            }
            (travelled, latency)
        };
        let old = run(false);
        let new = run(true);
        old_travel += old.0;
        new_travel += new.0;
        old_latency += old.1;
        new_latency += new.1;
        if new.1 > old.1 + 1e-9 {
            latency_regressions += 1;
        }
    }
    println!("走廊图128组: 单点策略总路程={old_travel:.3}, 区域策略总路程={new_travel:.3}, 单点加权到达代价={old_latency:.3}, 区域加权到达代价={new_latency:.3}, 到达代价退步组数={latency_regressions}");
    assert_eq!(latency_regressions, 0);
    let width = 256usize;
    let edges = |id: usize, out: &mut Vec<(usize, f64)>| {
        if id % width > 0 {
            out.push((id - 1, 0.1));
        }
        if id % width + 1 < width {
            out.push((id + 1, 0.1));
        }
        if id >= width {
            out.push((id - width, 0.1));
        }
        if id + width < width * width {
            out.push((id + width, 0.1));
        }
    };
    let targets = [0, 255, 65280, 65535, 4096, 4351, 61440, 61695];
    let mut timings = Vec::new();
    for _ in 0..15 {
        let begin = Instant::now();
        let root = shortest_costs(width * width, 32896, Some(&targets), edges);
        let candidates: Vec<_> = targets
            .iter()
            .enumerate()
            .map(|(i, &node)| Candidate {
                position: [4.0 * i as f64, 0.0, 0.0],
                gain: 1.0,
                travel_cost: root[node],
            })
            .collect();
        let selected = select(&candidates, |source, goals| {
            let ids: Vec<_> = goals.iter().map(|&i| targets[i]).collect();
            let distances = shortest_costs(width * width, targets[source], Some(&ids), edges);
            ids.iter().map(|&id| distances[id]).collect()
        })
        .unwrap();
        std::hint::black_box(selected);
        timings.push(begin.elapsed().as_secs_f64() * 1000.0);
    }
    timings.sort_by(f64::total_cmp);
    println!(
        "65536节点四邻接图: 一次起点查询+最多八次区域查询+区域排序, 15次中位={:.3}ms, 最大={:.3}ms",
        timings[7], timings[14]
    );
}
