//! 二维与三维共用的有限区域访问顺序；地图拥有方负责全部通行语义。
#![deny(unsafe_code)]
#![warn(missing_docs)]

pub mod observation;

use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap};

/// 一次图查询得到的可达候选。代价使用同一安全图的路径长度（米）。
#[derive(Clone, Debug)]
pub struct Candidate {
    /// 真实接近点或支撑点，不能填未知前沿本身。
    pub position: [f64; 3],
    /// 非负观测收益；同一空间区域使用一个代表值，避免重复累加重叠视点。
    pub gain: f64,
    /// 从当前机器人到该点的实际路径代价。
    pub travel_cost: f64,
}

/// 区域顺序只用于下一目标选择，不能作为运动路径。
#[derive(Clone, Debug)]
pub struct Selection {
    /// 等待保护实际选择该区域时的决策年龄；普通路线选择为零。
    pub priority_wait_rounds: u64,
    /// 输入候选数组的下标。
    pub index: usize,
    /// 本轮有限区域访问顺序，各项仍是输入候选下标。
    pub regional_order: Vec<usize>,
    /// 完整访问顺序的路径代价。
    pub route_cost: f64,
    /// 输入中实际有效的区域总数，可以大于本轮参与排序的上限。
    pub region_count: usize,
}

/// 有限图上的一次到多点最短路径；没有目标列表时遍历全部可达节点。
/// 边必须由调用方检查轮廓、方向、禁区与边界，非有限或负权边不参与。
/// 输出以节点下标索引，未到达节点为无穷；指定目标时不保证其它节点已收敛。
pub fn shortest_costs(
    node_count: usize,
    start: usize,
    targets: Option<&[usize]>,
    mut edges: impl FnMut(usize, &mut Vec<(usize, f64)>),
) -> Vec<f64> {
    let mut distances = vec![f64::INFINITY; node_count];
    if start >= node_count {
        return distances;
    }
    let mut pending = targets.map(|targets| {
        let mut pending = std::collections::BTreeSet::new();
        pending.extend(targets.iter().copied().filter(|&v| v < node_count));
        pending
    });
    let mut queue = BinaryHeap::new();
    distances[start] = 0.0;
    queue.push(Visit(0.0, start));
    let mut buffer = Vec::with_capacity(8);
    while let Some(Visit(cost, node)) = queue.pop() {
        if cost > distances[node] {
            continue;
        }
        if let Some(pending) = pending.as_mut() {
            pending.remove(&node);
            if pending.is_empty() {
                break;
            }
        }
        buffer.clear();
        edges(node, &mut buffer);
        for &(next, edge) in &buffer {
            if next >= node_count || !edge.is_finite() || edge < 0.0 {
                continue;
            }
            let next_cost = cost + edge;
            if next_cost < distances[next] {
                distances[next] = next_cost;
                queue.push(Visit(next_cost, next));
            }
        }
    }
    distances
}

#[derive(PartialEq)]
struct Visit(f64, usize);
impl Eq for Visit {}
impl PartialOrd for Visit {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for Visit {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .0
            .total_cmp(&self.0)
            .then_with(|| other.1.cmp(&self.1))
    }
}

/// 将候选压缩为四米平面、一米高度区域，再求至多八区域的最小加权到达代价。
/// 每个区域保留从当前点看收益/代价最好的代表；未入选区域下次仍重新参与。
/// `costs`接收一个候选下标和一批目标下标，必须返回有向图上的真实成本，
/// 不可达为无穷。查询失败必须返回无穷，不能回退为直线距离。
pub fn select(
    candidates: &[Candidate],
    costs: impl FnMut(usize, &[usize]) -> Vec<f64>,
) -> Option<Selection> {
    select_with_priority(candidates, None, None, costs)
}

fn region_key(p: [f64; 3]) -> [i64; 3] {
    [
        (p[0] / 4.0).floor() as i64,
        (p[1] / 4.0).floor() as i64,
        p[2].floor() as i64,
    ]
}
fn valid_candidate(c: &Candidate) -> bool {
    c.position.iter().all(|v| v.is_finite() && v.abs() < 1e12)
        && c.gain.is_finite()
        && c.gain > 0.0
        && c.travel_cost.is_finite()
        && c.travel_cost >= 0.0
}

/// 同一探索会话的区域等待账本；地图/任务切换必须随原会话一起重置。
#[derive(Clone, Debug, Default)]
pub struct RegionalSchedule {
    round: u64,
    last_served: BTreeMap<[i64; 3], u64>,
}
// JSON对象键不能表示三维数组，持久账本使用有界键值列表并检查重复。
impl serde::Serialize for RegionalSchedule {
    fn serialize<S:serde::Serializer>(&self,serializer:S)->Result<S::Ok,S::Error> {
        serde::Serialize::serialize(&(self.round,self.last_served.iter().map(|(k,v)|(*k,*v)).collect::<Vec<_>>()),serializer)
    }
}
impl<'de> serde::Deserialize<'de> for RegionalSchedule {
    fn deserialize<D:serde::Deserializer<'de>>(deserializer:D)->Result<Self,D::Error> {
        let (round,entries):(u64,Vec<([i64;3],u64)>)=serde::Deserialize::deserialize(deserializer)?;
        if entries.len()>65536 || entries.iter().any(|(key,value)|*value>round || key.iter().any(|n|n.unsigned_abs()>1_000_000_000_000)) {
            return Err(serde::de::Error::custom("区域进度超出边界"));
        }
        let len=entries.len();let last_served:BTreeMap<_,_>=entries.into_iter().collect();
        if len!=last_served.len(){return Err(serde::de::Error::custom("区域进度有重复键"));}
        Ok(Self {round,last_served})
    }
}
impl RegionalSchedule {
    /// 该会话已确认的有效目标终态数量。
    pub fn completed_rounds(&self) -> u64 {
        self.round
    }
    /// 清除该会话的区域状态，不改变任何目标或运动状态。
    pub fn reset(&mut self) {
        *self = Self::default();
    }
    /// 仅在原目标收到匹配的到达/失败反馈时调用；重复轮询和取消不算服务一次。
    pub fn record_outcome(&mut self, position: [f64; 3]) {
        if position.iter().any(|v| !v.is_finite() || v.abs() >= 1e12) {
            return;
        }
        self.round = self.round.saturating_add(1);
        self.last_served.insert(region_key(position), self.round);
    }
    /// 连续可达且等待至少八个目标终态的最老区域优先进入代表集。
    /// 在不损失该代表集最大可访问收益时优先从它出发，不能为了年龄越过有向限制。
    /// 一次查询只维护当前有效区域，内存不会随全部历史地图无限增长。
    pub fn select(
        &mut self,
        candidates: &[Candidate],
        costs: impl FnMut(usize, &[usize]) -> Vec<f64>,
    ) -> Option<Selection> {
        self.select_observed(candidates, None, costs)
    }

    /// 标定视域给出地图单元集合时，路线收益按并集计数，重叠视点不重复累计。
    pub fn select_observed(
        &mut self,
        candidates: &[Candidate],
        observations: Option<&[std::collections::BTreeSet<[i32; 3]>]>,
        costs: impl FnMut(usize, &[usize]) -> Vec<f64>,
    ) -> Option<Selection> {
        if observations.is_some_and(|sets| sets.len() != candidates.len()) {
            return None;
        }
        let present: std::collections::BTreeSet<_> = candidates
            .iter()
            .filter(|c| valid_candidate(c))
            .map(|c| region_key(c.position))
            .collect();
        self.last_served.retain(|key, _| present.contains(key));
        for key in present {
            self.last_served.entry(key).or_insert(self.round);
        }
        let priority = self
            .last_served
            .iter()
            .filter(|(_, served)| self.round.saturating_sub(**served) >= 8)
            .min_by_key(|(key, served)| (**served, **key))
            .map(|(&key, &served)| (key, self.round.saturating_sub(served)));
        let mut result = select_with_priority(
            candidates,
            priority.map(|(key, _)| key),
            observations,
            costs,
        )?;
        if let Some((key, wait)) = priority {
            if region_key(candidates[result.index].position) == key {
                result.priority_wait_rounds = wait;
            }
        }
        Some(result)
    }
}

fn select_with_priority(
    candidates: &[Candidate],
    priority: Option<[i64; 3]>,
    observations: Option<&[std::collections::BTreeSet<[i32; 3]>]>,
    mut costs: impl FnMut(usize, &[usize]) -> Vec<f64>,
) -> Option<Selection> {
    let utility = |i: usize| candidates[i].gain / (1.0 + candidates[i].travel_cost);
    let compare = |a: usize, b: usize| {
        utility(b)
            .total_cmp(&utility(a))
            .then_with(|| candidates[a].position[0].total_cmp(&candidates[b].position[0]))
            .then_with(|| candidates[a].position[1].total_cmp(&candidates[b].position[1]))
            .then_with(|| candidates[a].position[2].total_cmp(&candidates[b].position[2]))
    };
    let mut regions = BTreeMap::new();
    for (i, candidate) in candidates
        .iter()
        .enumerate()
        .filter(|(i, c)| valid_candidate(c) && observations.is_none_or(|sets| !sets[*i].is_empty()))
    {
        regions
            .entry(region_key(candidate.position))
            .and_modify(|best: &mut usize| {
                if compare(i, *best).is_lt() {
                    *best = i;
                }
            })
            .or_insert(i);
    }
    let preferred = priority.and_then(|key| regions.get(&key).copied());
    let region_count = regions.len();
    let mut representatives: Vec<_> = regions.into_values().collect();
    representatives.sort_by(|&a, &b| compare(a, b));
    representatives.truncate(8);
    if let Some(preferred) = preferred {
        if !representatives.contains(&preferred) {
            *representatives
                .last_mut()
                .expect("等待区域属于当前有效集合") = preferred;
        }
    }
    let n = representatives.len();
    if n == 0 {
        return None;
    }
    if n == 1 {
        let index = representatives[0];
        return Some(Selection {
            priority_wait_rounds: 0,
            index,
            regional_order: vec![index],
            route_cost: candidates[index].travel_cost,
            region_count,
        });
    }
    let matrix: Vec<Vec<f64>> = representatives
        .iter()
        .map(|&source| {
            let row = costs(source, &representatives);
            (0..n)
                .map(|j| {
                    row.get(j)
                        .copied()
                        .filter(|v| v.is_finite() && *v >= 0.0)
                        .unwrap_or(f64::INFINITY)
                })
                .collect()
        })
        .collect();
    // 动态规划枚举访问集合与末端。延伸一条边时，尚未访问的收益都增加同样的等待。
    // 最多256×8个状态；先最大化可访问收益，再最小化加权到达代价。
    let max_gain = representatives
        .iter()
        .map(|&i| candidates[i].gain)
        .fold(0.0, f64::max);
    let gains: Vec<_> = representatives
        .iter()
        .map(|&i| candidates[i].gain / max_gain)
        .collect();
    let mut total_gain: f64 = gains.iter().sum();
    let count = 1usize << n;
    let mut gain = vec![0.0; count];
    for mask in 1..count {
        let i = mask.trailing_zeros() as usize;
        gain[mask] = gain[mask & !(1 << i)] + gains[i];
    }
    if let Some(observations) = observations {
        let mut membership = BTreeMap::<[i32; 3], usize>::new();
        for (i, &candidate) in representatives.iter().enumerate() {
            for &cell in &observations[candidate] {
                *membership.entry(cell).or_default() |= 1 << i;
            }
        }
        let mut histogram = vec![0.0; count];
        for mask in membership.into_values() {
            histogram[mask] += 1.0;
        }
        let scale = histogram.iter().sum::<f64>().max(1.0);
        for mask in 0..count {
            gain[mask] = histogram
                .iter()
                .enumerate()
                .filter(|(members, _)| members & mask != 0)
                .map(|(_, count)| count / scale)
                .sum();
        }
        total_gain = gain[count - 1];
    }
    let preferred = preferred.and_then(|index| representatives.iter().position(|&i| i == index));
    // 比任一有限路线的全部加权到达代价都大的惩罚，只改变同一可访问收益下的起点顺序。
    let max_edge = matrix
        .iter()
        .flatten()
        .copied()
        .filter(|v| v.is_finite())
        .chain(representatives.iter().map(|&i| candidates[i].travel_cost))
        .fold(0.0, f64::max);
    let penalty = max_edge * (n as f64 + 1.0) * total_gain + 1.0;
    let penalty = if penalty.is_finite() { penalty } else { 0.0 };
    let mut dp = vec![f64::INFINITY; count * n];
    let mut previous = vec![usize::MAX; count * n];
    for i in 0..n {
        dp[(1 << i) * n + i] = candidates[representatives[i]].travel_cost * total_gain
            + if preferred.is_some_and(|preferred| preferred != i) {
                penalty
            } else {
                0.0
            };
    }
    for mask in 1..count {
        for last in 0..n {
            if mask & (1 << last) == 0 || !dp[mask * n + last].is_finite() {
                continue;
            }
            for next in 0..n {
                if mask & (1 << next) != 0 {
                    continue;
                }
                let value =
                    dp[mask * n + last] + matrix[last][next] * (total_gain - gain[mask]).max(0.0);
                let index = (mask | (1 << next)) * n + next;
                if value < dp[index] {
                    dp[index] = value;
                    previous[index] = last;
                }
            }
        }
    }
    let mut best: Option<(usize, usize)> = None;
    for mask in 1..count {
        for last in 0..n {
            if !dp[mask * n + last].is_finite() {
                continue;
            }
            if best.is_none_or(|(m, l)| {
                gain[mask] > gain[m] + 1e-12
                    || ((gain[mask] - gain[m]).abs() <= 1e-12
                        && (dp[mask * n + last] < dp[m * n + l] - 1e-12
                            || ((dp[mask * n + last] - dp[m * n + l]).abs() <= 1e-12
                                && mask.count_ones() < m.count_ones())))
            }) {
                best = Some((mask, last));
            }
        }
    }
    let (mut mask, mut last) = best?;
    let mut order = Vec::new();
    loop {
        order.push(last);
        let prev = previous[mask * n + last];
        if prev == usize::MAX {
            break;
        }
        mask &= !(1 << last);
        last = prev;
    }
    order.reverse();
    let route_cost = candidates[representatives[order[0]]].travel_cost
        + order
            .windows(2)
            .map(|pair| matrix[pair[0]][pair[1]])
            .sum::<f64>();
    let regional_order: Vec<_> = order.iter().map(|&i| representatives[i]).collect();
    Some(Selection {
        priority_wait_rounds: 0,
        index: regional_order[0],
        regional_order,
        route_cost,
        region_count,
    })
}

#[cfg(test)]
mod tests {
    #[test]
    fn overlapping_viewpoints_do_not_require_visiting_the_same_information_twice() {
        use std::collections::BTreeSet;
        let candidates = vec![
            Candidate {
                position: [0.0, 0.0, 0.0],
                gain: 2.0,
                travel_cost: 1.0,
            },
            Candidate {
                position: [4.0, 0.0, 0.0],
                gain: 2.0,
                travel_cost: 1.0,
            },
            Candidate {
                position: [8.0, 0.0, 0.0],
                gain: 1.0,
                travel_cost: 1.0,
            },
        ];
        let cells = vec![
            BTreeSet::from([[1, 0, 0], [2, 0, 0]]),
            BTreeSet::from([[1, 0, 0], [2, 0, 0]]),
            BTreeSet::from([[1, 0, 30]]),
        ];
        let selection = RegionalSchedule::default()
            .select_observed(&candidates, Some(&cells), |_, targets| {
                vec![1.0; targets.len()]
            })
            .unwrap();
        assert_eq!(selection.regional_order.len(), 2);
        assert!(
            selection.regional_order.contains(&2),
            "另一高度的观察单元必须独立计入"
        );
        let union: BTreeSet<_> = selection
            .regional_order
            .iter()
            .flat_map(|&i| cells[i].iter().copied())
            .collect();
        assert_eq!(union.len(), 3);
    }

    #[test]
    fn persistent_low_gain_regions_are_served_and_polling_alone_does_not_age() {
        let candidates: Vec<_> = (0..20)
            .map(|i| Candidate {
                position: [4.0 * i as f64, 0.0, 0.0],
                gain: if i == 0 { 1000.0 } else { 1.0 },
                travel_cost: 1.0 + i as f64,
            })
            .collect();
        let costs = |source: usize, targets: &[usize]| {
            targets
                .iter()
                .map(|&i| (candidates[source].position[0] - candidates[i].position[0]).abs())
                .collect()
        };
        let mut schedule = RegionalSchedule::default();
        for _ in 0..100 {
            assert_eq!(
                schedule
                    .select(&candidates, costs)
                    .unwrap()
                    .priority_wait_rounds,
                0
            );
        }
        assert_eq!(schedule.completed_rounds(), 0);
        for _ in 0..8 {
            schedule.record_outcome(candidates[0].position);
        }
        let mut visited = std::collections::BTreeSet::new();
        for _ in 0..19 {
            let result = schedule.select(&candidates, costs).unwrap();
            assert!(result.priority_wait_rounds >= 8);
            visited.insert(result.index);
            schedule.record_outcome(candidates[result.index].position);
        }
        assert_eq!(
            visited,
            (1..20).collect(),
            "持续低收益区域不能被高收益新视点永久挤出"
        );
        schedule.reset();
        assert_eq!(schedule.completed_rounds(), 0);
        assert_eq!(
            schedule
                .select(&candidates, costs)
                .unwrap()
                .priority_wait_rounds,
            0
        );
    }

    #[test]
    fn waiting_priority_does_not_take_a_one_way_exit_before_other_reachable_regions() {
        let candidates: Vec<_> = (0..3)
            .map(|i| Candidate {
                position: [4.0 * i as f64, 0.0, 0.0],
                gain: 1.0,
                travel_cost: i as f64 + 1.0,
            })
            .collect();
        let matrix = [
            [0.0, f64::INFINITY, f64::INFINITY],
            [2.0, 0.0, 1.0],
            [3.0, 1.0, 0.0],
        ];
        let costs =
            |source: usize, targets: &[usize]| targets.iter().map(|&i| matrix[source][i]).collect();
        let mut schedule = RegionalSchedule::default();
        schedule.select(&candidates, costs).unwrap();
        for _ in 0..8 {
            schedule.record_outcome(candidates[1].position);
        }
        let selected = schedule.select(&candidates, costs).unwrap();
        assert_eq!(selected.index, 1);
        assert_eq!(selected.regional_order, [1, 2, 0]);
        assert_eq!(selected.priority_wait_rounds, 0);
    }

    #[test]
    fn disappeared_or_invalid_regions_do_not_leave_unbounded_or_inherited_age() {
        let candidate = Candidate {
            position: [0.0, 0.0, 0.0],
            gain: 1.0,
            travel_cost: 1.0,
        };
        let mut schedule = RegionalSchedule::default();
        schedule
            .select(std::slice::from_ref(&candidate), |_, _| vec![])
            .unwrap();
        for _ in 0..10 {
            schedule.record_outcome([8.0, 0.0, 0.0]);
        }
        assert!(schedule.select(&[], |_, _| vec![]).is_none());
        assert!(schedule.last_served.is_empty());
        assert_eq!(
            schedule
                .select(&[candidate], |_, _| vec![])
                .unwrap()
                .priority_wait_rounds,
            0
        );
    }

    use super::*;
    #[test]
    fn directed_paths_do_not_invent_reverse_edges_or_cross_invalid_edges() {
        let graph = [
            vec![(1, 3.0), (2, f64::NAN), (3, -1.0)],
            vec![(2, 4.0)],
            vec![],
            vec![],
        ];
        let d = shortest_costs(4, 0, None, |i, out| out.extend_from_slice(&graph[i]));
        assert_eq!(d, vec![0.0, 3.0, 7.0, f64::INFINITY]);
        let d = shortest_costs(4, 2, Some(&[0]), |i, out| out.extend_from_slice(&graph[i]));
        assert!(d[0].is_infinite());
    }
    #[test]
    fn route_order_preserves_a_one_way_exit_until_other_regions_are_visited() {
        let candidates = vec![
            Candidate {
                position: [0.0, 0.0, 0.0],
                gain: 1.0,
                travel_cost: 1.0,
            },
            Candidate {
                position: [4.0, 0.0, 0.0],
                gain: 1.0,
                travel_cost: 2.0,
            },
            Candidate {
                position: [8.0, 0.0, 0.0],
                gain: 1.0,
                travel_cost: 3.0,
            },
        ];
        let matrix = [
            [0.0, f64::INFINITY, f64::INFINITY],
            [2.0, 0.0, 1.0],
            [3.0, 1.0, 0.0],
        ];
        let result = select(&candidates, |i, goals| {
            goals.iter().map(|&j| matrix[i][j]).collect()
        })
        .unwrap();
        assert_eq!(result.index, 1);
        assert_eq!(result.regional_order, [1, 2, 0]);
        assert_eq!(result.route_cost, 6.0);
    }
    #[test]
    fn overlapping_layers_are_separate_and_duplicate_views_do_not_multiply_gain() {
        let mut candidates = vec![
            Candidate {
                position: [1.0, 1.0, 0.0],
                gain: 1.0,
                travel_cost: 1.0
            };
            100
        ];
        candidates.push(Candidate {
            position: [1.0, 1.0, 2.7],
            gain: 2.0,
            travel_cost: 4.0,
        });
        let result = select(&candidates, |_, goals| vec![5.0; goals.len()]).unwrap();
        assert_eq!(result.region_count, 2);
        assert_eq!(result.regional_order.len(), 2);
    }
    #[test]
    fn regional_search_is_bounded_and_order_is_independent_of_input_order() {
        let mut candidates: Vec<_> = (0..30)
            .map(|i| Candidate {
                position: [4.0 * i as f64, 0.0, 0.0],
                gain: 1.0,
                travel_cost: i as f64 + 1.0,
            })
            .collect();
        let choose = |c: &[Candidate]| {
            select(c, |i, goals| {
                goals
                    .iter()
                    .map(|&j| (c[i].position[0] - c[j].position[0]).abs())
                    .collect()
            })
            .unwrap()
        };
        let first = choose(&candidates);
        assert_eq!(first.region_count, 30);
        assert_eq!(first.regional_order.len(), 8);
        let position = candidates[first.index].position;
        candidates.reverse();
        let second = choose(&candidates);
        assert_eq!(position, candidates[second.index].position);
    }
}
