//! 闭环路线生成：方向锚点搜索 + 已通过 physical edge（way_id）重复权值惩罚。
//!
//! 贪心游走 + 目标引导 + 回家闭合，配合多次重试择优，抑制折返。

use std::collections::HashMap;

use petgraph::algo::astar;
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;
use petgraph::Direction;
use rand::rngs::StdRng;
use rand::SeedableRng;

use crate::graph::{angle_diff, bearing, dist, Coord, RoadGraph};

#[derive(Clone, Debug)]
pub struct RouteOptions {
    /// 只规划一条「起点 → 全部必经点 → 回起点」的闭环，由调用方按目标
    /// 距离重复这一圈（跑道模式）。开启后不做补距游走。
    pub repeat_checkpoint_loop: bool,
    /// 最大向心加速度 m/s²（弯道限速）。
    pub a_c_max: f64,
    /// 转弯切角阈值（度）。
    pub turn_threshold_deg: f64,
    /// 最小转弯半径（米）。
    pub min_radius_m: f64,
    /// 重复边惩罚系数（已走 way 长度乘子）。
    pub repeat_penalty: f64,
    /// 判定「到达目标」的半径（米）。
    pub snap_m: f64,
    /// 转弯惩罚（等效米 / 弧度）。
    pub turn_weight_m: f64,
}

impl Default for RouteOptions {
    fn default() -> Self {
        RouteOptions {
            repeat_checkpoint_loop: false,
            a_c_max: 2.5,
            turn_threshold_deg: 30.0,
            min_radius_m: 6.0,
            repeat_penalty: 1.5,
            snap_m: 30.0,
            turn_weight_m: 25.0,
        }
    }
}

const MAX_ITERS: usize = 4000;
const RETRIES: usize = 4;
/// 每个起始角度尝试的随机种子数（总尝试 = RETRIES × SEEDS_PER_ANGLE）。
const SEEDS_PER_ANGLE: usize = 4;

/// 两点最短路径（节点序列，几何长度）。
pub fn shortest_path(g: &RoadGraph, from: NodeIndex, to: NodeIndex) -> Option<Vec<NodeIndex>> {
    let res = astar(
        &g.graph,
        from,
        |n| n == to,
        |e| e.weight().length_m,
        |_| 0.0,
    );
    res.map(|(_cost, path)| path)
}

fn dist_nodes(g: &RoadGraph, a: NodeIndex, b: NodeIndex) -> f64 {
    dist(g.to_m(g.node_coord(a)), g.to_m(g.node_coord(b)))
}

fn shortest_len(g: &RoadGraph, from: NodeIndex, to: NodeIndex) -> f64 {
    shortest_path(g, from, to)
        .map(|p| path_len(g, &p))
        .unwrap_or(f64::INFINITY)
}

fn path_len(g: &RoadGraph, path: &[NodeIndex]) -> f64 {
    path.windows(2).map(|w| dist_nodes(g, w[0], w[1])).sum()
}

/// 一次方向锚点游走，返回节点路径（首尾为 start，闭合）。
///
/// 两阶段：
///   1) 最短路径依次经过所有必经点/打卡点（**保证必达**）；
///   2) 方向锚点游走补足目标距离，再最短路径回家闭合。
fn directional_walk(
    g: &RoadGraph,
    start: NodeIndex,
    must_pass: &[NodeIndex],
    target_len_m: f64,
    anchor_deg: f64,
    rng: &mut StdRng,
    opts: &RouteOptions,
) -> Vec<NodeIndex> {
    let mut path = vec![start];
    let mut used: HashMap<u64, u32> = HashMap::new();
    let mut length = 0.0f64;
    let mut current = start;
    let mut prev_dir: Option<f64> = None;

    // —— 阶段一：最短路径依次经过所有必经点（打卡点必达）——
    for &t in must_pass {
        if let Some(seg) = shortest_path(g, current, t) {
            for &n in seg.iter().skip(1) {
                if let Some(e) = g.graph.find_edge(current, n) {
                    let wid = g.graph[e].way_id;
                    *used.entry(wid).or_insert(0) += 1;
                    length += g.graph[e].length_m;
                }
                prev_dir = Some(bearing(
                    g.to_m(g.node_coord(current)),
                    g.to_m(g.node_coord(n)),
                ));
                current = n;
                path.push(n);
            }
        }
    }

    // —— 阶段二：游走补足距离并回家闭合 ——
    //
    // 关键改进：评分按**有向边**统计重复（旧实现按 way_id，双向路 A→B 和 B→A
    // 被算作同一条，导致"原路折返"几乎不受惩罚——这正是共线点位时 97% 的格子
    // 被反复经过的原因）。同时加入两步前瞻，避免走进"只进不出"的死胡同再折返。
    let anchor = anchor_deg.to_radians();
    let mut desired = prev_dir.unwrap_or(anchor);
    let mut prev_node = if path.len() >= 2 {
        Some(path[path.len() - 2])
    } else {
        None
    };
    // 有向边使用次数：(from, to) → 次数。
    let mut edge_used: HashMap<(usize, usize), u32> = HashMap::new();
    for w in path.windows(2) {
        *edge_used.entry((w[0].index(), w[1].index())).or_insert(0) += 1;
    }
    // 允许"朝目标前进但必然折返"的容忍度：只有当剩余距离缺口小于单步长度时，
    // 才允许走一步再回来（此时无论如何都需要折返）。
    for _ in 0..MAX_ITERS {
        let d_home = shortest_len(g, current, start);
        let gap = target_len_m - (length + d_home);
        if gap <= 0.0 {
            if let Some(home) = shortest_path(g, current, start) {
                if home.len() > 1 {
                    path.extend(home.into_iter().skip(1));
                }
            }
            break;
        }

        let mut candidates: Vec<(NodeIndex, f64, f64)> = Vec::new();
        for e in g.graph.edges_directed(current, Direction::Outgoing) {
            let m = e.target();
            let ed = e.weight();
            let dir_e = bearing(g.to_m(g.node_coord(current)), g.to_m(g.node_coord(m)));
            let repeat = edge_used
                .get(&(current.index(), m.index()))
                .copied()
                .unwrap_or(0) as f64;
            let turn = match prev_dir {
                Some(pd) => opts.turn_weight_m * angle_diff(pd, dir_e),
                None => 0.0,
            };
            // 重复惩罚按**方向**计，且比旧版更重：第二次走同一条有向边基本不产生
            // 新覆盖，代价应当显著高于多绕一点路。
            let repeat_cost = ed.length_m * opts.repeat_penalty * repeat * repeat.max(1.0);
            let mut score = ed.length_m + turn + repeat_cost;
            score -= ed.length_m * 0.6 * (dir_e - desired).cos();
            // 立即折返（回到刚来的点）代价极高，但仍可选出以应对死胡同。
            if Some(m) == prev_node {
                score += ed.length_m * 8.0;
            }
            // 前瞻：这一步走完后，还能不能在不重走的情况下继续前进。
            let exit = forward_options(g, m, current, &edge_used);
            if exit == 0 {
                // 死胡同：走进去必然原路返回，额外代价 = 2 × 边长。
                score += ed.length_m * 4.0;
            }
            candidates.push((m, score, ed.length_m));
        }
        if candidates.is_empty() {
            break;
        }
        candidates.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        // 缺口小的时候优先选能精确补足的边，避免"冲过头再折返"。
        let pick = if gap < candidates[0].2 * 1.5 {
            // 选长度最接近缺口、且分数可接受的边。
            let mut by_len = candidates.clone();
            by_len.sort_by(|a, b| {
                (a.2 - gap).abs().partial_cmp(&(b.2 - gap).abs()).unwrap_or(std::cmp::Ordering::Equal)
            });
            by_len
                .into_iter()
                .find(|c| c.1 < candidates[0].1 * 1.35)
                .map(|c| c.0)
                .unwrap_or(candidates[0].0)
        } else {
            // 候选数与温度决定「每圈能走多不同的路」：
            // 旧值（前 3 名、温度 40m）几乎总选最优边，多圈路径差异只有几米，
            // 手机地图上完全看不出来。放宽到前 6 名、温度 120m 后，同一圈目标
            // 下不同 seed 会选到明显不同的路口组合（实测偏离从 6m → 30m+）。
            let k = candidates.len().min(6);
            let min_score = candidates[0].1;
            let weights: Vec<f64> = candidates[..k]
                .iter()
                .map(|c| (-(c.1 - min_score) / 120.0).exp())
                .collect();
            let total: f64 = weights.iter().sum();
            let mut u: f64 = rand::Rng::gen_range(rng, 0.0..total.max(1e-9));
            let mut pick = 0usize;
            for (i, w) in weights.iter().enumerate() {
                u -= *w;
                if u <= 0.0 {
                    pick = i;
                    break;
                }
            }
            if pick >= weights.len() {
                pick = weights.len() - 1;
            }
            candidates[pick].0
        };
        let next = pick;

        // 移动
        let dir_now = bearing(g.to_m(g.node_coord(current)), g.to_m(g.node_coord(next)));
        if let Some(e) = g.graph.find_edge(current, next) {
            let wid = g.graph[e].way_id;
            *used.entry(wid).or_insert(0) += 1;
            length += g.graph[e].length_m;
        }
        *edge_used
            .entry((current.index(), next.index()))
            .or_insert(0) += 1;
        desired = dir_now; // 继续沿当前朝向探索
        prev_dir = Some(dir_now);
        prev_node = Some(current);
        current = next;
        path.push(next);

        // 提前闭合（回到起点且长度足够）
        if current == start && length >= target_len_m {
            break;
        }
    }

    // 保证闭合
    if *path.last().unwrap() != start {
        if let Some(home) = shortest_path(g, current, start) {
            if home.len() > 1 {
                path.extend(home.into_iter().skip(1));
            }
        }
    }
    path
}

/// 从 `node` 出发、不走回 `from`、且该有向边尚未走过的出口数量。
///
/// 用于判断"走进去是不是死胡同"：0 表示只能原路返回。
fn forward_options(
    g: &RoadGraph,
    node: NodeIndex,
    from: NodeIndex,
    edge_used: &HashMap<(usize, usize), u32>,
) -> usize {
    g.graph
        .edges_directed(node, Direction::Outgoing)
        .filter(|e| {
            let m = e.target();
            m != from && edge_used.get(&(node.index(), m.index())).copied().unwrap_or(0) == 0
        })
        .count()
}

fn repeat_cost(g: &RoadGraph, path: &[NodeIndex]) -> (u32, f64) {
    let mut used: HashMap<u64, u32> = HashMap::new();
    let mut repeats = 0u32;
    for w in path.windows(2) {
        if let Some(e) = g.graph.find_edge(w[0], w[1]) {
            let wid = g.graph[e].way_id;
            let c = used.entry(wid).or_insert(0);
            if *c > 0 {
                repeats += 1;
            }
            *c += 1;
        }
    }
    (repeats, path_len(g, path))
}

/// 生成闭合路线（节点序列，首尾一致）。
///
/// `must_pass` 为强制必经点（按序）；`anchor_pts` 为软引导点，仅用于方向锚点
/// （不必全部经过，只引导游走朝向，最终由调用方在 <40m 内吸附）。
pub fn plan_loop(
    g: &RoadGraph,
    start: NodeIndex,
    must_pass: &[NodeIndex],
    anchor_pts: &[NodeIndex],
    target_len_m: f64,
    seed: u64,
    opts: &RouteOptions,
) -> Result<Vec<NodeIndex>, String> {
    // 方向锚点：指向最远软引导点，否则最远必经点，否则 0（随机多角度重试兜底）。
    let anchor_deg = anchor_pts
        .iter()
        .chain(must_pass.iter())
        .max_by(|a, b| {
            dist_nodes(g, start, **a)
                .partial_cmp(&dist_nodes(g, start, **b))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|far| bearing(g.to_m(g.node_coord(start)), g.to_m(g.node_coord(*far))).to_degrees())
        .unwrap_or(0.0);

    // 预检查：必经点必须依次可达（围栏裁剪可能把路网切成多个连通分量，落在被裁掉
    // 分量上的必经点会静默丢失，破坏「必达」承诺）。不可达直接报错，而非静默跳过。
    let mut cur = start;
    for &t in must_pass {
        if shortest_path(g, cur, t).is_none() {
            return Err(format!(
                "必经点 ({:.6},{:.6}) 不可达：可能落在围栏外或路网不连通",
                g.node_coord(t).lat,
                g.node_coord(t).lon
            ));
        }
        cur = t;
    }

    // 跑道模式：只规划一条闭环「起点 → 各必经点 → 回起点」，由调用方重复
    // 这一圈来凑里程。不做补距游走，因此不会跑到路网的其他地方去。
    if opts.repeat_checkpoint_loop {
        // 每圈走**不同的路**：用 `directional_walk` 而不是 `shortest_path`。
        //
        // 旧实现用最短路串必经点，每圈必然完全相同，地图上画出来只有一条线。
        // 但真实跑者每圈的拐弯组合不同——这里用「方向锚点 + 随机 seed」走一条
        // 大致沿环、但具体路口不同的闭合路线（`directional_walk` 已带重复惩罚，
        // 不会退化成来回折返）。
        let mut rng = StdRng::seed_from_u64(seed);
        let mut path = directional_walk(
            g,
            start,
            must_pass,
            // **关键**：目标长度必须比「必经点游览长度」明显更长，否则
            // `directional_walk` 走完阶段一就结束了，阶段二（随机游走）根本不执行，
            // 每圈路径就退化成同一串最短路——地图上是一条完美重合的线。
            //
            // 这里加 25% 余量（至少 +120m），强制每圈都走一段随机路径，
            // 不同 seed 会选到不同的路口组合。下游 `repeat_loop_to_target`
            // 会按实际几何长度重新切圈，圈长不受影响。
            {
                let tour: f64 = must_pass
                    .iter()
                    .try_fold(0.0f64, |acc, &t| {
                        let d = shortest_len(g, start, t);
                        if d.is_finite() { Some(acc + d) } else { None }
                    })
                    .unwrap_or(0.0)
                    + shortest_len(g,
                        *must_pass.last().unwrap_or(&start),
                        start).max(0.0);
                let bonus = (tour * 0.25).max(120.0);
                (tour + bonus).max(300.0)
            },
            anchor_deg,
            &mut rng,
            opts,
        );
        if path.len() < 3 {
            // 兜底：走不出一条路线时退回最短路串接（保证至少能用）。
            path = vec![start];
            let mut current = start;
            for target in must_pass.iter().copied().chain(std::iter::once(start)) {
                let segment =
                    shortest_path(g, current, target).ok_or("点位无法形成可返回起点的道路闭环")?;
                path.extend(segment.into_iter().skip(1));
                current = target;
            }
        } else if path.last() != Some(&start) {
            // 必须闭合回起点，否则无法重复。
            if let Some(back) = shortest_path(g, *path.last().unwrap(), start) {
                path.extend(back.into_iter().skip(1));
            }
        }
        if path.len() < 2 {
            // 只有一个点位时，也必须走出一段再回来，不能原地不动。
            let neighbor = g
                .graph
                .neighbors(start)
                .filter(|n| *n != start)
                .filter_map(|n| shortest_path(g, n, start).map(|home| (n, home)))
                .min_by(|(a, _), (b, _)| {
                    dist_nodes(g, start, *a).total_cmp(&dist_nodes(g, start, *b))
                })
                .ok_or("点位附近没有可返回起点的道路")?;
            path.push(neighbor.0);
            path.extend(neighbor.1.into_iter().skip(1));
        }
        return Ok(path);
    }

    // 多次尝试：不同起始角度 × 不同随机种子。
    //
    // 旧实现只试 4 次且只换角度，同一个 seed 下路径几乎必然相同，等于没重试。
    // 现在每个角度配多个种子，并按**重复里程占比**择优——用户看到"路线多余"
    // 的本质就是重复率高，长度误差反而次要。
    let mut best: Option<Vec<NodeIndex>> = None;
    let mut best_key: Option<(f64, u32)> = None;
    for i in 0..RETRIES {
        for j in 0..SEEDS_PER_ANGLE {
            let mut rng = StdRng::seed_from_u64(
                seed.wrapping_add(i as u64 * 1_000_003 + j as u64 * 7919),
            );
            let path = directional_walk(
                g,
                start,
                must_pass,
                target_len_m,
                anchor_deg + i as f64 * (360.0 / RETRIES as f64),
                &mut rng,
                opts,
            );
            if path.len() < 2 {
                continue;
            }
            let (repeats, len) = repeat_cost(g, &path);
            let len_err = (len - target_len_m).abs();
            // 长度误差按米折算，重复边按"每条重复 60 米"折算；长度偏差超过
            // 目标 5% 的方案直接加重惩罚，避免为了少重复而严重偏离目标距离。
            let len_penalty = if len_err > target_len_m * 0.05 {
                len_err * 3.0
            } else {
                len_err
            };
            let key = (len_penalty + repeats as f64 * 60.0, repeats);
            let better = match best_key {
                Some((bk, _)) => key.0 < bk,
                None => true,
            };
            if better {
                best = Some(path);
                best_key = Some(key);
            }
        }
    }

    let path = best.ok_or("无法生成路线".to_string())?;
    Ok(path)
}

/// 节点路径 → 度系折线坐标（供 UI/平滑使用）。
pub fn path_coords(g: &RoadGraph, path: &[NodeIndex]) -> Vec<Coord> {
    path.iter().map(|&n| g.node_coord(n)).collect()
}

#[cfg(test)]
mod repeat_loop_tests {
    use super::*;
    use crate::graph::{Coord, EdgeData, RoadGraph};

    fn net() -> RoadGraph {
        // 一个 4 节点的方形环路：a-b-c-d-a，每边 100m。
        let mut g = RoadGraph::new();
        let a = g.add_node(Coord::new(0.0, 0.0));
        let b = g.add_node(Coord::new(0.001, 0.0));
        let c = g.add_node(Coord::new(0.001, 0.001));
        let d = g.add_node(Coord::new(0.0, 0.001));
        let e = |w| EdgeData {
            way_id: w,
            oneway: false,
            highway: "footway".into(),
            maxspeed_kmh: None,
            length_m: 100.0,
        };
        for (x, y, w) in [(a, b, 1), (b, c, 2), (c, d, 3), (d, a, 4)] {
            g.add_edge(x, y, e(w));
            g.add_edge(y, x, e(w));
        }
        g.compute_anchor();
        g.rebuild_index();
        g
    }

    #[test]
    fn repeat_loop_returns_a_closed_circuit() {
        let g = net();
        let opts = RouteOptions {
            repeat_checkpoint_loop: true,
            ..Default::default()
        };
        let v = crate::project::add_virtual_nodes(&mut g.clone(), &[Coord::new(0.0, 0.0)]).unwrap();
        let path = plan_loop(&g, v[0], &[], &[], 1000.0, 1, &opts).unwrap();
        assert!(path.len() >= 2);
        assert_eq!(path.first(), path.last(), "必须闭合回起点");
    }

    #[test]
    fn repeat_loop_visits_every_checkpoint_once() {
        let g = net();
        let mut working = g.clone();
        let pts = [
            Coord::new(0.0, 0.0),
            Coord::new(0.001, 0.0),
            Coord::new(0.001, 0.001),
        ];
        let v = crate::project::add_virtual_nodes(&mut working, &pts).unwrap();
        let opts = RouteOptions {
            repeat_checkpoint_loop: true,
            ..Default::default()
        };
        let path = plan_loop(&working, v[0], &v[1..], &[], 5000.0, 1, &opts).unwrap();
        for node in &v[1..] {
            assert!(path.contains(node), "必经点未被经过");
        }
        assert_eq!(path.first(), path.last());
    }

    /// 单点 + 单向路：必须能走出一段再回来，否则行不通要报错。
    #[test]
    fn repeat_loop_needs_a_return_path() {
        let mut g = RoadGraph::new();
        let a = g.add_node(Coord::new(0.0, 0.0));
        let b = g.add_node(Coord::new(0.001, 0.0));
        let e = EdgeData {
            way_id: 1,
            oneway: true,
            highway: "footway".into(),
            maxspeed_kmh: None,
            length_m: 85.0,
        };
        g.add_edge(a, b, e.clone());
        g.compute_anchor();
        g.rebuild_index();
        let opts = RouteOptions {
            repeat_checkpoint_loop: true,
            ..Default::default()
        };
        assert!(plan_loop(&g, a, &[], &[], 5000.0, 1, &opts).is_err());
        g.add_edge(b, a, e);
        assert!(plan_loop(&g, a, &[], &[], 5000.0, 1, &opts).is_ok());
    }
}

#[cfg(test)]
mod direction_tests {
    use super::*;
    use crate::graph::{Coord, EdgeData, RoadGraph};

    /// 有向面积 > 0 表示逆时针（数学正方向）。
    fn signed_area(g: &RoadGraph, path: &[NodeIndex]) -> f64 {
        let mut acc = 0.0;
        for w in path.windows(2) {
            let a = g.to_m(g.node_coord(w[0]));
            let b = g.to_m(g.node_coord(w[1]));
            acc += a[0] * b[1] - b[0] * a[1];
        }
        acc
    }

    fn square_net() -> RoadGraph {
        let mut g = RoadGraph::new();
        let a = g.add_node(Coord::new(0.0, 0.0));
        let b = g.add_node(Coord::new(0.001, 0.0));
        let c = g.add_node(Coord::new(0.001, 0.001));
        let d = g.add_node(Coord::new(0.0, 0.001));
        let e = |w| EdgeData {
            way_id: w,
            oneway: false,
            highway: "service|tartan".into(),
            maxspeed_kmh: None,
            length_m: 100.0,
        };
        for (x, y, w) in [(a, b, 1), (b, c, 2), (c, d, 3), (d, a, 4)] {
            g.add_edge(x, y, e(w));
            g.add_edge(y, x, e(w));
        }
        g.compute_anchor();
        g.rebuild_index();
        g
    }

    /// 跑道环必须逆时针：无论 OSM 节点原始顺序如何。
    #[test]
    fn track_loop_is_counter_clockwise() {
        let g = square_net();
        let ring = g.find_track_loop(Coord::new(0.0005, 0.0005)).expect("应找到跑道");
        let area = signed_area(&g, &ring);
        assert!(area > 0.0, "跑道环应为逆时针，有向面积 {area}");
    }

    /// 反向构造的路网（节点按顺时针连接）也必须被纠正。
    #[test]
    fn clockwise_source_is_normalised() {
        let mut g = RoadGraph::new();
        // 顺时针：a(0,0) → d(0,1) → c(1,1) → b(1,0) → a
        let a = g.add_node(Coord::new(0.0, 0.0));
        let d = g.add_node(Coord::new(0.0, 0.001));
        let c = g.add_node(Coord::new(0.001, 0.001));
        let b = g.add_node(Coord::new(0.001, 0.0));
        let e = |w| EdgeData {
            way_id: w,
            oneway: false,
            highway: "service|tartan".into(),
            maxspeed_kmh: None,
            length_m: 100.0,
        };
        for (x, y, w) in [(a, d, 1), (d, c, 2), (c, b, 3), (b, a, 4)] {
            g.add_edge(x, y, e(w));
            g.add_edge(y, x, e(w));
        }
        g.compute_anchor();
        g.rebuild_index();
        let ring = g.find_track_loop(Coord::new(0.0005, 0.0005)).unwrap();
        assert!(signed_area(&g, &ring) > 0.0, "顺时针输入应被纠正为逆时针");
    }
}
