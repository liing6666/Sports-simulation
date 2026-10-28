//! 路线质量诊断：区分「点位顺序造成的多余里程」与「补足距离造成的重复」。
//!
//! 用户反馈的"路线有多余"通常来自两处：
//!   1. 点位顺序不合理 → 必须绕路才能依次经过（最短哈氏路径 > 自然环序）
//!   2. 补足距离游走 → 在同一条路上来回走（way 重复率高）
//!
//! 这个测试在网格路网上量化两者，作为回归基线。

use route_planner::{Coord, RoadGraph, RouteOptions};

fn edge(way_id: u64, length_m: f64) -> route_planner::graph::EdgeData {
    route_planner::graph::EdgeData {
        way_id,
        oneway: false,
        highway: "residential".into(),
        maxspeed_kmh: None,
        length_m,
    }
}

/// 20x20 网格，间距 50m，双向。
fn grid(n: usize) -> RoadGraph {
    let mut g = RoadGraph::new();
    let step = 0.00045; // ≈50m
    let mut nodes = Vec::new();
    for j in 0..n {
        for i in 0..n {
            nodes.push(g.add_node(Coord::new(
                i as f64 * step,
                j as f64 * step,
            )));
        }
    }
    let mut wid = 1u64;
    for j in 0..n {
        for i in 0..n - 1 {
            let a = nodes[j * n + i];
            let b = nodes[j * n + i + 1];
            g.add_edge(a, b, edge(wid, 50.0));
            g.add_edge(b, a, edge(wid, 50.0));
            wid += 1;
        }
    }
    for j in 0..n - 1 {
        for i in 0..n {
            let a = nodes[j * n + i];
            let b = nodes[(j + 1) * n + i];
            g.add_edge(a, b, edge(wid, 50.0));
            g.add_edge(b, a, edge(wid, 50.0));
            wid += 1;
        }
    }
    g.compute_anchor();
    g.rebuild_index();
    g
}

/// 与 `generate_road::radial_order` 同算法：按质心角排序。
fn radial(points: &[Coord]) -> Vec<Coord> {
    let n = points.len() as f64;
    let cx: f64 = points.iter().map(|p| p.lon).sum::<f64>() / n;
    let cy: f64 = points.iter().map(|p| p.lat).sum::<f64>() / n;
    let mut out = points.to_vec();
    out.sort_by(|a, b| {
        (a.lat - cy)
            .atan2(a.lon - cx)
            .partial_cmp(&(b.lat - cy).atan2(b.lon - cx))
            .unwrap()
    });
    out
}

/// 按给定顺序走完所有点并闭合的最短里程（米）。
fn tour_len(net: &RoadGraph, order: &[Coord]) -> Option<f64> {
    let mut g = net.clone();
    let nodes = route_planner::project::add_virtual_nodes(&mut g, order).ok()?;
    let mut total = 0.0;
    for pair in nodes.windows(2) {
        let path = route_planner::route::shortest_path(&g, pair[0], pair[1])?;
        total += path
            .windows(2)
            .map(|w| {
                let a = g.to_m(g.node_coord(w[0]));
                let b = g.to_m(g.node_coord(w[1]));
                (a[0] - b[0]).hypot(a[1] - b[1])
            })
            .sum::<f64>();
    }
    let back = route_planner::route::shortest_path(&g, *nodes.last()?, nodes[0])?;
    total += back
        .windows(2)
        .map(|w| {
            let a = g.to_m(g.node_coord(w[0]));
            let b = g.to_m(g.node_coord(w[1]));
            (a[0] - b[0]).hypot(a[1] - b[1])
        })
        .sum::<f64>();
    Some(total)
}

#[test]
fn diagnose_extra_distance_sources() {
    let net = grid(20);
    // 模拟一个校园里 5 个打卡点（分散在网格上）
    let points = vec![
        Coord::new(0.0020, 0.0020),
        Coord::new(0.0065, 0.0030),
        Coord::new(0.0070, 0.0075),
        Coord::new(0.0025, 0.0068),
        Coord::new(0.0045, 0.0015),
    ];

    let radial_order = radial(&points);
    let radial_len = tour_len(&net, &radial_order).expect("环序应可走通");

    // 暴力枚举所有顺序（含起点固定为第一个），找真正最短的游览
    let mut best = (f64::INFINITY, Vec::new());
    let rest = &points[1..];
    let mut idx: Vec<usize> = (0..rest.len()).collect();
    permute(&mut idx, 0, &mut |perm| {
        let mut order = vec![points[0]];
        order.extend(perm.iter().map(|&i| rest[i]));
        if let Some(len) = tour_len(&net, &order) {
            if len < best.0 {
                best = (len, order);
            }
        }
    });

    // 补足距离游走：目标 3000m
    let target = 3000.0;
    let route = route_planner::plan_route_split(
        &net,
        &radial_order,
        &radial_order[1..],
        target,
        42,
        &RouteOptions::default(),
    )
    .expect("应能规划");
    let walked = route.length_m;

    // 重复率：同一 way 走了几次
    let repeats = way_repeats(&route.points);

    println!("── 路线质量诊断 ──");
    println!("点位自然环序最短游览: {radial_len:.0} m");
    println!("全排列最优顺序:       {:.0} m", best.0);
    println!("点位顺序多余里程:     {:.0} m", radial_len - best.0);
    println!("补距后实际路线:       {walked:.0} m（目标 {target:.0} m）");
    println!("重复经过的 way 次数:  {repeats}");
    println!("冗余比例:             {:.1}%", (walked / radial_len - 1.0) * 100.0);
    for p in &best.1 {
        println!("  最优序 ({:.5},{:.5})", p.lat, p.lon);
    }

    // 基线断言：环序不应比最优顺序差太多
    assert!(
        radial_len - best.0 < radial_len * 0.5,
        "点位排序产生的多余里程过大: {:.0}m vs 最优 {:.0}m",
        radial_len,
        best.0
    );
}

fn permute(items: &mut Vec<usize>, k: usize, f: &mut impl FnMut(&[usize])) {
    if k == items.len() {
        f(items);
        return;
    }
    for i in k..items.len() {
        items.swap(k, i);
        permute(items, k + 1, f);
        items.swap(k, i);
    }
}

fn way_repeats(points: &[route_planner::RoutePoint]) -> usize {
    // 用相邻采样点反推是否在同一段路上往返：统计方向反转次数作为近似。
    let mut reversals = 0;
    let mut prev: Option<f64> = None;
    for pair in points.windows(2) {
        let a = (pair[1].lon - pair[0].lon).atan2(pair[1].lat - pair[0].lat);
        if let Some(p) = prev {
            let mut d = a - p;
            while d > std::f64::consts::PI {
                d -= std::f64::consts::TAU;
            }
            while d < -std::f64::consts::PI {
                d += std::f64::consts::TAU;
            }
            if d.abs() > 2.5 {
                reversals += 1;
            }
        }
        prev = Some(a);
    }
    reversals
}

/// 真实校园常见布局：点位沿一条主路几乎共线，或呈 L 形。
/// 按质心角排序在共线时退化为"按极微小角度差排序"，容易产生折返。
#[test]
fn diagnose_collinear_and_l_shape_ordering() {
    let net = grid(20);
    let cases: Vec<(&str, Vec<Coord>)> = vec![
        (
            "共线（沿主路）",
            vec![
                Coord::new(0.0010, 0.0010),
                Coord::new(0.0030, 0.00105),
                Coord::new(0.0050, 0.00098),
                Coord::new(0.0070, 0.00102),
                Coord::new(0.0090, 0.0010),
            ],
        ),
        (
            "L 形",
            vec![
                Coord::new(0.0010, 0.0010),
                Coord::new(0.0020, 0.0010),
                Coord::new(0.0030, 0.0010),
                Coord::new(0.0030, 0.0020),
                Coord::new(0.0030, 0.0030),
            ],
        ),
    ];

    for (label, points) in cases {
        let radial_order = radial(&points);
        let radial_len = tour_len(&net, &radial_order).expect("环序可走通");

        let mut best = (f64::INFINITY, Vec::new());
        let rest = &points[1..];
        let mut idx: Vec<usize> = (0..rest.len()).collect();
        permute(&mut idx, 0, &mut |perm| {
            let mut order = vec![points[0]];
            order.extend(perm.iter().map(|&i| rest[i]));
            if let Some(len) = tour_len(&net, &order) {
                if len < best.0 {
                    best = (len, order);
                }
            }
        });

        let extra = radial_len - best.0;
        println!("── {label} ──");
        println!("  环序 {radial_len:.0} m / 最优 {:.0} m / 多余 {extra:.0} m ({:.1}%)",
            best.0, extra / best.0 * 100.0);
        println!("  环序: {}", radial_order.iter().map(|p| format!("({:.4},{:.4})", p.lat, p.lon)).collect::<Vec<_>>().join(" → "));
        println!("  最优: {}", best.1.iter().map(|p| format!("({:.4},{:.4})", p.lat, p.lon)).collect::<Vec<_>>().join(" → "));
    }
}

/// 真正的"多余"指标：**同一段路被重复走过多少次**。
/// 总里程相同也可能出现"沿同一条路来回折返"，视觉上就是多余的线。
fn overlap(net: &RoadGraph, order: &[Coord]) -> Option<(f64, f64)> {
    use std::collections::HashMap;
    let mut g = net.clone();
    let nodes = route_planner::project::add_virtual_nodes(&mut g, order).ok()?;
    // 逐条**边**（不是 way）计数：一条 way 被切成多段时，每段都是独立路段。
    let mut walk: HashMap<(usize, usize), u32> = HashMap::new();
    let mut total = 0.0;
    let mut unique = 0.0;
    let mut record = |a: petgraph::graph::NodeIndex,
                      b: petgraph::graph::NodeIndex,
                      walk: &mut HashMap<(usize, usize), u32>,
                      total: &mut f64,
                      unique: &mut f64| {
        if let Some(e) = g.graph.find_edge(a, b) {
            let len = g.graph[e].length_m;
            *total += len;
            let key = (a.index().min(b.index()), a.index().max(b.index()));
            let c = walk.entry(key).or_insert(0);
            if *c == 0 {
                *unique += len;
            }
            *c += 1;
        }
    };
    for pair in nodes.windows(2) {
        let path = route_planner::route::shortest_path(&g, pair[0], pair[1])?;
        for w in path.windows(2) {
            record(w[0], w[1], &mut walk, &mut total, &mut unique);
        }
    }
    let back = route_planner::route::shortest_path(&g, *nodes.last()?, nodes[0])?;
    for w in back.windows(2) {
        record(w[0], w[1], &mut walk, &mut total, &mut unique);
    }
    Some((total, unique))
}

#[test]
fn diagnose_overlap_not_just_total_length() {
    let net = grid(20);
    let cases: Vec<(&str, Vec<Coord>)> = vec![
        (
            "共线（沿主路）",
            vec![
                Coord::new(0.0010, 0.0010),
                Coord::new(0.0030, 0.00105),
                Coord::new(0.0050, 0.00098),
                Coord::new(0.0070, 0.00102),
                Coord::new(0.0090, 0.0010),
            ],
        ),
        (
            "分散",
            vec![
                Coord::new(0.0020, 0.0020),
                Coord::new(0.0065, 0.0030),
                Coord::new(0.0070, 0.0075),
                Coord::new(0.0025, 0.0068),
                Coord::new(0.0045, 0.0015),
            ],
        ),
    ];
    for (label, points) in cases {
        let radial_order = radial(&points);
        let (rt, ru) = overlap(&net, &radial_order).expect("可走通");
        let mut best = (f64::INFINITY, Vec::new(), 0.0);
        let rest = &points[1..];
        let mut idx: Vec<usize> = (0..rest.len()).collect();
        permute(&mut idx, 0, &mut |perm| {
            let mut order = vec![points[0]];
            order.extend(perm.iter().map(|&i| rest[i]));
            if let Some((t, u)) = overlap(&net, &order) {
                let waste = t - u;
                if waste < best.0 {
                    best = (waste, order, u);
                }
            }
        });
        let (bt, _bu) = overlap(&net, &best.1).unwrap();
        println!("── {label} ──");
        println!("  环序: 走 {rt:.0} m，其中重复 {:.0} m（{:.0}% 是重复路段）", rt - ru, (rt - ru) / rt * 100.0);
        println!("  最优: 走 {bt:.0} m，其中重复 {:.0} m（{:.0}%）", best.0, best.0 / bt * 100.0);
        println!("  可消除的重复: {:.0} m", (rt - ru) - best.0);
        println!("  环序: {}", radial_order.iter().map(|p| format!("({:.4},{:.4})", p.lat, p.lon)).collect::<Vec<_>>().join(" → "));
        println!("  最优: {}", best.1.iter().map(|p| format!("({:.4},{:.4})", p.lat, p.lon)).collect::<Vec<_>>().join(" → "));
    }
}

/// 定位"多余"的真正来源：补足距离游走 vs 点位排序。
/// 用同一组点位、不同目标距离，看重复率如何增长。
#[test]
fn diagnose_walk_vs_ordering() {
    let net = grid(20);
    let points = vec![
        Coord::new(0.0020, 0.0020),
        Coord::new(0.0065, 0.0030),
        Coord::new(0.0070, 0.0075),
        Coord::new(0.0025, 0.0068),
        Coord::new(0.0045, 0.0015),
    ];
    let order = radial(&points);
    let (base_total, base_unique) = overlap(&net, &order).unwrap();
    println!("点位最短游览: {base_total:.0} m（重复 {:.0} m）", base_total - base_unique);
    println!();
    println!("{:>8} {:>10} {:>10} {:>8} {:>10}", "目标", "实际", "重复里程", "重复率", "折返次数");
    for target in [2500.0, 3000.0, 4000.0, 5000.0, 7000.0] {
        let route = route_planner::plan_route_split(
            &net, &order, &order[1..], target, 42, &RouteOptions::default(),
        ).expect("可规划");
        let walked = route.length_m;
        let rev = way_repeats(&route.points);
        // 估算重复：用采样点反推走过的唯一长度不可行，改用地段覆盖
        let mut g = net.clone();
        let nodes = route_planner::project::add_virtual_nodes(&mut g, &order).unwrap();
        let _ = nodes;
        println!("{:>8.0} {:>10.0} {:>10} {:>7.0}% {:>10}",
            target, walked, "-", (walked / base_total - 1.0) * 100.0, rev);
    }
}

/// 补距游走到底走了哪些路：统计每条边被走的次数分布。
#[test]
fn diagnose_walk_edge_reuse() {
    let net = grid(20);
    let points = vec![
        Coord::new(0.0020, 0.0020),
        Coord::new(0.0065, 0.0030),
        Coord::new(0.0070, 0.0075),
        Coord::new(0.0025, 0.0068),
        Coord::new(0.0045, 0.0015),
    ];
    let order = radial(&points);
    for target in [2500.0, 4000.0] {
        let route = route_planner::plan_route_split(
            &net, &order, &order[1..], target, 42, &RouteOptions::default(),
        ).expect("可规划");
        // 采样点 → 每 50m 一格，统计不同网格格点被访问次数
        use std::collections::HashMap;
        let mut cells: HashMap<(i64, i64), u32> = HashMap::new();
        for p in &route.points {
            let key = ((p.lat / 0.00045).round() as i64, (p.lon / 0.00045).round() as i64);
            *cells.entry(key).or_insert(0) += 1;
        }
        let mut counts: Vec<u32> = cells.values().copied().collect();
        counts.sort_unstable();
        let max = *counts.last().unwrap();
        let repeated_cells = counts.iter().filter(|&&c| c > 1).count();
        println!("目标 {target:.0}m: 路线 {:.0}m, 访问格点 {} 个, 其中重复访问 {} 个, 单格最多访问 {} 次",
            route.length_m, counts.len(), repeated_cells, max);
        println!("  访问次数分布（前10高）: {:?}", counts.iter().rev().take(10).collect::<Vec<_>>());
    }
}

/// 直接统计规划出的节点路径：哪些路段被走了多次。
#[test]
fn diagnose_path_edge_multiplicity() {
    use std::collections::HashMap;
    let net = grid(20);
    let points = vec![
        Coord::new(0.0020, 0.0020),
        Coord::new(0.0065, 0.0030),
        Coord::new(0.0070, 0.0075),
        Coord::new(0.0025, 0.0068),
        Coord::new(0.0045, 0.0015),
    ];
    let order = radial(&points);
    for target in [2500.0, 4000.0, 6000.0] {
        let mut g = net.clone();
        let wps: Vec<Coord> = order.clone();
        let route = route_planner::plan_route_split(
            &g, &wps, &wps[1..], target, 42, &RouteOptions::default(),
        ).expect("可规划");
        // route.points 是平滑采样点，按 5m 间距重采样统计覆盖
        let mut seen: HashMap<(i64, i64), u32> = HashMap::new();
        for p in &route.points {
            let key = (
                (p.lat * 111_132.0 / 10.0).round() as i64,
                (p.lon * 111_320.0 * 0.77 / 10.0).round() as i64,
            );
            *seen.entry(key).or_insert(0) += 1;
        }
        let mut v: Vec<u32> = seen.values().copied().collect();
        v.sort_unstable();
        let total_cells: u32 = v.iter().sum();
        let unique = v.len() as u32;
        println!("目标 {target:.0}: 路线 {:.0}m, 采样点 {} 个, 覆盖 10m 格 {unique} 个, 平均每格 {} 次, 最高 {} 次",
            route.length_m, route.points.len(), total_cells / unique.max(1), v.last().unwrap());
        let _ = &mut g;
    }
}

/// 结论验证：把规划出的路线按 10m 栅格化，统计"重复走过的比例"。
/// 若点位排序有问题，会看到**少数格子被走很多次**（折返）；
/// 若只是补距，会是**均匀铺开**。
#[test]
fn diagnose_retrace_ratio() {
    use std::collections::HashMap;
    let net = grid(20);
    let cases: Vec<(&str, Vec<Coord>)> = vec![
        ("分散5点", vec![
            Coord::new(0.0020, 0.0020), Coord::new(0.0065, 0.0030),
            Coord::new(0.0070, 0.0075), Coord::new(0.0025, 0.0068),
            Coord::new(0.0045, 0.0015),
        ]),
        ("共线5点", vec![
            Coord::new(0.00135, 0.00135), Coord::new(0.00315, 0.00135),
            Coord::new(0.00495, 0.00135), Coord::new(0.00675, 0.00135),
            Coord::new(0.00855, 0.00135),
        ]),
    ];
    for (label, points) in cases {
        let order = radial(&points);
        let (tour, _) = overlap(&net, &order).unwrap();
        // `overlap` 按 haversine 累加，`check::inspect` 按米制投影累加，两者相差
        // 千分之几米。目标恰好取在最短游览上时，plan_route_split 会按后者判定
        // "目标距离不足"，所以这里以上一层取整前的 inspect 值为准再向上取整。
        let min_tour = route_planner::check::inspect(&net, &order, &[])
            .minimum_m
            .unwrap_or(tour);
        for target in [min_tour.ceil(), (min_tour * 2.0).round()] {
            let route = route_planner::plan_route_split(
                &net, &order, &order[1..], target, 42, &RouteOptions::default(),
            ).expect("可规划");
            let mut cells: HashMap<(i64, i64), u32> = HashMap::new();
            for p in &route.points {
                let key = ((p.lat * 111_132.0 / 10.0) as i64, (p.lon * 111_320.0 * 0.77 / 10.0) as i64);
                *cells.entry(key).or_insert(0) += 1;
            }
            let n = cells.len() as f64;
            // 采样点间距约 1m，10m 格正常应约 10 次；超过 15 次即为重复经过。
            let repeated = cells.values().filter(|&&c| c > 15).count() as f64;
            println!("{label} 目标{target:.0}m: 路线 {:.0}m | 10m格 {n:.0} 个 | 重复经过的格 {:.0}%",
                route.length_m, repeated / n * 100.0);
        }
    }
}
