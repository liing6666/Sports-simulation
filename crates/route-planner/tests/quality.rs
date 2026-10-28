//! 路线质量回归：用真实 OSM 夹具验证「跑道模式」与「真实道路模式」。
//!
//! 跑道模式（`find_track_loop` / `plan_track_loop`）的断言不依赖实现本身：
//! 跑道几何直接取自 OSM 原始 way（`parse_osm` 的 node_refs），因此"路线点是否
//! 落在跑道上"是对照着原始测绘数据量出来的，而不是拿实现去验证实现。

use std::collections::HashSet;
use std::path::PathBuf;

use petgraph::graph::NodeIndex;
use route_planner::graph::{Coord, RoadGraph};
use route_planner::{plan_route, plan_track_loop, RouteOptions};

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(name)
}

fn fixture_bytes(name: &str) -> Vec<u8> {
    let p = fixture(name);
    std::fs::read(&p).unwrap_or_else(|e| panic!("读取夹具 {} 失败: {e}", p.display()))
}

fn load(name: &str) -> RoadGraph {
    let bytes = fixture_bytes(name);
    route_planner::load_osm(&bytes).unwrap_or_else(|e| panic!("解析夹具 {name} 失败: {e}"))
}

fn dist_m(net: &RoadGraph, a: Coord, b: Coord) -> f64 {
    let (a, b) = (net.to_m(a), net.to_m(b));
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// 点的米制坐标到折线的最短距离（折线按度系给定，逐段投影到线段）。
fn dist_point_to_polyline_m(net: &RoadGraph, polyline: &[Coord], p: Coord) -> f64 {
    let p = net.to_m(p);
    polyline
        .windows(2)
        .map(|w| {
            let (a, b) = (net.to_m(w[0]), net.to_m(w[1]));
            let (proj, _) = route_planner::graph::project_to_segment(p, a, b);
            (p[0] - proj[0]).hypot(p[1] - proj[1])
        })
        .fold(f64::INFINITY, f64::min)
}

/// 折线总长（米）。
fn polyline_len_m(net: &RoadGraph, polyline: &[Coord]) -> f64 {
    polyline
        .windows(2)
        .map(|w| dist_m(net, w[0], w[1]))
        .sum::<f64>()
}

/// 有向面积（米²，鞋带公式 /2）。> 0 表示逆时针。
fn signed_area_m2(net: &RoadGraph, ring: &[Coord]) -> f64 {
    let mut acc = 0.0;
    for w in ring.windows(2) {
        let (a, b) = (net.to_m(w[0]), net.to_m(w[1]));
        acc += a[0] * b[1] - b[0] * a[1];
    }
    acc / 2.0
}

/// OSM 原始里的跑道 way（`surface=tartan` 或 `leisure=track`）节点序列。
/// 用原始 way 而不是图里的边，作为独立的地面真值。
fn raw_tracks(name: &str) -> Vec<Vec<Coord>> {
    let bytes = fixture_bytes(name);
    let parsed = route_planner::load::parse_osm(&bytes).expect("解析 OSM");
    let mut out = Vec::new();
    for r in &parsed.roads {
        if !(r.is_leisure_track || r.surface == "tartan") {
            continue;
        }
        let coords: Vec<Coord> = r
            .node_refs
            .iter()
            .filter_map(|id| parsed.nodes.get(id).copied())
            .collect();
        if coords.len() >= 4 {
            out.push(coords);
        }
    }
    out
}

fn centroid(pts: &[Coord]) -> Coord {
    let n = pts.len().max(1) as f64;
    Coord::new(
        pts.iter().map(|c| c.lon).sum::<f64>() / n,
        pts.iter().map(|c| c.lat).sum::<f64>() / n,
    )
}

/// 坐标量化成整数键（1e-7 度 ≈ 1cm），用于集合比较。
fn key(c: Coord) -> (i64, i64) {
    ((c.lon * 1e7).round() as i64, (c.lat * 1e7).round() as i64)
}

fn ring_coords(net: &RoadGraph, ring: &[NodeIndex]) -> Vec<Coord> {
    ring.iter().map(|&n| net.node_coord(n)).collect()
}

fn is_tartan(net: &RoadGraph, e: petgraph::graph::EdgeIndex) -> bool {
    net.graph[e].highway.contains("tartan")
}

/// 跑道 way 在图里的全部节点。
fn graph_track_nodes(net: &RoadGraph) -> HashSet<NodeIndex> {
    net.graph
        .edge_indices()
        .filter(|&e| is_tartan(net, e))
        .filter_map(|e| net.graph.edge_endpoints(e))
        .flat_map(|(a, b)| [a, b])
        .collect()
}

/// 1. 有跑道的真实路网里必须能找到跑道闭环。
#[test]
fn finds_track_loop_in_real_osm_with_tartan() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    assert!(
        !tracks.is_empty(),
        "dinghu-roads.osm 里应该有 leisure=track 跑道（way 1101480651）"
    );
    let near = centroid(&tracks[0]);

    let ring = net
        .find_track_loop(near)
        .unwrap_or_else(|| panic!("dinghu-roads.osm 里找得到跑道（near={near:?}），find_track_loop 却返回 None"));
    let coords = ring_coords(&net, &ring);
    let perimeter = polyline_len_m(&net, &coords);
    println!(
        "跑道环：{} 条边 / {} 个节点，周长 {perimeter:.1}m，距 near 最近 {:.1}m",
        ring.len() - 1,
        ring.len() - 1,
        ring.iter()
            .map(|&n| dist_m(&net, net.node_coord(n), near))
            .fold(f64::INFINITY, f64::min)
    );

    // 环上的节点集合必须与原始 way 的节点集合一致（没有借道、没有漏点）。
    let ring_keys: HashSet<(i64, i64)> = coords.iter().map(|&c| key(c)).collect();
    let raw_keys: HashSet<(i64, i64)> = tracks[0].iter().map(|&c| key(c)).collect();
    assert_eq!(
        ring_keys, raw_keys,
        "跑道环的节点集合与 OSM 原始跑道 way 不一致"
    );
}

/// 2. 没有跑道的路网里 `find_track_loop` 返回 None、`plan_track_loop` 返回 Err。
///
/// 两个校园边界夹具（dinghu.osm / chengdu-neusoft.osm）里没有 highway 道路，
/// 无法构建路网；这部分按约定打印说明后跳过。真正的"无跑道图"用同一份真实
/// 路网剔除 tartan 边构造。
#[test]
fn returns_none_when_network_has_no_tartan() {
    for name in ["dinghu.osm", "chengdu-neusoft.osm"] {
        let bytes = fixture_bytes(name);
        let parsed = route_planner::load::parse_osm(&bytes).expect("解析 OSM");
        let track_ways = parsed
            .roads
            .iter()
            .filter(|r| r.is_leisure_track || r.surface == "tartan")
            .count();
        println!(
            "{name}: {} 条可用道路，其中 {} 条跑道",
            parsed.roads.len(),
            track_ways
        );
        if parsed.roads.is_empty() {
            println!("  → 该夹具只有校园边界/建筑，没有 highway 道路，无法构建路网，跳过");
            continue;
        }
        let net = route_planner::load::load_osm(&bytes).expect("加载路网");
        if track_ways == 0 {
            let c = centroid(&net.edges_deg().iter().flatten().copied().collect::<Vec<_>>());
            assert!(
                net.find_track_loop(c).is_none(),
                "{name} 没有 tartan 跑道，find_track_loop 应返回 None"
            );
            println!("  → 无跑道，find_track_loop 返回 None（已验证）");
        } else {
            println!("  → 有跑道，由 finds_track_loop_in_real_osm_with_tartan 覆盖");
        }
    }

    // 真实路网 − 跑道边 = 有道路但没跑道的图。
    let mut net = load("dinghu-roads.osm");
    let before = net.graph.edge_count();
    net.graph.retain_edges(|g, e| !g[e].highway.contains("tartan"));
    net.compute_anchor();
    net.rebuild_index();
    assert!(net.graph.edge_count() < before, "应剔除了跑道边");
    assert!(
        graph_track_nodes(&net).is_empty(),
        "剔除后图里不应还有跑道节点标记"
    );

    let near = centroid(&net.edges_deg().iter().flatten().copied().collect::<Vec<_>>());
    println!(
        "剔除跑道后的真实路网：{} 条边，find_track_loop → {:?}",
        net.graph.edge_count(),
        net.find_track_loop(near).map(|r| r.len())
    );
    assert!(
        net.find_track_loop(near).is_none(),
        "没有跑道边时 find_track_loop 必须返回 None"
    );
    match plan_track_loop(&net, near, 2000.0, 42) {
        Ok(_) => panic!("没有跑道时 plan_track_loop 不应成功"),
        Err(e) => println!("plan_track_loop 正确报错：{e}"),
    }
}

/// 3. 跑道环必须是逆时针（有向面积 > 0），且首尾节点相同。
#[test]
fn track_loop_is_counter_clockwise_and_closed() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    let near = centroid(&tracks[0]);
    let ring = net.find_track_loop(near).expect("应找到跑道环");

    assert!(ring.len() >= 4, "跑道环节点太少：{}", ring.len());
    assert_eq!(
        ring.first(),
        ring.last(),
        "跑道环必须首尾闭合（同一节点）"
    );

    let coords = ring_coords(&net, &ring);
    let area = signed_area_m2(&net, &coords);
    let raw_area = signed_area_m2(&net, &tracks[0]);
    println!("跑道环有向面积 {area:.0}m²（原始 way {raw_area:.0}m²）");
    assert!(
        area > 0.0,
        "跑道环应为逆时针（有向面积 > 0），实际 {area:.0}m²"
    );
    assert!(
        area.abs() > 1000.0,
        "跑道环面积 {area:.0}m² 太小，可能退化成一条折返线"
    );

    // 原始 way 恰好是顺时针，实现必须把它翻转过来——证明上面的 > 0 不是巧合。
    if raw_area < 0.0 {
        println!("原始 OSM way 为顺时针，实现已翻转为逆时针");
    }

    // 每一步都必须是图中真实存在的有向跑道边，方向与环序一致。
    for w in ring.windows(2) {
        let e = net
            .graph
            .find_edge(w[0], w[1])
            .unwrap_or_else(|| panic!("跑道环相邻节点 {:?} → {:?} 之间没有边", w[0], w[1]));
        assert!(
            is_tartan(&net, e),
            "跑道环经过非跑道边（way {}）",
            net.graph[e].way_id
        );
    }
}

/// 4. `plan_track_loop` 的 route.length_m 必须精确等于目标距离（±1m）。
#[test]
fn planned_track_route_length_matches_target() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    let near = centroid(&tracks[0]);

    for target in [400.0f64, 1000.0, 2500.0, 3000.0] {
        let tl = plan_track_loop(&net, near, target, 42)
            .unwrap_or_else(|e| panic!("目标 {target}m 规划失败: {e}"));
        let coords: Vec<Coord> = tl.route.points.iter().map(|p| Coord::new(p.lon, p.lat)).collect();
        let geo_len = polyline_len_m(&net, &coords);
        let laps = target / tl.lap_m;
        println!(
            "目标 {target:.0}m → length_m={:.1}m，实际折线 {geo_len:.1}m，\
             {} 个采样点，{laps:.2} 圈，末点 s={:.1}",
            tl.route.length_m,
            tl.route.points.len(),
            tl.route.points.last().map(|p| p.s).unwrap_or(-1.0)
        );

        assert!(
            (tl.route.length_m - target).abs() < 1.0,
            "route.length_m={} 与目标 {target} 相差超过 1m",
            tl.route.length_m
        );
        // s 必须单调不减，且终点 s 等于 length_m（下游按 s 定位配速）。
        let mut prev = -1.0;
        for p in &tl.route.points {
            assert!(p.s >= prev - 1e-9, "采样点 s 非单调：{} → {}", prev, p.s);
            prev = p.s;
        }
        assert!(
            (prev - target).abs() < 1e-6,
            "末点 s={prev} 应等于目标 {target}"
        );
        // 实际几何长度也要贴合目标（截断插值不能把里程吃掉）。
        assert!(
            (geo_len - target).abs() < 5.0,
            "实际折线长 {geo_len:.1}m 与目标 {target}m 偏差超过 5m"
        );
    }
}

/// 5. 单圈长度 lap_m 必须合理（100–1000m），且与原始跑道周长一致。
#[test]
fn track_lap_length_is_plausible() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    let near = centroid(&tracks[0]);
    let raw_perimeter = polyline_len_m(&net, &tracks[0]);
    println!("原始跑道 way 周长 {raw_perimeter:.1}m");

    let tl = plan_track_loop(&net, near, 2000.0, 42).expect("规划跑道");
    println!("plan_track_loop lap_m = {:.2}m", tl.lap_m);
    assert!(
        (100.0..=1000.0).contains(&tl.lap_m),
        "lap_m={:.1}m 不在 100–1000m 的合理区间",
        tl.lap_m
    );
    assert!(
        (tl.lap_m - raw_perimeter).abs() < 1.0,
        "lap_m={:.2}m 与原始跑道周长 {raw_perimeter:.2}m 不一致",
        tl.lap_m
    );
    // 目标 2000m 应绕出整数圈（截断处允许半圈收尾）。
    println!(
        "目标 2000m 约 {:.2} 圈，采样点 {} 个",
        2000.0 / tl.lap_m,
        tl.route.points.len()
    );
    assert!(
        tl.route.points.len() as f64 > raw_perimeter / 2.0,
        "采样点过少，无法描述跑道形状"
    );
}

/// 6. 重复多圈后，路线点必须仍然全部落在真实跑道环上（点到跑道边 < 5m），
///    并且覆盖整圈（每个原始跑道顶点都能在路线附近找到）。
#[test]
fn repeated_laps_stay_on_tartan_track() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    let track = &tracks[0];
    let near = centroid(track);

    let target = 3000.0;
    let tl = plan_track_loop(&net, near, target, 42).expect("规划跑道");
    let route: Vec<Coord> = tl
        .route
        .points
        .iter()
        .map(|p| Coord::new(p.lon, p.lat))
        .collect();

    let worst_route = route
        .iter()
        .map(|&p| dist_point_to_polyline_m(&net, track, p))
        .fold(0.0f64, f64::max);
    // 反向覆盖：原始跑道上每个顶点都必须被路线经过（不能只在某个小段来回）。
    let worst_track = track
        .iter()
        .map(|&p| dist_point_to_polyline_m(&net, &route, p))
        .fold(0.0f64, f64::max);

    println!(
        "{target:.0}m（约 {:.1} 圈）：路线点 {} 个，路线→跑道最大 {worst_route:.3}m，\
         跑道顶点→路线最大 {worst_track:.3}m",
        target / tl.lap_m,
        route.len()
    );
    assert!(
        worst_route < 5.0,
        "路线偏离跑道 {worst_route:.2}m（应 < 5m）——重复跑圈后跑到了其它道路上"
    );
    assert!(
        worst_track < 5.0,
        "跑道顶点离路线最远 {worst_track:.2}m——没有覆盖整圈"
    );
}

/// 真实道路模式的回归：用 dinghu-roads 里真实的道路节点选出分散的打卡点，
/// 规划一条比"最短游览"更长的闭环，验证长度、必经点、贴路程度和闭合性。
#[test]
fn road_mode_route_is_sane() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    let near = centroid(&tracks[0]);

    let waypoints = spread_road_waypoints(&net, near, 4);
    println!(
        "真实道路模式打卡点：{:?}",
        waypoints
            .iter()
            .map(|c| (format!("{:.5}", c.lat), format!("{:.5}", c.lon)))
            .collect::<Vec<_>>()
    );

    // 目标距离必须明显大于「按序拜访全部打卡点并回到起点」的最短游览长度，
    // 否则路线长度由游览本身决定，比较目标距离没有意义。
    let min_tour = route_planner::check::inspect(&net, &waypoints, &[])
        .minimum_m
        .unwrap_or(f64::NAN);
    let target = (min_tour * 1.5).max(1500.0);
    println!("点位最短游览 {min_tour:.0}m → 目标距离 {target:.0}m");

    // 回归：目标距离低于「必经点最短游览」时，规划器必须返回明确 Err（含最短
    // 游览数值），而不是静默产出一条远长于目标的路线。旧版测试硬编码目标
    // 2000m、而当时选出的打卡点最短游览 3092m，于是稳定地得到 ~3112m。
    let too_short = min_tour * 0.7;
    let err = plan_route(&net, &waypoints, too_short, 42, &RouteOptions::default())
        .err()
        .unwrap_or_else(|| panic!("目标 {too_short:.0}m < 最短游览 {min_tour:.0}m，应报错"));
    println!("目标 {too_short:.0}m（< 最短游览 {min_tour:.0}m）正确报错：{err}");
    let parsed_min = route_planner::check::is_target_below_min_tour(&err)
        .unwrap_or_else(|| panic!("错误消息应能被上层识别为目标距离不足：{err}"));
    println!("上层从消息中解析出最短游览 {parsed_min:.0}m");
    assert!(
        (parsed_min - min_tour).abs() < 1.0,
        "消息里的最短游览 {parsed_min:.0}m 与 inspect 的 {min_tour:.0}m 不一致"
    );

    let route = plan_route(&net, &waypoints, target, 42, &RouteOptions::default())
        .unwrap_or_else(|e| panic!("真实道路模式规划失败: {e}"));
    let pts: Vec<Coord> = route.points.iter().map(|p| Coord::new(p.lon, p.lat)).collect();
    let geo_len = polyline_len_m(&net, &pts);
    println!(
        "真实道路模式：{} 个采样点，length_m={:.1}m，实际折线 {geo_len:.1}m",
        route.points.len(),
        route.length_m
    );

    assert!(route.points.len() > 10, "采样点过少");
    assert!(
        route.length_m > min_tour,
        "路线长度 {:.1}m 不应短于点位最短游览 {min_tour:.0}m",
        route.length_m
    );
    // 长度：默认选项（补齐到目标距离，允许 ±10%）。
    assert!(
        (route.length_m - target).abs() / target < 0.10,
        "路线长度 {:.1}m 偏离目标 {target:.0}m 超过 10%",
        route.length_m
    );

    // 每个必经点都要经过。产品侧用 `snap_m`（默认 30m）判定"到达"，这里同口径。
    let snap_m = RouteOptions::default().snap_m;
    for (i, &wp) in waypoints.iter().enumerate() {
        let d = dist_point_to_polyline_m(&net, &pts, wp);
        println!("  打卡点 {i} 距路线 {d:.1}m");
        assert!(
            d <= snap_m,
            "打卡点 {i} 距路线 {d:.1}m，超过到达半径 {snap_m}m"
        );
    }

    // 路线必须贴着路网：切角段在路口外侧，偏移量受切角半径约束（默认 6m）。
    let worst_road = pts
        .iter()
        .map(|&p| net.nearest_edge(net.to_m(p)).map(|(_, _, _, d)| d).unwrap_or(f64::INFINITY))
        .fold(0.0f64, f64::max);
    println!("路线点到最近道路的最大距离 {worst_road:.1}m");
    assert!(
        worst_road < 20.0,
        "路线点距道路最远 {worst_road:.1}m——平滑切角把路线带离了路网"
    );

    // 闭环：终点回到起点附近。
    let close_d = dist_m(&net, pts[0], *pts.last().unwrap());
    println!("首尾距离 {close_d:.1}m");
    assert!(close_d < 60.0, "路线未闭合，首尾相距 {close_d:.1}m");
}

/// 目标距离判定的边界：最短游览 −1m 必须 Err，恰好等于和 +500m 必须 Ok。
#[test]
fn target_length_boundary_around_minimum_tour() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    let near = centroid(&tracks[0]);
    let waypoints = spread_road_waypoints(&net, near, 4);
    let check = route_planner::check::inspect(&net, &waypoints, &[]);
    assert!(check.ok(), "{:?}", check.issues);
    let min = check.minimum_m.expect("应能给出最短游览");
    println!("最短游览 {min:.3}m（inspect）");

    // 恰好等于最短游览：放行。
    let at_min = plan_route(&net, &waypoints, min, 42, &RouteOptions::default())
        .unwrap_or_else(|e| panic!("目标 = 最短游览 {min:.1}m 应成功，实际报错：{e}"));
    println!("目标 = 最短游览 {min:.1}m → Ok，length_m={:.1}m", at_min.length_m);
    assert!(
        at_min.length_m >= min - 1.0,
        "路线 {:.1}m 短于最短游览 {min:.1}m",
        at_min.length_m
    );

    // 少 1 米：必须报错，且消息可被上层解析。
    let below = min - 1.0;
    let err = plan_route(&net, &waypoints, below, 42, &RouteOptions::default())
        .err()
        .unwrap_or_else(|| panic!("目标 {below:.1}m < 最短游览 {min:.1}m 应报错"));
    println!("目标 = 最短游览 −1m（{below:.1}m）→ Err：{err}");
    let parsed = route_planner::check::is_target_below_min_tour(&err)
        .unwrap_or_else(|| panic!("错误消息应带固定前缀：{err}"));
    assert!(
        (parsed - min).abs() < 1.0,
        "消息里的最短游览 {parsed:.0}m ≠ inspect 的 {min:.1}m"
    );

    // 高于最短游览 500 米：正常规划。
    let above = min + 500.0;
    let ok = plan_route(&net, &waypoints, above, 42, &RouteOptions::default())
        .unwrap_or_else(|e| panic!("目标 {above:.0}m 应成功：{e}"));
    println!("目标 = 最短游览 +500m（{above:.0}m）→ Ok，length_m={:.1}m", ok.length_m);
    assert!(
        (ok.length_m - above).abs() / above < 0.10,
        "length_m={:.1}m 偏离目标 {above:.0}m 超过 10%",
        ok.length_m
    );
}

/// 任务验收用例：真实夹具上目标 2000m（< 最短游览）必须 Err 且消息含最短游览数值；
/// 目标 3440m 必须 Ok 且长度在 ±10% 内。
#[test]
fn real_fixture_target_check_messages() {
    let net = load("dinghu-roads.osm");
    let tracks = raw_tracks("dinghu-roads.osm");
    let near = centroid(&tracks[0]);
    let waypoints = spread_road_waypoints(&net, near, 4);
    let min = route_planner::check::inspect(&net, &waypoints, &[])
        .minimum_m
        .expect("最短游览");

    let err = plan_route(&net, &waypoints, 2000.0, 42, &RouteOptions::default())
        .err()
        .unwrap_or_else(|| panic!("目标 2000m < 最短游览 {min:.0}m，应报错"));
    println!("目标 2000m → Err：{err}");
    assert!(err.starts_with(route_planner::check::TARGET_BELOW_MIN_TOUR_PREFIX));
    assert!(
        err.contains(&format!("{min:.0}")),
        "错误消息应含最短游览 {min:.0}，实际：{err}"
    );

    let route = plan_route(&net, &waypoints, 3440.0, 42, &RouteOptions::default())
        .unwrap_or_else(|e| panic!("目标 3440m 应成功：{e}"));
    println!("目标 3440m → Ok，length_m={:.1}m", route.length_m);
    assert!(
        (route.length_m - 3440.0).abs() / 3440.0 < 0.10,
        "length_m={:.1}m 偏离 3440m 超过 10%",
        route.length_m
    );
}

/// 有向图最大强连通分量：分量内任意两点互相可达，按任意顺序串起来都能闭合成环
/// （夹具以单行道为主，无向连通并不保证有向可达）。
fn largest_scc(net: &RoadGraph) -> HashSet<NodeIndex> {
    petgraph::algo::kosaraju_scc(&net.graph)
        .into_iter()
        .max_by_key(|c| c.len())
        .unwrap_or_default()
        .into_iter()
        .collect()
}

/// 从真实路网的最大强连通分量里选 `count` 个分散的、非跑道节点作为打卡点。
///
/// 候选点先收集成**有序 Vec**（按 `NodeIndex` 排序，索引由 OSM way 顺序决定，
/// 与哈希随机种子无关），再按坐标排序；后续 `min_by`/`max_by` 遇到距离并列时
/// 由这条全序兜底，因此选点结果在不同进程、不同哈希种子下完全一致。
fn spread_road_waypoints(net: &RoadGraph, near: Coord, count: usize) -> Vec<Coord> {
    let track = graph_track_nodes(net);
    let main = largest_scc(net);
    println!(
        "最大强连通分量 {} 个节点 / 全图 {} 个节点",
        main.len(),
        net.graph.node_count()
    );

    // 1) HashSet → 有序 Vec（按 NodeIndex）。
    let mut main_sorted: Vec<NodeIndex> = main.iter().copied().collect();
    main_sorted.sort_unstable_by_key(|n| n.index());

    // 2) 过滤：非跑道、出度 ≥ 2（排除尽头节点，避免必然折返）、near 周边 1200m。
    let mut cands: Vec<(NodeIndex, Coord, f64)> = main_sorted
        .into_iter()
        .filter(|n| !track.contains(n))
        .filter(|&n| net.graph.neighbors_directed(n, petgraph::Direction::Outgoing).count() >= 2)
        .map(|n| {
            let c = net.node_coord(n);
            (n, c, dist_m(net, c, near))
        })
        .filter(|&(_, _, d)| d <= 1200.0)
        .collect();
    // 3) 坐标全序（lon 再 lat，最后回落到 NodeIndex），保证并列时结果唯一。
    cands.sort_by(|a, b| {
        a.1.lon
            .total_cmp(&b.1.lon)
            .then(a.1.lat.total_cmp(&b.1.lat))
            .then(a.0.index().cmp(&b.0.index()))
    });
    // 去重（同一坐标只留索引最小的那个）。
    cands.dedup_by(|a, b| a.1.lon == b.1.lon && a.1.lat == b.1.lat);
    assert!(cands.len() >= count, "候选道路节点不足：{}", cands.len());

    // 4) 第一个点取离 near 最近者；并列时取有序 Vec 中的前者（不再依赖哈希顺序）。
    let first = cands
        .iter()
        .enumerate()
        .min_by(|(ia, a), (ib, b)| a.2.total_cmp(&b.2).then(ia.cmp(ib)))
        .map(|(_, c)| c.1)
        .unwrap();
    let mut chosen = vec![first];
    while chosen.len() < count {
        // 贪心最远点采样；并列时同样回落到有序 Vec 的下标。
        let next = cands
            .iter()
            .enumerate()
            .filter(|(_, c)| !chosen.contains(&c.1))
            .max_by(|(ia, a), (ib, b)| {
                let da = chosen
                    .iter()
                    .map(|&s| dist_m(net, a.1, s))
                    .fold(f64::INFINITY, f64::min);
                let db = chosen
                    .iter()
                    .map(|&s| dist_m(net, b.1, s))
                    .fold(f64::INFINITY, f64::min);
                // max_by：距离大者优先；并列时下标小者优先。
                da.total_cmp(&db).then(ib.cmp(ia))
            })
            .map(|(_, c)| c.1)
            .unwrap();
        chosen.push(next);
    }
    chosen
}
