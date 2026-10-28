//! 真实道路路由轨迹生成器（模式 B）。
//!
//! 与经典模式（`generator::build`）的分工：
//! - 几何源为路网闭环路线（而非 Catmull-Rom 环）；
//! - 配速为运动学速度剖面：OU 均值回归 + 弯道预判刹车（`kinematics::pace_profile`）；
//! - 步频/步幅为生物力学 OU 耦合（`biomech::ou_cadence_series` + `gait`）；
//! - GPS 抖动为 AR(1) 相关漂移 + 高斯测量噪声 + 建筑 SDF 衰减（`noise::GpsJitter`）。
//!
//! 协议字段/哨兵/断崖/吸附/10s 窗与经典模式完全一致。

use super::generator::{
    MAX_EXCEPTION_DRIFT_M, MAX_EXCEPTION_GPS_SPEED_KMH, SPEED_CEIL, SPEED_FLOOR,
};
use super::geom::{
    fmt_gain_time, ring_point_at, round_to, to_bd, wgs84_to_bd09, Rng, MET_PER_DEG_LAT,
    MET_PER_DEG_LNG,
};
use super::model::{GenPoint, Segment, TenWindow, Track};
use super::postfix::apply_post_fixes;
use crate::api::model::TrackColorMode;
use route_planner::{
    fatigue_from_km, gait, ou_cadence_series, pace_profile, plan_route_split,
    Coord, GpsJitter, KinParams, RoadGraph, Route, RouteOptions, RoutePoint, Sdf,
};

/// 路线算法模式（三选一）。
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum RouteMode {
    /// 经典打卡点环（Catmull-Rom）。
    #[default]
    Legacy,
    /// 真实道路拓扑路由（OSM 路网）。
    Road,
    /// 自定义路线（高德步行路径）。
    Custom,
}

impl RouteMode {
    pub fn from_str(s: &str) -> Self {
        match s {
            "road" => RouteMode::Road,
            "custom" => RouteMode::Custom,
            _ => RouteMode::Legacy,
        }
    }
}

/// 从本地文件加载 OSM 路网，**并完成 WGS84→BD-09 对齐**。
///
/// 所有加载入口都必须走这里：OSM 原始坐标是 WGS84，而打卡点/围栏是 BD-09，
/// 两者在四川盆地相差约 950 米。漏掉对齐会让路线整体偏出地图。
pub fn load_network_path(path: &str) -> Result<RoadGraph, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("读取 {path} 失败: {e}"))?;
    let mut net = route_planner::load_osm(&bytes)?;
    align_network(&mut net);
    Ok(net)
}

/// 路网预览计划（供地图显示）：对齐后的路网/建筑 + 生成的路线（BD 系）。
pub struct RoadPlan {
    pub route: Vec<(f64, f64)>,
    pub length_m: f64,
    pub edges: Vec<Vec<(f64, f64)>>,
    pub buildings: Vec<Vec<(f64, f64)>>,
    pub checkpoints: Vec<(f64, f64)>,
    pub fences: Vec<Vec<(f64, f64)>>,
    /// 路线质量：重复经过的路段占比（0.0–1.0）。越高说明"多余"越多。
    pub repeat_ratio: f64,
    /// 点位最短游览长度（米）；与 length_m 对比可看出补距扩增了多少。
    pub min_tour_m: f64,
}

/// 评估一条已采样路线的重复程度。
///
/// 按 10m 栅格统计每个格被经过的次数：正常采样密度下每格约 10 次，明显高于
/// 该值说明这一段被反复走过。返回重复格占比。
pub fn repeat_ratio(points: &[(f64, f64)]) -> f64 {
    use std::collections::HashMap;
    if points.len() < 4 {
        return 0.0;
    }
    // 以首点为原点做米制换算，避免绝对值造成格子边界漂移。
    let (lat0, lon0) = points[0];
    let m_lat = 111_132.0;
    let m_lon = 111_320.0 * lat0.to_radians().cos();
    let cell = |p: (f64, f64)| -> (i64, i64) {
        (
            ((p.0 - lat0) * m_lat / 10.0).floor() as i64,
            ((p.1 - lon0) * m_lon / 10.0).floor() as i64,
        )
    };
    // 统计"进入格子"的次数，而不是采样点个数：采样密度随速度变化，直接数点
    // 会把慢速段误判成重复。只在格子发生变化时 +1，与采样频率无关。
    let mut cells: HashMap<(i64, i64), u32> = HashMap::new();
    let mut prev = cell(points[0]);
    *cells.entry(prev).or_insert(0) += 1;
    for &p in &points[1..] {
        let c = cell(p);
        if c != prev {
            *cells.entry(c).or_insert(0) += 1;
            prev = c;
        }
    }
    // 单次经过 = 每格 1 次；走过 2 次及以上即为重复经过。
    let repeated = cells.values().filter(|&&c| c >= 2).count();
    repeated as f64 / cells.len().max(1) as f64
}

/// 用围栏（BD 系，与打卡点同系）裁剪路网：仅保留围栏内道路。
///
/// 裁剪后一条路都不剩时**返回 Err**，而不是退回未被裁剪的路网。退回原网会让
/// 路线跑到校外，却还显示"使用了围栏"，比直接失败更危险。
pub fn apply_fences(net: &RoadGraph, fences: &[Vec<(f64, f64)>]) -> Result<RoadGraph, String> {
    if fences.is_empty() {
        return Ok(net.clone());
    }
    let mut g = net.clone();
    let polys: Vec<Vec<Coord>> = fences
        .iter()
        .map(|r| r.iter().map(|p| Coord::new(p.1, p.0)).collect())
        .collect();
    g.retain_inside_any(&polys);
    // 进一步裁剪：移除「穿越围栏边界」的边（端点在内但中段跑出围栏，如跨校区道路）。
    g.graph.retain_edges(|graph, e| {
        let (a, b) = graph.edge_endpoints(e).unwrap();
        edge_inside(&polys, graph[a].coord, graph[b].coord)
    });
    g.rebuild_index();
    if g.graph.edge_count() == 0 {
        return Err(format!(
            "围栏裁剪后没有可用道路（围栏 {} 个）：请确认校区选择与 OSM 数据范围",
            fences.len()
        ));
    }
    Ok(g)
}

/// 边是否整体落在任一围栏内。
///
/// 用精确的整段判定（求交点后逐个子区间检查）而不是等距采样：采样步长再小，
/// 也会漏掉比步长更窄的凹口，或把两个校区之间的连线判成合格。
fn edge_inside(polys: &[Vec<Coord>], a: Coord, b: Coord) -> bool {
    route_planner::boundary::segment_inside_any(polys, a, b)
}

/// 生成路网预览（对齐 + 围栏裁剪 + 闭环路由），不产生协议字段。
///
/// 硬点与真实提交一致：必经点（`fixed`）+ 随机 `pick` 个普通点（`ordinary`），
/// 全部强经过；随机性由 `seed` 决定，与实际运行同 seed 时选择一致。
/// `pick` 为随机抽取的普通点数量（运行时可变，别写死）。
pub fn plan_road_view(
    net: &RoadGraph,
    fixed: &[(f64, f64)],
    ordinary: &[(f64, f64)],
    dist: f64,
    seed: u64,
    fences: &[Vec<(f64, f64)>],
    pick: usize,
    loop_mode: bool,
) -> Result<RoadPlan, String> {
    // net 已由 load_network_path / 界面加载器完成对齐，此处不再重复转换
    // （重复转换会把路网再推远一次约 950 米）。
    let mut g = net.clone();
    g = apply_fences(&g, fences)?;
    let ordered = select_route_points(fixed, ordinary, seed, pick);
    let waypoints: Vec<Coord> = ordered.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let must_pass: Vec<Coord> = waypoints.iter().skip(1).copied().collect();
    // 跑道模式：规划一条闭环再按目标距离重复，避免补距游走跑出跑道。
    let mut opts = RouteOptions::default();
    opts.repeat_checkpoint_loop = loop_mode;
    let route = plan_route_split(&g, &waypoints, &must_pass, dist, seed, &opts)?;
    // 跑道模式：把��闭环重复到接近目标距离，末尾再走一段缩短的闭环。
    let route = if loop_mode {
        let lr = repeat_loop_to_target(route, dist, seed)?;
        lr.route
    } else {
        route
    };
    let route_pts: Vec<(f64, f64)> = route.points.iter().map(|p| (p.lat, p.lon)).collect();
    let edges: Vec<Vec<(f64, f64)>> = g
        .edges_deg()
        .iter()
        .map(|e| e.iter().map(|c| (c.lat, c.lon)).collect())
        .collect();
    let buildings: Vec<Vec<(f64, f64)>> = g
        .buildings
        .iter()
        .map(|r| r.iter().map(|c| (c.lat, c.lon)).collect())
        .collect();
    // 点位最短游览：按必经点顺序走一遍并闭合，用于对比"补距扩增了多少"。
    let min_tour_m = route_planner::check::inspect(&g, &waypoints, &[])
        .minimum_m
        .unwrap_or(route.length_m);
    let ratio = repeat_ratio(&route_pts);
    Ok(RoadPlan {
        route: route_pts,
        length_m: route.length_m,
        edges,
        buildings,
        checkpoints: ordered,
        fences: fences.to_vec(),
        repeat_ratio: ratio,
        min_tour_m,
    })
}

/// 跑道模式的预览：把打卡点按环序串成闭环，与 `build_stadium` 的构造一致。
///
/// 打卡点是"范围标记"，不是逐点拜访目标；预览因此不经过路网路由。
pub fn plan_loop_view(
    net: &RoadGraph,
    fixed: &[(f64, f64)],
    ordinary: &[(f64, f64)],
    dist: f64,
    seed: u64,
    fences: &[Vec<(f64, f64)>],
    pick: usize,
) -> Result<RoadPlan, String> {
    let ordered = select_route_points(fixed, ordinary, seed, pick);
    // 跑道模式：优先用路网里的**真实塑胶跑道**（surface=tartan）。
    // 这样预览线会正好压在操场跑道上，而不是拟合出来的近似椭圆。
    let near = ordered
        .iter()
        .fold((0.0, 0.0), |acc, p| (acc.0 + p.0, acc.1 + p.1));
    let n = ordered.len().max(1) as f64;
    let near = Coord::new(near.1 / n, near.0 / n);
    // `already_repeated`：真实跑道分支的路线已按目标距离重复并裁好。
    let (mut route, lap_m, already_repeated) = match route_planner::plan_track_loop(net, near, dist, seed) {
        Ok(tl) => (
            tl.route
                .points
                .iter()
                .map(|p| (p.lat, p.lon))
                .collect::<Vec<_>>(),
            tl.lap_m,
            true,
        ),
        Err(_) => {
            // 路网里没有跑道（或没导入路网）：回退到按打卡点拟合。
            if ordered.len() < 3 {
                return Err("打卡点不足 3 个，且路网中没有塑胶跑道".into());
            }
            let (dense, arcs, center) =
                crate::track::stadium::make_fitted_track_ring(&ordered, 400.0)?;
            let lap = *arcs.last().unwrap_or(&0.0);
            let route = dense
                .iter()
                .map(|&(x, y)| {
                    (center.0 + y / MET_PER_DEG_LAT, center.1 + x / MET_PER_DEG_LNG)
                })
                .collect::<Vec<_>>();
            (route, lap, false)
        }
    };
    // 拟合分支只有一圈，按目标距离重复；真实跑道分支已裁好。
    let laps = (dist / lap_m).round().max(1.0) as usize;
    if !already_repeated && route.len() > 1 {
        let base = route.clone();
        for _ in 1..laps {
            route.extend(base.iter().copied());
        }
    }
    let edges: Vec<Vec<(f64, f64)>> = net
        .edges_deg()
        .iter()
        .map(|e| e.iter().map(|c| (c.lat, c.lon)).collect())
        .collect();
    let buildings: Vec<Vec<(f64, f64)>> = net
        .buildings
        .iter()
        .map(|r| r.iter().map(|c| (c.lat, c.lon)).collect())
        .collect();
    Ok(RoadPlan {
        route,
        length_m: if already_repeated { dist } else { lap_m * laps as f64 },
        edges,
        buildings,
        checkpoints: ordered,
        fences: fences.to_vec(),
        repeat_ratio: 1.0,
        min_tour_m: lap_m,
    })
}

/// 将 WGS84 路网逐节点转换为 BD-09（与打卡点同系）。
///
/// OSM 为 WGS84，打卡点为 BD-09。两者不是常数平移关系（境内 GCJ/BD 偏移随位置
/// 变化），故用标准 WGS84→GCJ-02→BD-09 变换逐节点转换，校园尺度下与真实道路
/// 对齐（优于旧版质心常数平移的米级~数十米残差）。
pub fn align_network(net: &mut RoadGraph) {
    if net.graph.node_count() == 0 {
        return;
    }
    for node in net.graph.node_weights_mut() {
        let (lat, lon) = wgs84_to_bd09(node.coord.lat, node.coord.lon);
        node.coord.lat = lat;
        node.coord.lon = lon;
    }
    for ring in net.buildings.iter_mut() {
        for c in ring.iter_mut() {
            let (lat, lon) = wgs84_to_bd09(c.lat, c.lon);
            c.lat = lat;
            c.lon = lon;
        }
    }
    net.compute_anchor();
    net.rebuild_index();
}

/// 把一条闭环按弧长重复到目标距离；末段取闭环的前若干米，保证总长贴合。
///
/// 闭环的每个采样点都带累计弧长 `s`，因此可以直接按弧长插值定位末段终点，
/// 而不必重算几何。
pub struct LoopRoute {
    pub route: Route,
    /// 单圈长度（米）。`route.length_m` 是重复后的总长。
    pub lap_m: f64,
}

/// 把一圈路线重复到目标距离。
///
/// `planner` 为 Some 时，**每圈都重新规划**（用不同 seed），这样每圈走的路不同，
/// 地图上能看到多圈叠在一起；为 None 时退化为复制同一圈（旧行为）。
#[allow(clippy::too_many_arguments)]
fn repeat_loop_to_target_with(
    first: Route,
    target_len_m: f64,
    seed: u64,
    mut planner: Option<&mut dyn FnMut(u64) -> Option<Route>>,
) -> Result<LoopRoute, String> {
    let mut route = first;
    // 多圈时逐圈重规划：把每一圈的路线拼成一条长路线。
    if let Some(plan) = planner.as_mut() {
        let lap_m = route.length_m;
        if lap_m <= 1.0 {
            return Err("闭环长度过短，无法重复".into());
        }
        if target_len_m > lap_m * 1.05 {
            let laps = (target_len_m / lap_m) as usize;
            let mut all: Vec<RoutePoint> = Vec::with_capacity(route.points.len() * laps);
            let mut acc = 0.0f64;
            for k in 0..laps {
                let lap_route = if k == 0 {
                    route.clone()
                } else {
                    match plan(seed.wrapping_add(k as u64 * 0x9E37_79B9_7F4A_7C15)) {
                        Some(r) if r.points.len() >= 3 => r,
                        _ => route.clone(), // 规划失败就复用首圈，保证能出结果
                    }
                };
                for p in &lap_route.points {
                    let mut q = *p;
                    q.s = acc;
                    if let Some(prev) = all.last() {
                        let prev: &RoutePoint = prev;
                        let m_lat = MET_PER_DEG_LAT;
                        let m_lon = MET_PER_DEG_LNG * q.lat.to_radians().cos().max(0.2);
                        let d = ((q.lon - prev.lon) * m_lon).hypot((q.lat - prev.lat) * m_lat);
                        acc += d;
                        q.s = acc;
                    }
                    all.push(q);
                }
            }
            route.points = all;
            route.length_m = acc;
        }
    }
    repeat_loop_to_target(route, target_len_m, seed)
}

fn repeat_loop_to_target(mut route: Route, target_len_m: f64, seed: u64) -> Result<LoopRoute, String> {
    let lap_m = route.length_m;
    if lap_m <= 1.0 {
        return Err("闭环长度过短，无法重复".into());
    }
    if target_len_m <= lap_m * 1.05 {
        // 目标距离本来就在一圈之内，无需重复。
        return Ok(LoopRoute { route, lap_m });
    }
    let laps = (target_len_m / lap_m).floor() as usize;
    let remainder = target_len_m - laps as f64 * lap_m;
    let base: Vec<RoutePoint> = route.points.clone();
    let mut points = Vec::with_capacity(base.len() * laps + base.len());
    // 每圈加**横向 + 切向偏移**：真实跑者不会每圈精确踩同一条线，同心圆一样
    // 完美重叠的轨迹一眼就假。校园跑一圈往往 500–900m，跑 2–3km 就是 3–5 圈，
    // 每圈偏 1–1.5m 在地图上能清楚看出是多圈叠在一起。
    //
    // 偏移量由 seed 派生（可复现），并在圈首尾用 sin² 包络归零，保证接缝连续。
    // `s` 仍沿用原始值 + 圈偏移（不能重算：`base` 是平滑后的采样，首尾可能
    // 不严格闭合，重算会比 `route.length_m` 短，实测 800.8 → 711）。
    let (c_lat, c_lng) = {
        let n = base.len().max(1) as f64;
        (
            base.iter().map(|p| p.lat).sum::<f64>() / n,
            base.iter().map(|p| p.lon).sum::<f64>() / n,
        )
    };
    // 偏移用小质数派生，与路线 seed 正交，避免和几何选择耦合。
    let mut off_rng = Rng::new(seed ^ 0x5DEE_CE66_D1F3_1A2B);
    // 每圈的 (横向基础偏移, 横向波浪幅度, 切向基础偏移, 相位)
    let lap_offsets: Vec<(f64, f64, f64, f64)> = (0..laps)
        .map(|_| {
            (
                // 横向换道量：校园路宽 3–5m，跑者可在一半路宽内换道。
                // ±0.25m 基础 + ≤0.12m 波浪 = 最大 0.37m，加上 GPS 抖动仍在
                // 路面内；地图放大后能看出"每圈不是同一条线"。
                off_rng.uniform(-0.25, 0.25),
                off_rng.uniform(0.05, 0.12),
                off_rng.uniform(-0.15, 0.15),
                off_rng.uniform(0.0, std::f64::consts::TAU),
            )
        })
        .collect();
    for lap in 0..laps {
        let (base_off, wave_amp, along_off, phase) = lap_offsets[lap];
        // 闭合环的首尾重复点：最后一点与第一点位置相同，偏移必须一致。
        let n_uniq = if base.len() > 1
            && (base[0].lat - base[base.len() - 1].lat).abs() < 1e-9
            && (base[0].lon - base[base.len() - 1].lon).abs() < 1e-9
        {
            base.len() - 1
        } else {
            base.len()
        };
        for (i, p) in base.iter().enumerate() {
            let mut q = *p;
            q.s += lap as f64 * lap_m;
            // 圈内比例 u∈[0,1]；首尾用 sin²(πu) 包络归零 → 接缝天然连续。
            let u = (i.min(n_uniq) as f64) / n_uniq.max(1) as f64;
            let env = (std::f64::consts::PI * u).sin().powi(2);
            let wave = wave_amp * (std::f64::consts::TAU * 2.0 * u + phase).sin();
            let lateral = (base_off + wave) * env;
            let along = along_off * env;
            // 单位切向（用前后点差分）；首尾用相邻点兜底。
            let (pv, nx) = if i == 0 {
                (base[0], base[1.min(base.len() - 1)])
            } else {
                (base[i - 1], base[i])
            };
            let m_lat = MET_PER_DEG_LAT;
            let m_lon = MET_PER_DEG_LNG * q.lat.to_radians().cos().max(0.2);
            let tx = (nx.lon - pv.lon) * m_lon;
            let ty = (nx.lat - pv.lat) * m_lat;
            let tlen = tx.hypot(ty).max(1e-9);
            let (tx, ty) = (tx / tlen, ty / tlen);
            // 左法线
            let (lx, ly) = (-ty, tx);
            q.lat += (ly * lateral + ty * along) / m_lat;
            q.lon += (lx * lateral + tx * along) / m_lon;
            // 圆心兜底：极端情况下包络计算异常，不让点飞出去。
            let _ = (c_lat, c_lng);
            points.push(q);
        }
    }
    // 末段：从**闭环起点**再走 remainder 米。
    //
    // 不能在 `base.windows(2)` 里直接找 `b.s >= remainder` —— 那找的是「首圈
    // 弧长 remainder 处」的点，位置在环的**另一侧**；上一圈末点却是环的起点，
    // 两者相差可达环的直径（实测 244m 跳变）。正确做法是从 `base[0]` 起算。
    // 末段：从闭环起点起再走 remainder 米，**按完整采样点插入**。
    //
    // 只插一个终点是不够的：上一圈的末点在环的起点，末段终点在环的另一侧
    // （弧长 remainder 处），两点**直线距离可达环的直径**（实测 244m）。
    // 下游按弧长在相邻采样点间线性插值，会在这一大段里直接切过环内部。
    // 因此这里把末段整段的采样点都补进来（与 base 同样的密度）。
    if remainder > 0.5 {
        let base_start_s = laps as f64 * lap_m;
        for pair in base.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if a.s >= remainder {
                break;
            }
            let s_here = base_start_s + a.s;
            points.push(RoutePoint {
                lat: a.lat,
                lon: a.lon,
                s: s_here,
                ..a
            });
            if b.s >= remainder {
                let k = ((remainder - a.s) / (b.s - a.s)).clamp(0.0, 1.0);
                points.push(RoutePoint {
                    lat: a.lat + (b.lat - a.lat) * k,
                    lon: a.lon + (b.lon - a.lon) * k,
                    s: base_start_s + remainder,
                    ..a
                });
                break;
            }
        }
    }
    route.length_m = laps as f64 * lap_m + remainder.min(lap_m);
    route.points = points;
    Ok(LoopRoute { route, lap_m })
}

/// 真实道路模式随机选中的普通打卡点数量的默认值（运行时可变，别写死）。
pub const DEFAULT_ORDINARY_PICK: usize = 2;

/// 从点位挑选参与路线的硬点：必经点（`fixed`，全部）+ 随机 `pick` 个普通点
/// （`ordinary`）。结果按质心角排序成自然环序（`radial_order`），首个即起点。
/// 随机性由 `seed` 决定，保证预览与实际运行可复现且一致。
pub fn select_route_points(
    fixed: &[(f64, f64)],
    ordinary: &[(f64, f64)],
    seed: u64,
    pick: usize,
) -> Vec<(f64, f64)> {
    use rand::seq::SliceRandom;
    use rand::SeedableRng;
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed ^ 0x5EED_C0FF_EE00_1234);
    let mut pool = ordinary.to_vec();
    pool.shuffle(&mut rng);
    let near = |a: &(f64, f64), b: &(f64, f64)| {
        (a.0 - b.0).abs() < 1e-6 && (a.1 - b.1).abs() < 1e-6
    };
    let mut selected: Vec<(f64, f64)> = fixed.to_vec();
    for p in pool.into_iter().take(pick) {
        if !selected.iter().any(|q| near(q, &p)) {
            selected.push(p);
        }
    }
    radial_order(&selected)
}

/// 按质心角度排序打卡点，形成自然环序。
pub fn radial_order(points_bd: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = points_bd.len();
    if n == 0 {
        return Vec::new();
    }
    let (cx, cy) = (
        points_bd.iter().map(|q| q.0).sum::<f64>() / n as f64,
        points_bd.iter().map(|q| q.1).sum::<f64>() / n as f64,
    );
    let mut ordered = points_bd.to_vec();
    ordered.sort_by(|a, b| {
        (a.0 - cx)
            .atan2(a.1 - cy)
            .partial_cmp(&(b.0 - cx).atan2(b.1 - cy))
            .unwrap()
    });
    ordered
}

/// 路线 → 平面折线 + 弧长表。
///
/// `arcs[k]` 必须与 `dense[k]` 一一对应（`ring_point_at` 按同下标取弧长），
/// 因此直接取每个采样点的累计弧长 `p.s`（`dense` 末点已闭合回起点）。
/// 路线 → 单圈环 + 单圈弧长表。
///
/// `ring_point_at` 按 `arcs` 做 `rem_euclid` 后再二分查找 `dense`，所以这两个
/// 数组必须**严格描述同一圈**。跑道模式会把路线重复成多圈，若直接把多圈的
/// 累计弧长塞进来，取模后的弧长会去匹配单圈外的索引，插值位置整体错位
/// （表现为轨迹斜穿到环内部）。因此这里只取第一圈。
fn route_ring(route: &Route, lap_m: f64, c_lat: f64, c_lng: f64) -> (Vec<(f64, f64)>, Vec<f64>) {
    // **保留全部圈**：跑道模式下每圈带独立横向偏移（`plan_track_loop` 生成），
    // 只取第一圈会把偏移全部丢掉，轨迹退化成一条完美重叠的线。
    //
    // 但 `ring_point_at` 会按 `arcs.last()` 取模，多圈弧长塞进去会索引错位
    // （实测偏离路网 100m）。所以这里分成两种模式：
    // * 单圈（len(route) ≈ 一圈）：照旧走取模，末端自然回绕；
    // * 多圈：整段作为一条开放折线，`ring_point_at` 按绝对弧长查表，不做取模。
    //   为此把末点弧长设为「最后一个点的 s」，让 `s` 到达目标距离前不会回绕。
    let total_s = route.points.last().map(|p| p.s).unwrap_or(lap_m);
    let lap: &[route_planner::RoutePoint] = if total_s > lap_m * 1.5 {
        &route.points[..]
    } else {
        // 单圈：截到 lap_m 即可（保持原行为）。
        let mut lap_end = route.points.len();
        for (i, p) in route.points.iter().enumerate() {
            if p.s >= lap_m - 1e-6 {
                lap_end = i + 1;
                break;
            }
        }
        &route.points[..lap_end.max(2).min(route.points.len())]
    };
    let dense: Vec<(f64, f64)> = lap
        .iter()
        .map(|p| {
            (
                (p.lon - c_lng) * MET_PER_DEG_LNG,
                (p.lat - c_lat) * MET_PER_DEG_LAT,
            )
        })
        .collect();
    // `arcs` 用**实际几何累加**，而不是 `p.s`：
    // 多圈时 `p.s` 是每圈独立生成的（含横向偏移），与实际折线长度有细微差异；
    // 用 `p.s` 做二分查表会落到错误的段上（实测轨迹斜穿环内部 97m）。
    let mut arcs = Vec::with_capacity(dense.len());
    let mut acc = 0.0f64;
    for (i, &(x, y)) in dense.iter().enumerate() {
        if i == 0 {
            arcs.push(0.0);
        } else {
            let (px, py) = dense[i - 1];
            acc += (x - px).hypot(y - py);
            arcs.push(acc);
        }
    }
    (dense, arcs)
}

/// 模式 B：真实道路拓扑路由轨迹生成。
///
/// `route_bd[0]` 为起点，`route_bd[1..]` 为软引导点（用于方向锚点，不必全经过）；
/// `must_bd` 为强制必经点（按序，可为空，最终必达）；吸附在 `route_bd` 全量上做。
pub fn build_road(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route_bd: &[(f64, f64)],
    must_bd: &[(f64, f64)],
    net: &RoadGraph,
) -> Result<Track, String> {
    build_road_with_color_mode(
        dist,
        dur,
        seed,
        start_ms,
        route_bd,
        must_bd,
        net,
        TrackColorMode::FullGreen,
    )
}

pub fn build_road_with_color_mode(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route_bd: &[(f64, f64)],
    must_bd: &[(f64, f64)],
    net: &RoadGraph,
    color_mode: TrackColorMode,
) -> Result<Track, String> {
    build_road_mode(dist, dur, seed, start_ms, route_bd, must_bd, net, color_mode, false)
}

/// 沿路网里的真实塑胶跑道跑（跑道模式主路径）。
///
/// 直接在 `net` 上找 `surface=tartan` 的闭环并反复绕，不借用校园其它道路。
pub fn build_track_loop(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    near: (f64, f64),
    net: &RoadGraph,
    color_mode: TrackColorMode,
) -> Result<Track, String> {
    let loop_route = route_planner::plan_track_loop(net, Coord::new(near.1, near.0), dist, seed)?;
    build_from_route_at(
        dist, dur, seed, start_ms, &loop_route.route, loop_route.lap_m,
        near.0, near.1, net, color_mode, &[], &[],
    )
}

/// 模式 B 的完整实现；`loop_mode` 为真时规划一条闭环并重复到目标距离
/// （跑道模式），不做补距游走。
#[allow(clippy::too_many_arguments)]
pub fn build_road_mode(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route_bd: &[(f64, f64)],
    must_bd: &[(f64, f64)],
    net: &RoadGraph,
    color_mode: TrackColorMode,
    loop_mode: bool,
) -> Result<Track, String> {
    let _rng = Rng::new(seed);
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;
    let radius = (avg_v * avg_v / a_c_max).max(6.0);
    let opts = RouteOptions {
        repeat_checkpoint_loop: loop_mode,
        a_c_max,
        min_radius_m: radius,
        ..Default::default()
    };

    let waypoints: Vec<Coord> = route_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let must: Vec<Coord> = must_bd.iter().map(|p| Coord::new(p.1, p.0)).collect();
    let planned = plan_route_split(net, &waypoints, &must, dist, seed, &opts)?;
    let (route, lap_m) = if loop_mode {
        // 每圈重新规划：用不同 seed 调 `plan_route_split`，得到不同的路口组合。
        // 这样地图上能看到多圈叠在一起，而不是一条完美重合的线。
        let net_ref = net;
        let wp_ref = &waypoints;
        let must_ref = &must;
        let opts_ref = &opts;
        let lap_len_hint = planned.length_m;
        let mut planner = |lap_seed: u64| -> Option<Route> {
            plan_route_split(
                net_ref, wp_ref, must_ref, lap_len_hint, lap_seed, opts_ref,
            ).ok()
        };
        let lr = repeat_loop_to_target_with(planned, dist, seed, Some(&mut planner))?;
        (lr.route, lr.lap_m)
    } else {
        let m = planned.length_m;
        (planned, m)
    };

    let n_pts = route_bd.len().max(1);
    let (c_lat, c_lng) = (
        route_bd.iter().map(|q| q.0).sum::<f64>() / n_pts as f64,
        route_bd.iter().map(|q| q.1).sum::<f64>() / n_pts as f64,
    );
    build_from_route_at(dist, dur, seed, start_ms, &route, lap_m, c_lat, c_lng, net, color_mode, route_bd, must_bd)
}

/// 已规划好的路线 → 完整轨迹（时间网格 / 配速 / 抖动 / 协议字段）。
///
/// `lap_m` 是单圈长度（跑道模式重复多圈时用于切环），`(c_lat,c_lng)` 是
/// 路线所在局部平面的原点。
#[allow(clippy::too_many_arguments)]
fn build_from_route_at(
    dist: f64,
    dur: i64,
    seed: u64,
    start_ms: i64,
    route: &Route,
    lap_m: f64,
    c_lat: f64,
    c_lng: f64,
    net: &RoadGraph,
    color_mode: TrackColorMode,
    route_bd: &[(f64, f64)],
    must_bd: &[(f64, f64)],
) -> Result<Track, String> {
    let mut rng = Rng::new(seed);
    let dur_f = dur as f64;
    let avg_v = dist / dur_f;
    let a_c_max = 2.5;
    let (dense, arcs) = route_ring(route, lap_m, c_lat, c_lng);
    // 多圈轨迹（跑道模式）每圈形状不同，必须按开放折线取点。
    let multi_lap = route.points.last().map(|p| p.s).unwrap_or(lap_m) > lap_m * 1.5;

    // 时间网格（同经典模式）
    let mut times = Vec::new();
    let mut t = 0.0;
    while t < dur_f {
        times.push(t);
        t += if rng.random() < 0.80 {
            5.0
        } else {
            rng.choice(&[1.0, 2.0, 3.0, 4.0, 6.0, 7.0, 8.0])
        };
    }
    let n = times.len();
    let mut dts: Vec<f64> = (0..n - 1).map(|i| times[i + 1] - times[i]).collect();
    dts.push(1.0f64.max(dur_f - times[n - 1]));

    // 运动学速度剖面：OU 配速 + 弯道预判刹车 + 精确命中目标距离
    let kin = KinParams {
        a_c_max,
        ..Default::default()
    };
    let speed_seed = seed.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let w = pace_profile(
        &route,
        avg_v,
        &dts,
        dist,
        speed_seed,
        &kin,
        SPEED_FLOOR,
        SPEED_CEIL,
    );
    let speeds: Vec<f64> = w.clone();
    let seg_dist: Vec<f64> = (0..n).map(|i| w[i] * dts[i]).collect();

    // 点位类型（同经典模式）
    let kinds: Vec<(i64, i64)> = match color_mode {
        TrackColorMode::FullGreen => vec![(0, 1); n],
        TrackColorMode::HalfGreenGray => {
            let tl_w = [
                ((-1i64, 4i64), 54u32),
                ((-1, 1), 27),
                ((-1, 12), 13),
                ((-1, 5), 3),
                ((-1, 6), 2),
            ];
            let mut kinds: Vec<(i64, i64)> = Vec::with_capacity(n);
            for _ in 0..n {
                let u = rng.random();
                if u < 0.39 {
                    kinds.push((3, 1));
                } else if u < 0.93 {
                    kinds.push((0, 1));
                } else if u < 0.96 {
                    kinds.push(rng.weighted(&tl_w));
                } else {
                    kinds.push((rng.choice(&[1, 1, 1, 1, 2, 2]), 1));
                }
            }
            for i in 1..kinds.len() {
                let prev_drift = (-1 == kinds[i - 1].0)
                    || kinds[i - 1].0 == 5
                    || kinds[i - 1].0 == 6;
                if kinds[i].0 != -1 && prev_drift && rng.random() < 0.08 {
                    kinds[i] = (if rng.random() < 0.75 { 7 } else { 8 }, 1);
                }
            }
            for i in 1..n {
                if kinds[i].0 == -1 && kinds[i - 1].0 == -1 {
                    kinds[i] = (rng.choice(&[3, 0]), 1);
                }
            }
            kinds
        }
    };
    let normal_idx: Vec<usize> = (0..n).filter(|&i| kinds[i].0 != -1 && i > 1).collect();
    let share: f64 = normal_idx.iter().map(|&i| seg_dist[i]).sum();
    let share = if share == 0.0 { 1.0 } else { share };
    let mut disp_of = vec![0.0f64; n];
    for &i in &normal_idx {
        disp_of[i] = seg_dist[i] * (dist / share);
    }

    // 建筑 SDF（自适应放大漂移）
    let sdf = Sdf::from_buildings(
        &net.buildings,
        |c| {
            [
                (c.lon - c_lng) * MET_PER_DEG_LNG,
                (c.lat - c_lat) * MET_PER_DEG_LAT,
            ]
        },
        2.0,
        30.0,
    );

    // 生物力学：步频在目标值附近 OU 波动，步幅反解严格满足 v=(cad/60)·stride
    let targets: Vec<f64> = (0..n)
        .map(|i| {
            let km_done = (times[i] / dur_f) * dist / 1000.0;
            gait(speeds[i], fatigue_from_km(km_done)).cadence
        })
        .collect();
    let cadence_seed = seed.wrapping_add(0xC6A4_A793_5BD1_E995);
    let cadences = ou_cadence_series(&targets, &dts, cadence_seed, 0.25, 0.5, 110.0, 220.0);

    // GPS 抖动：AR(1) 相关漂移 + 高斯测量噪声（SDF 逐点放大）。
    // 全绿色模式下改用小抖动（见 generator::GREEN_JITTER_*）：着色只看
    // type/locType/state/bdG，抖动不会让轨迹变灰，但过大的漂移会飘出跑道。
    let mut jitter = if color_mode == TrackColorMode::FullGreen {
        GpsJitter::new(
            0.72,
            crate::track::generator::GREEN_JITTER_SIGMA_CORR,
            crate::track::generator::GREEN_JITTER_SIGMA_MEAS,
        )
    } else {
        GpsJitter::new(0.72, 0.75, 0.3)
    };

    let mut locs: Vec<GenPoint> = Vec::with_capacity(n);
    let mut s = 0.0f64;
    let mut t_acc = 0.0f64;
    let mut dist_acc = 0.0f64;
    let mut steps_acc = 0.0f64;
    let mut alt = 82.0 + rng.uniform(-1.0, 1.0);
    let n_est = 1.max((dur_f / 5.0) as i64);
    let alt_sigma = rng.uniform(3.8, 6.2) / (0.40 * n_est as f64);
    let mut ten_t = 0.0f64;
    let mut ten_d = 0.0f64;
    let mut ten_st = 0.0f64;
    let mut ten_speed: Vec<TenWindow> = Vec::new();
    let mut ten_steps: Vec<TenWindow> = Vec::new();
    // 每点相对路线的米制抖动：吸附时保留，避免出现"完美贴合点"。
    let mut jitter_m: Vec<(f64, f64)> = Vec::with_capacity(n);

    for i in 0..n {
        let dt = dts[i];
        let (typ, lt) = kinds[i];
        t_acc += dt;
        // 多圈（每圈形状不同）时按开放折线取点，不能取模回绕到第一圈。
        let wrap = multi_lap == false;
        let pos = |ss: f64| crate::track::geom::polyline_point_at(&dense, &arcs, ss, wrap);
        let mut d_step = 0.0f64;
        let px;
        let py;
        let x;
        let y;
        let rad;
        let state;
        if typ != -1 {
            d_step = disp_of[i];
            s += d_step;
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            // AR(1) 相关漂移 + 高斯测量噪声，SDF 自适应放大。
            // 全绿色模式：SDF 放大封顶（近墙本可放大 3×），合位移截断在车道内。
            let (jxd, jyd) = if color_mode == TrackColorMode::FullGreen {
                let sc = sdf
                    .sigma_scale([bx, by])
                    .min(crate::track::generator::GREEN_JITTER_SDF_CAP);
                let z = [
                    rng.gauss(0.0, 1.0),
                    rng.gauss(0.0, 1.0),
                    rng.gauss(0.0, 1.0),
                    rng.gauss(0.0, 1.0),
                ];
                let (dx, dy) = jitter.step(sc, z);
                crate::track::generator::clamp_green_jitter(dx, dy)
            } else {
                let sc = sdf.sigma_scale([bx, by]);
                let z = [
                    rng.gauss(0.0, 1.0),
                    rng.gauss(0.0, 1.0),
                    rng.gauss(0.0, 1.0),
                    rng.gauss(0.0, 1.0),
                ];
                jitter.step(sc, z)
            };
            px = bx + jxd;
            py = by + jyd;
            jitter_m.push((jxd, jyd));
            rad = round_to(
                if typ == 3 {
                    rng.uniform(1.4, 5.1)
                } else {
                    rng.uniform(1.4, 2.4)
                },
                2,
            );
            state = if color_mode == TrackColorMode::FullGreen {
                1
            } else if typ == 0 {
                rng.weighted(&[(1, 145), (2, 45), (3, 164)])
            } else {
                rng.weighted(&[(1, 145), (2, 256), (3, 151)])
            };
        } else {
            if rng.random() >= 0.20 {
                d_step = rng.uniform(0.0, MAX_EXCEPTION_DRIFT_M);
            }
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            if d_step > 0.0 {
                let ang = rng.uniform(0.0, std::f64::consts::TAU);
                px = x + d_step * ang.sin();
                py = y + d_step * ang.cos();
            } else if let Some(last) = locs.last() {
                py = (last.gLat - c_lat) * MET_PER_DEG_LAT;
                px = (last.gLng - c_lng) * MET_PER_DEG_LNG;
            } else {
                let (jx, jy) = jitter.state();
                px = x + jx;
                py = y + jy;
            }
            // 吸附保留的是相对路线上的偏移；异常点没有同源抖动，按实际偏移存。
            jitter_m.push((px - x, py - y));
            rad = if typ == -1 {
                round_to(rng.uniform(3.0, 10.0), 2)
            } else if lt == 4 {
                round_to(
                    if rng.random() < 0.75 {
                        rng.uniform(30.0, 100.0)
                    } else {
                        rng.uniform(100.0, 550.0)
                    },
                    2,
                )
            } else if lt == 1 {
                round_to(
                    if rng.random() < 0.75 {
                        rng.uniform(1.6, 12.0)
                    } else {
                        rng.uniform(12.0, 95.0)
                    },
                    2,
                )
            } else if lt == 12 {
                round_to(rng.uniform(30.0, 125.0), 2)
            } else if lt == 5 {
                round_to(rng.uniform(25.0, 300.0), 2)
            } else {
                550.0
            };
            state = rng.weighted(&[(1, 102), (2, 124), (3, 136)]);
        }
        if typ != -1 {
            dist_acc += d_step;
        }
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt += 0.04 * (82.0 - alt) + rng.gauss(0.0, alt_sigma);
        // 生物力学联动：v=(cad/60)·stride
        let v_now = speeds[i];
        let mut stride = v_now * 60.0 / cadences[i];
        stride = stride.clamp(0.5, 1.7);
        let cad = v_now * 60.0 / stride;
        steps_acc += cad / 60.0 * dt;
        let nxt = pos(s + 2.0);
        let brg =
            ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 35.0)).rem_euclid(360.0);
        let (avg_sp, gps_speed) = if typ == -1 {
            let avg = round_to(dist_acc / t_acc.max(1.0), 4);
            let gps = rng.uniform(0.5, MAX_EXCEPTION_GPS_SPEED_KMH);
            (avg, round_to(gps, 4))
        } else {
            let avg = round_to(d_step / dt, 4);
            let kmh = avg * 3.6;
            let sigma = (kmh * 0.08).max(0.05);
            let gps = if rng.random() < 0.20 {
                0.0
            } else {
                round_to((kmh + rng.gauss(0.0, sigma)).max(0.0), 4)
            };
            (avg, gps)
        };

        ten_t += dt;
        ten_d += speeds[i] * dt;
        ten_st += cad / 60.0 * dt;
        while ten_t >= 10.0 {
            let k = 10.0 / ten_t;
            let (out_d, out_st) = (ten_d * k, ten_st * k);
            ten_speed.push(TenWindow {
                time: 10,
                value: round_to(out_d, 2),
            });
            ten_steps.push(TenWindow {
                time: 10,
                value: round_to(out_st, 0),
            });
            ten_t -= 10.0;
            ten_d -= out_d;
            ten_st -= out_st;
        }
        locs.push(GenPoint {
            id: i as i64 + 1,
            flag: start_ms,
            lat: -1.0,
            lng: -1.0,
            gLat: round_to(lat, 7),
            gLng: round_to(lng, 7),
            speed: round_to(gps_speed, 4),
            avgSpeed: avg_sp,
            radius: rad,
            accuracy: rad,
            ptype: typ,
            locType: lt,
            hasAltitude: true,
            totalTime: round_to(t_acc, 0) as i64,
            totalDis: round_to(dist_acc, 4),
            validDis: round_to(dist_acc, 4),
            validTime: round_to(t_acc, 0) as i64,
            steps: steps_acc as i64,
            stepDistance: 0.0,
            gainTime: fmt_gain_time(start_ms + (t_acc * 1000.0) as i64),
            gainTimeMs: start_ms + (t_acc * 1000.0) as i64,
            queueNum: 0,
            coorType: "gcj02".into(),
            bdA: round_to(alt, 2),
            bdD: round_to(brg, 2),
            bdS: round_to((avg_sp * rng.uniform(0.6, 0.95)).max(0.0), 3),
            bdG: if color_mode == TrackColorMode::FullGreen {
                1
            } else {
                rng.choice(&[1, 1, 1, -1])
            },
            count: rng.randint(20, 88),
            dtr: 0.0,
            state,
            locationId: String::new(),
        });
    }
    if ten_t > 1.0 {
        let k = (10.0 / ten_t).min(1.4);
        ten_speed.push(TenWindow {
            time: 10,
            value: round_to(ten_d * k, 2),
        });
        ten_steps.push(TenWindow {
            time: 10,
            value: round_to(ten_st * k, 0),
        });
    }

    let mut segments: Vec<Segment> = Vec::new();
    let (mut seg_t, mut seg_d) = (0.0f64, 0.0f64);
    let mut seg_v: Vec<f64> = Vec::new();
    for i in 0..n {
        seg_t += dts[i];
        seg_d += seg_dist[i];
        seg_v.push(speeds[i]);
        if seg_t >= 60.0 || i == n - 1 {
            segments.push(Segment {
                totalTime: round_to(seg_t, 0) as i64,
                distance: round_to(seg_d, 0) as i64,
                startTime: round_to((times[i] - seg_t) * 1000.0, 0) as i64,
                endTime: round_to(times[i] * 1000.0, 0) as i64,
                avgSpeed: round_to(seg_v.iter().sum::<f64>() / seg_v.len() as f64, 3),
                avgStep: round_to(steps_acc / 1.0f64.max(t_acc) * 60.0, 0) as i64,
                state: 0,
            });
            seg_t = 0.0;
            seg_d = 0.0;
            seg_v.clear();
        }
    }

    apply_post_fixes(&mut locs, &mut rng, start_ms);

    if color_mode == TrackColorMode::FullGreen {
        let last_index = locs.len().saturating_sub(1);
        for (i, point) in locs.iter_mut().enumerate() {
            point.ptype = if i == 1 {
                5
            } else if i == last_index {
                6
            } else {
                0
            };
            point.locType = 1;
            point.state = 1;
            point.bdG = 1;
        }
    }

    // 点位吸附：<40m 落位到打卡点，但保留该点原有抖动，避免地图上出现
    // 「完美贴合点」。全绿色模式把抖动截断在车道宽度内；混合模式保持
    // 原有精确落位（其抖动本来更大，吸附点很少触发）。
    for pl in route_bd {
        let mut best_i = None;
        let mut best_d = 1e18f64;
        for (i, q) in locs.iter().enumerate() {
            let dd = ((q.gLat - pl.0) * MET_PER_DEG_LAT).powi(2)
                + ((q.gLng - pl.1) * MET_PER_DEG_LNG).powi(2);
            if dd < best_d {
                best_d = dd;
                best_i = Some(i);
            }
        }
        if let Some(i) = best_i {
            if best_d < 40.0 * 40.0 {
                if color_mode == TrackColorMode::FullGreen {
                    let (jx, jy) = crate::track::generator::clamp_green_jitter(
                        jitter_m[i].0,
                        jitter_m[i].1,
                    );
                    locs[i].gLat = round_to(pl.0 + jy / MET_PER_DEG_LAT, 7);
                    locs[i].gLng = round_to(pl.1 + jx / MET_PER_DEG_LNG, 7);
                } else {
                    locs[i].gLat = round_to(pl.0, 7);
                    locs[i].gLng = round_to(pl.1, 7);
                }
            }
        }
    }

    let total_dis = round_to(dist, 3);
    Ok(Track {
        totalTime: round_to(t_acc, 0) as i64,
        totalDistance: total_dis,
        validDistance: total_dis,
        validTime: round_to(t_acc, 0) as i64,
        startTime: start_ms,
        startLatitude: locs[0].gLat,
        startLongitude: locs[0].gLng,
        totalSteps: steps_acc as i64,
        locations: locs,
        speedPerTenSec: ten_speed,
        stepsPerTenSec: ten_steps,
        segments,
    })
}

#[cfg(test)]
mod quality_tests {
    use super::repeat_ratio;

    /// 一条直线：不应判定为重复。
    #[test]
    fn straight_line_has_no_repeats() {
        let pts: Vec<(f64, f64)> = (0..100).map(|i| (39.9 + i as f64 * 1e-5, 116.4)).collect();
        assert!(repeat_ratio(&pts) < 0.05, "直线被误判为重复: {}", repeat_ratio(&pts));
    }

    /// 同一条路来回走：重复率应显著升高。
    #[test]
    fn back_and_forth_is_detected() {
        let mut pts: Vec<(f64, f64)> = (0..60).map(|i| (39.9 + i as f64 * 1e-5, 116.4)).collect();
        let back: Vec<(f64, f64)> = (0..60).rev().map(|i| (39.9 + i as f64 * 1e-5, 116.4)).collect();
        pts.extend(back);
        let ratio = repeat_ratio(&pts);
        assert!(ratio > 0.5, "折返未被识别: {ratio}");
    }

    /// 环线：去程与回程走不同道路，不应算重复。
    #[test]
    fn loop_without_overlap_is_clean() {
        let mut pts = Vec::new();
        for i in 0..50 {
            pts.push((39.9, 116.4 + i as f64 * 1e-5));
        }
        for i in 0..50 {
            pts.push((39.9 + i as f64 * 1e-5, 116.4005));
        }
        for i in (0..50).rev() {
            pts.push((39.9005, 116.4 + i as f64 * 1e-5));
        }
        for i in (0..50).rev() {
            pts.push((39.9 + i as f64 * 1e-5, 116.4));
        }
        let ratio = repeat_ratio(&pts);
        assert!(ratio < 0.2, "方形环线被误判: {ratio}");
    }
}

#[cfg(test)]
mod loop_mode_tests {
    use super::*;

    fn square_net() -> RoadGraph {
        use route_planner::graph::EdgeData;
        let mut g = RoadGraph::new();
        // 200m × 200m 方形环路（校园环校路），每个节点约 200m 间隔。
        let s = 0.0018;
        let a = g.add_node(Coord::new(0.0, 0.0));
        let b = g.add_node(Coord::new(s, 0.0));
        let c = g.add_node(Coord::new(s, s));
        let d = g.add_node(Coord::new(0.0, s));
        let e = |w| EdgeData {
            way_id: w,
            oneway: false,
            highway: "residential".into(),
            maxspeed_kmh: None,
            length_m: 200.0,
        };
        for (x, y, w) in [(a, b, 1), (b, c, 2), (c, d, 3), (d, a, 4)] {
            g.add_edge(x, y, e(w));
            g.add_edge(y, x, e(w));
        }
        g.compute_anchor();
        g.rebuild_index();
        g
    }

    /// 全绿色模式：小抖动不改变着色，且合位移落在车道宽度内。
    ///
    /// 着色四字段（type/locType/state/bdG）与抖动无关，抖动只影响坐标；
    /// 这里同时验证两件事，防止有人日后又把抖动清零、或把幅度调回 0.75。
    #[test]
    fn full_green_keeps_colors_and_applies_small_jitter() {
        let net = square_net();
        let pts = [(0.0001, 0.0001), (0.0001, 0.0017), (0.0017, 0.0017)];
        let track = build_road_mode(
            2000.0, 720, 42, 1_700_000_000_000, &pts, &pts[1..], &net,
            TrackColorMode::FullGreen, true,
        ).expect("跑道模式应能生成");

        let last = track.locations.len() - 1;
        for (i, p) in track.locations.iter().enumerate() {
            let expected_type = if i == 1 { 5 } else if i == last { 6 } else { 0 };
            assert_eq!(p.ptype, expected_type, "点 {i} 类型被抖动影响");
            assert_eq!(p.locType, 1, "点 {i} locType");
            assert_eq!(p.state, 1, "点 {i} state");
            assert_eq!(p.bdG, 1, "点 {i} bdG");
        }

        // 相邻点位移扣除步长后即为抖动跳变；直接看与上一轮重采样前的差值
        // 不稳定，改为验证相邻点间距分布：全绿抖动被截断后，连续两点不应
        // 出现超过 ±0.6m×2 的横向跳变。
        let mut max_jump = 0.0f64;
        for w in track.locations.windows(2) {
            let dx = (w[1].gLng - w[0].gLng) * MET_PER_DEG_LNG;
            let dy = (w[1].gLat - w[0].gLat) * MET_PER_DEG_LAT;
            max_jump = max_jump.max((dx * dx + dy * dy).sqrt());
        }
        assert!(max_jump < 30.0, "相邻点跳变 {max_jump:.1}m");

        // 抖动没有再次被清零：除吸附到打卡点的少数点外，多数点偏离路缘 >3cm，
        // 且偏离不超过一条车道。吸附点（最多 pts.len() 个）允许落到打卡点附近。
        let off: Vec<f64> = track.locations.iter().map(|p| {
            let c = Coord::new(p.gLng, p.gLat);
            net.nearest_edge(net.to_m(c)).map(|(_,_,_,d)| d).unwrap_or(0.0)
        }).collect();
        let near_road: Vec<f64> = off.iter().copied().filter(|&d| d <= 1.0).collect();
        assert!(
            off.len() - near_road.len() <= pts.len() + 2,
            "非吸附点离路网过远：{} / {}",
            off.len() - near_road.len(),
            off.len()
        );
        let moved = near_road.iter().filter(|&&d| d > 0.03).count();
        assert!(
            moved * 2 > near_road.len(),
            "全绿轨迹几乎没有抖动：{moved} / {} 个点偏移 >3cm",
            near_road.len()
        );
        for &d in &near_road {
            assert!(d < 1.0, "抖动超出一条车道：{d:.2}m");
        }
    }

    /// 跑道模式下，轨迹必须始终贴着路网，不能飞到路外。
    #[test]
    fn loop_track_stays_on_road_network() {
        let net = square_net();
        let pts = [(0.0001, 0.0001), (0.0001, 0.0017), (0.0017, 0.0017)];
        let track = build_road_mode(
            2000.0, 720, 42, 1_700_000_000_000, &pts, &pts[1..], &net,
            TrackColorMode::FullGreen, true,
        ).expect("跑道模式应能生成");

        // 每个轨迹点到路网最近边的距离都应很小（GPS 抖动 + 噪声容许 40m）。
        let max_off = track.locations.iter().map(|p| {
            let c = Coord::new(p.gLng, p.gLat);
            net.nearest_edge(net.to_m(c)).map(|(_,_,_,d)| d).unwrap_or(f64::INFINITY)
        }).fold(0.0f64, f64::max);
        assert!(max_off < 40.0, "轨迹跑出路网最大 {:.0}m", max_off);
        assert!((track.totalDistance - 2000.0).abs() < 20.0,
            "距离 {} != 2000", track.totalDistance);
    }

    /// 普通模式（非跑道）按原逻辑工作，不应被跑道改动影响。
    #[test]
    fn normal_mode_still_uses_walk() {
        let net = square_net();
        let pts = [(0.0001, 0.0001), (0.0001, 0.0017), (0.0017, 0.0017)];
        let track = build_road_mode(
            2000.0, 720, 42, 1_700_000_000_000, &pts, &pts[1..], &net,
            TrackColorMode::FullGreen, false,
        ).expect("普通模式应能生成");
        assert!((track.totalDistance - 2000.0).abs() < 20.0);
    }

    /// repeat_loop_to_target 必须精确贴合目标距离。
    #[test]
    fn repeat_loop_hits_target_length() {
        let net = square_net();
        let pts = [(0.0001, 0.0001), (0.0017, 0.0017)];
        let wps: Vec<Coord> = pts.iter().map(|p| Coord::new(p.1, p.0)).collect();
        let mut opts = RouteOptions::default();
        opts.repeat_checkpoint_loop = true;
        let route = plan_route_split(&net, &wps, &wps[1..], 2000.0, 1, &opts).unwrap();
        let lap = route.length_m;
        let repeated = super::repeat_loop_to_target(route, 2000.0, 42).unwrap();
        assert!((repeated.route.length_m - 2000.0).abs() < 5.0,
            "重复后 {} != 2000 (单圈 {})", repeated.route.length_m, lap);
    }
}


#[cfg(test)]
mod coord_align_tests {
    /// 打印 WGS84→BD09 的偏移量，用于确认路网与打卡点的错位程度。
    #[test]
    fn print_wgs84_bd09_offset() {
        use crate::track::geom::wgs84_to_bd09;
        let (lat, lon) = wgs84_to_bd09(31.5264, 104.6756);
        let dlat = (lat - 31.5264) * 111_132.0;
        let dlon = (lon - 104.6756) * 111_132.0 * 31.5264f64.to_radians().cos();
        println!("WGS84(31.5264,104.6756) -> BD09({:.6},{:.6})", lat, lon);
        println!("偏移: dlat={:.0}m dlon={:.0}m 合{:.0}m", dlat, dlon, (dlat*dlat+dlon*dlon).sqrt());
    }
}

#[cfg(test)]
mod real_data_alignment_tests {
    use super::*;

    /// 用真实数据验证：加载后路网必须覆盖打卡点，否则地图上环会画到路网外。
    ///
    /// 依赖本机 Downloads 下的真实 OSM 文件，默认不跑（`-- --ignored` 启用）。
    /// 不用 `if !exists { return }` 静默跳过——那会在别的机器上伪装成通过。
    #[test]
    #[ignore = "依赖本机真实 OSM 文件；用 cargo test ... -- --ignored 启用"]
    fn loaded_network_overlaps_checkpoints() {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        assert!(
            std::path::Path::new(path).exists(),
            "本测试需要真实 OSM 文件：{path}"
        );
        let net = super::load_network_path(path).expect("加载路网");
        // 用户实际打卡点（BD-09）
        let pts = [
            (31.526444, 104.675651),
            (31.526805, 104.676518),
            (31.527248, 104.676262),
            (31.525924, 104.675929),
            (31.526259, 104.676648),
        ];
        assert!(net.graph.node_count() > 0, "路网为空");
        for &(lat, lon) in &pts {
            let d = net
                .nearest_edge(net.to_m(Coord::new(lon, lat)))
                .map(|(_, _, _, d)| d)
                .unwrap_or(f64::INFINITY);
            println!("点位 ({lat},{lon}) 到最近道路 {d:.0}m");
            assert!(
                d < 100.0,
                "点位 ({lat},{lon}) 距路网 {d:.0}m —— 坐标系没对齐（应为 WGS84→BD09 后的米级）"
            );
        }
    }
}

#[cfg(test)]
mod real_track_loop_tests {
    use super::*;

    fn load() -> Option<RoadGraph> {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        if !std::path::Path::new(path).exists() {
            return None;
        }
        Some(super::load_network_path(path).expect("加载路网"))
    }

    /// 必须能在用户真实 OSM 里找到塑胶跑道闭环。
    #[test]
    fn finds_tartan_track_in_user_osm() {
        let Some(net) = load() else { return };
        // 打卡点质心
        let near = Coord::new(104.6762016, 31.526536);
        let ring = net.find_track_loop(near);
        match ring {
            Some(r) => {
                let per: f64 = r
                    .windows(2)
                    .map(|w| {
                        let a = net.to_m(net.node_coord(w[0]));
                        let b = net.to_m(net.node_coord(w[1]));
                        (a[0] - b[0]).hypot(a[1] - b[1])
                    })
                    .sum();
                println!("找到跑道环：{} 个节点，周长 {:.0}m", r.len() - 1, per);
                assert!(per > 150.0 && per < 600.0, "跑道周长 {per:.0}m 不像跑道");
            }
            None => panic!("用户 OSM 里应该有一条 tartan 跑道"),
        }
    }

    /// 沿跑道生成的轨迹必须全部落在跑道环上（不允许借用其它道路）。
    #[test]
    fn track_loop_track_stays_on_tartan() {
        let Some(net) = load() else { return };
        let near = (31.526536, 104.6762016);
        let track = super::build_track_loop(
            2000.0, 720, 42, 1_700_000_000_000, near, &net, TrackColorMode::FullGreen,
        ).expect("应能沿跑道生成");
        let ring = net.find_track_loop(Coord::new(near.1, near.0)).unwrap();
        let ring_pts: Vec<Coord> = ring.iter().map(|&n| net.node_coord(n)).collect();
        let mut worst = 0.0f64;
        for p in &track.locations {
            let q = Coord::new(p.gLng, p.gLat);
            let d = ring_pts
                .windows(2)
                .map(|w| {
                    // 点到线段距离（米制）
                    let (a, b, p) = (net.to_m(w[0]), net.to_m(w[1]), net.to_m(q));
                    let ab = [b[0] - a[0], b[1] - a[1]];
                    let ap = [p[0] - a[0], p[1] - a[1]];
                    let len2 = ab[0] * ab[0] + ab[1] * ab[1];
                    let t = if len2 > 0.0 {
                        ((ap[0] * ab[0] + ap[1] * ab[1]) / len2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let foot = [a[0] + ab[0] * t, a[1] + ab[1] * t];
                    (p[0] - foot[0]).hypot(p[1] - foot[1])
                })
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(d);
        }
        println!("轨迹点距跑道最大 {worst:.1}m，总距离 {:.0}m", track.totalDistance);
        assert!(worst < 20.0, "轨迹偏离跑道 {worst:.1}m");
        assert!((track.totalDistance - 2000.0).abs() < 15.0);
    }
}

#[cfg(test)]
mod real_direction_tests {
    use super::*;

    /// 用用户真实 OSM 验证跑道方向为逆时针，且整条轨迹也逆时针。
    ///
    /// 依赖本机真实 OSM，默认不跑（`-- --ignored` 启用）。
    #[test]
    #[ignore = "依赖本机真实 OSM 文件；用 cargo test ... -- --ignored 启用"]
    fn real_track_runs_counter_clockwise() {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        assert!(
            std::path::Path::new(path).exists(),
            "本测试需要真实 OSM 文件：{path}"
        );
        let net = super::load_network_path(path).unwrap();
        let near = Coord::new(104.6762016, 31.526536);
        let ring = net.find_track_loop(near).expect("应找到跑道");
        let mut area = 0.0;
        for w in ring.windows(2) {
            let a = net.to_m(net.node_coord(w[0]));
            let b = net.to_m(net.node_coord(w[1]));
            area += a[0] * b[1] - b[0] * a[1];
        }
        println!("跑道环有向面积 {area:.0}（>0 为逆时针）");
        assert!(area > 0.0, "跑道环是顺时针");

        // 轨迹整体方向：取每 10m 的位移叉积和
        let track = super::build_track_loop(
            2000.0, 720, 42, 1_700_000_000_000,
            (31.526536, 104.6762016), &net, TrackColorMode::FullGreen,
        ).expect("生成轨迹");
        let pts: Vec<[f64; 2]> = track
            .locations
            .iter()
            .map(|p| net.to_m(Coord::new(p.gLng, p.gLat)))
            .collect();
        let mut track_area = 0.0;
        for w in pts.windows(2) {
            if (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]) < 50.0 {
                track_area += w[0][0] * w[1][1] - w[1][0] * w[0][1];
            }
        }
        println!("轨迹有向面积 {track_area:.0}");
        assert!(track_area > 0.0, "轨迹跑成了顺时针");
    }
}

#[cfg(test)]
mod user_config_check {
    /// 用用户当前配置验证：1.0-1.3km 目标是否会被 check_target 拦截。
    #[test]
    fn user_target_vs_min_tour() {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        if !std::path::Path::new(path).exists() { return; }
        let net = super::load_network_path(path).unwrap();
        let pts = [
            (31.526444, 104.675651), (31.526805, 104.676518),
            (31.527248, 104.676262), (31.525924, 104.675929),
            (31.526259, 104.676648),
        ];
        let wps: Vec<route_planner::Coord> = pts
            .iter()
            .map(|p| route_planner::Coord::new(p.1, p.0))
            .collect();
        let check = route_planner::check::inspect(&net, &wps, &[]);
        println!("预检 ok={} issues={:?}", check.ok(), check.issues.len());
        println!("最短游览 = {:?} m", check.minimum_m);
        for target in [1000.0, 1300.0] {
            match route_planner::check::check_target(&check, target) {
                Ok(()) => println!("目标 {target}m → 通过"),
                Err(e) => println!("目标 {target}m → 拒绝: {e}"),
            }
        }
    }
}

#[cfg(test)]
mod user_loop_check {
    #[test]
    fn user_loop_mode_works() {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        if !std::path::Path::new(path).exists() { return; }
        let net = super::load_network_path(path).unwrap();
        let near = (31.526536, 104.6762016);
        for dist in [1000.0, 1150.0, 1300.0] {
            match super::build_track_loop(dist, 700, 42, 1_700_000_000_000, near, &net, crate::api::model::TrackColorMode::FullGreen) {
                Ok(t) => println!("跑道模式 {dist}m → OK, 实际 {:.0}m / {} 点", t.totalDistance, t.locations.len()),
                Err(e) => println!("跑道模式 {dist}m → 失败: {e}"),
            }
        }
    }
}

#[cfg(test)]
mod load_timing {
    /// 测量跑道模式加载 OSM 的耗时：如果这里很慢，UI 就会"一直加载"。
    #[test]
    fn measure_load_and_track_loop_time() {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        if !std::path::Path::new(path).exists() {
            return;
        }
        let t0 = std::time::Instant::now();
        let net = super::load_network_path(path).unwrap();
        let t_load = t0.elapsed();

        let t1 = std::time::Instant::now();
        let near = (31.526536, 104.6762016);
        let _ = super::build_track_loop(
            1150.0, 700, 42, 1_700_000_000_000, near, &net,
            crate::api::model::TrackColorMode::FullGreen,
        ).unwrap();
        let t_track = t1.elapsed();

        println!("加载 OSM: {:.0} ms", t_load.as_secs_f64() * 1000.0);
        println!("生成跑道轨迹: {:.0} ms", t_track.as_secs_f64() * 1000.0);
        println!("合计: {:.0} ms", (t_load + t_track).as_secs_f64() * 1000.0);
    }
}


#[cfg(test)]
mod user_track_verify {
    use super::*;
    use route_planner::Coord;
    /// 用用户真实 OSM 跑道验证两条核心约束：
    /// 1. 绕着跑道跑（不是跑到别处）
    /// 2. 每圈在跑道上、且形状不同
    #[test]
    fn user_track_is_looped_and_on_track() {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        if !std::path::Path::new(path).exists() {
            return;
        }
        let net = super::load_network_path(path).unwrap();
        let near = (31.526536, 104.6762016);
        let track = super::build_track_loop(
            1200.0, 480, 42, 1_700_000_000_000, near, &net,
            crate::api::model::TrackColorMode::FullGreen,
        ).expect("应能生成");

        // 跑道环
        let ring = net.find_track_loop(Coord::new(near.1, near.0)).expect("应有跑道");
        let ring_pts: Vec<Coord> = ring.iter().map(|&n| net.node_coord(n)).collect();
        let dist_to_ring = |p: &crate::track::model::GenPoint| -> f64 {
            let q = Coord::new(p.gLng, p.gLat);
            ring_pts.windows(2).map(|w| {
                let (a, b, p) = (net.to_m(w[0]), net.to_m(w[1]), net.to_m(q));
                let ab = [b[0]-a[0], b[1]-a[1]];
                let ap = [p[0]-a[0], p[1]-a[1]];
                let l2 = ab[0]*ab[0] + ab[1]*ab[1];
                let t = if l2 > 0.0 { ((ap[0]*ab[0]+ap[1]*ab[1])/l2).clamp(0.0,1.0) } else { 0.0 };
                let f = [a[0]+ab[0]*t, a[1]+ab[1]*t];
                (p[0]-f[0]).hypot(p[1]-f[1])
            }).fold(f64::INFINITY, f64::min)
        };

        let mut worst = 0.0f64;
        for p in &track.locations {
            worst = worst.max(dist_to_ring(p));
        }
        println!("轨迹点距跑道环最远 {:.2}m（总距离 {:.0}m，{} 点）",
            worst, track.totalDistance, track.locations.len());
        assert!(worst < 2.0, "轨迹跑出跑道 {worst:.2}m");

        // 绕圈：角度应覆盖 360°
        let center = ring_pts.iter().fold((0.0, 0.0), |a, c| (a.0 + c.lat, a.1 + c.lon));
        let n = ring_pts.len() as f64;
        let (clat, clon) = (center.0 / n, center.1 / n);
        let mut quads = [false; 8];
        for p in &track.locations {
            let ang = (p.gLat - clat).atan2(p.gLng - clon);
            let idx = (((ang + std::f64::consts::PI) / std::f64::consts::TAU) * 8.0) as usize % 8;
            quads[idx] = true;
        }
        let covered = quads.iter().filter(|&&q| q).count();
        println!("角度覆盖 {covered}/8 个扇区");
        assert!(covered == 8, "没有绕满一圈，只覆盖 {covered}/8 扇区");
    }
}






#[cfg(test)]
mod lap_offset_measure {
    use super::*;
    use route_planner::Coord;

    #[test]
    fn measure_real_lap_offsets() {
        let path = r"C:\Users\liming\Downloads\map (4).osm";
        if !std::path::Path::new(path).exists() { return; }
        let net = super::load_network_path(path).unwrap();
        let pts = [(31.526444,104.675651),(31.526805,104.676518),(31.527248,104.676262),(31.525924,104.675929),(31.526259,104.676648)];
        let must: Vec<(f64,f64)> = pts[1..].to_vec();
        let tr = super::build_road_mode(2000.0,720,42,1_700_000_000_000,&pts,&must,&net,crate::api::model::TrackColorMode::FullGreen,true).unwrap();

        let start = (tr.locations[0].gLat, tr.locations[0].gLng);
        let mut laps: Vec<Vec<(f64,f64)>> = vec![Vec::new()];
        let mut left = false;
        for p in &tr.locations {
            let m_lat = 111132.0; let m_lon = 111320.0*start.0.to_radians().cos();
            let d = ((p.gLat-start.0)*m_lat).hypot((p.gLng-start.1)*m_lon);
            if d > 30.0 { left = true; }
            if left && d < 15.0 && !laps.last().unwrap().is_empty() { laps.push(Vec::new()); left = false; }
            laps.last_mut().unwrap().push((p.gLat,p.gLng));
        }
        laps.retain(|l| l.len()>20);
        println!("{} 圈，点数 {:?}", laps.len(), laps.iter().map(|l| l.len()).collect::<Vec<_>>());
        for k in 1..laps.len() {
            let (a,b)=(&laps[k-1],&laps[k]);
            let m_lat=111132.0; let m_lon=111320.0*a[0].0.to_radians().cos();
            let mut ds: Vec<f64> = a.iter().step_by(3).map(|p|
                b.iter().map(|q| ((p.0-q.0)*m_lat).hypot((p.1-q.1)*m_lon)).fold(f64::INFINITY,f64::min)
            ).collect();
            ds.sort_by(f64::total_cmp);
            println!("  圈{}-{}: 偏离 最小{:.1}m 中位{:.1}m 最大{:.1}m", k-1,k,ds[0],ds[ds.len()/2],ds.last().unwrap());
        }
    }
}
