//! 有向路网图：`petgraph` DiGraph + `rstar` 空间索引 + 等距投影。
//!
//! 节点/边坐标以「度」存储（工作系，已由调用方对齐到 BD 工作系），
//! 距离/曲率计算通过本地等距投影（锚点 + `cos(lat)` 缩放）转成米制平面。

use std::collections::HashMap;

use crate::boundary;
use petgraph::graph::{DiGraph, EdgeIndex, NodeIndex};
use rstar::{RTree, RTreeObject, AABB};

pub const MET_PER_DEG_LAT: f64 = 111_132.0;
pub const MET_PER_DEG_LNG_EQ: f64 = 111_320.0;

/// 经纬度（度，工作系）。
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Coord {
    pub lon: f64,
    pub lat: f64,
}

impl Coord {
    pub fn new(lon: f64, lat: f64) -> Self {
        Coord { lon, lat }
    }
}

#[derive(Clone, Debug)]
pub struct NodeData {
    pub coord: Coord,
}

#[derive(Clone, Debug)]
pub struct EdgeData {
    /// OSM way id（同一物理道路切分出的多段共享，用于重复惩罚）。
    pub way_id: u64,
    pub oneway: bool,
    pub highway: String,
    pub maxspeed_kmh: Option<f64>,
    pub length_m: f64,
}

/// rstar 空间索引条目：一条有向边在米制平面的两端点。
/// 存端点而非边索引：切分边时 `remove_edge` 的 swap-remove 会让边索引失效，
/// 端点索引（节点不随切分删除）则可经 `find_edge` 重新定位当前有效边。
#[derive(Clone, Copy)]
struct Seg {
    a: [f64; 2],
    b: [f64; 2],
    from: NodeIndex,
    to: NodeIndex,
}

impl RTreeObject for Seg {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.a, self.b)
    }
}

/// 点到线段距离（用于 rstar 邻域查询：返回与查询点距离 ≤ 半径的边）。
impl rstar::PointDistance for Seg {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let (proj, _) = project_to_segment(*point, self.a, self.b);
        let (dx, dy) = (proj[0] - point[0], proj[1] - point[1]);
        dx * dx + dy * dy
    }
}

#[derive(Clone)]
pub struct RoadGraph {
    pub graph: DiGraph<NodeData, EdgeData>,
    /// 投影锚点（度）。
    pub anchor: Coord,
    /// 经度每度对应米数（按锚点纬度校正）。
    pub meter_per_lng: f64,
    /// 建筑外环（度）。
    pub buildings: Vec<Vec<Coord>>,
    index: RTree<Seg>,
}

impl RoadGraph {
    pub fn new() -> Self {
        RoadGraph {
            graph: DiGraph::new(),
            anchor: Coord::new(0.0, 0.0),
            meter_per_lng: MET_PER_DEG_LNG_EQ,
            buildings: Vec::new(),
            index: RTree::new(),
        }
    }

    /// 度 → 米制平面（相对锚点）。
    pub fn to_m(&self, c: Coord) -> [f64; 2] {
        [
            (c.lon - self.anchor.lon) * self.meter_per_lng,
            (c.lat - self.anchor.lat) * MET_PER_DEG_LAT,
        ]
    }

    /// 米制平面 → 度。
    pub fn to_deg(&self, p: [f64; 2]) -> Coord {
        Coord::new(
            self.anchor.lon + p[0] / self.meter_per_lng,
            self.anchor.lat + p[1] / MET_PER_DEG_LAT,
        )
    }

    pub fn node_coord(&self, n: NodeIndex) -> Coord {
        self.graph[n].coord
    }

    /// 添加节点，返回索引。
    pub fn add_node(&mut self, coord: Coord) -> NodeIndex {
        self.graph.add_node(NodeData { coord })
    }

    /// 添加有向边。
    pub fn add_edge(&mut self, from: NodeIndex, to: NodeIndex, data: EdgeData) -> EdgeIndex {
        self.graph.add_edge(from, to, data)
    }

    /// 设置投影锚点（由所有节点质心计算）并重建米制缩放。
    pub fn compute_anchor(&mut self) {
        let n = self.graph.node_count();
        if n == 0 {
            return;
        }
        let (mut slat, mut slon) = (0.0f64, 0.0f64);
        for node in self.graph.node_indices() {
            let c = self.graph[node].coord;
            slat += c.lat;
            slon += c.lon;
        }
        let (lat, lon) = (slat / n as f64, slon / n as f64);
        self.anchor = Coord::new(lon, lat);
        self.meter_per_lng = MET_PER_DEG_LNG_EQ * lat.to_radians().cos().abs().max(0.2);
    }

    /// 重建空间索引（新增节点/边或切分后调用）。
    pub fn rebuild_index(&mut self) {
        let mut items = Vec::with_capacity(self.graph.edge_count());
        for e in self.graph.edge_indices() {
            let (a, b) = self.graph.edge_endpoints(e).unwrap();
            let (pa, pb) = (
                self.to_m(self.graph[a].coord),
                self.to_m(self.graph[b].coord),
            );
            items.push(Seg {
                a: pa,
                b: pb,
                from: a,
                to: b,
            });
        }
        self.index = RTree::bulk_load(items);
    }

    /// 查询离给定米制平面点最近的边：返回 (边, 垂足参数 t∈[0,1], 垂足点, 距离)。
    pub fn nearest_edge(&self, p: [f64; 2]) -> Option<(EdgeIndex, f64, [f64; 2], f64)> {
        let mut radius = 500.0f64;
        let mut best: Option<(EdgeIndex, f64, [f64; 2], f64)> = None;
        while best.is_none() && radius <= 20_000.0 {
            let mut best_d = f64::INFINITY;
            for seg in self.index.locate_within_distance(p, radius * radius) {
                // 跳过已被切分删除的过期条目（端点对应边已不存在）。
                let edge = match self.graph.find_edge(seg.from, seg.to) {
                    Some(e) => e,
                    None => continue,
                };
                let (proj, t) = project_to_segment(p, seg.a, seg.b);
                let d = dist(p, proj);
                if d < best_d {
                    best_d = d;
                    best = Some((edge, t, proj, d));
                }
            }
            if best.is_some() {
                break;
            }
            radius *= 4.0;
        }
        best
    }

    /// 增量插入一条有向边到空间索引（切分边新增子段时调用，避免全量重建）。
    pub fn index_insert_edge(&mut self, from: NodeIndex, to: NodeIndex) {
        let a = self.to_m(self.graph[from].coord);
        let b = self.to_m(self.graph[to].coord);
        self.index.insert(Seg { a, b, from, to });
    }

    /// 整体平移（对齐 WGS84 → BD 工作系，常数偏移）。
    pub fn shift(&mut self, dlat: f64, dlng: f64) {
        for node in self.graph.node_weights_mut() {
            node.coord.lat += dlat;
            node.coord.lon += dlng;
        }
        for ring in self.buildings.iter_mut() {
            for c in ring.iter_mut() {
                c.lat += dlat;
                c.lon += dlng;
            }
        }
        self.anchor.lat += dlat;
        self.anchor.lon += dlng;
        self.rebuild_index();
    }

    /// 仅保留落在任一多边形（围栏）内的节点及其边，并重建索引。
    ///
    /// 判据是**整条边**都在围栏内，而不是只看两端节点：两端都在凹形围栏内、
    /// 中间却穿过凹口的两点连边，靠单点判定会被误保留，生成的路线会越界。
    pub fn retain_inside_any(&mut self, polygons: &[Vec<Coord>]) {
        if polygons.is_empty() {
            return;
        }
        self.graph.retain_edges(|g, e| {
            let (a, b) = g.edge_endpoints(e).unwrap_or_else(|| unreachable!());
            boundary::segment_inside_any(polygons, g[a].coord, g[b].coord)
        });
        self.graph
            .retain_nodes(|g, idx| polygons.iter().any(|p| point_in_polygon(p, g[idx].coord)));
        self.compute_anchor();
        self.rebuild_index();
    }

    /// 找出路网里的**塑胶跑道闭环**。
    ///
    /// 返回离 `near` 最近的那一圈，按行进顺序排列的节点序列（首尾闭合）。
    /// 找不到返回 None —— 调用方据此回退到其它模式。
    ///
    /// 判定条件：只由跑道边组成、闭合、周长在 100–1000m 之间。
    ///
    /// 注意：这里**不直接**看 OSM 的 `surface=tartan` / `leisure=track` 标签，
    /// 而是看边上的 `highway` 是否带 `|tartan` 标记。该标记由
    /// [`crate::load`] 在解析时给 `surface=tartan` **或** `leisure=track`
    /// 两类 way 统一打上。要改判定范围请改那边的标记逻辑，不要在这里
    /// 另加标签判断——否则两条路径会不一致。
    pub fn find_track_loop(&self, near: Coord) -> Option<Vec<NodeIndex>> {
        use std::collections::{HashMap, HashSet};

        let is_track = |e: EdgeIndex| -> bool {
            self.graph
                .edge_weight(e)
                .map(|d| d.highway.contains("tartan"))
                .unwrap_or(false)
        };
        let track_edges: HashSet<EdgeIndex> =
            self.graph.edge_indices().filter(|&e| is_track(e)).collect();
        if track_edges.is_empty() {
            return None;
        }
        let track_nodes: HashSet<NodeIndex> = track_edges
            .iter()
            .filter_map(|&e| self.graph.edge_endpoints(e))
            .flat_map(|(a, b)| [a, b])
            .collect();

        // 跑道节点按相邻关系连成环：每个节点在跑道图里通常恰好 2 个邻居。
        let mut adj: HashMap<NodeIndex, Vec<NodeIndex>> = HashMap::new();
        for &e in &track_edges {
            if let Some((a, b)) = self.graph.edge_endpoints(e) {
                adj.entry(a).or_default().push(b);
                adj.entry(b).or_default().push(a);
            }
        }
        for v in adj.values_mut() {
            v.sort_unstable();
            v.dedup();
        }

        // 逐连通分量走一圈，选出离 `near` 最近、周长最像跑道的那个。
        let mut best: Option<(f64, Vec<NodeIndex>, f64)> = None; // (距 near, 环, 周长)
        let mut seen: HashSet<NodeIndex> = HashSet::new();
        for &start in &track_nodes {
            if seen.contains(&start) {
                continue;
            }
            // 收集该连通分量的全部节点。
            let mut comp = Vec::new();
            let mut stack = vec![start];
            seen.insert(start);
            while let Some(n) = stack.pop() {
                comp.push(n);
                if let Some(ns) = adj.get(&n) {
                    for &m in ns {
                        if seen.insert(m) {
                            stack.push(m);
                        }
                    }
                }
            }
            if comp.len() < 3 {
                continue;
            }
            // 每个节点都必须是度 2（简单环），否则是岔路口，跳过。
            if comp.iter().any(|n| adj.get(n).map(|v| v.len()).unwrap_or(0) != 2) {
                continue;
            }
            // 沿环走一圈。
            let mut ring = Vec::with_capacity(comp.len());
            let mut prev: Option<NodeIndex> = None;
            let mut cur = comp[0];
            loop {
                ring.push(cur);
                let next = adj[&cur]
                    .iter()
                    .copied()
                    .find(|&m| Some(m) != prev)
                    .unwrap_or(comp[0]);
                prev = Some(cur);
                cur = next;
                if cur == comp[0] || ring.len() > comp.len() {
                    break;
                }
            }
            if ring.len() != comp.len() {
                continue; // 没走成完整环
            }
            ring.push(ring[0]); // 闭合

            // 强制逆时针：田径比赛与学校体测都按逆时针（左手弯道）跑。
            // 有向面积 > 0 表示逆时针；否则反转环序。
            // 注意保持首尾闭合：反转后仍以同一节点开头。
            if ring.len() >= 4 {
                let signed = {
                    let mut acc = 0.0;
                    for w in ring.windows(2) {
                        let a = self.to_m(self.node_coord(w[0]));
                        let b = self.to_m(self.node_coord(w[1]));
                        acc += a[0] * b[1] - b[0] * a[1];
                    }
                    acc
                };
                if signed < 0.0 {
                    ring.reverse();
                }
            }

            let perimeter: f64 = ring
                .windows(2)
                .map(|w| dist(self.to_m(self.node_coord(w[0])), self.to_m(self.node_coord(w[1]))))
                .sum();
            if !(100.0..=1000.0).contains(&perimeter) {
                continue;
            }
            let center = self.to_m(near);
            let d = ring
                .iter()
                .map(|&n| dist(self.to_m(self.node_coord(n)), center))
                .fold(f64::INFINITY, f64::min);
            if best.as_ref().map(|(bd, _, _)| d < *bd).unwrap_or(true) {
                best = Some((d, ring, perimeter));
            }
        }
        best.map(|(_, ring, _)| ring)
    }

    /// 所有边（度系折线，供 UI 绘制）。
    pub fn edges_deg(&self) -> Vec<Vec<Coord>> {
        self.graph
            .edge_indices()
            .map(|e| {
                let (a, b) = self.graph.edge_endpoints(e).unwrap();
                vec![self.graph[a].coord, self.graph[b].coord]
            })
            .collect()
    }
}

impl Default for RoadGraph {
    fn default() -> Self {
        Self::new()
    }
}

/// 点到线段垂足投影，返回 (垂足点, 归一化参数 t)。
pub fn project_to_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> ([f64; 2], f64) {
    let ab = [b[0] - a[0], b[1] - a[1]];
    let ap = [p[0] - a[0], p[1] - a[1]];
    let denom = ab[0] * ab[0] + ab[1] * ab[1];
    let t = if denom <= 0.0 {
        0.0
    } else {
        ((ap[0] * ab[0] + ap[1] * ab[1]) / denom).clamp(0.0, 1.0)
    };
    ([a[0] + t * ab[0], a[1] + t * ab[1]], t)
}

pub fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    (dx * dx + dy * dy).sqrt()
}

/// 方位角（弧度，正北为 0，顺时针），用于方向锚点。
pub fn bearing(a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    dy.atan2(dx)
}

/// 两方位角的最小夹角（弧度，[0, π]）。
pub fn angle_diff(a: f64, b: f64) -> f64 {
    let mut d = (a - b).rem_euclid(std::f64::consts::TAU);
    if d > std::f64::consts::PI {
        d = std::f64::consts::TAU - d;
    }
    d
}

/// 由节点 id 序列重建路径所需的信息（way_id → 使用计数）。
pub fn default_used() -> HashMap<u64, u32> {
    HashMap::new()
}

/// 射线法点是否在多边形内（经纬度按 (lon,lat)=(x,y) 处理，校园尺度足够）。
pub fn point_in_polygon(poly: &[Coord], p: Coord) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let (x, y) = (p.lon, p.lat);
    let mut inside = false;
    let mut j = n - 1;
    for i in 0..n {
        let (xi, yi) = (poly[i].lon, poly[i].lat);
        let (xj, yj) = (poly[j].lon, poly[j].lat);
        let intersects = ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        if intersects {
            inside = !inside;
        }
        j = i;
    }
    inside
}
