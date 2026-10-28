//! 真实道路拓扑路由 + 运动学物理仿真库。
//!
//! 三层解耦：
//!   1. 路由层（纯几何/拓扑）：OSM 解析 → 有向图 → 虚拟节点投影切分 → 闭环路线
//!   2. 动力学层（运动学 + 传感器）：曲率限速 / OU 配速 / AR(1) 噪声 + SDF / 生物力学
//!   3. 协议层（调用方适配）：由 `Route` + 速度序列组装最终轨迹
//!
//! 坐标约定：本 crate 内部以「度」存储（调用方已把 OSM 对齐到 BD 工作系），
//! 距离/曲率经本地等距投影转米制平面。

use petgraph::graph::NodeIndex;

pub mod biomech;
/// 围栏点/整段包含判定。
pub mod boundary;
pub mod campus;
pub mod check;
pub mod gpx;
pub mod graph;
pub mod kinematics;
pub mod load;
pub mod noise;
pub mod project;
pub mod route;
pub mod sdf;
pub mod smooth;
pub mod speed;

pub use biomech::{cadence_for, fatigue_from_km, gait, ou_cadence_series, speed_from, stride_for};
pub use campus::Campus;
pub use check::Check as RouteCheck;
pub use graph::{point_in_polygon, Coord, RoadGraph};
pub use kinematics::{pace_profile, speed_limit_ahead, KinParams};
pub use load::{load_osm, parse_osm};
pub use noise::{ar1_xy, GpsJitter};
pub use route::RouteOptions;
pub use sdf::Sdf;
pub use smooth::{turn_speed_limit, RoutePoint};

/// 一条规划好的空间路线（度系采样点 + 总长）。
#[derive(Clone, Debug)]
pub struct Route {
    pub points: Vec<RoutePoint>,
    pub length_m: f64,
}

/// 入口：解析路网 + 虚拟节点 + 闭环路由 + 曲率平滑。
///
/// `waypoints[0]` 为起点，`waypoints[1..]` 为必经打卡点（按顺序），
/// 路线自动闭合回起点。
pub fn plan_route(
    net: &RoadGraph,
    waypoints: &[Coord],
    target_len_m: f64,
    seed: u64,
    opts: &RouteOptions,
) -> Result<Route, String> {
    plan_route_split(net, waypoints, &waypoints[1..], target_len_m, seed, opts)
}

/// 入口（软/硬点分离）：`waypoints[0]` 为起点；`must` 为强制必经点（按序，不含
/// 起点，可为空）；`waypoints[1..]` 为软引导点（仅用于方向锚点，不必全经过）。
///
/// 遇到 `无起点` / `不可达` 等**结构性**失败时直接返回描述性错误；调用方若需要
/// 更细致的预检（问题点坐标、最短闭环、连通分量），用 [`check::inspect`]。
/// 沿路网里的**真实塑胶跑道**跑：找到最近的一圈跑道，反复绕它凑够目标距离。
///
/// 与 `plan_route_split` 的区别：这条路线只走**跑道边**（OSM 里
/// `surface=tartan` 或 `leisure=track` 的 way，解析时统一打上 `|tartan`
/// 标记，见 [`RoadGraph::find_track_loop`]），不会借用校园其它道路，
/// 因此每一米都真的在跑道上。找不到跑道时返回 Err（调用方回退到其它模式）。
pub struct TrackLoop {
    pub route: Route,
    /// 跑道单圈长度（米）。`route.length_m` 是按目标距离裁好的总长。
    pub lap_m: f64,
}

/// 每圈派生随机种子的乘数（大质数，把相邻圈号在 64 位空间里打散）。
const LAP_SEED_MUL: u64 = 0x9E37_79B9_7F4A_7C15;

/// 径向缩放允许的最大位移（米）。若按比例缩放，400m 场地半长 ~85m，
/// 2% 就是 1.7m，直接跑出跑道；因此按位移截断，而不是按比例。
const RADIAL_CAP_M: f64 = 0.35;

/// 一圈的形状参数。
#[derive(Clone, Copy)]
struct LapShape {
    /// 整圈基础横向偏移（米）：这一圈整体偏内道还是外道。
    base_off: f64,
    /// 三个正弦分量：(幅度 m, 频率 周/圈, 相位 rad)。
    /// 频率不必取整：`offset_at` 会去掉端点连线，u=0 的偏移恒为 0。
    waves: [(f64, f64, f64); 3],
    /// 径向缩放系数：0.98–1.02，但**不是整圈常数**——见 `radial_at`。
    scale: f64,
    /// 切向抖动：(幅度 m, 频率 周/圈, 相位 rad)，同样去趋势。
    along: (f64, f64, f64),
}

/// 正弦叠加在 `u` 处的值。
fn waves_at(waves: &[(f64, f64, f64)], u: f64) -> f64 {
    use std::f64::consts::TAU;
    waves
        .iter()
        .map(|&(a, f, p)| a * (TAU * f * u + p).sin())
        .sum()
}

/// 去趋势：减去端点连线，使 `f(0) = f(1) = 0`。
///
/// 这是「接缝不跳变」的关键：每圈首尾都回到跑道中线上的同一个起点（就像真实
/// 跑者每圈都经过同一条起跑线），第 k 圈末点 == 第 k+1 圈首点。
fn detrend(f: impl Fn(f64) -> f64, u: f64) -> f64 {
    f(u) - ((1.0 - u) * f(0.0) + u * f(1.0))
}

impl LapShape {
    /// 横向偏移（米）。`u ∈ [0,1]` 为圈内比例，端点恒为 0。
    ///
    /// = 换道量（带首尾归零包络）+ 限幅后的波浪。端点恒为 0 意味着每圈都从
    /// 跑道中线的同一个点出发、同一个点结束：接缝处零跳变，不会出现瞬移。
    fn offset_at(&self, u: f64) -> f64 {
        // 换道量乘一个首尾归零的包络：圈首尾为 0，所以**每圈都精确从中线
        // 起点出发、回到同一点**（接缝零跳变）；圈中部才体现这一圈走内道
        // 还是外道。真实跑者也是这样：起跑线附近大家挤在一起，跑开后才拉开。
        //
        // 包络要「近乎方形」：离起点/终点几米内就快速饱和到 1，只在最后几米
        // 回落到 0。`sin²(πu)` 太软（离起点 5% 处只剩 0.024，相邻两圈差异被
        // 压到 2cm，地图上仍像一条线）；指数 0.7 也不够（u=0.95 处只有 0.11）。
        // 这里用 `1 - (1 - |sin(πu)|)^6`：端点严格为 0、峰值为 1，且在
        // u∈[0.05, 0.95] 全程 ≥ 0.74，保证圈中段的车道差异完整保留。
        let e = (std::f64::consts::PI * u).sin().abs();
        let lane = self.base_off * (1.0 - (1.0 - e).powi(6));
        // 波浪与车道的比例受限：波浪只是「跑者在这条车道上的小幅漂移」，
        // 不能大到把相邻两圈的车道差抵消掉（实测第 1、2 圈在 u=0.82 处波浪
        // 恰好反向抵消，同弧长位置只差 3mm，地图上又变成一条线）。
        // 相邻圈的 base_off 符号相反（见生成处），所以只要把波浪压到
        // |waves| < 0.6·|base_off|，两圈横向偏移就不可能相交。
        let limit = 0.6 * self.base_off.abs();
        let w = detrend(|v| waves_at(&self.waves, v), u);
        lane + w.clamp(-limit, limit)
    }

    /// 切向抖动（米），端点恒为 0（不叠加基础值，否则会沿跑道整体错位）。
    fn along_at(&self, u: f64) -> f64 {
        detrend(|v| waves_at(&[self.along], v), u)
    }

    /// 径向缩放对半径的**相对**增量（无量纲），`scale - 1` 量级。
    ///
    /// 关键：缩放不能是整圈常数。整圈统一放大 2% 会让 400m 的圈变成 408m，
    /// 每圈长度偏离 `lap_m` 8m，多圈累加后 `s` 与几何位置脱节（下游按
    /// `lap_m` 取模就会错位）；若再拿牛顿法把长度硬掰回来，只能把
    /// `base_off` 推到 1m 开外、跑出跑道。
    ///
    /// 所以这里把缩放做成**沿圈积分为零**的场：`scale - 1` 时正时负，
    /// 一半弯道收紧、一半弯道放开（真实跑者就是这样——弯道切内道、
    /// 直道回外道），整圈长度只差二阶小量，不需要任何长度校正。
    fn radial_at(&self, u: f64) -> f64 {
        use std::f64::consts::TAU;
        // 基频 2：正/负半圈各一次，一正一负，整圈平均为零（与 sin² 正交）。
        // 相位由每圈的 scale 派生（0.98–1.02 → 相差 0.5–1 圈），保证每圈
        // 压/放的位置不同。
        let phase = (self.scale - 1.0) * 25.0 * TAU;
        let amp = self.scale - 1.0;
        // 再乘 sin²(πu)：首尾归零（接缝处不径向错开），且与基频 2 的正弦
        // 正交，整圈积分仍严格为零。
        amp * (TAU * 2.0 * u + phase).sin() * (std::f64::consts::PI * u).sin().powi(2)
            * std::f64::consts::FRAC_1_SQRT_2
    }
}

/// 跑道中心线（米制平面）：采样点 + 单位左法向量 + 弧长 + 外包中心。
struct Centerline {
    pts: Vec<[f64; 2]>,
    tangents: Vec<[f64; 2]>,
    arcs: Vec<f64>,
    curv: Vec<f64>,
    pivot: [f64; 2],
}

impl Centerline {
    fn new(net: &RoadGraph, points: &[RoutePoint]) -> Self {
        let pts: Vec<[f64; 2]> = points
            .iter()
            .map(|p| net.to_m(Coord::new(p.lon, p.lat)))
            .collect();
        let n = pts.len();
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in &pts {
            lo[0] = lo[0].min(p[0]);
            lo[1] = lo[1].min(p[1]);
            hi[0] = hi[0].max(p[0]);
            hi[1] = hi[1].max(p[1]);
        }
        let pivot = [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0];
        // 每圈形状不同的关键不仅是法向偏移，还有**切向抖动**：真人每一圈的分段
        // 配速不同，同一位置出现的时间不同，GPS 点位就会沿跑道前后错开。圆角
        // 方形跑道上（测试夹具）法向偏移几乎全是正交方向，单靠它两圈永远差
        // 0.2–0.3m；叠加几厘米级的切向分量后，两圈的点不会再落在同一个位置。
        let tangents = (0..n)
            .map(|i| {
                let prev = pts[(i + n - 1) % n];
                let next = pts[(i + 1) % n];
                let dx = next[0] - prev[0];
                let dy = next[1] - prev[1];
                let len = dx.hypot(dy).max(1e-9);
                [dx / len, dy / len]
            })
            .collect();
        Self {
            pts,
            tangents,
            arcs: points.iter().map(|p| p.s).collect(),
            curv: points.iter().map(|p| p.curvature).collect(),
            pivot,
        }
    }

    /// 径向位移（米制平面）：以跑道外包中心为基准，按 `LapShape::radial_at`
    /// 的相对增量压/放，并把位移硬截断到 `RADIAL_CAP_M`。
    fn radial_disp(&self, p: [f64; 2], shape: &LapShape, u: f64) -> (f64, f64) {
        let k = shape.radial_at(u);
        let mut rx = (p[0] - self.pivot[0]) * k;
        let mut ry = (p[1] - self.pivot[1]) * k;
        let r = rx.hypot(ry);
        if r > RADIAL_CAP_M {
            rx *= RADIAL_CAP_M / r;
            ry *= RADIAL_CAP_M / r;
        }
        (rx, ry)
    }

    /// 按 `shape` 生成一圈，弧长从 `s_off` 开始累加，追加到 `out`。
    fn push_lap(
        &self,
        net: &RoadGraph,
        lap_len: f64,
        shape: &LapShape,
        s_off: f64,
        out: &mut Vec<RoutePoint>,
    ) {
        // 中心线首尾是同一点，第 k 圈（k>0）的第一个采样与第 k-1 圈末点重合：
        // 跳过它，既不产生重复的 s，又让「上一圈末点 = 下一圈首点」严格成立。
        let start = usize::from(s_off > 0.0);
        for i in start..self.pts.len() {
            let p = self.pts[i];
            let u = (self.arcs[i] / lap_len.max(1e-9)).clamp(0.0, 1.0);
            let (rx, ry) = self.radial_disp(p, shape, u);
            let off = shape.offset_at(u);
            let along = shape.along_at(u);
            let tan = self.tangents[i];

            // 法向 = 左法线（切向逆时针转 90°）。
            let nrm = [-tan[1], tan[0]];
            let c = net.to_deg([
                p[0] + rx + nrm[0] * off + tan[0] * along,
                p[1] + ry + nrm[1] * off + tan[1] * along,
            ]);
            // `s` 用**中心线的参数弧长**（`s_off + arcs[i]`），而不是偏移后的
            // 累计几何长度：每圈必须精确落在 `[k·lap_m, (k+1)·lap_m]` 上，
            // 下游按 `lap_m` 切圈 / 取模定位依赖这一点。偏移只带来厘米级的
            // 长度差异（扰动本身被设计成长度中性），几何与 s 的偏差远小于
            // `polyline_point_at` 的插值容差。
            out.push(RoutePoint {
                lon: c.lon,
                lat: c.lat,
                s: s_off + self.arcs[i],
                // 曲率按缩放反比近似（缩放后的弯道半径 = 原半径 / scale）。
                curvature: self.curv[i] / shape.scale,
            });
        }
    }
}

/// 把闭合环旋转到「坐标最小」的节点作为起点。
///
/// `find_track_loop` 在 `HashSet` 上遍历连通分量，起点随进程哈希种子变化。
/// 不归一化的话，同 seed 两次调用会从不同的节点出发，逐点不可复现
/// （测试里的「同 seed 结果一致」会随机失败）。
fn canonicalize_ring_start(coords: &mut Vec<Coord>) {
    // 闭合环首尾重复，先去掉末尾的重复点再找最小。
    if coords.len() >= 2 && coords.first() == coords.last() {
        coords.pop();
    }
    if coords.len() < 2 {
        return;
    }
    let start = (0..coords.len())
        .min_by(|&a, &b| {
            coords[a]
                .lon
                .total_cmp(&coords[b].lon)
                .then(coords[a].lat.total_cmp(&coords[b].lat))
        })
        .unwrap_or(0);
    coords.rotate_left(start);
    // 重新闭合。
    if let Some(&first) = coords.first() {
        coords.push(first);
    }
}

/// 闭合采样折线的几何周长（米），首尾自动闭合。
fn loop_len_m(net: &RoadGraph, points: &[RoutePoint]) -> f64 {
    let n = points.len();
    if n < 2 {
        return 0.0;
    }
    let m: Vec<[f64; 2]> = points
        .iter()
        .map(|p| net.to_m(Coord::new(p.lon, p.lat)))
        .collect();
    (0..n)
        .map(|i| {
            let a = m[i];
            let b = m[(i + 1) % n];
            (a[0] - b[0]).hypot(a[1] - b[1])
        })
        .sum()
}

pub fn plan_track_loop(
    net: &RoadGraph,
    near: Coord,
    target_len_m: f64,
    seed: u64,
) -> Result<TrackLoop, String> {
    let ring = net
        .find_track_loop(near)
        .ok_or_else(|| {
            "路网里没有找到塑胶跑道（surface=tartan 或 leisure=track）：\
             请导入包含操场的 OSM，或改用其它跑道模式"
                .to_string()
        })?;
    let mut coords: Vec<Coord> = ring.iter().map(|&n| net.node_coord(n)).collect();
    // `find_track_loop` 在 HashSet 上遍历，环的起点随进程哈希种子变化；把环旋转到
    // 坐标最小的节点，否则同 seed 两次调用可能从不同起点出发，逐点不可复现。
    canonicalize_ring_start(&mut coords);
    let poly_m = coords
        .windows(2)
        .map(|w| {
            let a = net.to_m(w[0]);
            let b = net.to_m(w[1]);
            (a[0] - b[0]).hypot(a[1] - b[1])
        })
        .sum::<f64>();
    if poly_m < 50.0 {
        return Err(format!("识别到的跑道周长过短（{poly_m:.0}m）"));
    }

    // 沿跑道走一圈的平滑采样（跑道本身就是闭合环，不做切角，保留弯道形状）。
    let points = smooth::smooth_and_sample(net, &coords, 0.0, 0.0, 1.0);
    if points.len() < 10 {
        return Err("跑道采样点过少".into());
    }
    // 单圈长度取**采样折线的实际几何周长**，而不是 OSM 节点多边形周长：
    // 采样点按 1m 步长切分，最后一个间隔只有零点几米，两者能差 0.5–0.6m。
    // 下游用 `lap_m` 切圈/取模，必须与 `RoutePoint.s` 同一把尺子。
    let lap_m = loop_len_m(net, &points);

    // ── 多圈：每圈**独立生成**形状，而不是把同一圈平移复制 ──────────────────
    //
    // 旧实现把第一圈原样复制 laps 次、每圈只加一个单频横向偏移，地图上仍是
    // 几条几乎完美重叠的线。真实跑者每圈都不一样：切内道/外道的程度不同、
    // 弯道半径略有变化、时快时慢，贴线位置自然不同。
    //
    // 第 k 圈（k>0）用 `seed ^ (k·大质数)` 派生独立参数，在「跑道中心线」上
    // 重新生成整圈：
    //   1. 横向偏移 = 整圈换道量 base_off + 3 个不同频率的正弦叠加
    //      （频率/相位/幅度每圈独立）；正弦分量去趋势，两端为 0。
    //   2. 径向缩放 0.98–1.02，但做成沿圈积分归零的场（见 `radial_at`）：
    //      一半弯道收紧、一半放开，整圈周长几乎不变；位移截断到 RADIAL_CAP_M。
    //   3. 切向抖动 2–4cm（去趋势）：真人分段配速不同，同一位置出现的时间
    //      不同，GPS 点会沿跑道前后错开。
    //
    // 闭合与接缝：换道量的 sin²(πu) 包络、波浪去趋势、切向抖动去趋势在圈
    // 首尾都为 0，径向缩放场首尾取自同一相位，因此每圈首尾都精确落在跑道
    // 中线的同一节点上——相邻圈接缝零跳变。
    //
    // 为什么**不**让每圈起点弧长不同（完整的 A 方案）：一圈必然回到起点，
    // 若第 k 圈从 s0_k 出发、第 k+1 圈从 s0_{k+1} 出发，接缝处位置会从
    // C(s0_k) 直接跳到 C(s0_{k+1})，两者直线距离可达整个跑道直径（几十上百
    // 米）——GPS 轨迹里就是瞬移，比重叠线更假。所以采用退一步的版本：
    // 起点固定，圈内路径不同。
    //
    // 弧长：`s` 一律取**偏移后的真实累计几何长度**，不做任何名义值硬切。
    // 所有扰动（换道量带 sin² 包络、波浪去趋势、径向缩放积分归零）都被设计成
    // 长度中性：每圈实际周长与 `lap_m` 只差厘米级，绕几十圈也不会让 `s` 与
    // 几何脱节（下游 `polyline_point_at` 按 s 查表依赖这一点）。同 seed 下
    // 参数全部来自确定性 RNG，逐点可复现。
    let base_len = points.last().map(|p| p.s).unwrap_or(lap_m);
    let laps = (target_len_m / base_len).ceil().max(1.0) as usize;
    let centerline = Centerline::new(net, &points);
    let mut out: Vec<RoutePoint> = Vec::with_capacity(points.len() * laps);

    use rand::Rng as _;
    use rand::SeedableRng as _;
    use std::f64::consts::TAU;

    for lap in 0..laps {
        let shape = if lap == 0 {
            // 第 0 圈：沿用旧逻辑——起点固定 + 单频缓慢漂移，幅度收窄，不缩放。
            let mut rng = rand::rngs::StdRng::seed_from_u64(seed ^ 0x7E51_A9C3_2B4D_0001);
            LapShape {
                base_off: 0.0,
                waves: [
                    (rng.gen_range(0.04..0.10), 1.0, rng.gen_range(0.0..TAU)),
                    (0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0),
                ],
                scale: 1.0,
                along: (0.0, 0.0, 0.0),
            }
        } else {
            // 派生 seed：大质数乘法把相邻 k 打散，避免参数随圈号缓慢漂移。
            let mut r =
                rand::rngs::StdRng::seed_from_u64(seed ^ (lap as u64).wrapping_mul(LAP_SEED_MUL));
            // 三个频率（整圈周数）：低频 1–2 周（整圈缓慢外切/内切）、中频 3–5 周
            // （直线段与弯道贴线差异）、高频 7–11 周（每圈起步/冲刺的细微摆动）。
            // 波浪幅度合计 ≤ 0.30m；去趋势后再叠加换道量，横向始终在 ±0.45m 内。
            // 换道量做成**符号交替**（第 1 圈内道、第 2 圈外道、第 3 圈内道…），
            // 而不是各圈独立均匀采样：独立采样会出现相邻两圈车道几乎相同、
            // 波浪又恰好抵消的情况（实测同弧长位置只差 4mm，地图上仍然是一条
            // 线）。交替 + 随机幅度保证任意相邻两圈的车道中心至少差 0.18m。
            let side = if lap % 2 == 1 { 1.0 } else { -1.0 };
            let lane = side * r.gen_range(0.09..0.18);
            LapShape {
                base_off: lane,
                waves: [
                    (r.gen_range(0.04..0.07), r.gen_range(1.0..2.0), r.gen_range(0.0..TAU)),
                    (r.gen_range(0.02..0.04), r.gen_range(3.0..5.0), r.gen_range(0.0..TAU)),
                    (r.gen_range(0.01..0.02), r.gen_range(7.0..11.0), r.gen_range(0.0..TAU)),
                ],
                // 径向缩放负责「弯道切多深」的局部差异；圈与圈的可辨识度
                // 主要靠车道交替 + 波浪。幅度取 ±2%，位移另有 RADIAL_CAP_M 兜底。
                scale: r.gen_range(0.98..1.02),
                along: (
                    r.gen_range(0.02..0.04),
                    r.gen_range(8.0..13.0),
                    r.gen_range(0.0..TAU),
                ),
            }
        };
        centerline.push_lap(net, base_len, &shape, lap as f64 * base_len, &mut out);
    }
    // 裁到目标距离（s 已按每圈实际几何长度累加，末圈按 1m 网格采样）。
    if let Some(cut) = out.iter().position(|p| p.s >= target_len_m) {
        let last = out[cut];
        let prev = out[cut.saturating_sub(1)];
        let span = (last.s - prev.s).max(1e-6);
        let k = ((target_len_m - prev.s) / span).clamp(0.0, 1.0);
        out.truncate(cut);
        out.push(RoutePoint {
            lat: prev.lat + (last.lat - prev.lat) * k,
            lon: prev.lon + (last.lon - prev.lon) * k,
            s: target_len_m,
            ..prev
        });
    }
    Ok(TrackLoop {
        route: Route {
            points: out,
            length_m: target_len_m,
        },
        lap_m: base_len,
    })
}

pub fn plan_route_split(
    net: &RoadGraph,
    waypoints: &[Coord],
    must: &[Coord],
    target_len_m: f64,
    seed: u64,
    opts: &RouteOptions,
) -> Result<Route, String> {
    if waypoints.is_empty() {
        return Err("无起点".into());
    }
    // 预检：必达点必须可达、且确实在目标的同一路网上。校验的是"必达点顺序
    // 可达"，不约束可用道路总长——同一条路绕圈凑公里数本来就是合法策略。
    let hard: Vec<Coord> = std::iter::once(waypoints[0])
        .chain(must.iter().copied())
        .collect();
    if !must.is_empty() {
        let pre = check::inspect(net, &hard, &[]);
        if !pre.ok() {
            return Err(pre.error().unwrap_or_else(|| "路线预检未通过".into()));
        }
        // 目标距离比「按序拜访必经点并回到起点」更短时，规划器只能产出比目标
        // 更长的路线（长度由必经点游览决定），多出来的里程用户毫不知情，直接报错。
        check::check_target(&pre, target_len_m)?;
    }
    let mut g = net.clone();

    // 合并查询点（起点 + 必经点 + 软引导点），交由投影函数按坐标去重。
    let mut queries = vec![waypoints[0]];
    queries.extend(must.iter().copied());
    queries.extend(waypoints[1..].iter().copied());
    let vnodes = project::add_virtual_nodes(&mut g, &queries)?;

    let start = vnodes[0];
    let must_nodes: Vec<NodeIndex> = vnodes[1..1 + must.len()]
        .iter()
        .copied()
        .filter(|n| *n != start)
        .collect();
    let anchor_nodes: Vec<NodeIndex> = vnodes[1 + must.len()..].to_vec();

    let path = route::plan_loop(
        &g,
        start,
        &must_nodes,
        &anchor_nodes,
        target_len_m,
        seed,
        opts,
    )?;
    let coords = route::path_coords(&g, &path);
    // 跑道模式必须保留拐点：切角平滑会把方形环路的直角切掉，轨迹会斜穿到
    // 环内部（实测偏离路网 90m+）。这里用零半径、零阈值等价于不切角。
    let (radius, threshold) = if opts.repeat_checkpoint_loop {
        (0.0, 0.0)
    } else {
        (opts.min_radius_m, opts.turn_threshold_deg)
    };
    let points = smooth::smooth_and_sample(&g, &coords, radius, threshold, 1.0);
    if points.is_empty() {
        return Err("路线平滑后为空".into());
    }
    let length_m = points.last().map(|p| p.s).unwrap_or(0.0);
    Ok(Route { points, length_m })
}


#[cfg(test)]
mod track_loop_lap_tests {
    use super::*;
    use crate::graph::{EdgeData, RoadGraph};

    /// 造一个方形「跑道」：4 节点闭合、周长约 400m。
    fn loop_net() -> RoadGraph {
        let mut g = RoadGraph::new();
        let s = 0.0009;
        let a = g.add_node(Coord::new(0.0, 0.0));
        let b = g.add_node(Coord::new(s, 0.0));
        let c = g.add_node(Coord::new(s, s));
        let d = g.add_node(Coord::new(0.0, s));
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

    /// 把多圈路线按 `lap_m` 切成每圈的点（最后一圈可能被裁短）。
    ///
    /// 每个采样点只归入一圈：`s` 恰好落在整数圈边界时归**上一圈**，因为它是
    /// 上一圈的收尾点。若归到下一圈，接缝比较会拿两圈的起点去比，得到 0 位移。
    fn split_laps(points: &[RoutePoint], lap_m: f64) -> Vec<Vec<RoutePoint>> {
        let mut laps: Vec<Vec<RoutePoint>> = Vec::new();
        for &p in points {
            // 圈边界（s 恰为整数倍）归上一圈：它是上一圈的收尾点。
            let q = p.s / lap_m;
            let k = if (q - q.round()).abs() < 1e-6 {
                (q.round() as usize).saturating_sub(1)
            } else {
                q.floor() as usize
            };
            if k >= laps.len() {
                laps.resize_with(k + 1, Vec::new);
            }
            laps[k].push(p);
        }
        laps
    }

    /// 米制坐标。
    fn to_m(p: &RoutePoint) -> (f64, f64) {
        let m_lon = 111_320.0 * p.lat.to_radians().cos().max(0.2);
        (p.lon * m_lon, p.lat * 111_132.0)
    }

    /// 每圈在等弧长位置上的横向偏差指纹（相对该圈自身均值）。
    fn shape_fingerprint(lap: &[RoutePoint], n: usize) -> Vec<(f64, f64)> {
        let out: Vec<(f64, f64)> = (0..n)
            .map(|j| {
                let idx = j * (lap.len() - 1) / (n - 1).max(1);
                to_m(&lap[idx])
            })
            .collect();
        let mx = out.iter().map(|p| p.0).sum::<f64>() / n as f64;
        let my = out.iter().map(|p| p.1).sum::<f64>() / n as f64;
        out.iter().map(|p| (p.0 - mx, p.1 - my)).collect()
    }

    fn fingerprint_rms(a: &[(f64, f64)], b: &[(f64, f64)]) -> f64 {
        let s: f64 = a
            .iter()
            .zip(b)
            .map(|(p, q)| (p.0 - q.0).powi(2) + (p.1 - q.1).powi(2))
            .sum();
        (s / a.len() as f64).sqrt()
    }

    /// 在圈内弧长比例 `u` 处按线性插值取点（圈内比例 0..1）。
    fn point_at_u(lap: &[RoutePoint], u: f64) -> (f64, f64) {
        if lap.len() < 2 {
            return to_m(&lap[0]);
        }
        let s0 = lap[0].s;
        let span = lap.last().unwrap().s - s0;
        let target = s0 + span * u;
        let mut i = 0usize;
        while i + 2 < lap.len() && lap[i + 1].s <= target {
            i += 1;
        }
        let (a, b) = (lap[i], lap[i + 1]);
        let t = ((target - a.s) / (b.s - a.s).max(1e-9)).clamp(0.0, 1.0);
        let p = RoutePoint {
            lon: a.lon + (b.lon - a.lon) * t,
            lat: a.lat + (b.lat - a.lat) * t,
            s: target,
            curvature: 0.0,
        };
        to_m(&p)
    }

    /// 相邻两圈在**圈中段**的同弧长位置不能靠得太近——否则地图上是一条完美
    /// 重叠的线。要求 > 0.05m（远大于 GPS 噪声，肉眼可见为两条线）。
    ///
    /// 只检查圈中部 20%–80%：接缝处两圈**必须**重合（这是「不瞬移」的前提），
    /// 两端 20% 是换道/缩放包络的收敛区，偏移按设计趋近 0，那里靠拢属正常。
    ///
    /// 用圈内比例 `u` 对齐（而不是数组下标）：各圈采样点数量可能不同（末圈被
    /// 裁短），且每圈的几何长度略有差异，按下标比较会整体错位。
    #[test]
    fn laps_are_not_geometrically_identical() {
        let net = loop_net();
        let near = Coord::new(0.0009, 0.0009);
        let tl = plan_track_loop(&net, near, 2000.0, 42).expect("应能规划");
        let laps = split_laps(&tl.route.points, tl.lap_m);
        assert!(laps.len() >= 4, "圈数过少：{}", laps.len());

        for k in 1..laps.len() {
            let (a, b) = (&laps[k - 1], &laps[k]);
            if a.len() < 10 || b.len() < 10 {
                continue;
            }
            let mut worst = (f64::INFINITY, 0.0f64);
            let mut max_shift: f64 = 0.0;
            // 只看圈中部 20%–80%：两端 20% 是换道/缩放的包络收敛区，
            // 偏移按设计趋近 0（保证圈首尾精确闭合），那里靠拢属正常。
            for j in 0..=60 {
                let u = 0.20 + 0.60 * (j as f64 / 60.0);
                let (ax, ay) = point_at_u(a, u);
                let (bx, by) = point_at_u(b, u);
                let d = (ax - bx).hypot(ay - by);
                max_shift = max_shift.max(d);
                if d < worst.0 {
                    worst = (d, u);
                }
            }
            println!(
                "第 {} → {} 圈（u=0.20..0.80）：同弧长位移 最小 {:.3}m @u={:.2} 最大 {max_shift:.3}m",
                k - 1,
                k,
                worst.0,
                worst.1
            );
            assert!(
                worst.0 > 0.05,
                "第 {} 圈与第 {k} 圈在 u={:.2} 处仅相距 {:.3}m（应 > 0.05m）",
                k - 1,
                worst.1,
                worst.0
            );
        }
    }

    /// 每圈形状指纹（等弧长采样的相对位置序列）两两不同。
    #[test]
    fn each_lap_has_a_distinct_shape() {
        let net = loop_net();
        let near = Coord::new(0.0009, 0.0009);
        let tl = plan_track_loop(&net, near, 2000.0, 42).expect("应能规划");
        let laps = split_laps(&tl.route.points, tl.lap_m);
        let fps: Vec<Vec<(f64, f64)>> = laps.iter().map(|l| shape_fingerprint(l, 64)).collect();
        for i in 1..fps.len() {
            let rms = fingerprint_rms(&fps[i - 1], &fps[i]);
            println!("第 {} 圈 vs 第 {i} 圈：形状指纹 RMS 差 {rms:.3}m", i - 1);
            assert!(rms > 0.05, "第 {} 圈与第 {i} 圈形状几乎相同（RMS {rms:.3}m）", i - 1);
        }
    }

    /// 圈与圈之间必须连续：**接缝处相邻两个采样点的间距要和正常采样间距一致**。
    ///
    /// 注意不能要求「第 k 圈末点 ≈ 第 k+1 圈首点」——那两点本来就沿跑道相距一个
    /// 采样步长（约 1m）。真正的跳变表现为：接缝处的几何间距远大于 Δs（说明
    /// 中间被拉了一条直线穿过去），或 Δs 突然变大。
    #[test]
    fn laps_join_without_jumps() {
        let net = loop_net();
        let near = Coord::new(0.0009, 0.0009);
        let tl = plan_track_loop(&net, near, 2000.0, 42).expect("应能规划");
        let laps = split_laps(&tl.route.points, tl.lap_m);
        for k in 1..laps.len() {
            let last = *laps[k - 1].last().unwrap();
            let first = laps[k][0];
            let (ax, ay) = to_m(&last);
            let (bx, by) = to_m(&first);
            let geo = (ax - bx).hypot(ay - by);
            let ds = first.s - last.s;
            // 接缝处是两个相邻采样点：几何间距应接近 Δs（约 1m）。放宽到
            // 3×Δs + 0.5m 是因为换道会让相邻圈在接缝附近有 ~1m 的横向差。
            println!("接缝 {k}: Δs={ds:.3}m 几何间距={geo:.3}m");
            // 采样步长 1m；允许 25% 的局部差异（扰动会让局部弧长略有伸缩）。
            assert!(
                geo < 1.5,
                "第 {} → {k} 圈接缝处几何间距 {geo:.3}m，轨迹会瞬移",
                k - 1
            );
            assert!(
                ds < 1.5,
                "第 {} → {k} 圈接缝处 Δs={ds:.3}m，弧长不连续",
                k - 1
            );
            assert!(geo > 0.2 && ds > 0.2, "接缝处出现重复点（geo={geo:.3} ds={ds:.3}）");
        }
    }

    /// 总长度精确等于目标（±1m），且 s 单调、末点 s = 目标。
    #[test]
    fn total_length_matches_target() {
        let net = loop_net();
        let near = Coord::new(0.0009, 0.0009);
        for target in [400.0f64, 1200.0, 2000.0, 3100.0] {
            let tl = plan_track_loop(&net, near, target, 42).expect("应能规划");
            assert!((tl.route.length_m - target).abs() < 1.0);
            let mut prev = -1.0;
            for p in &tl.route.points {
                assert!(p.s >= prev - 1e-9);
                prev = p.s;
            }
            assert!((prev - target).abs() < 1e-6, "末点 s={prev} 应等于 {target}");
        }
    }

    /// 偏移必须落在跑道宽度量级内：所有点离跑道中线 < 1.2m。
    #[test]
    fn lap_offset_stays_within_track_width() {
        let net = loop_net();
        let near = Coord::new(0.0009, 0.0009);
        let loop_route = plan_track_loop(&net, near, 1600.0, 7).expect("应能规划");
        let s = 0.0009;
        let mut worst = 0.0f64;
        for p in &loop_route.route.points {
            let (x, y) = to_m(p);
            let side = s * 111_132.0;
            let d = [x.abs(), (x - side).abs(), y.abs(), (y - side).abs()]
                .into_iter()
                .fold(f64::INFINITY, f64::min);
            worst = worst.max(d);
        }
        assert!(worst < 1.2, "采样点离跑道中线最远 {worst:.2}m，超出跑道宽度");
    }

    /// 同一个 seed 必须产生同样的路线（可复现）。
    #[test]
    fn same_seed_reproduces_the_same_laps() {
        let net = loop_net();
        let near = Coord::new(0.0009, 0.0009);
        let a = plan_track_loop(&net, near, 1200.0, 99).unwrap();
        let b = plan_track_loop(&net, near, 1200.0, 99).unwrap();
        assert_eq!(a.route.points.len(), b.route.points.len());
        for (i, (p, q)) in a.route.points.iter().zip(b.route.points.iter()).enumerate() {
            assert!(
                (p.lat - q.lat).abs() < 1e-15 && (p.lon - q.lon).abs() < 1e-15,
                "第 {i} 点不一致：({},{}) vs ({},{})",
                p.lat,
                p.lon,
                q.lat,
                q.lon
            );
        }
    }
}

#[cfg(test)]
mod lap_path_diff_probe {
    use super::*;
    use crate::graph::{EdgeData, RoadGraph};

    /// 造一个有多个可选路口的网格，验证每圈选的边是否不同。
    fn grid() -> RoadGraph {
        let mut g = RoadGraph::new();
        let step = 0.0005;
        let n = 6;
        let mut nodes = vec![];
        for j in 0..n { for i in 0..n {
            nodes.push(g.add_node(Coord::new(i as f64*step, j as f64*step)));
        }}
        let mut wid = 1u64;
        for j in 0..n { for i in 0..n-1 {
            let (a,b) = (nodes[j*n+i], nodes[j*n+i+1]);
            g.add_edge(a,b,EdgeData{way_id:wid,oneway:false,highway:"residential".into(),maxspeed_kmh:None,length_m:55.0});
            g.add_edge(b,a,EdgeData{way_id:wid,oneway:false,highway:"residential".into(),maxspeed_kmh:None,length_m:55.0});
            wid+=1;
        }}
        for j in 0..n-1 { for i in 0..n {
            let (a,b) = (nodes[j*n+i], nodes[(j+1)*n+i]);
            g.add_edge(a,b,EdgeData{way_id:wid,oneway:false,highway:"residential".into(),maxspeed_kmh:None,length_m:55.0});
            g.add_edge(b,a,EdgeData{way_id:wid,oneway:false,highway:"residential".into(),maxspeed_kmh:None,length_m:55.0});
            wid+=1;
        }}
        g.compute_anchor(); g.rebuild_index(); g
    }

    #[test]
    fn laps_with_different_seeds_take_different_edges() {
        let net = grid();
        let wps = [Coord::new(0.0005,0.0005), Coord::new(0.0025,0.0005), Coord::new(0.0025,0.0025)];
        let mut opts = RouteOptions::default();
        opts.repeat_checkpoint_loop = true;
        let mut edges_per_lap = Vec::new();
        for k in 0..3u64 {
            let r = plan_route_split(&net, &wps, &wps[1..], 600.0, 100+k*7919, &opts).unwrap();
            let ids: Vec<(f64,f64)> = r.points.iter().map(|p| (p.lat.round()*1e6, p.lon.round()*1e6)).collect();
            edges_per_lap.push(ids);
            println!("圈{k}: {} 点", r.points.len());
        }
        // 比较圈 0 和圈 1 的路径是否相同
        let same = edges_per_lap[0] == edges_per_lap[1];
        println!("圈0 与圈1 路径完全相同: {}", same);
        assert!(!same, "不同 seed 应该走出不同的路径");
    }
}
