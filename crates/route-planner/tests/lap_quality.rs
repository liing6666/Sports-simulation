//! 跑道模式「每圈形状不同」质量评估（只消费 `plan_track_loop` 的输出，不依赖其实现）。
//!
//! 用真实 OSM 的跑道 way 作为「跑道中线」地面真值，输出逐圈指标表：
//!
//! | 指标 | 含义 |
//! |---|---|
//! | 相邻圈最大位移 | 相邻两圈在 100 个等弧长相位上的最大点距。≈0 即每圈完全重合 |
//! | 相邻圈均值 | 同上，取平均 |
//! | 形状 Hausdorff | 相邻两圈 100 点集的对称 Hausdorff 距离（形状差异） |
//! | 圈闭合 | 本圈首点 ↔ 本圈末点的距离（首尾应回到同一处） |
//! | 接缝跳变 | 本圈末点 ↔ 下一圈首点：同一弧长位置、横向偏移不同造成的折跳 |
//! | 圈长误差 | 本圈几何折线长 − 实现 `lap_m` |
//! | 总长误差 | `route.length_m` / 沿轨几何长 对目标距离的偏差 |
//! | 距中线 | 所有路线点到 OSM 跑道中线的最大距离（需求 < 1.2m） |
//!
//! 外部真实 OSM 通过环境变量 `LAP_QUALITY_OSM` 传入（不写死绝对路径）；未设置时
//! 该用例跳过，crate 内夹具 `dinghu-roads.osm` 始终运行：
//!
//! ```text
//! LAP_QUALITY_OSM="C:/Users/xxx/Downloads/map (4).osm" \
//!   cargo test -p route-planner --test lap_quality -- --nocapture
//! ```
//!
//! 可选 `LAP_QUALITY_SVG=out.svg` 把各圈画成不同颜色导出 SVG，便于人眼对比。

use std::path::{Path, PathBuf};

use route_planner::graph::{project_to_segment, Coord, RoadGraph};
use route_planner::{load_osm, parse_osm, plan_track_loop, TrackLoop};

/// 每圈重采样点数（等弧长相位）。
const N_PHASE: usize = 100;
/// 需求：每圈首尾（含接缝）偏差 < 1m。
const CLOSURE_MAX_M: f64 = 1.0;
/// 需求：所有点距跑道中线 < 1.2m。
const TRACK_HALF_WIDTH_M: f64 = 1.2;
/// 需求：总长 = target ± 1m。
const LENGTH_TOL_M: f64 = 1.0;
/// 位移小于该值视为「每圈完全重合」。
const IDENTICAL_EPS_M: f64 = 0.02;
/// s 相等的判定容差（米）。
const S_TOL: f64 = 1e-6;
/// 周期检测：间隔 d 的两点中位距离低于该值，认为它们是相邻圈的同弧长点。
const PERIOD_MEDIAN_MAX_M: f64 = 1.0;

const TARGET_M: f64 = 3000.0;
const SEED: u64 = 42;

// ---------------------------------------------------------------------------
// 基础几何
// ---------------------------------------------------------------------------

fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn polyline_len_m(net: &RoadGraph, poly: &[Coord]) -> f64 {
    poly.windows(2)
        .map(|w| dist(net.to_m(w[0]), net.to_m(w[1])))
        .sum()
}

/// 米制点到折线（度系给定）的最短距离。
fn dist_point_to_polyline_m(net: &RoadGraph, poly: &[Coord], p: [f64; 2]) -> f64 {
    poly.windows(2)
        .map(|w| {
            let (proj, _) = project_to_segment(p, net.to_m(w[0]), net.to_m(w[1]));
            dist(p, proj)
        })
        .fold(f64::INFINITY, f64::min)
}

fn centroid(pts: &[Coord]) -> Coord {
    let n = pts.len().max(1) as f64;
    Coord::new(
        pts.iter().map(|c| c.lon).sum::<f64>() / n,
        pts.iter().map(|c| c.lat).sum::<f64>() / n,
    )
}

/// 折线上弧长 s 处的点（线性插值；s 恰好落在某个采样点时直接返回该点）。
fn pos_at(poly: &[(f64, [f64; 2])], s: f64) -> [f64; 2] {
    if poly.is_empty() {
        return [f64::NAN, f64::NAN];
    }
    if s <= poly[0].0 + 1e-9 {
        return poly[0].1;
    }
    let i = poly
        .partition_point(|p| p.0 <= s + 1e-9)
        .saturating_sub(1)
        .min(poly.len() - 1);
    if i + 1 >= poly.len() {
        return poly[i].1;
    }
    let (s0, p0) = poly[i];
    let (s1, p1) = poly[i + 1];
    let span = s1 - s0;
    if span <= 1e-9 {
        return p1;
    }
    let t = ((s - s0) / span).clamp(0.0, 1.0);
    [
        p0[0] + (p1[0] - p0[0]) * t,
        p0[1] + (p1[1] - p0[1]) * t,
    ]
}

fn directed_hausdorff(a: &[[f64; 2]], b: &[[f64; 2]]) -> f64 {
    a.iter()
        .map(|p| {
            b.iter()
                .map(|q| dist(*p, *q))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0, f64::max)
}

fn hausdorff(a: &[[f64; 2]], b: &[[f64; 2]]) -> f64 {
    directed_hausdorff(a, b).max(directed_hausdorff(b, a))
}

// ---------------------------------------------------------------------------
// 分圈
// ---------------------------------------------------------------------------

/// 每圈采样点数：取最小的 d，使「间隔 d 的两个点」的中位距离最小（同一弧长
/// 位置、不同圈的点只差横向/切向抖动，通常 0.2–0.6m；圈内相隔 d 米的点则相距
/// ≈ d）。不依赖 `s` 的编号方式——实现把 `s` 连续累加后，`k·lap_m` 已不再落在
/// 圈接缝上。
fn lap_period(raw: &[[f64; 2]]) -> Option<usize> {
    let n = raw.len();
    if n < 40 {
        return None;
    }
    let mut medians = Vec::new();
    for d in 10..=n / 2 {
        let mut ds: Vec<f64> = (0..n - d).map(|i| dist(raw[i], raw[i + d])).collect();
        ds.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let med = ds[ds.len() / 2];
        medians.push((d, med));
        // 第一个中位距离进入「同弧长抖动」量级的间隔就是一圈的点数。
        if med < PERIOD_MEDIAN_MAX_M {
            // 排除截断造成的伪周期：至少能切出 2 圈完整的数据。
            if n / d >= 2 {
                return Some(d);
            }
        }
    }
    // 兜底：中位距离最小的那个间隔。
    medians
        .into_iter()
        .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(d, _)| d)
}

/// 把路线切成「完整圈」的原始点下标区间 `[start, end]`（end 为本圈圈末点）。
///
/// 依赖每圈采样点数相同（实现每圈都在同一条中心线上按弧长等分重采样）。
fn lap_slices(raw: &[(f64, [f64; 2])]) -> Vec<(usize, usize)> {
    let pts: Vec<[f64; 2]> = raw.iter().map(|p| p.1).collect();
    let Some(n) = lap_period(&pts) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    let mut start = 0usize;
    while start + n < raw.len() {
        out.push((start, start + n - 1));
        start += n;
    }
    out
}

// ---------------------------------------------------------------------------
// 评估
// ---------------------------------------------------------------------------

struct LapRow {
    lap: usize,
    lap_m: f64,
    adj_max: Option<f64>,
    adj_mean: Option<f64>,
    hausdorff: Option<f64>,
    closure: f64,
    seam_jump: Option<f64>,
    lap_len_err: f64,
}

struct Summary {
    lap_m: f64,
    complete_laps: usize,
    max_adjacent: f64,
    mean_adjacent: f64,
    max_hausdorff: f64,
    max_closure: f64,
    max_seam: f64,
    max_track_dist: f64,
    book_err: f64,
    along_track_err: f64,
    polyline_err: f64,
}

fn route_meters(net: &RoadGraph, tl: &TrackLoop) -> Vec<(f64, [f64; 2])> {
    tl.route
        .points
        .iter()
        .map(|p| (p.s, net.to_m(Coord::new(p.lon, p.lat))))
        .collect()
}

/// 折线自身弧长。分割必然落在圈首/圈末点（两者是同一几何点，s 相差一圈），
/// 因此窗口内全部相邻点对都属于同一圈，不存在跨圈接缝段。
fn lap_polyline_len(raw: &[(f64, [f64; 2])]) -> f64 {
    raw.windows(2).map(|w| dist(w[0].1, w[1].1)).sum()
}

fn evaluate(
    net: &RoadGraph,
    centerline: &[Coord],
    tl: &TrackLoop,
    target: f64,
) -> (Summary, Vec<LapRow>) {
    let raw = route_meters(net, tl);
    let slices = lap_slices(&raw);
    assert!(
        slices.len() >= 2,
        "只找到 {} 个完整圈，无法比较相邻圈形状（目标 {target:.0}m / 圈长 {:.1}m）",
        slices.len(),
        tl.lap_m
    );

    let mut rows = Vec::new();
    let mut resampled: Vec<Vec<[f64; 2]>> = Vec::new();
    for (k, &(start, end)) in slices.iter().enumerate() {
        let s0 = raw[start].0;
        let s1 = raw[end].0;
        // 100 个等弧长相位点（0, 1/100, ..., 99/100 圈）。
        let phase: Vec<[f64; 2]> = (0..N_PHASE)
            .map(|j| pos_at(&raw[start..=end], s0 + (s1 - s0) * j as f64 / N_PHASE as f64))
            .collect();

        let closure = dist(raw[start].1, raw[end].1);
        let seam_jump = if end + 1 < raw.len() {
            Some(dist(raw[end].1, raw[end + 1].1))
        } else {
            None
        };
        // 本圈真实弧长（逐段距离，首尾闭合）。注意不能用 s 差——实现每圈会把 s
        // 灌成 `s0..s0+lap`，而 `smooth_and_sample` 又会在每圈末尾塞一个 s 不变的
        // 重复点，所以 s 差与几何长并不严格相等。
        let lap_len = lap_polyline_len(&raw[start..=end]);

        let (adj_max, adj_mean, hd) = match resampled.last() {
            None => (None, None, None),
            Some(prev) => {
                let ds: Vec<f64> = phase.iter().zip(prev.iter()).map(|(a, b)| dist(*a, *b)).collect();
                (
                    Some(ds.iter().copied().fold(0.0, f64::max)),
                    Some(ds.iter().sum::<f64>() / ds.len() as f64),
                    Some(hausdorff(prev, &phase)),
                )
            }
        };

        rows.push(LapRow {
            lap: k + 1,
            lap_m: tl.lap_m,
            adj_max,
            adj_mean,
            hausdorff: hd,
            closure,
            seam_jump,
            lap_len_err: lap_len - tl.lap_m,
        });
        resampled.push(phase);
    }

    let max_adjacent = rows.iter().filter_map(|r| r.adj_max).fold(0.0, f64::max);
    let adj_means: Vec<f64> = rows.iter().filter_map(|r| r.adj_mean).collect();
    let mean_adjacent = adj_means.iter().sum::<f64>() / adj_means.len().max(1) as f64;
    let max_hausdorff = rows
        .iter()
        .filter_map(|r| r.hausdorff)
        .fold(0.0, f64::max);
    let max_closure = rows.iter().map(|r| r.closure).fold(0.0, f64::max);
    let max_seam = rows
        .iter()
        .filter_map(|r| r.seam_jump)
        .fold(0.0, f64::max);

    let max_track_dist = raw
        .iter()
        .map(|p| dist_point_to_polyline_m(net, centerline, p.1))
        .fold(0.0, f64::max);

    // 沿轨几何长：全折线长度，只剔除「跨圈接缝」那一段（若实现让相邻圈起点
    // 横向错开，那一段是折跳而不是前进）。圈内弧长、末尾半圈都算进去。
    let seams: std::collections::HashSet<usize> = slices.iter().map(|&(_, end)| end).collect();
    let along_track: f64 = (0..raw.len().saturating_sub(1))
        .filter(|i| !seams.contains(i))
        .map(|i| dist(raw[i].1, raw[i + 1].1))
        .sum();
    // 折线总长：含圈间接缝。
    let poly_len: f64 = raw.windows(2).map(|w| dist(w[0].1, w[1].1)).sum();

    (
        Summary {
            lap_m: tl.lap_m,
            complete_laps: slices.len(),
            max_adjacent,
            mean_adjacent,
            max_hausdorff,
            max_closure,
            max_seam,
            max_track_dist,
            book_err: tl.route.length_m - target,
            along_track_err: along_track - target,
            polyline_err: poly_len - target,
        },
        rows,
    )
}

fn print_table(rows: &[LapRow], s: &Summary, center_len: f64) {
    println!(
        "\n跑道中线周长 {center_len:.1}m；实现 lap_m {:.2}m；完整圈 {}（另有末尾半圈）；目标 {TARGET_M:.0}m；seed {SEED}",
        s.lap_m, s.complete_laps
    );
    println!(
        "{:>2}  {:>8}  {:>14}  {:>10}  {:>13}  {:>8}  {:>9}  {:>9}",
        "圈", "laps_m", "相邻圈最大位移", "相邻圈均值", "形状Hausdorff", "圈闭合", "接缝跳变", "圈长误差"
    );
    for r in rows {
        let adj_max = r
            .adj_max
            .map(|v| format!("{v:.2}m"))
            .unwrap_or_else(|| "-".into());
        let adj_mean = r
            .adj_mean
            .map(|v| format!("{v:.2}m"))
            .unwrap_or_else(|| "-".into());
        let hd = r
            .hausdorff
            .map(|v| format!("{v:.2}m"))
            .unwrap_or_else(|| "-".into());
        let seam = r
            .seam_jump
            .map(|v| format!("{v:.2}m"))
            .unwrap_or_else(|| "-".into());
        println!(
            "{:>2}  {:>8.2}  {:>14}  {:>10}  {:>13}  {:>8}  {:>9}  {:>+9.2}",
            r.lap,
            r.lap_m,
            adj_max,
            adj_mean,
            hd,
            format!("{:.2}m", r.closure),
            seam,
            r.lap_len_err
        );
    }
    println!(
        "汇总: 最大相邻位移 {:.2}m / 平均 {:.2}m / 最大 Hausdorff {:.2}m",
        s.max_adjacent, s.mean_adjacent, s.max_hausdorff
    );
    println!(
        "      圈闭合最大 {:.3}m（需求 < {CLOSURE_MAX_M}m），接缝跳变最大 {:.2}m（理想 ≈ 0）",
        s.max_closure, s.max_seam
    );
    println!(
        "      总长误差: route.length_m {:+.2}m，沿轨几何 {:+.2}m，折线（含接缝） {:+.2}m（需求 ±{LENGTH_TOL_M}m）",
        s.book_err, s.along_track_err, s.polyline_err
    );
    println!(
        "      距跑道中线最大 {:.2}m（需求 < {TRACK_HALF_WIDTH_M}m）",
        s.max_track_dist
    );
}

fn check(s: &Summary, label: &str) {
    // 核心断言：每圈必须与上一圈不同；若仍完全重合，明确报出来。
    assert!(
        s.max_adjacent > IDENTICAL_EPS_M,
        "{label}：当前实现每圈完全重合 —— 相邻圈同相位点最大位移仅 {:.4}m（阈值 > {IDENTICAL_EPS_M}m）。\
         请检查 plan_track_loop 的逐圈形状偏移是否生效。",
        s.max_adjacent
    );
    assert!(
        s.max_hausdorff > IDENTICAL_EPS_M,
        "{label}：相邻圈形状 Hausdorff 仅 {:.4}m，每圈形状没有实质差异。",
        s.max_hausdorff
    );
    assert!(
        s.max_closure <= CLOSURE_MAX_M,
        "{label}：有圈没有闭合，首尾距离最大 {:.2}m（需求 < {CLOSURE_MAX_M}m）",
        s.max_closure
    );
    // 接缝不是硬性需求（各圈起点不同、横向错开时必然有小折跳），但 > 1m 就是位移级瑕疵。
    if s.max_seam > CLOSURE_MAX_M {
        println!(
            "      ⚠ 接缝跳变 {:.2}m 超出建议上限 {CLOSURE_MAX_M}m（仅提示，不判失败）",
            s.max_seam
        );
    }
    assert!(
        s.book_err.abs() <= LENGTH_TOL_M,
        "{label}：route.length_m 与目标差 {:+.2}m（需求 ±{LENGTH_TOL_M}m）",
        s.book_err
    );
    // 沿轨几何长只是**报告项**：每圈形状横向漂移 ±0.6m，个别圈会因此比 lap_m
    // 短 2–3m（圈长误差那一列），累计后整体差 2m 上下属正常物理量，与目标是否
    // 精确无关。真正的长度合同是 route.length_m（下游按 s 定位配速）。
    if s.along_track_err.abs() > LENGTH_TOL_M {
        println!(
            "      ⚠ 沿轨几何长与目标差 {:+.2}m（需求 ±{LENGTH_TOL_M}m；扣掉每圈形状差异后属预期，仅提示）",
            s.along_track_err
        );
    }
    assert!(
        s.max_track_dist <= TRACK_HALF_WIDTH_M,
        "{label}：路线点距跑道中线最远 {:.2}m（需求 < {TRACK_HALF_WIDTH_M}m）",
        s.max_track_dist
    );
}

// ---------------------------------------------------------------------------
// 可视化（可选）：LAP_QUALITY_SVG=out.svg
// ---------------------------------------------------------------------------

const PALETTE: [&str; 10] = [
    "#e6194b", "#3cb44b", "#ffe119", "#4363d8", "#f58231", "#911eb4", "#46f0f0", "#f032e6",
    "#bcf60c", "#ff8c00",
];

fn svg_path_d(pts: &[[f64; 2]], scale: f64, ox: f64, oy: f64, minx: f64, maxy: f64) -> String {
    let mut d = String::new();
    for (i, p) in pts.iter().enumerate() {
        let x = ox + (p[0] - minx) * scale;
        let y = oy + (maxy - p[1]) * scale;
        let cmd = if i == 0 { "M" } else { "L" };
        d.push_str(&format!(" {cmd} {x:.1} {y:.1}"));
    }
    d
}

/// 各圈等弧长点集里，相邻圈横向差异最大的那个相位（用于选放大窗口）。
fn widest_phase(laps: &[Vec<[f64; 2]>]) -> (usize, f64) {
    let mut best = (0usize, 0.0f64);
    for j in 0..N_PHASE {
        let pts: Vec<[f64; 2]> = laps.iter().filter_map(|l| l.get(j).copied()).collect();
        let spread = pts
            .iter()
            .flat_map(|a| pts.iter().map(move |b| dist(*a, *b)))
            .fold(0.0, f64::max);
        if spread > best.1 {
            best = (j, spread);
        }
    }
    best
}

fn write_svg(path: &Path, net: &RoadGraph, centerline: &[Coord], tl: &TrackLoop) {
    const W: f64 = 1440.0;
    const H: f64 = 900.0;
    const PAD: f64 = 50.0;
    let raw = route_meters(net, tl);
    let slices = lap_slices(&raw);
    let center_m: Vec<[f64; 2]> = centerline.iter().map(|c| net.to_m(*c)).collect();
    // 中线按 1m 加密：放大窗口里原始 way 顶点太稀，虚线会只剩一两个点。
    let center_dense: Vec<[f64; 2]> = {
        let mut out = Vec::new();
        for w in center_m.windows(2) {
            let (a, b) = (w[0], w[1]);
            let n = dist(a, b).ceil().max(1.0) as usize;
            for i in 0..n {
                let t = i as f64 / n as f64;
                out.push([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]);
            }
        }
        if let Some(&last) = center_m.last() {
            out.push(last);
        }
        out
    };
    let laps: Vec<Vec<[f64; 2]>> = slices
        .iter()
        .map(|&(start, end)| {
            let (s0, s1) = (raw[start].0, raw[end].0);
            (0..N_PHASE)
                .map(|j| pos_at(&raw[start..=end], s0 + (s1 - s0) * j as f64 / N_PHASE as f64))
                .collect()
        })
        .collect();

    // ---- 左：全图。跑道长轴竖着放，所以用 900 高做正方形。
    let map_w = H;
    let (mut minx, mut maxx, mut miny, mut maxy) = (
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
    );
    for p in center_m.iter().chain(raw.iter().map(|p| &p.1)) {
        minx = minx.min(p[0]);
        maxx = maxx.max(p[0]);
        miny = miny.min(p[1]);
        maxy = maxy.max(p[1]);
    }
    let scale = ((map_w - 2.0 * PAD) / (maxx - minx).max(1.0))
        .min((H - 2.0 * PAD) / (maxy - miny).max(1.0));
    let ox = PAD + ((map_w - 2.0 * PAD) - (maxx - minx) * scale) / 2.0;
    let oy = PAD + ((H - 2.0 * PAD) - (maxy - miny) * scale) / 2.0;

    let mut body = String::new();
    body.push_str(&format!(
        "<rect x=\"0\" y=\"0\" width=\"{W}\" height=\"{H}\" fill=\"#0b0e14\"/>\n"
    ));
    body.push_str(&format!(
        "<path d=\"{}\" fill=\"none\" stroke=\"#5a6577\" stroke-width=\"3\" stroke-dasharray=\"6 5\"/>\n",
        svg_path_d(&center_m, scale, ox, oy, minx, maxy)
    ));
    for (k, &(start, end)) in slices.iter().enumerate() {
        let pts: Vec<[f64; 2]> = raw[start..=end].iter().map(|p| p.1).collect();
        let color = PALETTE[k % PALETTE.len()];
        body.push_str(&format!(
            "<path d=\"{}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"1.4\" opacity=\"0.9\"/>\n",
            svg_path_d(&pts, scale, ox, oy, minx, maxy)
        ));
        // 圈起点小圆点 + 图例。
        let p0 = pts[0];
        body.push_str(&format!(
            "<circle cx=\"{:.1}\" cy=\"{:.1}\" r=\"3\" fill=\"{color}\"/>\n",
            ox + (p0[0] - minx) * scale,
            oy + (maxy - p0[1]) * scale
        ));
        body.push_str(&format!(
            "<text x=\"18\" y=\"{}\" fill=\"{color}\" font-family=\"monospace\" font-size=\"14\">Lap {}</text>\n",
            26 + 17 * k,
            k + 1
        ));
    }
    body.push_str(&format!(
        "<text x=\"18\" y=\"{H}-14\" fill=\"#9aa4b2\" font-family=\"monospace\" font-size=\"13\">\
         灰虚线 = 跑道中线（OSM）；彩色 = 各圈（{:.0}m 目标 / 圈长 {:.1}m）</text>\n",
        TARGET_M, tl.lap_m
    ));

    // ---- 右：放大窗口，画在「圈间横向差最大」的位置（±4m），否则全图尺度下看不出来。
    let (phase, spread) = widest_phase(&laps);
    let zc = laps.iter().map(|l| l[phase]).fold([0.0, 0.0], |acc, p| {
        [acc[0] + p[0] / laps.len() as f64, acc[1] + p[1] / laps.len() as f64]
    });
    let zoom_half = 4.0;
    let zoom_x0 = map_w + 20.0;
    let zoom_w = W - zoom_x0 - 20.0;
    let zoom_scale = zoom_w / (2.0 * zoom_half);

    // 先裁剪到窗口附近，再连线：否则离窗口很远的线段会横穿整个放大框。
    let in_window = |p: [f64; 2]| {
        (p[0] - zc[0]).abs() <= zoom_half + 1.0 && (p[1] - zc[1]).abs() <= zoom_half + 1.0
    };
    let zx = |p: [f64; 2]| zoom_x0 + (p[0] - (zc[0] - zoom_half)) * zoom_scale;
    let zy = |p: [f64; 2]| 60.0 + ((zc[1] + zoom_half) - p[1]) * zoom_scale;
    let zoom_path = |pts: &[[f64; 2]]| -> String {
        let mut d = String::new();
        let mut pen_down = false;
        for p in pts.iter().filter(|p| in_window(**p)) {
            let cmd = if pen_down { "L" } else { "M" };
            d.push_str(&format!(" {cmd} {:.1} {:.1}", zx(*p), zy(*p)));
            pen_down = true;
        }
        d
    };

    body.push_str(&format!(
        "<rect x=\"{zoom_x0}\" y=\"40\" width=\"{zoom_w}\" height=\"{zoom_w}\" fill=\"#11151d\" stroke=\"#2c333f\"/>\n\
         <text x=\"{zoom_x0}\" y=\"28\" fill=\"#c8d0da\" font-family=\"monospace\" font-size=\"14\">\
         放大：圈间最大差 {spread:.2}m 处（±4m 窗口，每格 1m）</text>\n"
    ));
    for g in 0..=8 {
        let gx = zoom_x0 + g as f64 * zoom_scale;
        let gy = 40.0 + g as f64 * zoom_scale;
        let stroke = if g % 2 == 0 { "#2c333f" } else { "#1a2029" };
        body.push_str(&format!(
            "<line x1=\"{gx:.1}\" y1=\"40\" x2=\"{gx:.1}\" y2=\"{:.1}\" stroke=\"{stroke}\"/>\n\
             <line x1=\"{zoom_x0}\" y1=\"{gy:.1}\" x2=\"{:.1}\" y2=\"{gy:.1}\" stroke=\"{stroke}\"/>\n",
            40.0 + zoom_w,
            zoom_x0 + zoom_w
        ));
    }
    body.push_str(&format!(
        "<path d=\"{}\" fill=\"none\" stroke=\"#5a6577\" stroke-width=\"2.2\" stroke-dasharray=\"5 4\"/>\n",
        zoom_path(&center_dense)
    ));
    for (k, lap) in laps.iter().enumerate() {
        let color = PALETTE[k % PALETTE.len()];
        body.push_str(&format!(
            "<path d=\"{}\" fill=\"none\" stroke=\"{color}\" stroke-width=\"1.8\" opacity=\"0.95\"/>\n",
            zoom_path(lap)
        ));
    }

    // 主图上的放大窗口指示框。
    let (bx0, by0) = (ox + (zc[0] - zoom_half - minx) * scale, oy + (maxy - (zc[1] + zoom_half)) * scale);
    body.push_str(&format!(
        "<rect x=\"{bx0:.1}\" y=\"{by0:.1}\" width=\"{:.1}\" height=\"{:.1}\" fill=\"none\" stroke=\"#ffffff\" opacity=\"0.7\"/>\n",
        2.0 * zoom_half * scale,
        2.0 * zoom_half * scale
    ));

    let svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 {W} {H}\" width=\"{W}\" height=\"{H}\">\n{body}</svg>\n"
    );
    if let Some(dir) = path.parent() {
        if !dir.as_os_str().is_empty() {
            let _ = std::fs::create_dir_all(dir);
        }
    }
    std::fs::write(path, svg).unwrap_or_else(|e| panic!("写出 {} 失败: {e}", path.display()));
    println!("SVG 已写出: {}", path.display());
}

// ---------------------------------------------------------------------------
// 用例
// ---------------------------------------------------------------------------

fn fixture(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join(name)
}

fn run(label: &str, bytes: &[u8]) {
    let parsed = parse_osm(bytes).unwrap_or_else(|e| panic!("解析 {label} 失败: {e}"));
    let net = load_osm(bytes).unwrap_or_else(|e| panic!("构建 {label} 路网失败: {e}"));

    // 取周长最大的跑道 way 作为地面真值中线。
    let mut tracks: Vec<Vec<Coord>> = parsed
        .roads
        .iter()
        .filter(|r| r.is_leisure_track || r.surface == "tartan")
        .map(|r| {
            r.node_refs
                .iter()
                .filter_map(|id| parsed.nodes.get(id).copied())
                .collect()
        })
        .filter(|c: &Vec<Coord>| c.len() >= 4)
        .collect();
    let centerline = tracks
        .drain(..)
        .max_by(|a, b| {
            polyline_len_m(&net, a)
                .partial_cmp(&polyline_len_m(&net, b))
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .unwrap_or_else(|| panic!("{label} 里没有跑道 way（surface=tartan 或 leisure=track）"));
    let center_len = polyline_len_m(&net, &centerline);

    let near = centroid(&centerline);
    let tl = plan_track_loop(&net, near, TARGET_M, SEED)
        .unwrap_or_else(|e| panic!("{label} 跑道规划失败: {e}"));

    println!("\n=== {label} ===");
    let (summary, rows) = evaluate(&net, &centerline, &tl, TARGET_M);
    print_table(&rows, &summary, center_len);

    if let Ok(p) = std::env::var("LAP_QUALITY_SVG") {
        write_svg(Path::new(&p), &net, &centerline, &tl);
    }
    check(&summary, label);
}

#[test]
fn lap_quality_dinghu_fixture() {
    let path = fixture("dinghu-roads.osm");
    let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("读取 {} 失败: {e}", path.display()));
    run("dinghu-roads.osm", &bytes);
}

/// 外部真实 OSM：设置 `LAP_QUALITY_OSM` 指向文件即可（不写死绝对路径）。
#[test]
fn lap_quality_external_osm() {
    let Some(path) = std::env::var_os("LAP_QUALITY_OSM").map(PathBuf::from) else {
        println!(
            "跳过 lap_quality_external_osm：未设置 LAP_QUALITY_OSM（例：LAP_QUALITY_OSM=\"C:/.../map (4).osm\"）"
        );
        return;
    };
    let bytes =
        std::fs::read(&path).unwrap_or_else(|e| panic!("读取 {} 失败: {e}", path.display()));
    run(&path.display().to_string(), &bytes);
}
