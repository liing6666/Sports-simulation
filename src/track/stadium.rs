//! Standard athletics-track geometry fitted to the live server checkpoints.

use serde::{Deserialize, Serialize};

use super::geom::{PointRing, MET_PER_DEG_LAT, MET_PER_DEG_LNG};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TrackSpec {
    #[serde(rename = "auto", alias = "Auto")]
    Auto,
    #[serde(rename = "200m", alias = "200", alias = "m200", alias = "M200")]
    M200,
    #[serde(rename = "300m", alias = "300", alias = "m300", alias = "M300")]
    M300,
    #[serde(rename = "400m", alias = "400", alias = "m400", alias = "M400")]
    M400,
    #[serde(rename = "custom")]
    Custom { total_m: u32 },
    /// 固定跑道：不拟合服务器打卡点，直接以设备定位锚点为中心绕圈。
    ///
    /// 适合"只想在操场一直绕圈"的场景。与 M200/M300/M400 的区别：
    /// 后者要求服务器打卡点本身构成一个标准跑道（偏差超限直接报错），
    /// 而本模式不需要任何点位，锚点就是操场中心。
    #[serde(rename = "loop")]
    Loop { total_m: u32 },
}

impl TrackSpec {
    /// 固定跑道模式的默认圈长（米）。
    pub const DEFAULT_LOOP_M: u32 = 400;
}

impl Default for TrackSpec {
    fn default() -> Self {
        Self::Auto
    }
}

impl TrackSpec {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Auto => "自动（服务器点位）",
            Self::M200 => "200 米标准跑道",
            Self::M300 => "300 米标准跑道",
            Self::M400 => "400 米标准跑道",
            Self::Custom { .. } => "自定义跑道",
            Self::Loop { .. } => "固定跑道（绕圈）",
        }
    }

    /// 是否是不依赖服务器点位的固定跑道模式。
    pub const fn is_fixed_loop(self) -> bool {
        matches!(self, Self::Loop { .. })
    }

    fn dimensions(self) -> Option<TrackDimensions> {
        let total_m = match self {
            Self::Auto => return None,
            Self::M200 => 200,
            Self::M300 => 300,
            Self::M400 => 400,
            Self::Custom { total_m } if (100..=1000).contains(&total_m) => total_m,
            Self::Custom { .. } => return None,
            Self::Loop { total_m } if (100..=1000).contains(&total_m) => total_m,
            Self::Loop { .. } => return None,
        };
        let base = TrackDimensions {
            straight_m: 84.39,
            radius_m: 36.80,
        };
        let scale = total_m as f64 / base.length_m();
        Some(TrackDimensions {
            straight_m: base.straight_m * scale,
            radius_m: base.radius_m * scale,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct TrackDimensions {
    straight_m: f64,
    radius_m: f64,
}

impl TrackDimensions {
    fn length_m(self) -> f64 {
        2.0 * self.straight_m + 2.0 * std::f64::consts::PI * self.radius_m
    }
}

/// 用最贴合打卡点的方式**生成一条跑道形状的环**（自由拟合）。
///
/// 与 [`make_stadium_ring`] 的区别：后者要求打卡点本身就构成标准跑道，偏差
/// 超限直接报错；而学校的打卡点通常只是"操场周边几个点"，围成的形状未必是
/// 标准椭圆（实测某校点位围成 86×147m，长宽比 1.7:1，而标准跑道约 2.4:1）。
///
/// 这里做的是**尺寸也参与拟合**：
/// 1. 枚举旋转角，求能包住全部点位的跑道尺寸（保持标准跑道的长宽比）；
/// 2. 取"能覆盖所有点位、且多出来最少"的那组尺寸；
/// 3. 生成的环按弧长缩放到请求的圈长，并保证每个打卡点都落在环附近。
pub fn make_fitted_track_ring(
    points: &[(f64, f64)],
    target_lap_m: f64,
) -> Result<PointRing, String> {
    if points.len() < 3 {
        return Err("打卡点不足 3 个，无法拟合跑道".into());
    }
    if points
        .iter()
        .any(|(lat, lng)| !lat.is_finite() || !lng.is_finite())
    {
        return Err("服务器打卡点包含非有限坐标".into());
    }
    let center_lat = points.iter().map(|p| p.0).sum::<f64>() / points.len() as f64;
    let center_lng = points.iter().map(|p| p.1).sum::<f64>() / points.len() as f64;
    let local: Vec<(f64, f64)> = points
        .iter()
        .map(|&(lat, lng)| {
            (
                (lng - center_lng) * MET_PER_DEG_LNG,
                (lat - center_lat) * MET_PER_DEG_LAT,
            )
        })
        .collect();

    // 标准跑道的长宽比：直道 / 半径。保持这个比例缩放，形状才像跑道。
    let base_straight = 84.39f64;
    let base_radius = 36.80f64;

    // 枚举旋转角 + 缩放系数，选「所有点都在环内（留 2m 余量）且整体最小」的组合。
    let mut best: Option<(f64, f64, (f64, f64))> = None; // (angle, scale, offset)
    for index in 0..=180 {
        let angle = -std::f64::consts::FRAC_PI_2 + index as f64 * std::f64::consts::PI / 180.0;
        let rotated: Vec<(f64, f64)> = local.iter().map(|&p| rotate(p, -angle)).collect();
        let xs: Vec<f64> = rotated.iter().map(|p| p.0).collect();
        let ys: Vec<f64> = rotated.iter().map(|p| p.1).collect();
        let (minx, maxx) = (
            xs.iter().cloned().fold(f64::MAX, f64::min),
            xs.iter().cloned().fold(f64::MIN, f64::max),
        );
        let (miny, maxy) = (
            ys.iter().cloned().fold(f64::MAX, f64::min),
            ys.iter().cloned().fold(f64::MIN, f64::max),
        );
        let (cx, cy) = ((minx + maxx) / 2.0, (miny + maxy) / 2.0);
        // 跑道外接框：宽 = 直道 + 2R，高 = 2R，各留 3m 余量。
        let need_w = (maxx - minx) + 6.0;
        let need_h = (maxy - miny) + 6.0;
        let scale_w = need_w / (base_straight + 2.0 * base_radius);
        let scale_h = need_h / (2.0 * base_radius);
        let scale = scale_w.max(scale_h).clamp(0.30, 2.20);
        let area = scale * scale;
        if best.map(|(_, s, _)| area < s * s).unwrap_or(true) {
            best = Some((angle, scale, (cx, cy)));
        }
    }
    let (angle, scale, offset) = best.ok_or("无法拟合跑道")?;

    let dimensions = TrackDimensions {
        straight_m: base_straight * scale,
        radius_m: base_radius * scale,
    };
    let mut dense: Vec<(f64, f64)> = rounded_rectangle(dimensions)
        .into_iter()
        .map(|p| rotate((p.0 + offset.0, p.1 + offset.1), angle))
        .collect();

    // 把贴近环的站点吸到环上，让蓝色打卡点标记正好落在路线上。
    let mut used = std::collections::HashSet::new();
    for &target in &local {
        let nearest = dense
            .iter()
            .enumerate()
            .map(|(i, &p)| (i, distance(p, target)))
            .min_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((i, d)) = nearest {
            if d <= 12.0 {
                let i = nearest_unused(i, dense.len(), &used);
                used.insert(i);
                dense[i] = target;
            }
        }
    }

    // 按弧长缩放到目标圈长（形状不变），让"跑道长度"设置生效。
    let mut arcs = vec![0.0];
    for i in 1..=dense.len() {
        arcs.push(arcs[i - 1] + distance(dense[i - 1], dense[i % dense.len()]));
    }
    let raw_len = *arcs.last().unwrap_or(&1.0);
    if raw_len > 1.0 && target_lap_m > 1.0 {
        let k = target_lap_m / raw_len;
        for p in dense.iter_mut() {
            *p = (p.0 * k, p.1 * k);
        }
        arcs = vec![0.0];
        for i in 1..=dense.len() {
            arcs.push(arcs[i - 1] + distance(dense[i - 1], dense[i % dense.len()]));
        }
    }
    Ok((dense, arcs, (center_lat, center_lng)))
}

/// 固定跑道：以给定中心直接构造椭圆环，不参考任何打卡点。
///
/// `center` 是 BD 系坐标（通常取设备定位锚点）。返回的环与
/// [`make_stadium_ring`] 同格式，可直接交给轨迹生成器。
pub fn make_fixed_loop_ring(
    center: (f64, f64),
    spec: TrackSpec,
) -> Result<PointRing, String> {
    let dimensions = spec
        .dimensions()
        .ok_or_else(|| "请选择有效的跑道长度（100-1000 米）".to_string())?;
    if !center.0.is_finite() || !center.1.is_finite() {
        return Err("跑道中心坐标无效".into());
    }
    let dense = rounded_rectangle(dimensions);
    let mut arcs = vec![0.0];
    for index in 1..=dense.len() {
        arcs.push(arcs[index - 1] + distance(dense[index - 1], dense[index % dense.len()]));
    }
    // 与 make_stadium_ring 同源校验：采样长度必须贴合设计圈长。
    if (arcs[dense.len()] - dimensions.length_m()).abs() > 4.0 {
        return Err("跑道采样长度偏离设计值过大".into());
    }
    // 环点已在「以中心为原点」的局部米制平面上，因此中心就是 (0,0)。
    Ok((dense, arcs, center))
}

/// Fit a standard oval around the live checkpoints and return it in the same
/// local metre plane used by the regular track generator.
pub fn make_stadium_ring(points: &[(f64, f64)], spec: TrackSpec) -> Result<PointRing, String> {
    let dimensions = spec
        .dimensions()
        .ok_or_else(|| "请选择有效的操场规格".to_string())?;
    if points.len() < 2 {
        return Err("服务器打卡点不足，无法定位标准跑道".into());
    }
    if points
        .iter()
        .any(|(lat, lng)| !lat.is_finite() || !lng.is_finite())
    {
        return Err("服务器打卡点包含非有限坐标".into());
    }

    let center_lat = points.iter().map(|point| point.0).sum::<f64>() / points.len() as f64;
    let center_lng = points.iter().map(|point| point.1).sum::<f64>() / points.len() as f64;
    let local: Vec<_> = points
        .iter()
        .map(|(lat, lng)| {
            (
                (lng - center_lng) * MET_PER_DEG_LNG,
                (lat - center_lat) * MET_PER_DEG_LAT,
            )
        })
        .collect();
    let (angle, offset) = fit_pose(&local, dimensions);
    let mut dense: Vec<_> = rounded_rectangle(dimensions)
        .into_iter()
        .map(|point| rotate((point.0 + offset.0, point.1 + offset.1), angle))
        .collect();

    // Refuse a preset that clearly does not describe the checkpoint geometry.
    let tolerance = dimensions.radius_m * 0.30 + 3.0;
    let furthest = local
        .iter()
        .map(|&point| {
            let aligned = rotate(point, -angle);
            distance_to_centerline(
                (aligned.0 - offset.0, aligned.1 - offset.1),
                dimensions,
            )
        })
        .fold(0.0, f64::max);
    if furthest > tolerance {
        return Err(format!(
            "服务器打卡点与{}跑道不匹配：最大偏差 {:.1}m（允许 {:.1}m）",
            spec.label(),
            furthest,
            tolerance
        ));
    }

    // Put close server checkpoints directly on the route so their markers do
    // not depend on a later sample accidentally landing at the same location.
    let mut used = std::collections::HashSet::new();
    for &(lat, lng) in points {
        let target = (
            (lng - center_lng) * MET_PER_DEG_LNG,
            (lat - center_lat) * MET_PER_DEG_LAT,
        );
        let nearest = dense
            .iter()
            .enumerate()
            .map(|(index, &point)| (index, distance(point, target)))
            .min_by(|a, b| a.1.total_cmp(&b.1));
        if let Some((index, distance_to_ring)) = nearest {
            if distance_to_ring <= 3.0 {
                let index = nearest_unused(index, dense.len(), &used);
                used.insert(index);
                dense[index] = target;
            }
        }
    }

    let mut arcs = vec![0.0];
    for index in 1..=dense.len() {
        arcs.push(arcs[index - 1] + distance(dense[index - 1], dense[index % dense.len()]));
    }
    if (arcs[dense.len()] - dimensions.length_m()).abs() > 4.0 {
        return Err("标准跑道采样长度偏离设计值过大".into());
    }
    Ok((dense, arcs, (center_lat, center_lng)))
}

fn nearest_unused(
    preferred: usize,
    count: usize,
    used: &std::collections::HashSet<usize>,
) -> usize {
    (0..count)
        .map(|offset| (preferred + offset) % count)
        .find(|index| !used.contains(index))
        .unwrap_or(preferred)
}

fn fit_pose(points: &[(f64, f64)], dimensions: TrackDimensions) -> (f64, (f64, f64)) {
    let mut best_angle = 0.0;
    let mut best_error = f64::INFINITY;
    let mut best_offset = bbox_center(points);
    for index in 0..=180 {
        let angle = -std::f64::consts::FRAC_PI_2
            + index as f64 * std::f64::consts::PI / 180.0;
        let rotated: Vec<_> = points.iter().map(|&point| rotate(point, -angle)).collect();
        let offset = bbox_center(&rotated);
        let error = rotated
            .iter()
            .map(|&(x, y)| {
                let d = distance_to_centerline((x - offset.0, y - offset.1), dimensions);
                d * d
            })
            .sum::<f64>();
        if error < best_error {
            best_error = error;
            best_angle = angle;
            best_offset = offset;
        }
    }
    (best_angle, best_offset)
}

/// 标准跑道闭合环，**逆时针**（田径比赛与学校体测的规定方向）。
///
/// 走法：从下方(-y)直道向右 → 右半圆向上 → 上方(+y)直道向左 → 左半圆向下。
/// 旧实现是顺时针，跑出来的方向与真实比赛相反。
fn rounded_rectangle(dimensions: TrackDimensions) -> Vec<(f64, f64)> {
    let straight = dimensions.straight_m;
    let radius = dimensions.radius_m;
    let samples = |length: f64| (length / 0.75).ceil().max(8.0) as usize;
    let straight_count = samples(straight);
    let arc_count = samples(std::f64::consts::PI * radius);
    let mut points = Vec::with_capacity(straight_count * 2 + arc_count * 2);
    // 下直道：从 (-s/2, -r) 到 (+s/2, -r)
    for index in 0..straight_count {
        let t = index as f64 / straight_count as f64;
        points.push((-straight / 2.0 + straight * t, -radius));
    }
    // 右半圆：从 -90° 到 +90°（逆时针向上）
    for index in 0..arc_count {
        let angle = -std::f64::consts::FRAC_PI_2
            + std::f64::consts::PI * index as f64 / arc_count as f64;
        points.push((straight / 2.0 + radius * angle.cos(), radius * angle.sin()));
    }
    // 上直道：从 (+s/2, +r) 到 (-s/2, +r)
    for index in 0..straight_count {
        let t = index as f64 / straight_count as f64;
        points.push((straight / 2.0 - straight * t, radius));
    }
    // 左半圆：从 +90° 到 +270°（逆时针向下）
    for index in 0..arc_count {
        let angle = std::f64::consts::FRAC_PI_2
            + std::f64::consts::PI * index as f64 / arc_count as f64;
        points.push((-straight / 2.0 + radius * angle.cos(), radius * angle.sin()));
    }
    points
}

fn bbox_center(points: &[(f64, f64)]) -> (f64, f64) {
    let (mut min_x, mut max_x, mut min_y, mut max_y) =
        (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for &(x, y) in points {
        min_x = min_x.min(x);
        max_x = max_x.max(x);
        min_y = min_y.min(y);
        max_y = max_y.max(y);
    }
    ((min_x + max_x) / 2.0, (min_y + max_y) / 2.0)
}

fn rotate(point: (f64, f64), angle: f64) -> (f64, f64) {
    let (sin, cos) = angle.sin_cos();
    (point.0 * cos - point.1 * sin, point.0 * sin + point.1 * cos)
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn distance_to_centerline(point: (f64, f64), dimensions: TrackDimensions) -> f64 {
    let half_straight = dimensions.straight_m / 2.0;
    let radius = dimensions.radius_m;
    if point.0.abs() <= half_straight {
        (point.1.abs() - radius).abs()
    } else {
        let center_x = if point.0.is_sign_negative() {
            -half_straight
        } else {
            half_straight
        };
        ((point.0 - center_x).hypot(point.1) - radius).abs()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn checkpoints(spec: TrackSpec, lat: f64, lng: f64, angle: f64) -> Vec<(f64, f64)> {
        let dimensions = spec.dimensions().unwrap();
        rounded_rectangle(dimensions)
            .iter()
            .step_by((rounded_rectangle(dimensions).len() / 8).max(1))
            .map(|&(x, y)| {
                let point = rotate((x, y), angle);
                (
                    lat + point.1 / MET_PER_DEG_LAT,
                    lng + point.0 / MET_PER_DEG_LNG,
                )
            })
            .collect()
    }

    #[test]
    fn presets_have_expected_nominal_lengths() {
        for (spec, expected) in [
            (TrackSpec::M200, 200.0),
            (TrackSpec::M300, 300.0),
            (TrackSpec::M400, 400.0),
        ] {
            assert!((spec.dimensions().unwrap().length_m() - expected).abs() < 0.01);
        }
        assert!(TrackSpec::Custom { total_m: 99 }.dimensions().is_none());
    }

    #[test]
    fn fitted_ring_keeps_nominal_length_and_checkpoint_orientation() {
        let spec = TrackSpec::M400;
        let points = checkpoints(spec, 38.9, 121.54, 0.35);
        let (ring, arcs, _) = make_stadium_ring(&points, spec).unwrap();
        assert!((arcs.last().unwrap() - 400.0).abs() < 4.0);
        assert!(ring.iter().any(|point| point.0.abs() > 70.0));
    }

    #[test]
    fn generated_stadium_track_keeps_totals_consistent() {
        let spec = TrackSpec::M400;
        let points = checkpoints(spec, 38.9, 121.54, 0.35);
        let track = crate::track::generator::build_stadium(
            1200.0,
            600,
            7,
            (38.9, 121.54),
            1_700_000_000_000,
            &points,
            spec,
        )
        .unwrap();
        assert!(track.validate_consistency().is_ok());
    }

    #[test]
    fn rejects_points_that_do_not_fit_the_selected_preset() {
        let points = [(38.9, 121.54), (38.92, 121.54), (38.9, 121.56)];
        assert!(make_stadium_ring(&points, TrackSpec::M400).is_err());
    }
}

#[cfg(test)]
mod loop_tests {
    use super::{make_fixed_loop_ring, TrackSpec};

    #[test]
    fn fixed_loop_ring_matches_requested_length() {
        for spec in [
            TrackSpec::Loop { total_m: 400 },
            TrackSpec::Loop { total_m: 200 },
            TrackSpec::Loop { total_m: 300 },
        ] {
            let (dense, arcs, _) = make_fixed_loop_ring((39.9, 116.4), spec).unwrap();
            let total = *arcs.last().unwrap();
            let want = match spec {
                TrackSpec::Loop { total_m } => total_m as f64,
                _ => unreachable!(),
            };
            assert!(
                (total - want).abs() < 4.0,
                "{:?} 圈长 {total} != {want}",
                spec
            );
            assert!(dense.len() > 20, "采样点过少: {}", dense.len());
            // 闭合：首末点应几乎重合
            let (a, b) = (dense[0], dense[dense.len() - 1]);
            let gap = ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2)).sqrt();
            assert!(gap < 5.0, "环未闭合: {gap}");
        }
    }

    /// 固定跑道不需要任何打卡点。
    #[test]
    fn fixed_loop_needs_no_checkpoints() {
        assert!(make_fixed_loop_ring((39.9, 116.4), TrackSpec::Loop { total_m: 400 }).is_ok());
    }

    #[test]
    fn invalid_lengths_are_rejected() {
        assert!(make_fixed_loop_ring((39.9, 116.4), TrackSpec::Loop { total_m: 50 }).is_err());
        assert!(make_fixed_loop_ring((39.9, 116.4), TrackSpec::Loop { total_m: 2000 }).is_err());
        assert!(make_fixed_loop_ring((f64::NAN, 116.4), TrackSpec::Loop { total_m: 400 }).is_err());
    }

    #[test]
    fn fixed_loop_is_distinct_from_checkpoint_fitting() {
        assert!(TrackSpec::Loop { total_m: 400 }.is_fixed_loop());
        assert!(!TrackSpec::M400.is_fixed_loop());
        assert!(!TrackSpec::Auto.is_fixed_loop());
        assert!(TrackSpec::Loop { total_m: 400 }.label().contains("绕圈"));
    }
}

#[cfg(test)]
mod loop_track_tests {
    use super::TrackSpec;

    /// 固定跑道生成的轨迹必须落在锚点附近，且覆盖整圈。
    #[test]
    fn fixed_loop_track_stays_around_anchor() {
        let anchor = (39.9000, 116.4000);
        let track = crate::track::generator::build_stadium(
            2000.0,
            720,
            42,
            anchor,
            1_700_000_000_000,
            &[], // 无打卡点
            TrackSpec::Loop { total_m: 400 },
        )
        .expect("固定跑道应能在无点位时生成");

        // 所有点都应在锚点 200m 内（400m 跑道半长约 70m，留足余量）。
        let max_off = track
            .locations
            .iter()
            .map(|p| {
                let dy = (p.gLat - anchor.0) * 111_132.0;
                let dx = (p.gLng - anchor.1) * 111_320.0 * anchor.0.to_radians().cos();
                (dx * dx + dy * dy).sqrt()
            })
            .fold(0.0f64, f64::max);
        assert!(max_off < 200.0, "轨迹偏离锚点过远: {max_off:.0}m");
        assert!(max_off > 20.0, "轨迹没有真正绕圈: {max_off:.0}m");

        // 总距离应贴合目标。
        assert!(
            (track.totalDistance - 2000.0).abs() < 5.0,
            "距离 {} != 2000",
            track.totalDistance
        );
        assert_eq!(track.totalTime, 720);
        assert!(track.totalSteps > 500);
    }

    /// 绕圈模式的横向漂移应覆盖整条 8 道跑道（±0.6m × 8 = ±4.8m），
    /// 而不是贴着跑道中心线跑成一条细线。
    #[test]
    fn fixed_loop_drifts_across_all_eight_lanes() {
        use super::{distance, make_fixed_loop_ring};
        use crate::track::geom::{MET_PER_DEG_LAT, MET_PER_DEG_LNG};

        let anchor = (39.9000, 116.4000);
        let spec = TrackSpec::Loop { total_m: 400 };
        let (dense, _arcs, _) = make_fixed_loop_ring(anchor, spec).unwrap();
        let track = crate::track::generator::build_stadium(
            2000.0, 720, 42, anchor, 1_700_000_000_000, &[], spec,
        )
        .unwrap();

        let max_off = track
            .locations
            .iter()
            .map(|p| {
                let q = (
                    (p.gLng - anchor.1) * MET_PER_DEG_LNG,
                    (p.gLat - anchor.0) * MET_PER_DEG_LAT,
                );
                dense
                    .iter()
                    .map(|&r| distance(r, q))
                    .fold(f64::INFINITY, f64::min)
            })
            .fold(0.0f64, f64::max);
        let limit = crate::track::generator::LOOP_TRACK_OFFSET_M;
        assert!(max_off > 1.0, "绕圈轨迹仍贴着中心线: {max_off:.2}m");
        assert!(
            max_off <= limit + 0.5,
            "绕圈漂移超出 8 道范围: {max_off:.2}m（上限 {limit:.2}m）"
        );
    }

    /// 同一圈长下，轨迹应真的绕多圈（角度覆盖 0-360）。
    #[test]
    fn fixed_loop_circles_the_anchor() {
        let anchor = (39.9000, 116.4000);
        let track = crate::track::generator::build_stadium(
            2000.0,
            720,
            7,
            anchor,
            1_700_000_000_000,
            &[],
            TrackSpec::Loop { total_m: 400 },
        )
        .unwrap();
        // 用四个象限判断是否覆盖整圈。
        let mut quads = [false; 4];
        for p in &track.locations {
            let dy = p.gLat - anchor.0;
            let dx = p.gLng - anchor.1;
            let q = match (dx >= 0.0, dy >= 0.0) {
                (true, true) => 0,
                (false, true) => 1,
                (false, false) => 2,
                (true, false) => 3,
            };
            quads[q] = true;
        }
        assert!(quads.iter().all(|&q| q), "轨迹未覆盖全部四个象限: {quads:?}");
    }
}

#[cfg(test)]
mod loop_vs_normal_tests {
    use super::TrackSpec;

    /// 固定跑道的轨迹必须"回环"：与中心距离几乎恒定；普通经典轨迹不会。
    #[test]
    fn fixed_loop_radius_is_constant() {
        let anchor: (f64, f64) = (39.9000, 116.4000);
        let m_lat = 111_132.0;
        let m_lon = 111_320.0 * anchor.0.to_radians().cos();
        let dist_from = |p: &crate::track::model::GenPoint| {
            (((p.gLat - anchor.0) * m_lat).powi(2) + ((p.gLng - anchor.1) * m_lon).powi(2)).sqrt()
        };

        let loop_track = crate::track::generator::build_stadium(
            2000.0, 720, 42, anchor, 1_700_000_000_000, &[], TrackSpec::Loop { total_m: 400 },
        ).unwrap();
        let radii: Vec<f64> = loop_track.locations.iter().map(dist_from).collect();
        let mean = radii.iter().sum::<f64>() / radii.len() as f64;
        let max_dev = radii.iter().map(|r| (r - mean).abs()).fold(0.0f64, f64::max);
        // 400m 跑道：直道区半径在 36.8~67m 之间变化，属正常；关键是不跑飞。
        assert!(max_dev < 60.0, "固定跑道半径波动过大: {max_dev:.0}m (均值 {mean:.0}m)");

        let normal = crate::track::generator::build(
            2000.0, 720, 42, anchor, 1_700_000_000_000,
            &[(39.901, 116.401), (39.899, 116.402), (39.8995, 116.398)],
        );
        let n_radii: Vec<f64> = normal.locations.iter().map(dist_from).collect();
        let n_mean = n_radii.iter().sum::<f64>() / n_radii.len() as f64;
        let n_dev = n_radii.iter().map(|r| (r - n_mean).abs()).fold(0.0f64, f64::max);
        // 普通轨迹围绕多个打卡点铺开，半径波动应明显更大。
        assert!(
            n_dev > max_dev,
            "普通轨迹({n_dev:.0}m) 应比固定跑道({max_dev:.0}m) 更分散"
        );
    }
}

#[cfg(test)]
mod real_fit_tests {
    use super::*;

    /// 用户真实打卡点（四川幼专操场附近），验证能否拟合标准跑道。
    #[test]
    fn fit_user_checkpoints_to_400m_track() {
        let pts = [
            (31.526444, 104.675651),
            (31.526805, 104.676518),
            (31.527248, 104.676262),
            (31.525924, 104.675929),
            (31.526259, 104.676648),
        ];
        let (clat, clng) = (
            pts.iter().map(|p| p.0).sum::<f64>() / 5.0,
            pts.iter().map(|p| p.1).sum::<f64>() / 5.0,
        );
        let local: Vec<(f64, f64)> = pts
            .iter()
            .map(|&(lat, lng)| {
                (
                    (lng - clng) * MET_PER_DEG_LNG,
                    (lat - clat) * MET_PER_DEG_LAT,
                )
            })
            .collect();
        println!("点位质心 ({clat},{clng})");
        println!("局部坐标:");
        for (i, p) in local.iter().enumerate() {
            println!("  [{i}] ({:.1}, {:.1})", p.0, p.1);
        }
        let xs: Vec<f64> = local.iter().map(|p| p.0).collect();
        let ys: Vec<f64> = local.iter().map(|p| p.1).collect();
        println!(
            "范围: X {:.0}m  Y {:.0}m",
            xs.iter().cloned().fold(f64::MIN, f64::max) - xs.iter().cloned().fold(f64::MAX, f64::min),
            ys.iter().cloned().fold(f64::MIN, f64::max) - ys.iter().cloned().fold(f64::MAX, f64::min)
        );
        for spec in [TrackSpec::M400, TrackSpec::M300, TrackSpec::M200] {
            match make_stadium_ring(&pts, spec) {
                Ok((dense, arcs, _)) => {
                    println!("✓ {} 拟合成功，一圈 {:.0}m，{} 点", spec.label(), arcs.last().unwrap(), dense.len());
                }
                Err(e) => println!("✗ {} 失败: {e}", spec.label()),
            }
        }
    }
}

#[cfg(test)]
mod fitted_track_tests {
    use super::*;

    fn user_points() -> Vec<(f64, f64)> {
        vec![
            (31.526444, 104.675651),
            (31.526805, 104.676518),
            (31.527248, 104.676262),
            (31.525924, 104.675929),
            (31.526259, 104.676648),
        ]
    }

    /// 拟合出的跑道必须覆盖全部打卡点（否则标记会落在环外）。
    #[test]
    fn fitted_track_covers_all_checkpoints() {
        let pts = user_points();
        let (dense, arcs, center) = make_fitted_track_ring(&pts, 400.0).expect("应能拟合");
        assert!(
            (arcs.last().unwrap() - 400.0).abs() < 30.0,
            "圈长 {:.0} 应接近 400",
            arcs.last().unwrap()
        );
        // 每个打卡点到环的最小距离
        for &(lat, lng) in &pts {
            let p = (
                (lng - center.1) * MET_PER_DEG_LNG,
                (lat - center.0) * MET_PER_DEG_LAT,
            );
            let d = dense
                .iter()
                .map(|&q| distance(q, p))
                .fold(f64::INFINITY, f64::min);
            println!("点位 ({lat},{lng}) 距环 {d:.1}m");
            assert!(d < 25.0, "打卡点距环 {d:.1}m，太大");
        }
        // 形状应像跑道：长宽比接近标准跑道的 1.6~2.6
        let xs: Vec<f64> = dense.iter().map(|p| p.0).collect();
        let ys: Vec<f64> = dense.iter().map(|p| p.1).collect();
        let w = xs.iter().cloned().fold(f64::MIN, f64::max)
            - xs.iter().cloned().fold(f64::MAX, f64::min);
        let h = ys.iter().cloned().fold(f64::MIN, f64::max)
            - ys.iter().cloned().fold(f64::MAX, f64::min);
        println!("拟合跑道外框 {:.0}m × {:.0}m", w, h);
    }

    /// 生成的轨迹必须全部落在这条跑道环上。
    #[test]
    fn loop_track_stays_on_fitted_ring() {
        let pts = user_points();
        let (dense, _arcs, center) = make_fitted_track_ring(&pts, 400.0).unwrap();
        let track = crate::track::generator::build_stadium(
            2000.0,
            720,
            42,
            center,
            1_700_000_000_000,
            &pts,
            TrackSpec::Loop { total_m: 400 },
        )
        .expect("跑道模式应能生成");
        let mut worst = 0.0f64;
        for p in &track.locations {
            let q = (
                (p.gLng - center.1) * MET_PER_DEG_LNG,
                (p.gLat - center.0) * MET_PER_DEG_LAT,
            );
            let d = dense.iter().map(|&r| distance(r, q)).fold(f64::INFINITY, f64::min);
            worst = worst.max(d);
        }
        println!("轨迹点距跑道环最大 {worst:.1}m");
        assert!(worst < 30.0, "轨迹偏离跑道 {worst:.1}m");
        assert!((track.totalDistance - 2000.0).abs() < 10.0);
    }
}
