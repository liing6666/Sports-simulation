//! 围栏几何判定：点包含 + **整段**折线包含。
//!
//! 为什么不能按间距采样：凹形校园的窄缺口、两个校区之间的连线、以及围栏
//! 边界本身都可能落进采样间隔里，采样会把这些越界判成合格。这里改为先求
//! 线段与多边形边界的全部交点，再逐个子区间判断中点是否被覆盖——长度任意
//! 小的缺口都能被发现。

use crate::{point_in_polygon, Coord};

const EPS: f64 = 1e-12;

fn cross(a: (f64, f64), b: (f64, f64)) -> f64 {
    a.0 * b.1 - a.1 * b.0
}

fn sub(a: Coord, b: Coord) -> (f64, f64) {
    (a.lon - b.lon, a.lat - b.lat)
}

fn lerp(a: Coord, b: Coord, t: f64) -> Coord {
    Coord::new(a.lon + (b.lon - a.lon) * t, a.lat + (b.lat - a.lat) * t)
}

/// 点是否在边界上（含端点与共线情形，容差按线段长度缩放）。
fn on_edge(a: Coord, b: Coord, p: Coord) -> bool {
    let d = sub(b, a);
    let q = sub(p, a);
    let len = d.0.hypot(d.1);
    if len < EPS {
        return q.0.hypot(q.1) < EPS;
    }
    cross(d, q).abs() <= EPS * len
        && p.lon >= a.lon.min(b.lon) - EPS
        && p.lon <= a.lon.max(b.lon) + EPS
        && p.lat >= a.lat.min(b.lat) - EPS
        && p.lat <= a.lat.max(b.lat) + EPS
}

/// 线段 p→q 与线段 a→b 的交点参数（p→q 上的 t，a→b 上的 u）。
fn segment_intersection(p: Coord, q: Coord, a: Coord, b: Coord) -> Option<(f64, f64)> {
    let r = sub(q, p);
    let s = sub(b, a);
    let denom = cross(r, s);
    if denom.abs() < EPS {
        return None; // 平行或共线：由 on_edge 覆盖
    }
    let qp = sub(a, p);
    let t = cross(qp, s) / denom;
    let u = cross(qp, r) / denom;
    if (-EPS..=1.0 + EPS).contains(&t) && (-EPS..=1.0 + EPS).contains(&u) {
        Some((t.clamp(0.0, 1.0), u.clamp(0.0, 1.0)))
    } else {
        None
    }
}

/// 点是否被任一多边形覆盖（含落在边界上）。
pub fn contains(polys: &[Vec<Coord>], p: Coord) -> bool {
    if !p.lon.is_finite() || !p.lat.is_finite() {
        return false;
    }
    polys.iter().any(|poly| {
        poly.len() >= 3
            && (point_in_polygon(poly, p)
                || poly
                    .iter()
                    .zip(poly.iter().cycle().skip(1))
                    .any(|(&a, &b)| on_edge(a, b, p)))
    })
}

/// 线段是否整体位于**同一个**多边形内。
///
/// 取线段上所有交点参数排序，相邻参数之间的开子区间用中点判定。用同一个
/// 多边形覆盖整个子区间，避免把两个不相交校区各占一半的越界连线判成合格。
pub fn segment_inside(poly: &[Coord], p: Coord, q: Coord) -> bool {
    if poly.len() < 3 || !contains(&[poly.to_vec()], p) || !contains(&[poly.to_vec()], q) {
        return false;
    }
    let mut cuts = vec![0.0f64, 1.0];
    for (&a, &b) in poly.iter().zip(poly.iter().cycle().skip(1)) {
        if let Some((t, _)) = segment_intersection(p, q, a, b) {
            cuts.push(t);
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    let poly_only = [poly.to_vec()];
    for pair in cuts.windows(2) {
        let (lo, hi) = (pair[0], pair[1]);
        if hi - lo < 1e-12 {
            continue;
        }
        let mid = lerp(p, q, (lo + hi) / 2.0);
        if !contains(&poly_only, mid) {
            return false;
        }
    }
    true
}

/// 线段是否整体位于**同一块**围栏内。
///
/// 注意：这要求整段落在其中一个多边形里。三角化后的校园边界不能用它——
/// 线段会跨越相邻三角形的公共边，应当用 [`segment_inside_union`]。
pub fn segment_inside_any(polys: &[Vec<Coord>], p: Coord, q: Coord) -> bool {
    if polys.is_empty() {
        return true;
    }
    polys.iter().any(|poly| segment_inside(poly, p, q))
}

/// 线段是否整体位于**多个多边形的并集**内。
///
/// 按所有多边形的边求交点参数，再逐个子区间取中点判定是否被并集覆盖。
/// 三角化后的校园边界（一个校区被切成许多三角形）必须用这个版本，
/// 否则线段跨过三角形公共边时会被误判为越界。
pub fn segment_inside_union(polys: &[Vec<Coord>], p: Coord, q: Coord) -> bool {
    if polys.is_empty() {
        return true;
    }
    if !contains(polys, p) || !contains(polys, q) {
        return false;
    }
    let mut cuts = vec![0.0f64, 1.0];
    for poly in polys {
        for (&a, &b) in poly.iter().zip(poly.iter().cycle().skip(1)) {
            if let Some((t, _)) = segment_intersection(p, q, a, b) {
                cuts.push(t);
            }
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    for pair in cuts.windows(2) {
        let (lo, hi) = (pair[0], pair[1]);
        if hi - lo < 1e-12 {
            continue;
        }
        if !contains(polys, lerp(p, q, (lo + hi) / 2.0)) {
            return false;
        }
    }
    true
}

/// 折线是否整体位于任一围栏内（逐段检查）。
pub fn polyline_inside_any(polys: &[Vec<Coord>], points: &[Coord]) -> bool {
    if polys.is_empty() {
        return true;
    }
    points
        .windows(2)
        .all(|pair| segment_inside_any(polys, pair[0], pair[1]))
        && points.first().map(|p| contains(polys, *p)).unwrap_or(true)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 凹形校园：上边中间有向上的凹口（缺口两侧 x 覆盖到 1.0）。
    fn notched_poly() -> Vec<Coord> {
        vec![
            Coord::new(0.0, 0.0),
            Coord::new(1.0, 0.0),
            Coord::new(1.0, 0.4),
            Coord::new(0.55, 0.4),
            Coord::new(0.55, 0.2),
            Coord::new(0.45, 0.2),
            Coord::new(0.45, 0.4),
            Coord::new(0.0, 0.4),
        ]
    }

    #[test]
    fn segment_crossing_a_narrow_notch_is_rejected() {
        let poly = notched_poly();
        // 两端都在围栏内，但连线从凹口下方穿过 —— 采样间隔大于缺口宽度时会漏判。
        let a = Coord::new(0.2, 0.3);
        let b = Coord::new(0.8, 0.3);
        assert!(!segment_inside(&poly, a, b));
        assert!(!segment_inside_any(&[poly], a, b));
    }

    #[test]
    fn contained_segment_is_accepted() {
        let poly = notched_poly();
        assert!(segment_inside(&poly, Coord::new(0.1, 0.1), Coord::new(0.9, 0.1)));
    }

    #[test]
    fn segment_into_the_notch_is_rejected() {
        let poly = notched_poly();
        // 终点在凹口里的点本身不被包含，直接拒绝。
        assert!(!segment_inside(&poly, Coord::new(0.5, 0.3), Coord::new(0.5, 0.35)));
    }

    #[test]
    fn two_separate_campuses_cannot_be_joined_by_one_segment() {
        let east = vec![
            Coord::new(0.0, 0.0),
            Coord::new(0.3, 0.0),
            Coord::new(0.3, 0.3),
            Coord::new(0.0, 0.3),
        ];
        let west = vec![
            Coord::new(1.0, 0.0),
            Coord::new(1.3, 0.0),
            Coord::new(1.3, 0.3),
            Coord::new(1.0, 0.3),
        ];
        // 两端各在一个校区内，连线却横穿中间空地。
        assert!(!segment_inside_any(
            &[east, west],
            Coord::new(0.25, 0.15),
            Coord::new(1.05, 0.15)
        ));
    }

    #[test]
    fn polyline_checks_every_segment() {
        let poly = notched_poly();
        let ok = [
            Coord::new(0.1, 0.1),
            Coord::new(0.9, 0.1),
            Coord::new(0.9, 0.35),
        ];
        assert!(polyline_inside_any(&[poly.clone()], &ok));
        let mut bad = ok.to_vec();
        bad.push(Coord::new(0.1, 0.35));
        assert!(!polyline_inside_any(&[poly], &bad));
    }

    #[test]
    fn no_fence_means_no_constraint() {
        assert!(segment_inside_any(&[], Coord::new(0.0, 0.0), Coord::new(9.0, 9.0)));
    }
}
