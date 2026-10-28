//! 电子围栏：POST /api/v1/getGeoFenceForRun。
//! 返回 data.geoFences[]，每个围栏 points[] 为坐标点（与打卡点同系，BD-09）。
//!
//! 服务端点结构可能只填充 GCJ 字段（glat/glon）而把 BD 字段（lat/lon）置 0，
//! 故优先取 lat/lon，若为 0/缺失则回退 glat/glon 并做 GCJ→BD 转换，避免缓存全 0。

use super::client::{get_field, ApiClient};
use crate::track::geom::gcj02_to_bd09;
use serde_json::{json, Value};

pub const FENCE_PATH: &str = "/api/v1/getGeoFenceForRun";

/// 单个围栏：几何 + 服务端身份。
#[derive(Debug, Clone)]
pub struct Fence {
    /// 围栏在服务端的 id（fixed_point_json 的 runAreaId 通常取这个值）。
    pub id: Option<i64>,
    pub name: String,
    /// 闭合环，(lat, lon)，BD-09。
    pub ring: Vec<(f64, f64)>,
}

/// 按锚点选中一个校区后的结果。
#[derive(Debug, Clone)]
pub struct SelectedFence {
    /// 裁剪路网时使用的多边形（多个围栏重叠时取包含锚点的全部，保证不自断）。
    pub polygons: Vec<Vec<(f64, f64)>>,
    /// 用于 fixed_point_json / OBS 的 runAreaId。
    pub run_area_id: i64,
    /// 使用的围栏 id（fallback 时用于日志）。
    pub fence_id: Option<i64>,
    pub name: String,
    /// 命中的围栏下标（在原始 fences 列表里）。
    pub index: usize,
}

pub struct GeoFenceResponse {
    pub polygons: Vec<Vec<(f64, f64)>>,
    pub area: crate::track::wire::RunAreaMeta,
    /// 带身份的围栏列表；`polygons` 是它的几何投影，两者顺序一致。
    pub fences: Vec<Fence>,
}

/// 拉取围栏多边形列表（每个围栏一个闭合环，(lat, lon)，BD-09）。
pub fn fetch_geo_fence(client: &mut ApiClient) -> Result<Vec<Vec<(f64, f64)>>, String> {
    Ok(fetch_geo_fence_with_area(client)?.polygons)
}

/// 拉取围栏并保留服务端原始元数据，供 fixed_point_json/OBS 回显绿色边界。
pub fn fetch_geo_fence_with_area(client: &mut ApiClient) -> Result<GeoFenceResponse, String> {
    let body = json!({ "geoFenceUpdateTime": 0 }).to_string();
    let biz = client.call("POST", FENCE_PATH, &body, &[])?;
    let fences = get_field(&biz, "geoFences")
        .cloned()
        .or_else(|| {
            let data = super::client::parse_data_field(&biz);
            data.get("geoFences").cloned()
        })
        .and_then(|value| match value {
            Value::Array(items) => Some(items),
            Value::String(text) => serde_json::from_str(&text).ok(),
            _ => None,
        })
        .unwrap_or_default();

    let mut out: Vec<Vec<(f64, f64)>> = Vec::new();
    let mut parsed: Vec<Fence> = Vec::new();
    for f in &fences {
        let Some(pts) = f.get("points").and_then(|v| v.as_array()) else {
            continue;
        };
        let ring: Vec<(f64, f64)> = pts
            .iter()
            .filter_map(point_xy)
            .filter(|(lat, lon)| !(*lat == 0.0 && *lon == 0.0))
            .collect();
        if ring.len() >= 3 {
            out.push(ring.clone());
            parsed.push(Fence {
                id: f
                    .get("id")
                    .and_then(value_as_i64)
                    .or_else(|| f.get("runAreaId").and_then(value_as_i64))
                    .or_else(|| f.get("areaId").and_then(value_as_i64)),
                name: f
                    .get("name")
                    .and_then(|v| v.as_str())
                    .unwrap_or("")
                    .to_string(),
                ring,
            });
        }
    }
    let mut area = crate::api::points::area_from_payload(&biz, &[]);
    // The standalone endpoint may return lon/glon/pointsNumber, while the
    // detail payload used by the official client uses lng/glng and the compact
    // fence-point shape.  Keep the server-provided geometry, but emit that
    // canonical shape so the detail map can recognize the green boundary.
    if !out.is_empty() {
        area.geo_fences_json = canonical_fences_json(&fences);
        area.freedom_show_fence = area.geo_fences_json != "[]";
    }
    Ok(GeoFenceResponse {
        polygons: out,
        area,
        fences: parsed,
    })
}

fn value_as_i64(value: &Value) -> Option<i64> {
    value
        .as_i64()
        .or_else(|| value.as_u64().and_then(|n| i64::try_from(n).ok()))
        .or_else(|| value.as_str().and_then(|s| s.trim().parse().ok()))
}

/// 按设备定位锚点选中校区。
///
/// 学校可能返回多个围栏（多校区/多跑步区域）。旧实现把所有围栏**取并集**裁剪
/// 路网，却拿第一个围栏的 id 当 `runAreaId`：路网横跨两个校区，提交的区域 id
/// 又未必是锚点所在的那个，规划结果自然不对。
///
/// 规则：
/// 1. 锚点落在唯一/多个围栏内 → 用这些围栏的几何，`runAreaId` 取命中围栏中
///    第一个有 id 的那个。
/// 2. 锚点不在任何围栏内 → 取离锚点最近的围栏，并在 `fallback` 标记为真，
///    调用方应当提示用户核对定位锚点。
pub struct FencePick {
    pub selected: SelectedFence,
    /// 锚点不在任何围栏内，按最近围栏兜底。
    pub fallback: bool,
    /// 锚点到所选围栏的距离（米），仅 fallback 时有意义。
    pub distance_m: f64,
}

pub fn select_by_anchor(
    fences: &[Fence],
    anchor: (f64, f64),
    service_area_id: i64,
) -> Result<FencePick, String> {
    if fences.is_empty() {
        return Err("没有可用围栏".into());
    }
    let containing: Vec<usize> = fences
        .iter()
        .enumerate()
        .filter(|(_, f)| ring_contains(&f.ring, anchor))
        .map(|(i, _)| i)
        .collect();

    if let Some(&first) = containing.first() {
        // 重叠围栏（例如校内分区）合并使用，避免把路网裁成互不相连的碎片。
        let polygons = containing.iter().map(|&i| fences[i].ring.clone()).collect();
        let run_area_id = containing
            .iter()
            .filter_map(|&i| fences[i].id)
            .next()
            .unwrap_or(service_area_id);
        return Ok(FencePick {
            selected: SelectedFence {
                polygons,
                run_area_id,
                fence_id: fences[first].id,
                name: fences[first].name.clone(),
                index: first,
            },
            fallback: false,
            distance_m: 0.0,
        });
    }

    // 锚点不在任何围栏内：取最近围栏兜底，并把距离交回调用方提示。
    let (index, distance_m) = fences
        .iter()
        .enumerate()
        .map(|(i, f)| (i, ring_min_distance(&f.ring, anchor)))
        .min_by(|a, b| a.1.total_cmp(&b.1))
        .ok_or("没有可用围栏")?;
    let fence = &fences[index];
    Ok(FencePick {
        selected: SelectedFence {
            polygons: vec![fence.ring.clone()],
            run_area_id: fence.id.unwrap_or(service_area_id),
            fence_id: fence.id,
            name: fence.name.clone(),
            index,
        },
        fallback: true,
        distance_m,
    })
}

/// 从裸多边形列表（缓存里的旧格式）中按锚点挑出所在校区。
///
/// 缓存没有围栏 id，因此只能按几何选；选不到（锚点不在任何围栏内）时返回 None，
/// 由调用方决定是兜底还是放弃。
pub fn pick_from_polygons(
    polygons: &[Vec<(f64, f64)>],
    anchor: (f64, f64),
) -> Option<SelectedFence> {
    let fences: Vec<Fence> = polygons
        .iter()
        .map(|ring| Fence {
            id: None,
            name: String::new(),
            ring: ring.clone(),
        })
        .collect();
    let pick = select_by_anchor(&fences, anchor, -1).ok()?;
    (!pick.fallback).then_some(pick.selected)
}

/// 把单个围栏环转成 fixed_point_json 需要的 `geoFencesJson` 形状。
pub fn canonical_json_for(ring: &[(f64, f64)]) -> String {
    let points: Vec<Value> = ring
        .iter()
        .map(|&(lat, lon)| {
            let (glat, glon) = crate::track::wire::bd09_to_gcj02(lat, lon);
            json!({ "lat": lat, "lng": lon, "glat": glat, "glng": glon })
        })
        .collect();
    json!([{ "points": points }]).to_string()
}

fn ring_contains(ring: &[(f64, f64)], p: (f64, f64)) -> bool {
    let poly: Vec<route_planner::Coord> =
        ring.iter().map(|&(lat, lon)| route_planner::Coord::new(lon, lat)).collect();
    route_planner::boundary::contains(&[poly], route_planner::Coord::new(p.1, p.0))
}

/// 点到环的最小距离（米，近似平面）。
fn ring_min_distance(ring: &[(f64, f64)], p: (f64, f64)) -> f64 {
    const LAT_M: f64 = 111_132.0;
    let lng_m = LAT_M * (p.0.to_radians().cos());
    let pt = [p.1 * lng_m, p.0 * LAT_M];
    let mut best = f64::INFINITY;
    for pair in ring.iter().zip(ring.iter().cycle().skip(1)) {
        let a = [pair.0 .1 * lng_m, pair.0 .0 * LAT_M];
        let b = [pair.1 .1 * lng_m, pair.1 .0 * LAT_M];
        let (foot, _) = route_planner::graph::project_to_segment(pt, a, b);
        best = best.min((pt[0] - foot[0]).hypot(pt[1] - foot[1]));
    }
    best
}

fn canonical_fences_json(fences: &[Value]) -> String {
    let canonical: Vec<Value> = fences
        .iter()
        .filter_map(|fence| {
            let points = fence.get("points")?.as_array()?;
            let points: Vec<Value> = points
                .iter()
                .filter_map(|point| {
                    let mut output = serde_json::Map::new();
                    for (canonical, aliases) in [
                        ("lat", &["lat", "latitude"][..]),
                        ("lng", &["lng", "lon", "longitude"][..]),
                        ("glat", &["glat"][..]),
                        ("glng", &["glng", "glon"][..]),
                    ] {
                        if let Some(value) = aliases.iter().find_map(|name| point.get(*name)) {
                            output.insert(canonical.into(), value.clone());
                        }
                    }
                    (output.contains_key("lat") && output.contains_key("lng"))
                        .then_some(Value::Object(output))
                })
                .collect();
            if points.len() < 3 {
                return None;
            }
            let mut output = serde_json::Map::new();
            for name in ["id", "name"] {
                if let Some(value) = fence.get(name) {
                    output.insert(name.into(), value.clone());
                }
            }
            output.insert("points".into(), Value::Array(points));
            Some(Value::Object(output))
        })
        .collect();
    if canonical.is_empty() {
        "[]".into()
    } else {
        Value::Array(canonical).to_string()
    }
}

/// 取一个点的 (lat, lon)（BD-09）。优先 lat/lon，缺失或全 0 时回退 glat/glon（GCJ→BD）。
/// 供围栏与 policy 必经点共用（两端点均实测存在「只填 GCJ、BD 置 0」的情况）。
pub(crate) fn point_xy(p: &Value) -> Option<(f64, f64)> {
    let lat = field_num(p, &["lat", "latitude"]);
    let lon = field_num(p, &["lon", "lng", "longitude"]);
    match (lat, lon) {
        (Some(a), Some(o)) if !(a == 0.0 && o == 0.0) => Some((a, o)),
        _ => {
            let (ga, go) = (field_num(p, &["glat"]), field_num(p, &["glon"]));
            match (ga, go) {
                (Some(a), Some(o)) if !(a == 0.0 && o == 0.0) => Some(gcj02_to_bd09(a, o)),
                _ => None,
            }
        }
    }
}

fn field_num(p: &Value, keys: &[&str]) -> Option<f64> {
    keys.iter().find_map(|k| p.get(*k).and_then(|v| v.as_f64()))
}

#[cfg(test)]
mod tests {
    use super::{canonical_fences_json, point_xy};
    use serde_json::{json, Value};

    /// 两个相隔约 5km 的校区（模拟"学校有两套线路"）。
    fn two_campuses() -> Vec<super::Fence> {
        let square = |lat0: f64, lon0: f64| {
            vec![
                (lat0, lon0),
                (lat0, lon0 + 0.01),
                (lat0 + 0.01, lon0 + 0.01),
                (lat0 + 0.01, lon0),
            ]
        };
        vec![
            super::Fence {
                id: Some(1774),
                name: "东校区".into(),
                ring: square(39.90, 116.40),
            },
            super::Fence {
                id: Some(1888),
                name: "西校区".into(),
                ring: square(39.90, 116.45),
            },
        ]
    }

    #[test]
    fn anchor_selects_only_its_own_campus() {
        let fences = two_campuses();
        let east = super::select_by_anchor(&fences, (39.905, 116.405), -1).unwrap();
        assert!(!east.fallback);
        assert_eq!(east.selected.run_area_id, 1774);
        assert_eq!(east.selected.polygons.len(), 1, "不能把另一个校区的围栏并进来");
        assert_eq!(east.selected.index, 0);

        let west = super::select_by_anchor(&fences, (39.905, 116.455), -1).unwrap();
        assert_eq!(west.selected.run_area_id, 1888, "runAreaId 必须跟随锚点");
        assert_eq!(west.selected.index, 1);
    }

    #[test]
    fn anchor_outside_every_campus_falls_back_to_nearest() {
        let fences = two_campuses();
        let pick = super::select_by_anchor(&fences, (39.905, 116.445), -1).unwrap();
        assert!(pick.fallback, "锚点在围栏外必须标记 fallback");
        assert!(pick.distance_m > 0.0);
        // 116.445 距西校区(116.45)更近。
        assert_eq!(pick.selected.run_area_id, 1888);
    }

    #[test]
    fn overlapping_fences_are_merged_to_avoid_fragmenting_the_network() {
        let mut fences = two_campuses();
        // 东校区内部再划一个子区域，两环都包含锚点。
        fences.push(super::Fence {
            id: Some(2000),
            name: "东校区操场".into(),
            ring: vec![
                (39.901, 116.401),
                (39.901, 116.405),
                (39.905, 116.405),
                (39.905, 116.401),
            ],
        });
        let pick = super::select_by_anchor(&fences, (39.903, 116.403), -1).unwrap();
        assert_eq!(pick.selected.polygons.len(), 2, "重叠围栏应合并使用");
        assert_eq!(pick.selected.run_area_id, 1774, "取第一个有 id 的围栏");
    }

    #[test]
    fn empty_fence_list_is_an_error() {
        assert!(super::select_by_anchor(&[], (39.9, 116.4), -1).is_err());
    }

    #[test]
    fn cached_polygons_without_ids_still_select_by_geometry() {
        let fences = two_campuses();
        let polygons: Vec<Vec<(f64, f64)>> = fences.iter().map(|f| f.ring.clone()).collect();
        let sel = super::pick_from_polygons(&polygons, (39.905, 116.405)).unwrap();
        assert_eq!(sel.polygons.len(), 1);
        assert_eq!(sel.index, 0);
        // 锚点不在任何围栏内时不返回（由调用方决定兜底策略）。
        assert!(super::pick_from_polygons(&polygons, (0.0, 0.0)).is_none());
    }

    #[test]
    fn point_xy_prefers_bd_then_gcj_fallback() {
        // BD lat/lon 正常
        let p = json!({ "lat": 38.9, "lon": 121.5 });
        assert_eq!(point_xy(&p), Some((38.9, 121.5)));
        // 经纬度字段用 lng
        let p = json!({ "lat": 38.9, "lng": 121.5 });
        assert_eq!(point_xy(&p), Some((38.9, 121.5)));
        // BD 全 0，回退 glat/glon（GCJ）并转 BD
        let p =
            json!({ "lat": 0.0, "lon": 0.0, "glat": 38.8956025774013, "glon": 121.5337497718317 });
        let (lat, lon) = point_xy(&p).unwrap();
        assert!((lat - 38.901678).abs() < 1e-6, "lat={lat}");
        assert!((lon - 121.540241).abs() < 1e-6, "lon={lon}");
        // 全缺
        let p = json!({ "foo": 1 });
        assert_eq!(point_xy(&p), None);
    }

    #[test]
    fn canonical_fence_uses_detail_map_coordinate_names() {
        let value = canonical_fences_json(&[json!({
            "id": 1774,
            "name": "学校",
            "points": [
                {"lat": 39.4, "lon": 116.2, "glat": 39.3, "glon": 116.1},
                {"lat": 39.4, "lon": 116.3, "glat": 39.3, "glon": 116.2},
                {"lat": 39.5, "lon": 116.3, "glat": 39.4, "glon": 116.2}
            ]
        })]);
        let parsed: Value = serde_json::from_str(&value).unwrap();
        assert_eq!(parsed[0]["id"], 1774);
        assert_eq!(parsed[0]["points"][0]["lng"], 116.2);
        assert_eq!(parsed[0]["points"][0]["glng"], 116.1);
        assert!(parsed[0]["points"][0].get("lon").is_none());
    }
}
