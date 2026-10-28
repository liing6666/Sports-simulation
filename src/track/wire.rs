//! OBS 对象组装。
//!
//! 10 键对象，每值 gzip+base64；BD→GCJ 单次转换；27 键协议点集；
//! 10s 窗（speed/step_freq）；每 1000m 一圈；五点 fixed_point_json。
#![allow(non_snake_case)]

use chrono::{Local, TimeZone};
use flate2::write::GzEncoder;
use flate2::Compression;
use serde_json::{json, Value};
use std::io::Write;

use super::geom::round_to;
use super::model::{GenPoint, Track};

const X_PI: f64 = std::f64::consts::PI * 3000.0 / 180.0;

/// The run-area metadata returned by the point endpoint.
///
/// Older callers can keep using the default value (free-run semantics), while
/// campus runs pass through the server-provided area id and fence JSON so the
/// record detail page can draw the same green boundary as the official app.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RunAreaMeta {
    pub run_area_id: i64,
    pub geo_fences_json: String,
    pub freedom_show_fence: bool,
}

impl Default for RunAreaMeta {
    fn default() -> Self {
        Self {
            run_area_id: -1,
            geo_fences_json: "[]".into(),
            freedom_show_fence: false,
        }
    }
}

/// 百度 BD-09 → 高德 GCJ-02。
pub fn bd09_to_gcj02(bd_lat: f64, bd_lng: f64) -> (f64, f64) {
    let x = bd_lng - 0.0065;
    let y = bd_lat - 0.006;
    let z = (x * x + y * y).sqrt() - 0.00002 * (y * X_PI).sin();
    let theta = y.atan2(x) - 0.000003 * (x * X_PI).cos();
    (z * theta.sin(), z * theta.cos())
}

/// gzip + base64。
pub fn gz(data: &[u8]) -> String {
    let mut enc = GzEncoder::new(Vec::new(), Compression::default());
    enc.write_all(data).expect("gzip 写入内存失败");
    let out = enc.finish().expect("gzip 收尾失败");
    super::super::crypto::envelope::b64_encode(&out)
}

fn gz_json(v: &Value) -> String {
    gz(v.to_string().as_bytes())
}

fn gz_str(v: &str) -> String {
    gz(v.as_bytes())
}

/// 27 键协议点集（gen 点 → OBS 点；gLat/gLng 由 BD 转 GCJ）。
pub fn conv_point(p: &GenPoint, start_ms: i64) -> Value {
    let (glat, glng) = bd09_to_gcj02(p.gLat, p.gLng);
    json!({
        "avgSpeed": round_to(p.avgSpeed, 4),
        "bdA": round_to(p.bdA, 2),
        "bdD": round_to(p.bdD, 2),
        "bdG": p.bdG,
        "bdS": round_to(p.bdS, 4),
        "coorType": "gcj02",
        "count": p.count,
        "dtr": 0.0,
        "flag": start_ms,
        "gLat": round_to(glat, 7),
        "gLng": round_to(glng, 7),
        "gainTime": p.gainTime,
        "id": p.id,
        "lat": -1.0,
        "lng": -1.0,
        "locType": p.locType,
        "locationId": "",
        "queueNum": p.queueNum,
        "radius": round_to(p.radius, 2),
        "speed": round_to(p.speed, 4),
        "state": p.state,
        // 累计步数：客户端按轨迹点还原十秒窗/圈步数时需要，缺失会让步频显示为 0。
        "steps": p.steps,
        "stepDistance": 0.0,
        "totalDis": round_to(p.totalDis, 4),
        "totalTime": p.totalTime,
        "type": p.ptype,
        "validDis": round_to(p.validDis, 4),
        "validTime": p.validTime,
    })
}

/// 五点实体（实时点位 → 跑完态 isPass=true）。
pub fn five_point_payload(points: &[Value], start_ms: i64) -> Vec<Value> {
    points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            json!({
                "flag": start_ms,
                "glat": p["glat"].as_f64().unwrap_or(0.0),
                "glon": p["glon"].as_f64().unwrap_or(0.0),
                "id": i as i64 + 1,
                // 跑完态的 fivePointJson 要把本次命中的打卡点标为固定点。
                // 点位接口有时只把起点标成 1（其余返回 0），但已验证的
                // 成功载荷要求所有已通过点均为 isFixed=1。
                "isFixed": 1,
                "isPass": true,
                "lat": p["lat"].as_f64().unwrap_or(0.0),
                "lon": p["lon"].as_f64().unwrap_or(0.0),
                "pointName": p["pointName"].as_str().unwrap_or(""),
                "position": 999,
                "state": 0,
            })
        })
        .collect()
}

/// 提交 body 的 fivePointJson 包装串。
#[allow(dead_code)]
pub fn five_point_wrapper(points: &[Value], start_ms: i64) -> String {
    five_point_wrapper_with_area(points, start_ms, &RunAreaMeta::default())
}

/// Build the fixed-point wrapper while preserving the server's campus area
/// and geofence metadata.
pub fn five_point_wrapper_with_area(points: &[Value], start_ms: i64, area: &RunAreaMeta) -> String {
    let five = five_point_payload(points, start_ms);
    json!({
        "useZip": false,
        "fivePointJson": Value::Array(five).to_string(),
        "runAreaId": area.run_area_id,
        "geoFencesJson": area.geo_fences_json,
        "freedomShowFence": area.freedom_show_fence,
    })
    .to_string()
}

/// 校验提交用五点轨迹仍是同一次点位请求产生的有效数据。
pub fn validate_five_point_wrapper(wrapper: &str) -> Result<(), String> {
    let outer: Value =
        serde_json::from_str(wrapper).map_err(|e| format!("五点轨迹 JSON 无效: {e}"))?;
    let raw = outer
        .get("fivePointJson")
        .and_then(Value::as_str)
        .ok_or("五点轨迹缺少 fivePointJson")?;
    let points: Vec<Value> =
        serde_json::from_str(raw).map_err(|e| format!("五点轨迹数组无效: {e}"))?;
    if points.is_empty() {
        return Err("五点轨迹不能为空".into());
    }
    for point in points {
        let lat = point.get("lat").and_then(Value::as_f64).unwrap_or(0.0);
        let lon = point.get("lon").and_then(Value::as_f64).unwrap_or(0.0);
        let glat = point.get("glat").and_then(Value::as_f64).unwrap_or(0.0);
        let glon = point.get("glon").and_then(Value::as_f64).unwrap_or(0.0);
        if (lat == 0.0 && lon == 0.0) || (glat == 0.0 && glon == 0.0) {
            return Err("五点轨迹包含缺失坐标".into());
        }
        crate::location::Coordinate::new(lat, lon, 0.0)?;
        crate::location::Coordinate::new(glat, glon, 0.0)?;
    }
    validate_fence_metadata(&outer)?;
    Ok(())
}

/// 围栏是学校能力差异字段：没有围栏时允许自由跑提交；如果服务端
/// 返回了围栏，则保持完整元数据并拒绝损坏或隐藏的围栏，避免详情页误显示。
fn validate_fence_metadata(outer: &Value) -> Result<(), String> {
    let Some(value) = outer.get("geoFencesJson") else {
        return Ok(());
    };
    let parsed = match value {
        Value::Null => return Ok(()),
        Value::String(text) if text.trim().is_empty() => return Ok(()),
        Value::String(text) => serde_json::from_str::<Value>(text)
            .map_err(|e| format!("五点轨迹围栏 JSON 无效: {e}"))?,
        other => other.clone(),
    };
    let has_fences = match parsed {
        Value::Null => false,
        Value::Array(items) => !items.is_empty(),
        _ => return Err("五点轨迹围栏必须是数组".into()),
    };
    if !has_fences {
        return Ok(());
    }
    let run_area_id = outer
        .get("runAreaId")
        .and_then(|v| v.as_i64().or_else(|| v.as_str()?.parse().ok()))
        .unwrap_or(-1);
    if run_area_id < -1 {
        return Err("五点轨迹 runAreaId 无效".into());
    }
    if outer.get("freedomShowFence").and_then(Value::as_bool) != Some(true) {
        return Err("五点轨迹围栏元数据未启用".into());
    }
    Ok(())
}

#[cfg(test)]
mod validation_tests {
    use super::*;
    #[test] fn rejects_empty_or_malformed_five_point_payload() { assert!(validate_five_point_wrapper("{}").is_err()); assert!(validate_five_point_wrapper(r#"{"fivePointJson":"[]"}"#).is_err()); }

    fn five_point_wrapper_without_fence_fields() -> Value {
        json!({
            "fivePointJson": r#"[{"lat":39.4,"lon":116.2,"glat":39.4,"glon":116.2}]"#
        })
    }

    #[test]
    fn five_point_validation_allows_missing_fence_metadata() {
        assert!(validate_five_point_wrapper(
            &five_point_wrapper_without_fence_fields().to_string()
        )
        .is_ok());
    }

    #[test]
    fn five_point_validation_allows_empty_fence_metadata() {
        let mut wrapper = five_point_wrapper_without_fence_fields();
        wrapper["runAreaId"] = json!(-1);
        wrapper["geoFencesJson"] = json!("[]");
        wrapper["freedomShowFence"] = json!(false);
        assert!(validate_five_point_wrapper(&wrapper.to_string()).is_ok());
    }

    #[test]
    fn five_point_validation_allows_null_fence_metadata() {
        let mut wrapper = five_point_wrapper_without_fence_fields();
        wrapper["runAreaId"] = Value::Null;
        wrapper["geoFencesJson"] = Value::Null;
        wrapper["freedomShowFence"] = Value::Null;
        assert!(validate_five_point_wrapper(&wrapper.to_string()).is_ok());
    }

    #[test]
    fn five_point_validation_accepts_valid_fence_metadata() {
        let mut wrapper = five_point_wrapper_without_fence_fields();
        wrapper["runAreaId"] = json!(-1);
        wrapper["geoFencesJson"] = json!(r#"[{"id":1}]"#);
        wrapper["freedomShowFence"] = json!(true);
        assert!(validate_five_point_wrapper(&wrapper.to_string()).is_ok());
    }

    #[test]
    fn five_point_validation_rejects_hidden_non_empty_fence() {
        let mut wrapper = five_point_wrapper_without_fence_fields();
        wrapper["runAreaId"] = json!(-1);
        wrapper["geoFencesJson"] = json!(r#"[{"id":1}]"#);
        wrapper["freedomShowFence"] = json!(false);
        assert!(validate_five_point_wrapper(&wrapper.to_string()).is_err());
    }

    #[test]
    fn five_point_validation_rejects_malformed_present_fence() {
        let mut wrapper = five_point_wrapper_without_fence_fields();
        wrapper["runAreaId"] = json!(-1);
        wrapper["geoFencesJson"] = json!("not-json");
        wrapper["freedomShowFence"] = json!(true);
        assert!(validate_five_point_wrapper(&wrapper.to_string()).is_err());
    }
    fn wrapper_without_fence() -> Value {
        json!({"fivePointJson": r#"[{"lat":39.4,"lon":116.2,"glat":39.4,"glon":116.2}]"#})
    }

    #[test]
    fn fence_metadata_is_optional() {
        assert!(validate_five_point_wrapper(&wrapper_without_fence().to_string()).is_ok());
        let mut empty = wrapper_without_fence();
        empty["runAreaId"] = json!(-1);
        empty["geoFencesJson"] = json!("[]");
        empty["freedomShowFence"] = json!(false);
        assert!(validate_five_point_wrapper(&empty.to_string()).is_ok());
    }

    #[test]
    fn valid_fence_metadata_is_kept_but_invalid_present_metadata_is_rejected() {
        let mut valid = wrapper_without_fence();
        valid["geoFencesJson"] = json!(r#"[{"id":1}]"#);
        valid["freedomShowFence"] = json!(true);
        assert!(validate_five_point_wrapper(&valid.to_string()).is_ok());

        let mut hidden = valid.clone();
        hidden["freedomShowFence"] = json!(false);
        assert!(validate_five_point_wrapper(&hidden.to_string()).is_err());

        let mut malformed = valid;
        malformed["geoFencesJson"] = json!("not-json");
        assert!(validate_five_point_wrapper(&malformed.to_string()).is_err());
    }

    #[test]
    fn area_metadata_is_written_to_fixed_point_wrapper() {
        let area = RunAreaMeta { run_area_id: 42, geo_fences_json: "[{\"x\":1}]".into(), freedom_show_fence: true };
        let wrapper = five_point_wrapper_with_area(&[json!({"lat": 39.9, "lon": 116.4, "glat": 39.9, "glon": 116.4})], 1_700_000_000_000, &area);
        let value: Value = serde_json::from_str(&wrapper).unwrap();
        assert_eq!(value["runAreaId"], 42);
        assert_eq!(value["geoFencesJson"], "[{\"x\":1}]");
        assert_eq!(value["freedomShowFence"], true);
    }

    #[test]
    fn completed_fixed_points_are_marked_fixed_even_if_source_point_is_zero() {
        let points = vec![json!({
            "lat": 39.9,
            "lon": 116.4,
            "glat": 39.9,
            "glon": 116.4,
            "isFixed": 0,
        })];
        let value: Value = serde_json::from_str(&five_point_wrapper(&points, 1_700_000_000_000)).unwrap();
        let raw = value["fivePointJson"].as_str().unwrap();
        let fixed: Value = serde_json::from_str(raw).unwrap();
        assert_eq!(fixed[0]["isFixed"], 1);
        assert_eq!(fixed[0]["isPass"], true);
    }

    #[test]
    fn laps_are_rebuilt_from_overridden_altitude() {
        let points = vec![(38.901678, 121.540241), (38.902564, 121.541233)];
        let mut track = crate::track::generator::build(
            1200.0,
            600,
            7,
            (38.9, 121.54),
            1_700_000_000_000,
            &points,
        );
        crate::track::altitude::override_bd_a(&mut track, 36.75).unwrap();
        let laps = build_laps(&track, track.startTime);
        assert!(!laps.is_empty());
        assert!(laps.iter().all(|lap| lap["elevationGain"] == 0.0));
        assert!(laps.iter().all(|lap| lap["endAltAbs"] == 36.75));
        assert!(laps.iter().all(|lap| lap["endAltRel"] == 0.0));
    }

    #[test]
    fn lap_metrics_use_seconds_per_kilometre_and_stride_centimetres() {
        let points = vec![(38.901678, 121.540241), (38.902564, 121.541233)];
        let mut track = crate::track::generator::build(
            1200.0,
            600,
            7,
            (38.9, 121.54),
            1_700_000_000_000,
            &points,
        );
        let mut start = track.locations[0].clone();
        start.totalTime = 0;
        start.totalDis = 0.0;
        start.steps = 0;
        let mut finish = start.clone();
        finish.totalTime = 600;
        finish.totalDis = 1000.0;
        finish.steps = 1200;
        track.locations = vec![start, finish];

        let lap = build_laps(&track, track.startTime).remove(0);
        // 秒/公里：10 分 00 秒/km 编码为 600；旧实现写的是分钟/公里（10.0）。
        assert_eq!(lap["avgPace"].as_f64(), Some(600.0));
        assert_eq!(lap["avgStride"].as_f64(), Some(83.33));
        assert_eq!(lap["distance"].as_f64(), Some(1000.0));
        assert_eq!(lap["isFullLap"].as_bool(), Some(true));
    }

    /// 圈边界必须精确落在 1000m，而不是落在越过分段后的那个采样点上。
    #[test]
    fn lap_boundaries_are_interpolated_at_exact_kilometres() {
        let mut track = crate::track::generator::build(
            1200.0, 600, 7, (38.9, 121.54), 1_700_000_000_000,
            &[(38.901678, 121.540241), (38.902564, 121.541233)],
        );
        let template = track.locations[0].clone();
        let point = |t: i64, d: f64, steps: i64| {
            let mut p = template.clone();
            p.totalTime = t;
            p.totalDis = d;
            p.steps = steps;
            p
        };
        // 采样点跨过 1000m（1080）与 2000m（2160），两者都必须被切开。
        track.locations = vec![
            point(0, 0.0, 0),
            point(600, 1080.0, 1200),
            point(1200, 2160.0, 2400),
            point(1380, 2500.0, 2800),
        ];
        let laps = build_laps(&track, track.startTime);
        assert_eq!(laps.len(), 3);
        assert!((laps[0]["distance"].as_f64().unwrap() - 1000.0).abs() < 1e-3);
        assert!((laps[1]["distance"].as_f64().unwrap() - 1000.0).abs() < 1e-3);
        assert!((laps[2]["distance"].as_f64().unwrap() - 500.0).abs() < 1e-3);
        assert_eq!(laps[2]["isFullLap"].as_bool(), Some(false));
        // 1000m 落在 0→1080m 分段的 92.59% 处，时间按比例插值。
        assert_eq!(laps[0]["duration"].as_i64(), Some(556));
        assert!((laps[0]["avgPace"].as_f64().unwrap() - 556.0).abs() < 0.01);
        // 累计时长与步数守恒。
        assert_eq!(laps[2]["cumulativeDuration"].as_i64(), Some(1380));
        assert_eq!(laps[0]["step"].as_i64().unwrap() + laps[1]["step"].as_i64().unwrap() + laps[2]["step"].as_i64().unwrap(), 2800);
    }

    /// 零步数不能伪造成巨大步幅，零时长不能伪造成 1 秒配速。
    #[test]
    fn zero_steps_and_zero_duration_are_null_not_fabricated() {
        let mut track = crate::track::generator::build(
            1200.0, 600, 7, (38.9, 121.54), 1_700_000_000_000,
            &[(38.901678, 121.540241), (38.902564, 121.541233)],
        );
        let mut start = track.locations[0].clone();
        start.totalTime = 0;
        start.totalDis = 0.0;
        start.steps = 0;
        let mut finish = start.clone();
        finish.totalTime = 0;
        finish.totalDis = 1000.0;
        finish.steps = 0;
        track.locations = vec![start, finish];

        let lap = build_laps(&track, track.startTime).remove(0);
        assert!(lap["avgStride"].is_null());
        assert!(lap["avgPace"].is_null());
        assert!(lap["avgCadence"].is_null());
        assert_eq!(lap["duration"].as_i64(), Some(0));
    }

    /// 解码真实组装的 gzip+base64 laps_json 断言单位，避免只测中间结构。
    #[test]
    fn encoded_laps_json_round_trips_with_seconds_per_kilometre() {
        let mut track = crate::track::generator::build(
            1200.0, 600, 7, (38.9, 121.54), 1_700_000_000_000,
            &[(38.901678, 121.540241), (38.902564, 121.541233)],
        );
        let template = track.locations[0].clone();
        let point = |t: i64, d: f64, steps: i64| {
            let mut p = template.clone();
            p.totalTime = t;
            p.totalDis = d;
            p.steps = steps;
            p
        };
        track.locations = vec![
            point(0, 0.0, 0),
            point(410, 1000.0, 900),
            point(799, 2000.0, 1800),
            point(924, 2400.0, 2100),
        ];
        track.totalTime = 924;
        track.totalDistance = 2400.0;
        track.startTime = 1_700_000_000_000;

        let obs = build_obs_object(&track, 4242, "UUID", 7, &[]);
        let raw = de_gz(obs["laps_json"].as_str().unwrap());
        let laps: Value = serde_json::from_str(raw.trim_end_matches('\0')).unwrap();
        let laps = laps.as_array().unwrap();
        assert_eq!(laps.len(), 3);
        assert!((laps[0]["avgPace"].as_f64().unwrap() - 410.0).abs() < 0.01);
        assert!((laps[1]["avgPace"].as_f64().unwrap() - 389.0).abs() < 0.01);
        assert!(laps[2]["avgPace"].as_f64().unwrap() > 300.0);
        assert!((laps[0]["distance"].as_f64().unwrap() - 1000.0).abs() < 1e-3);
        assert!(laps[0]["avgStride"].as_f64().unwrap() > 100.0);
        // 圈用时之和必须等于总时长，不能因插值漂移。
        let sum: i64 = laps.iter().map(|l| l["duration"].as_i64().unwrap()).sum();
        assert_eq!(sum, 924);
    }

    fn de_gz(encoded: &str) -> String {
        use flate2::read::GzDecoder;
        use std::io::Read;
        let bytes = crate::crypto::envelope::b64_decode(encoded).unwrap();
        let mut out = String::new();
        GzDecoder::new(&bytes[..]).read_to_string(&mut out).unwrap();
        out
    }
}

/// 10s 时间窗，id=(rrid%100000)*1000+窗口序秒（6 个真人样本跨 9 月记录验证一致；
/// 旧版 App 样本为全局序号，不适用当前版本）。
fn build_windows(track: &Track, rrid: i64) -> (Vec<Value>, Vec<Value>) {
    let start_ms = track.startTime;
    let total_time = track.totalTime;
    let mut sp = Vec::new();
    let mut stf = Vec::new();
    for (i, a) in track.speedPerTenSec.iter().enumerate() {
        let b = &track.stepsPerTenSec[i];
        let lo = (i * 10) as i64;
        let hi = (i * 10 + 10) as i64;
        let hi = hi.min(total_time);
        let id = (rrid % 100000) * 1000 + hi;
        sp.push(json!({
            "beginTime": start_ms + lo * 1000,
            "distance": a.value,
            "endTime": start_ms + hi * 1000,
            "flag": start_ms,
            "id": id,
            "queueNum": 0,
            "state": 0,
        }));
        stf.push(json!({
            "avgDiff": 0.0,
            "beginTime": start_ms + lo * 1000,
            "endTime": start_ms + hi * 1000,
            "flag": start_ms,
            "id": id,
            "maxDiff": 0.0,
            "minDiff": 1000.0,
            "queueNum": 0,
            "state": 0,
            "stepsNum": b.value as i64,
        }));
    }
    (sp, stf)
}

/// 累计量在给定累计距离处的线性插值（圈边界落在相邻采样点之间时使用）。
///
/// 边界按距离而不是按采样点查找：旧实现用 `d_now - prev_d >= 1000` 在采样点上
/// 切圈，每圈实际都略短于 1000m，且单步跨过 2km 时只切出 1 圈。
fn counters_at_distance(locs: &[GenPoint], distance: f64) -> (i64, i64, f64) {
    let Some(first) = locs.first() else {
        return (0, 0, 0.0);
    };
    if distance <= first.totalDis {
        return (first.totalTime, first.steps, first.bdA);
    }
    for pair in locs.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        if b.totalDis >= distance && b.totalDis > a.totalDis {
            let k = (distance - a.totalDis) / (b.totalDis - a.totalDis);
            let time = a.totalTime + ((b.totalTime - a.totalTime) as f64 * k).round() as i64;
            let steps = (a.steps as f64 + (b.steps - a.steps) as f64 * k).round() as i64;
            return (time, steps, a.bdA + (b.bdA - a.bdA) * k);
        }
    }
    let last = locs.last().unwrap();
    (last.totalTime, last.steps, last.bdA)
}

/// 圈（每 1000m 一圈，末圈 isFullLap=false）。
///
/// `avgPace` 单位为**秒/公里**（客户端据此格式化为分:秒）；记录级 `speed` 才是
/// 千分之一分钟/公里。`avgStride` 单位厘米，零步时为 null 而不是伪造巨大步幅。
fn build_laps(track: &Track, start_ms: i64) -> Vec<Value> {
    let mut laps = Vec::new();
    let locs = &track.locations;
    let Some(last) = locs.last() else {
        return laps;
    };
    let alt0 = locs.first().map(|p| p.bdA).unwrap_or(0.0);
    let mut prev_d = 0.0f64;
    let mut prev_t = 0i64;
    let mut prev_steps = 0i64;
    let mut index = 0i64;

    while prev_d < last.totalDis {
        let boundary = (prev_d + 1000.0).min(last.totalDis);
        let (t_now, steps, altitude) = counters_at_distance(locs, boundary);
        // 圈内爬升：只累计落在 [prev_d, boundary] 内的那部分正增量，
        // 跨边界的分段按重叠比例计入，和值与圈长一样精确。
        let mut gain = 0.0f64;
        for pair in locs.windows(2) {
            let (a, b) = (&pair[0], &pair[1]);
            let span = b.totalDis - a.totalDis;
            if span <= 0.0 || b.totalDis <= prev_d || a.totalDis >= boundary {
                continue;
            }
            let overlap = b.totalDis.min(boundary) - a.totalDis.max(prev_d);
            let dd = b.bdA - a.bdA;
            if dd > 0.0 {
                gain += super::altitude::positive_ascent_delta(dd) * (overlap / span);
            }
        }
        let lap_d = boundary - prev_d;
        let lap_t = (t_now - prev_t).max(0);
        let lap_steps = (steps - prev_steps).max(0);
        index += 1;
        laps.push(json!({
            "avgCadence": if lap_t > 0 { json!(round_to(lap_steps as f64 / (lap_t as f64 / 60.0), 2)) } else { Value::Null },
            "avgPace": if lap_t > 0 && lap_d > 0.0 { json!(round_to(lap_t as f64 / (lap_d / 1000.0), 2)) } else { Value::Null },
            "avgStride": if lap_steps > 0 { json!(round_to(lap_d / lap_steps as f64 * 100.0, 2)) } else { Value::Null },
            "cumulativeDuration": t_now,
            "distance": round_to(lap_d, 4),
            "duration": lap_t,
            "elevationGain": round_to(gain, 2),
            "endAltAbs": round_to(altitude, 2),
            "endAltRel": round_to(altitude - alt0, 2),
            "flag": start_ms,
            "id": index,
            "isFullLap": lap_d >= 1000.0,
            "lapIndex": index,
            "step": lap_steps,
        }));
        prev_d = boundary;
        prev_t = t_now;
        prev_steps = steps;
    }
    laps
}

/// 组装 10 键 OBS 对象（值均 gzip+base64；segment_json/runFaceCheck 为空串 gzip）。
pub fn build_obs_object(
    track: &Track,
    rrid: i64,
    uuid: &str,
    uid: i64,
    live_points: &[Value],
) -> Value {
    build_obs_object_with_area(track, rrid, uuid, uid, live_points, &RunAreaMeta::default())
}

/// Build the OBS payload with the same area metadata used by submission.
pub fn build_obs_object_with_area(
    track: &Track,
    rrid: i64,
    uuid: &str,
    uid: i64,
    live_points: &[Value],
    area: &RunAreaMeta,
) -> Value {
    let start_ms = track.startTime;
    let pts: Vec<Value> = track
        .locations
        .iter()
        .map(|p| conv_point(p, start_ms))
        .collect();
    let run_wrap = json!({ "allLocJson": Value::Array(pts).to_string(), "useZip": false });
    let (sp, stf) = build_windows(track, rrid);
    let laps = build_laps(track, start_ms);
    let five = five_point_payload(live_points, start_ms);
    let fx = json!({
        "fivePointJson": Value::Array(five).to_string(),
        "freedomShowFence": area.freedom_show_fence,
        "geoFencesJson": area.geo_fences_json,
        "runAreaId": area.run_area_id,
        "useZip": false,
    });
    json!({
        "rrid": gz_str(&rrid.to_string()),
        "uuid": gz_str(uuid),
        "uid": gz_str(&uid.to_string()),
        "run_data": gz_json(&run_wrap),
        "fixed_point_json": gz_json(&fx),
        "segment_json": gz_str(""),
        "speed_json": gz_json(&Value::Array(sp)),
        "step_freq_json": gz_json(&Value::Array(stf)),
        "laps_json": gz_json(&Value::Array(laps)),
        "runFaceCheck": gz_str(""),
    })
}

/// OBS objectKey（两个都要传）。
pub fn obs_keys(track: &Track, rrid: i64, uuid: &str) -> Vec<String> {
    let t0 = Local
        .timestamp_millis_opt(track.startTime)
        .single()
        .map(|t| t.format("%Y%m%d%H").to_string())
        .unwrap_or_default();
    vec![
        format!("run_data/{t0}/{uuid}.json"),
        format!("run_data/{}/{}.json", rrid / 1000000, rrid),
    ]
}

