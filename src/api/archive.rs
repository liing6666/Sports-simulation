//! 跑步数据本地存档：轨迹文件、点位/围栏快照、失败现场包。
//!
//! 目录结构（与 config.json 同级，升级时保留）：
//! ```text
//! runs/
//!   2026-10-26_073015_<uuid8>/
//!     track.gpx            原始轨迹（WGS84/BD 双点，可导入其他工具查看）
//!     track.json           协议点集（与 OBS run_data 同源，便于逐点比对）
//!     meta.json            参数、统计、校区/围栏选择结果
//!     points.json          实时点位快照
//!     fences.json          围栏快照（含选中的 runAreaId）
//!     result.json          提交结果（rrid/obs/verify），失败时为错误原因
//!     diagnostic.json      仅失败/结果未知时生成，白名单脱敏
//! ```
//!
//! 原则：存档失败**不能**阻断跑步流程，所有写入失败只记日志。

use chrono::TimeZone;
use serde_json::{json, Value};
use std::path::{Path, PathBuf};

/// 存档根目录名。
pub const RUNS_DIR: &str = "runs";

/// 一次跑步存档的写入句柄。
pub struct RunArchive {
    dir: PathBuf,
}

impl RunArchive {
    /// 创建本次存档目录。`key` 通常是 UUID，取前 8 位做目录名后缀。
    pub fn create(key: &str, start_ms: i64) -> Result<Self, String> {
        let root = crate::platform::data_dir().join(RUNS_DIR);
        std::fs::create_dir_all(&root).map_err(|e| format!("无法建立 runs 目录：{e}"))?;
        let stamp = chrono::Local
            .timestamp_millis_opt(start_ms)
            .single()
            .map(|t| t.format("%Y-%m-%d_%H%M%S").to_string())
            .unwrap_or_else(|| "unknown".into());
        let short: String = key.chars().take(8).collect();
        let dir = root.join(format!("{stamp}_{short}"));
        std::fs::create_dir_all(&dir).map_err(|e| format!("无法建立存档目录：{e}"))?;
        Ok(Self { dir })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    /// 写一个 JSON 文件；失败只返回错误，由调用方决定是否记日志。
    pub fn write_json(&self, name: &str, value: &Value) -> Result<(), String> {
        let text = serde_json::to_string_pretty(value)
            .map_err(|e| format!("{name} 序列化失败：{e}"))?;
        std::fs::write(self.dir.join(name), text).map_err(|e| format!("{name} 写入失败：{e}"))
    }

    fn write_text(&self, name: &str, text: &str) -> Result<(), String> {
        std::fs::write(self.dir.join(name), text).map_err(|e| format!("{name} 写入失败：{e}"))
    }
}

/// 一个轨迹点（存档用，字段与协议点集一致）。
pub struct TrackRow<'a> {
    pub lat: f64,
    pub lon: f64,
    pub total_time: i64,
    pub total_dis: f64,
    pub steps: i64,
    pub bd_a: f64,
    pub bd: (f64, f64),
    pub _marker: std::marker::PhantomData<&'a ()>,
}

/// 生成 GPX 1.1 文本。
///
/// `trkpt` 使用 GCJ02 坐标（`gLat/gLng`），与提交给服务端的点一致；
/// 同时写入 `<extensions>` 里的 BD 坐标，避免坐标系歧义。
pub fn build_gpx(name: &str, start_ms: i64, rows: &[TrackRow<'_>]) -> String {
    let when = chrono::Local
        .timestamp_millis_opt(start_ms)
        .single()
        .map(|t| t.format("%Y-%m-%dT%H:%M:%S%:z").to_string())
        .unwrap_or_default();
    let mut out = String::with_capacity(rows.len() * 160 + 512);
    out.push_str(r#"<?xml version="1.0" encoding="UTF-8"?>"#);
    out.push('\n');
    out.push_str(
        r#"<gpx version="1.1" creator="NekoSportsWorldTool" xmlns="http://www.topografix.com/GPX/1/1">"#,
    );
    out.push('\n');
    out.push_str(&format!(
        "  <metadata><name>{}</name><time>{}</time></metadata>\n",
        xml_escape(name),
        when
    ));
    out.push_str("  <trk><name>");
    out.push_str(&xml_escape(name));
    out.push_str("</name><trkseg>\n");
    for row in rows {
        out.push_str(&format!(
            "    <trkpt lat=\"{:.7}\" lon=\"{:.7}\"><ele>{:.2}</ele><time>{}</time>\
<extensions><nsw:bd lat=\"{:.7}\" lon=\"{:.7}\" steps=\"{}\" totalDis=\"{:.2}\"/></extensions></trkpt>\n",
            row.lat,
            row.lon,
            row.bd_a,
            when_at(start_ms, row.total_time),
            row.bd.0,
            row.bd.1,
            row.steps,
            row.total_dis,
            // 命名空间在 extensions 里声明一次即可，这里用简化写法
        ));
    }
    out.push_str("  </trkseg></trk>\n</gpx>\n");
    // 补上自定义命名空间声明，否则 ns 前缀未定义，导入工具会报错。
    out.replace(
        "xmlns=\"http://www.topografix.com/GPX/1/1\">",
        "xmlns=\"http://www.topografix.com/GPX/1/1\" xmlns:nsw=\"https://github.com/liming2500/NekoSportsWorldTool\">",
    )
}

fn when_at(start_ms: i64, offset_s: i64) -> String {
    chrono::Local
        .timestamp_millis_opt(start_ms + offset_s * 1000)
        .single()
        .map(|t| t.format("%Y-%m-%dT%H:%M:%SZ").to_string())
        .unwrap_or_default()
}

fn xml_escape(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// 轨迹存档：GPX + 协议点集。
pub fn save_track(
    archive: &RunArchive,
    uuid: &str,
    start_ms: i64,
    points: &[Value],
) -> Result<(), String> {
    let rows: Vec<TrackRow<'_>> = points
        .iter()
        .map(|p| TrackRow {
            lat: p["gLat"].as_f64().unwrap_or(0.0),
            lon: p["gLng"].as_f64().unwrap_or(0.0),
            total_time: p["totalTime"].as_i64().unwrap_or(0),
            total_dis: p["totalDis"].as_f64().unwrap_or(0.0),
            steps: p["steps"].as_i64().unwrap_or(0),
            bd_a: p["bdA"].as_f64().unwrap_or(0.0),
            bd: (p["lat"].as_f64().unwrap_or(0.0), p["lng"].as_f64().unwrap_or(0.0)),
            _marker: std::marker::PhantomData,
        })
        .collect();
    archive.write_text("track.gpx", &build_gpx(uuid, start_ms, &rows))?;
    archive.write_json("track.json", &Value::Array(points.to_vec()))
}

/// 点位 + 围栏 + 校区选择快照。
pub fn save_area_snapshot(
    archive: &RunArchive,
    points: &[Value],
    selected_fence_id: Option<i64>,
    run_area_id: i64,
    fences: Option<&Value>,
) -> Result<(), String> {
    archive.write_json("points.json", &Value::Array(points.to_vec()))?;
    archive.write_json(
        "fences.json",
        &json!({
            "selected_run_area_id": run_area_id,
            "selected_fence_id": selected_fence_id,
            "fences": fences.cloned().unwrap_or(Value::Null),
        }),
    )
}

/// 失败/结果未知时的诊断包（脱敏，不含账号、Token、坐标明细）。
pub fn save_diagnostic(
    archive: &RunArchive,
    stage: &str,
    error: &str,
    extra: &Value,
) -> Result<(), String> {
    archive.write_json(
        "diagnostic.json",
        &json!({
            "stage": stage,
            "category": crate::diagnostics::category(error),
            "message": crate::diagnostics::safe_message(error),
            "extra": extra,
            "privacy": "不含账号、姓名、Token、设备标识、原始响应、文件路径或轨迹坐标",
        }),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_root(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "nekoarchive-{tag}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn gpx_is_well_formed_and_carries_cumulative_fields() {
        let rows = vec![
            TrackRow { lat: 39.9, lon: 116.4, total_time: 0, total_dis: 0.0, steps: 0, bd_a: 12.5, bd: (39.91, 116.41), _marker: std::marker::PhantomData },
            TrackRow { lat: 39.9001, lon: 116.4001, total_time: 10, total_dis: 12.0, steps: 20, bd_a: 12.8, bd: (39.9101, 116.4101), _marker: std::marker::PhantomData },
        ];
        let gpx = build_gpx("TEST", 1_700_000_000_000, &rows);
        assert!(gpx.starts_with("<?xml"));
        assert!(gpx.contains("xmlns:nsw="));
        assert_eq!(gpx.matches("<trkpt").count(), 2);
        assert!(gpx.contains(r#"steps="20""#));
        assert!(gpx.contains(r#"totalDis="12.00""#));
        assert!(gpx.ends_with("</gpx>\n"));
    }

    #[test]
    fn gpx_escapes_user_controlled_name() {
        let gpx = build_gpx("<script>&", 0, &[]);
        assert!(gpx.contains("&lt;script&gt;&amp;"));
        assert!(!gpx.contains("<script>"));
    }

    #[test]
    fn archive_writes_all_snapshot_files() {
        let root = temp_root("files");
        let archive = RunArchive {
            dir: root.join("run"),
        };
        std::fs::create_dir_all(archive.dir()).unwrap();
        let points = vec![json!({"gLat": 39.9, "gLng": 116.4, "totalTime": 0, "totalDis": 0.0, "steps": 0, "bdA": 0.0, "lat": 39.9, "lng": 116.4})];
        save_track(&archive, "UUID1234", 1_700_000_000_000, &points).unwrap();
        save_area_snapshot(&archive, &points, Some(1774), 42, None).unwrap();
        save_diagnostic(&archive, "submit", "HTTP 401 token=secret", &json!({})).unwrap();

        assert!(archive.dir().join("track.gpx").is_file());
        assert!(archive.dir().join("track.json").is_file());
        assert!(archive.dir().join("points.json").is_file());
        let fences = std::fs::read_to_string(archive.dir().join("fences.json")).unwrap();
        assert!(fences.contains("\"selected_fence_id\": 1774"));
        assert!(fences.contains("\"selected_run_area_id\": 42"));
        let diag = std::fs::read_to_string(archive.dir().join("diagnostic.json")).unwrap();
        assert!(!diag.contains("secret"), "诊断包泄漏了 Token");
        assert!(diag.contains("登录已失效"));
        let _ = std::fs::remove_dir_all(root);
    }
}
