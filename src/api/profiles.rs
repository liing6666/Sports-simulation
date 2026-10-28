//! 多套配置档位（PROFILE SLOTS）。
//!
//! 每个档位保存一组跑步参数（距离/配速/海拔/路线模式/点名数/操场规格等），
//! 与账号解耦：同一个人可以给「日常 2km」「冲量 5km」「操场 400m」各存一套；
//! 也可以在一个档位里存另一个账号的登录信息。
//!
//! 设计要点：
//! * 档位文件与 `config.json` 同级（`profiles.json`），升级时保留。
//! * 应用档位 = 把档位字段写进 `Config` 并立即落盘，因此老代码路径完全不用改。
//! * 密码单独存放且默认不写入：`remember_password = false` 时只存账号名。
//! * 导入/导出走独立的 `ProfileExport` 信封，不动 `ProfileSlot` 的 serde 结构，
//!   导出默认脱敏（密码字段直接从 JSON 中删掉），需显式勾选才保留。

use serde::{Deserialize, Serialize};

/// 导入/导出文件的格式标识与版本号。
pub const EXPORT_FORMAT: &str = "neko-profiles";
pub const EXPORT_VERSION: u32 = 1;

fn is_false(v: &bool) -> bool {
    !*v
}

/// 导出/导入文件的外层信封。
///
/// 与 `ProfileBook` 分开定义，因此 `ProfileSlot` 的 serde 结构保持不变，
/// 既有的 `profiles.json` / `config.json` 兼容性不受影响。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileExport {
    pub format: String,
    pub version: u32,
    /// 为真表示导出时已把所有 password 字段删除。
    #[serde(default, skip_serializing_if = "is_false")]
    pub exported_without_password: bool,
    #[serde(default)]
    pub slots: Vec<ProfileSlot>,
}

impl ProfileExport {
    /// 导入前校验格式与版本；不认识的格式/版本返回错误而不是 panic。
    pub fn validate(self) -> Result<Vec<ProfileSlot>, String> {
        if self.format != EXPORT_FORMAT {
            return Err(format!("不是档位文件（format = {:?}）", self.format));
        }
        if self.version != EXPORT_VERSION {
            return Err(format!(
                "不支持的档位文件版本 {}（当前支持 {EXPORT_VERSION}）",
                self.version
            ));
        }
        if self.slots.is_empty() {
            return Err("档位文件里没有任何档位".into());
        }
        Ok(self.slots)
    }

    /// 导入：解析 + 校验。
    pub fn read(text: &str) -> Result<Vec<ProfileSlot>, String> {
        let parsed: ProfileExport =
            serde_json::from_str(text).map_err(|e| format!("解析档位文件失败：{e}"))?;
        parsed.validate()
    }
}

/// 生成导出用的 JSON 文本。
///
/// `include_password = false` 时从**序列化后的 JSON 对象**里逐个删除 `password` 键，
/// 而不是把值改成空串——文件里不会残留任何原始密码。
pub fn export_json(book: &ProfileBook, include_password: bool) -> Result<String, String> {
    let mut slots =
        serde_json::to_value(&book.slots).map_err(|e| format!("序列化档位失败：{e}"))?;
    if !include_password {
        if let Some(items) = slots.as_array_mut() {
            for slot in items {
                if let Some(obj) = slot.as_object_mut() {
                    obj.remove("password");
                    // remember_password 只对密码有意义；脱敏后一并去掉，避免导入后
                    // 应用该档位时把「记住的密码」当成空串写回 config。
                    obj.remove("remember_password");
                }
            }
        }
    }

    // 只导出 slots 与格式头，不导出 active（导入方不应继承「当前选中」）。
    let mut envelope = serde_json::Map::new();
    envelope.insert("format".into(), EXPORT_FORMAT.into());
    envelope.insert("version".into(), EXPORT_VERSION.into());
    if !include_password {
        envelope.insert("exported_without_password".into(), true.into());
    }
    envelope.insert("slots".into(), slots);
    serde_json::to_string_pretty(&serde_json::Value::Object(envelope))
        .map_err(|e| format!("生成导出 JSON 失败：{e}"))
}

/// 合并导入的档位：同名追加「 (导入)」后缀，绝不覆盖现有档位。
///
/// 返回加入后的名称列表，调用方负责 `save()`。
pub fn merge_slots(book: &mut ProfileBook, imported: Vec<ProfileSlot>) -> Vec<String> {
    let mut added = Vec::with_capacity(imported.len());
    for mut slot in imported {
        let base = if slot.name.trim().is_empty() {
            format!("档位 {}", book.slots.len() + 1)
        } else {
            slot.name.trim().to_string()
        };
        let name = unique_import_name(book, &base);
        slot.name = name.clone();
        added.push(name);
        book.slots.push(slot);
    }
    added
}

fn unique_import_name(book: &ProfileBook, base: &str) -> String {
    // 第一次冲突加「 (导入)」，仍冲突则继续加序号。
    let candidate = format!("{base} (导入)");
    if !book.slots.iter().any(|s| s.name == candidate) {
        return candidate;
    }
    let mut n = 2;
    loop {
        let candidate = format!("{base} (导入 {n})");
        if !book.slots.iter().any(|s| s.name == candidate) {
            return candidate;
        }
        n += 1;
    }
}

/// 档位文件：固定槽位 + 当前选中项。
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProfileBook {
    #[serde(default)]
    pub slots: Vec<ProfileSlot>,
    /// 当前应用的槽位下标；None 表示未应用任何档位（跟随当前 config）。
    #[serde(default)]
    pub active: Option<usize>,
}

impl Default for ProfileBook {
    fn default() -> Self {
        Self {
            slots: Vec::new(),
            active: None,
        }
    }
}

/// 一个配置档位。
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct ProfileSlot {
    pub name: String,
    /// 账号（手机号）。留空表示该档位不绑定账号，只切换参数。
    #[serde(default)]
    pub username: String,
    /// 密码。仅在 `remember_password` 为真时写入。
    #[serde(default)]
    pub password: String,
    #[serde(default)]
    pub remember_password: bool,

    // ---- 跑步参数 ----
    #[serde(default = "default_f32")]
    pub dist_min: f32,
    #[serde(default = "default_f32")]
    pub dist_max: f32,
    #[serde(default = "default_pace_min")]
    pub pace_min: f32,
    #[serde(default = "default_pace_max")]
    pub pace_max: f32,
    #[serde(default)]
    pub face_check: bool,
    #[serde(default)]
    pub manual_altitude: Option<f64>,
    #[serde(default)]
    pub manual_altitude_range: Option<crate::track::altitude::AltitudeRange>,
    #[serde(default = "default_true")]
    pub manual_altitude_enabled: bool,
    #[serde(default)]
    pub track_color_mode: crate::api::model::TrackColorMode,
    #[serde(default = "default_route_mode")]
    pub route_mode: String,
    #[serde(default = "default_ordinary_pick")]
    pub ordinary_pick: usize,
    #[serde(default)]
    pub track_spec: crate::track::stadium::TrackSpec,
    #[serde(default)]
    pub osm_path: String,
    #[serde(default)]
    pub amap_key: String,
    #[serde(default)]
    pub custom_origin: String,
    #[serde(default)]
    pub custom_destination: String,
    #[serde(default = "default_true")]
    pub custom_loop: bool,
    #[serde(default)]
    pub custom_laps: i64,
}

impl Default for ProfileSlot {
    fn default() -> Self {
        Self {
            name: String::new(),
            username: String::new(),
            password: String::new(),
            remember_password: false,
            dist_min: 1.0,
            dist_max: 1.5,
            pace_min: 360.0,
            pace_max: 480.0,
            face_check: true,
            manual_altitude: None,
            manual_altitude_range: None,
            manual_altitude_enabled: false,
            track_color_mode: Default::default(),
            route_mode: "legacy".into(),
            ordinary_pick: 2,
            track_spec: Default::default(),
            osm_path: String::new(),
            amap_key: String::new(),
            custom_origin: String::new(),
            custom_destination: String::new(),
            custom_loop: true,
            custom_laps: 0,
        }
    }
}

fn default_f32() -> f32 {
    1.0
}
fn default_pace_min() -> f32 {
    360.0
}
fn default_pace_max() -> f32 {
    480.0
}
fn default_true() -> bool {
    true
}
fn default_route_mode() -> String {
    "legacy".into()
}
fn default_ordinary_pick() -> usize {
    2
}

impl ProfileBook {
    pub fn load() -> Self {
        crate::api::model::read_named("profiles.json").unwrap_or_default()
    }

    pub fn save(&self) -> Result<(), String> {
        crate::api::model::write_named("profiles.json", self)
    }

    /// 从当前 Config 抓一份快照，用于「保存为档位」。
    pub fn capture(name: &str, config: &crate::api::model::Config) -> ProfileSlot {
        ProfileSlot {
            name: name.to_string(),
            username: config.username.clone(),
            password: if config.remember {
                config.password.clone()
            } else {
                String::new()
            },
            remember_password: config.remember,
            dist_min: config.dist_min,
            dist_max: config.dist_max,
            pace_min: config.pace_min,
            pace_max: config.pace_max,
            face_check: config.face_check,
            manual_altitude: config.manual_altitude,
            manual_altitude_range: config.manual_altitude_range,
            manual_altitude_enabled: config.manual_altitude.is_some()
                || config.manual_altitude_range.is_some(),
            track_color_mode: config.track_color_mode,
            route_mode: config.route_mode.clone(),
            ordinary_pick: config.ordinary_pick,
            track_spec: config.track_spec,
            osm_path: config.osm_path.clone(),
            amap_key: config.amap_key.clone(),
            custom_origin: config.custom_origin.clone(),
            custom_destination: config.custom_destination.clone(),
            custom_loop: config.custom_loop,
            custom_laps: config.custom_laps,
        }
    }

    /// 把档位写进 Config（不落盘，由调用方决定何时 save）。
    ///
    /// 只覆盖档位负责的字段：`username/password` 仅在档位确实绑定了账号时才覆盖，
    /// 否则「纯参数档位」会把当前登录账号冲掉。
    pub fn apply_to(slot: &ProfileSlot, config: &mut crate::api::model::Config) {
        if !slot.username.is_empty() {
            config.username = slot.username.clone();
            config.password = slot.password.clone();
            config.remember = slot.remember_password;
        }
        config.dist_min = slot.dist_min;
        config.dist_max = slot.dist_max;
        config.pace_min = slot.pace_min;
        config.pace_max = slot.pace_max;
        config.face_check = slot.face_check;
        config.manual_altitude = if slot.manual_altitude_enabled {
            slot.manual_altitude
        } else {
            None
        };
        config.manual_altitude_range = if slot.manual_altitude_enabled {
            slot.manual_altitude_range
        } else {
            None
        };
        config.track_color_mode = slot.track_color_mode;
        config.route_mode = slot.route_mode.clone();
        config.ordinary_pick = slot.ordinary_pick;
        config.track_spec = slot.track_spec;
        config.osm_path = slot.osm_path.clone();
        config.amap_key = slot.amap_key.clone();
        config.custom_origin = slot.custom_origin.clone();
        config.custom_destination = slot.custom_destination.clone();
        config.custom_loop = slot.custom_loop;
        config.custom_laps = slot.custom_laps;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::model::Config;

    fn slot_with_account() -> ProfileSlot {
        ProfileSlot {
            name: "账号A".into(),
            username: "13800000000".into(),
            password: "secret".into(),
            remember_password: true,
            dist_min: 2.0,
            ..Default::default()
        }
    }

    #[test]
    fn applying_a_slot_overwrites_run_parameters() {
        let mut config = Config::default();
        ProfileBook::apply_to(&slot_with_account(), &mut config);
        assert_eq!(config.username, "13800000000");
        assert_eq!(config.password, "secret");
        assert!(config.remember);
        assert_eq!(config.dist_min, 2.0);
    }

    #[test]
    fn parameter_only_slot_does_not_touch_the_logged_in_account() {
        let mut config = Config {
            username: "current".into(),
            password: "currentpw".into(),
            remember: true,
            ..Default::default()
        };
        let slot = ProfileSlot {
            name: "冲量 5km".into(),
            dist_min: 5.0,
            ..Default::default()
        };
        ProfileBook::apply_to(&slot, &mut config);
        assert_eq!(config.username, "current");
        assert_eq!(config.password, "currentpw");
        assert_eq!(config.dist_min, 5.0);
    }

    #[test]
    fn capture_omits_password_when_not_remembered() {
        let config = Config {
            username: "u".into(),
            password: "p".into(),
            remember: false,
            ..Default::default()
        };
        let slot = ProfileBook::capture("x", &config);
        assert!(slot.password.is_empty(), "未勾选记住密码时不应保存密码");
        assert!(!slot.remember_password);
    }

    #[test]
    fn disabled_altitude_override_clears_on_apply() {
        let mut config = Config {
            manual_altitude: Some(17.0),
            ..Default::default()
        };
        let slot = ProfileSlot {
            manual_altitude_enabled: false,
            manual_altitude: Some(99.0),
            ..Default::default()
        };
        ProfileBook::apply_to(&slot, &mut config);
        assert_eq!(config.manual_altitude, None);
    }

    #[test]
    fn round_trip_through_json_preserves_fields() {
        let book = ProfileBook {
            slots: vec![slot_with_account()],
            active: Some(0),
        };
        let text = serde_json::to_string(&book).unwrap();
        let back: ProfileBook = serde_json::from_str(&text).unwrap();
        assert_eq!(back.slots.len(), 1);
        assert_eq!(back.slots[0].name, "账号A");
        assert_eq!(back.active, Some(0));
    }

    fn book_with_secret() -> ProfileBook {
        ProfileBook {
            slots: vec![
                ProfileSlot {
                    name: "账号A".into(),
                    username: "13800000000".into(),
                    password: "S3cr3t-Pa55".into(),
                    remember_password: true,
                    ..Default::default()
                },
                ProfileSlot {
                    name: "账号B".into(),
                    username: "13900000000".into(),
                    password: "another-Secret-42".into(),
                    remember_password: true,
                    ..Default::default()
                },
            ],
            active: Some(1),
        }
    }

    #[test]
    fn default_export_contains_no_password_strings_or_keys() {
        let book = book_with_secret();
        let text = export_json(&book, false).unwrap();
        for secret in ["S3cr3t-Pa55", "another-Secret-42"] {
            assert!(!text.contains(secret), "脱敏导出的 JSON 里残留了密码：{secret}");
        }
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert_eq!(value["exported_without_password"], true);
        for slot in value["slots"].as_array().unwrap() {
            assert!(
                slot.get("password").is_none(),
                "password 键必须被删除而不是置空：{slot}"
            );
        }
        // 非密码字段要保留。
        assert_eq!(value["slots"][0]["username"], "13800000000");
        assert_eq!(value["slots"][0]["name"], "账号A");
    }

    #[test]
    fn export_with_password_keeps_them_and_omits_the_flag() {
        let book = book_with_secret();
        let text = export_json(&book, true).unwrap();
        assert!(text.contains("S3cr3t-Pa55"));
        let value: serde_json::Value = serde_json::from_str(&text).unwrap();
        assert!(value.get("exported_without_password").is_none());
        assert_eq!(value["format"], EXPORT_FORMAT);
        assert_eq!(value["version"], EXPORT_VERSION);
        assert_eq!(value["slots"].as_array().unwrap().len(), 2);
    }

    #[test]
    fn export_round_trips_through_import() {
        let text = export_json(&book_with_secret(), true).unwrap();
        let slots = ProfileExport::read(&text).unwrap();
        assert_eq!(slots.len(), 2);
        assert_eq!(slots[0].password, "S3cr3t-Pa55");
        assert_eq!(slots[1].dist_min, ProfileSlot::default().dist_min);
    }

    #[test]
    fn stripped_export_imports_with_empty_password() {
        let text = export_json(&book_with_secret(), false).unwrap();
        let slots = ProfileExport::read(&text).unwrap();
        assert_eq!(slots.len(), 2);
        for slot in &slots {
            assert!(slot.password.is_empty());
            assert!(!slot.remember_password);
            assert_eq!(slot.username.len(), 11);
        }
    }

    #[test]
    fn importing_invalid_json_returns_error_without_panicking() {
        for bad in [
            "",
            "not json",
            "{",
            "[]",
            r#"{"format":"other","version":1,"slots":[{"name":"x"}]}"#,
            r#"{"format":"neko-profiles","version":999,"slots":[{"name":"x"}]}"#,
            r#"{"format":"neko-profiles","version":1,"slots":[]}"#,
            r#"{"format":"neko-profiles","version":1,"slots":[{"name":123}]}"#,
        ] {
            let err = ProfileExport::read(bad).unwrap_err();
            assert!(!err.is_empty());
        }
    }

    #[test]
    fn import_merge_appends_suffix_instead_of_overwriting() {
        let mut book = ProfileBook {
            slots: vec![ProfileSlot {
                name: "日常".into(),
                password: "keep-me".into(),
                ..Default::default()
            }],
            active: Some(0),
        };
        let imported = vec![
            ProfileSlot {
                name: "日常".into(),
                dist_min: 5.0,
                ..Default::default()
            },
            ProfileSlot {
                name: "日常".into(),
                dist_min: 3.0,
                ..Default::default()
            },
        ];
        let added = merge_slots(&mut book, imported);
        assert_eq!(added, vec!["日常 (导入)", "日常 (导入 2)"]);
        assert_eq!(book.slots.len(), 3);
        // 原有档位必须原样保留。
        assert_eq!(book.slots[0].name, "日常");
        assert_eq!(book.slots[0].password, "keep-me");
        assert_eq!(book.active, Some(0));
    }
}
