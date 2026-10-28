//! 运动项目（校园跑 / 户外跑 / 室外锻炼 / AI 运动）。
//!
//! 各项目在服务端用两个维度区分：
//! * `runMode` —— 拉策略/学期数据时的模式参数（`runModePolicy`、`getPersonalSemesterInfo`）
//! * `sportType` —— 提交成绩时的运动类型（`runnings/save/record`）
//!
//! 已通过真实账号只读探测确认的事实：
//! * `runMode = 1` 是**校园跑**：现在（22:xx）请求会返回 11016「不在允许的时间段内」。
//! * `runMode = 2/3/4` 都返回 200，配置相同（`policy=0`、`geoFence=null`、无点位），
//!   说明它们是**自由跑类**项目（不需要围栏与打卡点）。
//! * 校园跑的可跑时段来自 `runRuleModel.validTime`，实测为 `06:20:00–22:00:00`；
//!   户外跑**没有时间限制**（用户确认，且探测时 runMode=2 未被 11016 拒绝）。
//! * `sportType = 1` 是校园跑；`sportType = 6` 是**户外跑**（已由 rrid 1343567950
//!   的真实记录详情确认：0.09km / 0:36 / 235spm，与用户指认的户外跑数据一致）。
//!   注意 AI 运动的记录走独立接口，不在这个 `sportType` 体系里。
//!
//! **未确认项**：室外锻炼的 `runMode` 与 `sportType` 具体值。这些值必须从官方
//! App 的一次真实记录里读出来，不能靠猜——猜错会把成绩提交到错误的运动类型。
//! 在确认之前 `FIELD.sport_type` 保持 `None`，调用方必须在提交前拦截并提示。

/// 一个运动项目的服务端参数。
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SportKind {
    /// 界面显示名。
    pub label: &'static str,
    /// 拉策略 / 学期数据用的 `runMode`。
    pub run_mode: i64,
    /// 提交成绩用的 `sportType`；`None` 表示尚未经真实抓包确认。
    pub sport_type: Option<i64>,
    /// 是否需要校园围栏与打卡点。
    pub needs_campus: bool,
}

impl SportKind {
    /// 校园跑：有围栏、有打卡点、限时 06:20–22:00。
    pub const CAMPUS: Self = Self {
        label: "校园跑",
        run_mode: 1,
        sport_type: Some(1),
        needs_campus: true,
    };

    /// 户外跑：无围栏、无打卡点、**不限时**。
    ///
    /// `sportType = 6` 已由用户真实记录确认：
    /// rrid 1343567950（2026-10-23 16:31，0.09km / 0:36 / 235spm）的详情
    /// 返回 `sportType: 6`，与用户指认的户外跑数据完全一致。
    pub const OUTDOOR: Self = Self {
        label: "户外跑",
        run_mode: 2,
        sport_type: Some(6),
        needs_campus: false,
    };

    /// 室外锻炼：同样属于自由跑类，`sportType` 尚未经真实记录确认。
    pub const FIELD: Self = Self {
        label: "室外锻炼",
        run_mode: 3,
        sport_type: None,
        needs_campus: false,
    };

    /// 界面上可选的跑步项目（AI 运动走独立入口，不在此列）。
    pub const SELECTABLE: [Self; 3] = [Self::CAMPUS, Self::OUTDOOR, Self::FIELD];

    /// 提交前校验：`sportType` 未确认时禁止提交。
    pub fn require_sport_type(self) -> Result<i64, String> {
        self.sport_type.ok_or_else(|| {
            format!(
                "「{}」的提交类型尚未确认：请先用官方 App 跑一次该项目，\
                 再用本工具读取记录列表确认真实 sportType，避免提交到错误的运动类型",
                self.label
            )
        })
    }

    /// 从配置里的字符串解析（`campus` / `outdoor` / `field`）。
    pub fn from_key(key: &str) -> Self {
        match key {
            "outdoor" => Self::OUTDOOR,
            "field" => Self::FIELD,
            _ => Self::CAMPUS,
        }
    }

    /// 配置持久化用的键。
    pub fn key(self) -> &'static str {
        match self {
            Self::OUTDOOR => "outdoor",
            Self::FIELD => "field",
            _ => "campus",
        }
    }
}

/// 校园跑的时间段（来自 `runRuleModel.validTime` 实测值）。
///
/// 仅用于界面提示与本地预判；真正的判定以服务端返回为准。
pub const CAMPUS_WINDOW: (&str, &str) = ("06:20", "22:00");

/// 当前时刻是否落在校园跑可跑时段内（按本地时间）。
pub fn campus_window_open(now: chrono::DateTime<chrono::Local>) -> bool {
    use chrono::Timelike;
    let minutes = now.hour() as i64 * 60 + now.minute() as i64;
    let parse = |s: &str| -> i64 {
        let mut it = s.split(':');
        let h: i64 = it.next().and_then(|v| v.parse().ok()).unwrap_or(0);
        let m: i64 = it.next().and_then(|v| v.parse().ok()).unwrap_or(0);
        h * 60 + m
    };
    let (start, end) = (parse(CAMPUS_WINDOW.0), parse(CAMPUS_WINDOW.1));
    minutes >= start && minutes < end
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;

    fn at(h: u32, m: u32) -> chrono::DateTime<chrono::Local> {
        chrono::Local
            .with_ymd_and_hms(2026, 10, 26, h, m, 0)
            .single()
            .unwrap()
    }

    #[test]
    fn campus_window_matches_the_server_configured_range() {
        assert!(!campus_window_open(at(6, 19)));
        assert!(campus_window_open(at(6, 20)));
        assert!(campus_window_open(at(12, 0)));
        assert!(campus_window_open(at(21, 59)));
        assert!(!campus_window_open(at(22, 0)));
        assert!(!campus_window_open(at(23, 30)));
    }

    /// 未确认 sportType 的项目必须拒绝提交，不能默认成 1（那会提交成校园跑）。
    #[test]
    fn unconfirmed_sport_types_are_rejected_not_guessed() {
        // 户外跑的 sportType 已由真实记录确认（rrid 1343567950）。
        assert_eq!(SportKind::OUTDOOR.require_sport_type().unwrap(), 6);
        // 室外锻炼仍未确认，必须拒绝而不是猜一个值。
        assert!(SportKind::FIELD.require_sport_type().is_err());
        assert_eq!(SportKind::CAMPUS.require_sport_type().unwrap(), 1);
    }

    #[test]
    fn keys_round_trip() {
        for kind in SportKind::SELECTABLE {
            assert_eq!(SportKind::from_key(kind.key()), kind);
        }
        // 未知键回退到校园跑，保持旧配置可用。
        assert_eq!(SportKind::from_key("nonsense"), SportKind::CAMPUS);
    }

    /// 只有校园跑需要围栏与打卡点。
    #[test]
    fn only_campus_needs_fence_and_points() {
        assert!(SportKind::CAMPUS.needs_campus);
        assert!(!SportKind::OUTDOOR.needs_campus);
        assert!(!SportKind::FIELD.needs_campus);
    }
}
