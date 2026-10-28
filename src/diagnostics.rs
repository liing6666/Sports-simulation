//! 一键诊断报告：白名单输出，只给分类不给原文。
//!
//! 用户复制出去求帮助的内容里最容易夹带账号、Token、设备号、学校编号、
//! 文件路径和轨迹坐标。这里的原则是**默认不导出**：只把错误串映射成固定
//! 分类标签，原始文本只用于分类判定，绝不进入输出。

/// 把原始错误串归到一个固定分类；未能识别的一律标为未分类。
pub fn category(text: &str) -> &'static str {
    // 顺序有意义：越具体的原因越靠前，避免被泛化的"失败"抢先命中。
    // 路网获取的专项分类必须与 `crate::road_status::classify` 同序（专项在泛化前）：
    // 「所有地图服务均不可用：…timeout」原文带 timeout，排在超时之后会被误判。
    for (needle, label) in [
        ("所有地图服务均不可用", "地图服务不可达"),
        ("没有返回可用围栏", "学校围栏缺失"),
        ("学校没有有效围栏", "学校围栏缺失"),
        ("跨度过大", "路网下载范围跨度过大"),
        ("超过 24 MiB", "路网数据过大"),
        ("不是有效 XML", "路网数据无效"),
        ("不完整数据", "路网数据无效"),
        ("围栏", "电子围栏缺失、变化或未确认"),
        ("校区", "校区/区域不匹配"),
        ("不连通", "校内道路不连通"),
        ("不可达", "打卡点位不可达"),
        ("超过 40", "点位离路超过判定范围"),
        ("不足", "目标距离不足"),
        ("闭环", "校内道路无法闭环"),
        ("超时", "网络超时"),
        ("timeout", "网络超时"),
        ("timed out", "网络超时"),
        ("401", "登录已失效"),
        ("登录", "登录状态或学校信息"),
        ("解密", "响应解密失败"),
        ("验签", "响应验签失败"),
        ("下载", "路网下载或缓存处理"),
        ("缓存", "路网下载或缓存处理"),
        ("取消", "用户取消"),
        ("失败", "操作失败"),
        ("错误", "操作错误"),
    ] {
        if text.contains(needle) {
            return label;
        }
    }
    "未分类状态（原文为保护隐私不导出）"
}

/// 可安全写进诊断包的错误摘要。
///
/// 只保留**服务端业务码**和分类标签：业务码能定位问题（如 10121 设备风险、
/// 10603 点位限流），而原文里可能夹带 token/坐标/路径。其余内容一律不回显。
pub fn safe_message(error: &str) -> String {
    let category = self::category(error);
    // 提取 "业务错误 <code>" 或 "HTTP <code>" 里的数字码。
    for marker in ["业务错误 ", "HTTP "] {
        if let Some(rest) = error.split(marker).nth(1) {
            let code: String = rest.chars().take_while(|c| c.is_ascii_digit()).collect();
            if !code.is_empty() {
                return format!("{category}（错误的服务端业务码 {code}，原文未导出）");
            }
        }
    }
    format!("{category}（原文未导出）")
}

/// 一段时间任务的可导出诊断摘要。所有字段都经过白名单筛选。
pub struct Report {
    pub status: &'static str,
    pub road_status: &'static str,
    pub road_source: &'static str,
    pub offline_preview: bool,
    pub road_nodes: usize,
    pub road_edges: usize,
}

impl Report {
    pub fn to_value(&self) -> serde_json::Value {
        serde_json::json!({
            "version": env!("CARGO_PKG_VERSION"),
            "os": std::env::consts::OS,
            "arch": std::env::consts::ARCH,
            "status": self.status,
            "road_status": self.road_status,
            "road_source": self.road_source,
            "offline_preview": self.offline_preview,
            "road_nodes": self.road_nodes,
            "road_edges": self.road_edges,
            "privacy": "不含账号、姓名、Token、设备标识、原始日志/响应、文件路径、学校编号或轨迹坐标",
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{category, Report};

    #[test]
    fn categories_cover_the_common_failures() {
        assert_eq!(category("围栏为空"), "电子围栏缺失、变化或未确认");
        assert_eq!(category("点位不可达"), "打卡点位不可达");
        assert_eq!(category("网络错误: timeout"), "网络超时");
        assert_eq!(category("业务错误 10603: 限流"), "操作错误");
        assert_eq!(category(""), "未分类状态（原文为保护隐私不导出）");
    }

    #[test]
    fn safe_message_keeps_business_code_but_drops_the_rest() {
        let msg = super::safe_message("业务错误 10121: 设备风险 uid=123 token=abc");
        assert!(msg.contains("10121"));
        assert!(!msg.contains("123"), "账号泄漏: {msg}");
        assert!(!msg.contains("abc"), "Token 泄漏: {msg}");
        assert!(super::safe_message("网络错误: timeout").contains("网络超时"));
    }

    /// 路网获取的五个专项分类与 `road_status::classify` 对齐，且原文从不进入输出。
    #[test]
    fn road_failures_are_categorized_without_leaking_the_original() {
        for (e, label) in [
            (
                "所有地图服务均不可用：https://overpass-api.de/api/interpreter: 自动获取地图失败: timeout；uid=123 token=super-secret /Users/Alice",
                "地图服务不可达",
            ),
            (
                "当前学校没有有效围栏，无法确定路网范围 uid=123 token=super-secret /home/Alice",
                "学校围栏缺失",
            ),
            (
                "路网下载范围跨度过大，请确认当前学校和点位后重试 lat=30.1 lon=104.2 uid=123",
                "路网下载范围跨度过大",
            ),
            ("路网响应超过 24 MiB uid=123 token=super-secret", "路网数据过大"),
            (
                "路网不是有效 XML uid=123 token=super-secret /Users/Alice",
                "路网数据无效",
            ),
            (
                "路网服务返回错误或不完整数据，请稍后重试 uid=123 password=hunter2",
                "路网数据无效",
            ),
        ] {
            assert_eq!(category(e), label, "分类不符: {e}");
            let msg = super::safe_message(e);
            assert!(msg.contains(label), "缺少分类「{label}」: {msg}");
            for secret in ["123", "super-secret", "Alice", "hunter2", "overpass", "30.1", "104.2"] {
                assert!(!msg.contains(secret), "诊断摘要泄漏了 {secret}: {msg}");
            }
        }
    }

    /// 诊断报告同样只出分类：五类路网失败各走一遍 Report。
    #[test]
    fn road_failures_never_reach_the_report_as_raw_text() {
        for (e, label) in [
            ("所有地图服务均不可用：timeout uid=123", "地图服务不可达"),
            ("没有返回可用围栏 token=super-secret", "学校围栏缺失"),
            ("跨度过大 /Users/Alice", "路网下载范围跨度过大"),
            ("超过 24 MiB uid=123", "路网数据过大"),
            ("不是有效 XML password=hunter2", "路网数据无效"),
        ] {
            let report = Report {
                status: category(e),
                road_status: category(e),
                road_source: "school",
                offline_preview: false,
                road_nodes: 0,
                road_edges: 0,
            }
            .to_value()
            .to_string();
            assert!(report.contains(label), "报告缺少分类「{label}」: {report}");
            for secret in ["123", "super-secret", "Alice", "hunter2"] {
                assert!(!report.contains(secret), "报告泄漏了 {secret}: {report}");
            }
        }
    }

    /// 专项分支必须排在「超时」「围栏」之前，但不能抢走既有分类。
    #[test]
    fn road_categories_keep_the_same_precedence_as_road_status() {
        assert_eq!(
            category("所有地图服务均不可用：重试超时 timeout"),
            "地图服务不可达",
            "专项分类被「超时」抢先命中"
        );
        // 既有行为不变：「没有有效围栏」仍走泛化围栏分类。
        assert_eq!(category("没有有效围栏"), "电子围栏缺失、变化或未确认");
        assert_eq!(category("网络错误: timeout"), "网络超时");
        assert_eq!(category("下载失败"), "路网下载或缓存处理");
        assert_eq!(category("点位不可达"), "打卡点位不可达");
    }

    #[test]
    fn secrets_in_errors_are_never_exported() {
        let report = Report {
            status: category("登录失败 uid=123 token=super-secret /Users/Alice"),
            road_status: category("下载失败 password=hunter2"),
            road_source: "school",
            offline_preview: false,
            road_nodes: 2,
            road_edges: 3,
        }
        .to_value()
        .to_string();
        for secret in ["123", "super-secret", "Alice", "hunter2", "Users"] {
            assert!(!report.contains(secret), "诊断报告泄漏了 {secret}");
        }
        assert!(report.contains("登录状态"));
        assert!(report.contains("路网下载"));
    }
}
