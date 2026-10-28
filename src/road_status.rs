//! 把内部错误串翻译成「[分类] 原因 + 建议动作」。
//!
//! 直接抛 `String` 时用户只能看到"路由失败"，不知道该改什么。这里把已知
//! 失败归成互斥几类，并给出唯一可执行的动作；分类判定与 [`crate::diagnostics`]
//! 共用同一组关键词，避免两边说法不一致。

use crate::api::client::is_session_expired;

/// 失败分类及其建议动作。
pub fn explain(error: &str) -> String {
    let (kind, help) = classify(error);
    format!("[{kind}] {error}\n建议：{help}")
}

/// 仅返回分类标签（不含原文），供诊断报告使用。
pub fn classify(error: &str) -> (&'static str, &'static str) {
    if is_session_expired(error) {
        (
            "登录已失效",
            "在账号栏重新登录；成功后重新获取学校围栏和路网，本地地图仍可查看",
        )
    // 路网获取的专项分类：必须排在「超时」「围栏」「下载」等泛化关键词之前，
    // 否则「所有地图服务均不可用：…timeout」会被归成普通超时，丢了手动导入这条出路。
    } else if error.contains("所有地图服务均不可用") {
        (
            "地图服务不可达",
            "检查网络或代理后重试；也可在「路网」页点「浏览…」手动导入 OSM 文件",
        )
    } else if error.contains("没有返回可用围栏") || error.contains("学校没有有效围栏") {
        (
            "学校围栏缺失",
            "到「路网」页按校名搜索并确认校园边界，确认后再自动获取路网",
        )
    } else if error.contains("跨度过大") {
        (
            "路网下载范围跨度过大",
            "确认学校与点位在同一校区后重试；仍失败则在「路网」页用「浏览…」手动导入",
        )
    } else if error.contains("超过 24 MiB") {
        (
            "路网数据过大",
            "缩小下载范围，或在「路网」页用「浏览…」手动导入裁剪后的 OSM 文件",
        )
    } else if error.contains("不是有效 XML") || error.contains("不完整数据") {
        (
            "路网数据无效",
            "稍后重试自动获取；仍失败则在「路网」页用「浏览…」手动导入完整 OSM 文件",
        )
    } else if error.contains("11016") || error.contains("不在允许的时间段") {
        (
            "校园跑时段已过",
            "换到学校允许的时间段再跑；当前时段服务端会拒绝校园跑提交",
        )
    } else if error.contains("取消") {
        (
            "已取消",
            "可在当前操作结束后重试；已发出的网络请求不能立刻撤回",
        )
    } else if error.contains("timeout") || error.contains("timed out") || error.contains("超时") {
        ("网络超时", "检查网络后重试，或改用缓存的离线预览")
    } else if error.contains("没有可用") || error.contains("空路网") || error.contains("没有道路") {
        (
            "学校缺少道路数据",
            "更新路网或手动导入完整 OSM；不会向校外补足距离",
        )
    } else if error.contains("围栏") || error.contains("校区") {
        (
            "围栏或校区问题",
            "在线刷新校区列表、核对设备定位锚点，并确认学校边界",
        )
    } else if error.contains("不可达") || error.contains("不连通") || error.contains("超过 40") {
        (
            "点位不可达",
            "查看地图上的问题点，确认校区选择并补充缺失的道路",
        )
    } else if error.contains("不足") {
        ("目标距离不足", "按路线检查给出的最短闭环距离增大目标里程")
    } else if error.contains("闭环") || error.contains("有效路线") {
        (
            "校内道路无法闭合",
            "更新或补充校内道路，检查单向限制和断开的道路",
        )
    } else if error.contains("下载") || error.contains("network") || error.contains("HTTP") {
        ("路网网络请求失败", "检查网络后重试；已有缓存可离线查看")
    } else {
        ("处理失败", "查看具体原因，修正输入后重试")
    }
}

#[cfg(test)]
mod tests {
    use super::{classify, explain};

    #[test]
    fn failures_have_distinct_actions() {
        for (e, label) in [
            (crate::api::client::SESSION_EXPIRED, "登录已失效"),
            ("网络错误: timeout", "网络超时"),
            ("没有可用的 OSM 道路", "学校缺少道路数据"),
            ("没有有效围栏", "围栏或校区问题"),
            ("点位不可达", "点位不可达"),
            ("目标距离不足", "目标距离不足"),
            ("路线无法闭环", "校内道路无法闭合"),
            ("操作已取消", "已取消"),
        ] {
            assert_eq!(classify(e).0, label, "分类不符: {e}");
            assert!(explain(e).starts_with(&format!("[{label}]")));
        }
    }

    #[test]
    fn unknown_errors_fall_back_to_a_generic_action() {
        assert_eq!(classify("某种未知情况").0, "处理失败");
    }

    /// 路网获取每个失败原因都有独立分类和可执行建议，且建议里点名了下一步操作。
    #[test]
    fn road_download_failures_point_at_a_concrete_action() {
        for (e, label, hint) in [
            (
                "所有地图服务均不可用：https://overpass-api.de/api/interpreter: 自动获取地图失败: timeout；https://api.openstreetmap.org/api/0.6/map: 502",
                "地图服务不可达",
                "手动导入",
            ),
            (
                "当前学校没有有效围栏，无法确定路网范围",
                "学校围栏缺失",
                "搜索并确认校园边界",
            ),
            (
                "路网下载范围跨度过大，请确认当前学校和点位后重试",
                "路网下载范围跨度过大",
                "确认学校与点位",
            ),
            ("路网响应超过 24 MiB", "路网数据过大", "手动导入"),
            ("路网不是有效 XML", "路网数据无效", "手动导入"),
            (
                "路网服务返回错误或不完整数据，请稍后重试",
                "路网数据无效",
                "手动导入",
            ),
        ] {
            let (got, help) = classify(e);
            assert_eq!(got, label, "分类不符: {e}");
            assert!(help.contains(hint), "建议缺少「{hint}」: {help}");

            let shown = explain(e);
            assert!(shown.starts_with(&format!("[{label}] ")), "缺少分类前缀: {shown}");
            assert_eq!(shown.lines().count(), 2, "应为「原因 + 建议」两行: {shown}");
            assert!(shown.lines().nth(1).unwrap().starts_with("建议："));
        }
    }

    /// 专项分类插在泛化关键词之前，但不能反过来抢走既有分类。
    #[test]
    fn road_categories_do_not_hijack_existing_classifications() {
        // 「所有地图服务均不可用」串里常夹带 timeout，必须仍归为地图服务不可达。
        assert_eq!(
            classify("所有地图服务均不可用：重试超时 timeout").0,
            "地图服务不可达"
        );
        // 「围栏缺失」专属路线不能吞掉原有的「有效围栏」泛化分类。
        assert_eq!(classify("没有有效围栏").0, "围栏或校区问题");
        assert_eq!(classify("点位不可达").0, "点位不可达");
        assert_eq!(classify("路线无法闭环").0, "校内道路无法闭合");
        assert_eq!(classify("网络错误: timeout").0, "网络超时");
        assert_eq!(classify("操作已取消").0, "已取消");
    }
}
