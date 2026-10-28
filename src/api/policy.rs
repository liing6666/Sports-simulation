//! 跑步策略：POST /api/v70103/runModePolicy。
//! 返回 data.runRuleModel.minDistance（提交时 selDistance 用它）与 data.policy。

use super::client::{get_field, ApiClient};
use super::fence::point_xy;
use serde_json::{json, Value};

pub const POLICY_PATH: &str = "/api/v70103/runModePolicy";

pub struct PolicyInfo {
    pub timestamp: i64,
    pub policy: i64,
    pub min_distance: i64,
    pub valid_time: i64,
    /// 必经点（BD 系，与打卡点同系），policy 响应里若有则返回。
    pub must_points: Vec<(f64, f64)>,
    /// 服务端可选的校园区域/绿色围栏元数据；没有配置时为自由跑默认值。
    pub area: crate::track::wire::RunAreaMeta,
}

/// 将已解密的业务响应解析为稳定的策略模型，供网络路径和 fixture 测试共用。
pub(crate) fn parse_policy_response(biz: &Value) -> Result<PolicyInfo, String> {
    let timestamp = get_field(biz, "timestamp")
        .and_then(|t| t.as_i64())
        .ok_or("policy 响应缺 timestamp")?;
    let policy = get_field(biz, "policy")
        .and_then(|t| t.as_i64())
        .unwrap_or(0);
    let rule = get_field(biz, "runRuleModel")
        .cloned()
        .unwrap_or(Value::Null);
    Ok(PolicyInfo {
        timestamp,
        policy,
        min_distance: rule
            .get("minDistance")
            .and_then(|t| t.as_i64())
            .unwrap_or(1000),
        valid_time: rule.get("validTime").and_then(|t| t.as_i64()).unwrap_or(0),
        must_points: extract_must_points(biz),
        area: super::points::area_from_payload(biz, &[]),
    })
}

/// 从 policy 响应 `data` 中防御式提取必经点列表（字段名不确定，逐个尝试）。
fn extract_must_points(v: &serde_json::Value) -> Vec<(f64, f64)> {
    let data = v.get("data").unwrap_or(v);
    for name in [
        "pointList",
        "runPointList",
        "mustPointList",
        "passPointList",
        "mustPoints",
        "passPoints",
        "runPoints",
        "points",
        "nodeList",
        "checkPointList",
    ] {
        let Some(arr) = data.get(name).and_then(|x| x.as_array()) else {
            continue;
        };
        // 复用 fence 的坐标系解析：BD 优先，缺失/全 0 回退 glat/glon（GCJ→BD）并过滤 (0,0)。
        let pts: Vec<(f64, f64)> = arr.iter().filter_map(point_xy).collect();
        if !pts.is_empty() {
            return pts;
        }
    }
    Vec::new()
}

/// body：{"runMode":<runMode>,"ruleUpdateTime":0,"geoFenceUpdateTime":0,"selectUnid":<unid>,"operateType":0}
///
/// `runMode` 区分运动项目（1=校园跑，2=户外跑，3=室外锻炼）。校园跑有 11016
/// 时段限制，户外跑没有。
pub fn fetch_policy_for(client: &mut ApiClient, run_mode: i64) -> Result<PolicyInfo, String> {
    let unid = client
        .login
        .as_ref()
        .map(|s| s.unid.clone())
        .unwrap_or_default();
    let select_unid = unid.parse::<i64>().unwrap_or(0);
    let body = json!({
        "runMode": run_mode,
        "ruleUpdateTime": 0,
        "geoFenceUpdateTime": 0,
        "selectUnid": select_unid,
        "operateType": 0,
    })
    .to_string();
    let biz = client.call("POST", POLICY_PATH, &body, &[])?;
    parse_policy_response(&biz)
}

/// 默认按校园跑拉策略（保留旧调用点）。
pub fn fetch_policy(client: &mut ApiClient) -> Result<PolicyInfo, String> {
    fetch_policy_for(client, 1)
}

#[cfg(test)]
mod tests {
    use super::parse_policy_response;

    #[test]
    fn parses_nested_policy_fixture_and_gcj_fallback_point() {
        let raw = include_str!("fixtures/policy_nested.json");
        let value: serde_json::Value = serde_json::from_str(raw).unwrap();
        let policy = parse_policy_response(&value).unwrap();
        assert_eq!(policy.timestamp, 1_700_000_000_123);
        assert_eq!(policy.policy, 1);
        assert_eq!(policy.min_distance, 2000);
        assert_eq!(policy.valid_time, 86400);
        assert_eq!(policy.must_points.len(), 2);
        assert!((policy.must_points[0].0 - 39.9001).abs() < 1e-9);
        assert!((policy.must_points[1].0 - 39.9062).abs() < 0.01);
    }
}

#[cfg(test)]
mod runmode_probe {
    /// 只读探测：不同 runMode 值的策略响应差异，用于确定各项目的参数。
    /// 不提交任何成绩，只调 policy 接口。
    #[test]
    #[ignore = "需要登录态；用 --ignored 运行"]
    fn probe_run_modes() {
        use crate::api::client::ApiClient;
        let identity = crate::api::model::load_identity();
        let sess = crate::api::model::load_session();
        if !sess.is_logged_in() {
            println!("未登录，跳过");
            return;
        }
        for mode in 1..=4 {
            let mut client = ApiClient::new(identity.clone(), Some(sess.clone()));
            let body = serde_json::json!({
                "runMode": mode,
                "ruleUpdateTime": 0,
                "geoFenceUpdateTime": 0,
                "selectUnid": sess.unid.parse::<i64>().unwrap_or(0),
                "operateType": 0,
            })
            .to_string();
            match client.call("POST", super::POLICY_PATH, &body, &[]) {
                Ok(biz) => {
                    let data = serde_json::to_string(&biz).unwrap_or_default();
                    println!("runMode={mode} → OK: {}", &data[..data.len().min(300)]);
                }
                Err(e) => println!("runMode={mode} → Err: {e}"),
            }
        }
    }
}
