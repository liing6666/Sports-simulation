//! HTTP 客户端封装。
//!
//! 通用请求封装：body = 信封(明文, insert)，headerSign = 信封(header, observed)，
//! 响应两层解密到业务 JSON；同一实例复用同一 keyDataFour。

use crate::api::model::Session;
use crate::crypto::decrypt::{decrypt_response, derive_paes_key, Decrypted};
use crate::crypto::envelope::{
    build_envelope, build_envelope_ts, rsa_public_key, EnvelopeSession, OuterOrder,
};
use crate::crypto::header::{build_header_for, HeaderIdentity, UA_IOS};

/// 登录凭证失效的统一标记。
///
/// 任何一次认证 401 都返回这个串，调用方据此清会话/提示重新登录；
/// 地图等公共服务的 403/429 不能误判成登录失效，见 [`is_session_expired`]。
pub const SESSION_EXPIRED: &str = "登录已失效：请重新登录（本工具不会自动使用旧 Token）";

/// 网络/传输层错误是否是认证失效。
pub fn is_session_expired(text: &str) -> bool {
    text.contains(SESSION_EXPIRED) || text.contains("HTTP 401")
}

/// 认证失效的判定只针对业务接口：HTTP 401，或服务端返回的业务码 401/10001。
fn auth_failure(http_status: u16) -> bool {
    http_status == 401
}

pub fn make_agent() -> ureq::Agent {
    ureq::AgentBuilder::new()
        .timeout(std::time::Duration::from_secs(30))
        .build()
}

pub struct ApiClient {
    pub agent: ureq::Agent,
    pub session: EnvelopeSession,
    pub identity: HeaderIdentity,
    pub login: Option<Session>,
}

pub struct RequestOutcome {
    pub http_status: u16,
    pub decrypted: Option<Decrypted>,
    pub raw_len: usize,
    /// 解密失败时的服务端原始返回摘要（诊断用）。
    pub raw_head: String,
}

impl ApiClient {
    pub fn new(identity: HeaderIdentity, login: Option<Session>) -> Self {
        Self {
            agent: make_agent(),
            session: EnvelopeSession::new(),
            identity,
            login,
        }
    }

    fn uid(&self) -> i64 {
        self.login.as_ref().map(|s| s.uid).unwrap_or(-1)
    }

    fn token(&self) -> String {
        self.login
            .as_ref()
            .map(|s| s.token.clone())
            .unwrap_or_default()
    }

    /// 发送信封请求（header/headerSign 用 observed 序，body 用 insert 序）。
    /// method 为 "POST" 或 "GET"（GET 同样携带信封 body）。
    pub fn envelope_request(
        &mut self,
        method: &str,
        url: &str,
        body_plain: &str,
        ua: &str,
        extra_headers: &[(String, String)],
    ) -> Result<RequestOutcome, String> {
        let uid = self.uid();
        let token = self.token();
        let (header_plain, _) = build_header_for(&self.identity, uid, &token, None);
        let header_env = build_envelope(&mut self.session, &header_plain, OuterOrder::Observed);
        // body 时间戳 = header ts + 1（原生两次独立读取毫秒）
        let body_env = build_envelope_ts(
            &mut self.session,
            body_plain,
            OuterOrder::Insert,
            header_env.ts_ms + 1,
        );

        let mut req = self.agent.request(method, url);
        req = req.set("Content-Type", "application/json; charset=utf-8");
        req = req.set("User-Agent", ua);
        req = req.set("appVersion", "7.3.40");
        req = req.set("headerSign", &header_env.json);
        for (k, v) in extra_headers {
            req = req.set(k, v);
        }
        let resp = match req.send_string(&body_env.json) {
            Ok(r) => r,
            // 401 是认证失效，不是网络故障：统一文案，便于上层识别并清会话。
            Err(ureq::Error::Status(401, _)) => {
                return Err(SESSION_EXPIRED.to_string());
            }
            Err(e) => return Err(ureq_err(e)),
        };
        let status = resp.status();
        let raw = resp.into_string().unwrap_or_default();
        let raw_len = raw.len();
        if auth_failure(status) {
            return Err(SESSION_EXPIRED.to_string());
        }
        let key = derive_paes_key(
            &body_env.key_data[0],
            &body_env.key_data[1],
            &body_env.key_data[2],
            &body_env.key_data[3],
        );
        let (decrypted, raw_head) = match decrypt_response(raw.as_bytes(), &key, &rsa_public_key())
        {
            Ok(d) => (Some(d), String::new()),
            Err(e) => (
                None,
                crate::textlog::redact_text(&format!(
                    "{e} | 原文: {}",
                    crate::textlog::truncate(&raw, 160)
                )),
            ),
        };
        Ok(RequestOutcome {
            http_status: status,
            decrypted,
            raw_len,
            raw_head,
        })
    }

    /// 标准业务请求（RUN 域名）：返回业务 JSON；error != 10000 时 Err。
    pub fn call(
        &mut self,
        method: &str,
        path: &str,
        body_plain: &str,
        extra_headers: &[(String, String)],
    ) -> Result<serde_json::Value, String> {
        self.call_host(
            crate::api::model::HOST,
            method,
            path,
            body_plain,
            extra_headers,
        )
    }

    /// 指定域名业务请求（排行榜/违规名单走 DISCOVERY，信封链相同）。
    /// 每次调用打印敏感字段打码后的解密响应日志。
    pub fn call_host(
        &mut self,
        host: &str,
        method: &str,
        path: &str,
        body_plain: &str,
        extra_headers: &[(String, String)],
    ) -> Result<serde_json::Value, String> {
        let url = format!("{}{}", host, path);
        let out = self.envelope_request(method, &url, body_plain, UA_IOS, extra_headers)?;
        let dec = out.decrypted.ok_or_else(|| {
            format!(
                "HTTP {} 响应解密失败（len={}）原文: {}",
                out.http_status, out.raw_len, out.raw_head
            )
        })?;
        let business = match check_business(&dec.business) {
            Ok(b) => b,
            Err(e) => {
                eprintln!(
                    "[resp] {} -> Err: {}",
                    path,
                    crate::textlog::redact_text(&e)
                );
                return Err(e);
            }
        };
        let biz_str = serde_json::to_string(&crate::textlog::redact_json(&business)).unwrap_or_default();
        eprintln!("[resp] {} -> {} ({}B) {}", path, out.http_status, out.raw_len, biz_str);
        Ok(business)
    }
}

/// 业务结果检查：error == 10000 → Ok(business)；否则 Err(描述)。
/// 校园跑不在服务端允许的时间段内。
///
/// 服务端在 `runModePolicy` 等接口用这个码拒绝时，说明当前时刻不能进行
/// **校园跑**。这不是网络故障、也不是程序错误，提示语必须区分开，
/// 否则用户会反复重试或以为软件坏了。
pub const RUN_TIME_WINDOW_CLOSED: &str =
    "校园跑当前不在允许的时间段内（服务端 11016）：请换到可跑时段，或改用其他跑步方式";

/// 业务错误是否为「校园跑时间段已过」。
pub fn is_run_time_window_closed(text: &str) -> bool {
    text.contains("11016") || text.contains(RUN_TIME_WINDOW_CLOSED)
}

pub fn check_business(v: &serde_json::Value) -> Result<serde_json::Value, String> {
    let err = v.get("error").and_then(|e| e.as_i64()).unwrap_or(0);
    if err == 10000 {
        Ok(v.clone())
    } else if matches!(err, 401 | 10001) {
        // 业务层认证失败，与 HTTP 401 归一到同一标记。
        Err(SESSION_EXPIRED.to_string())
    } else if err == 11016 {
        // 时间段限制：服务端明确拒绝，不是故障，给出可执行提示。
        Err(RUN_TIME_WINDOW_CLOSED.to_string())
    } else {
        let msg = v
            .get("message")
            .or_else(|| v.get("msg"))
            .and_then(|m| m.as_str())
            .unwrap_or("(无消息)");
        Err(crate::textlog::redact_text(&format!(
            "业务错误 {err}: {msg}"
        )))
    }
}

/// 业务字段查找：先顶层，再 data 子对象。
pub fn get_field<'a>(biz: &'a serde_json::Value, name: &str) -> Option<&'a serde_json::Value> {
    if let Some(v) = biz.get(name) {
        if !v.is_null() {
            return Some(v);
        }
    }
    biz.get("data").and_then(|d| d.get(name))
}

/// data 字段如果是 JSON 字符串则解析（Obs 一层包裹语义）。
pub fn parse_data_field(biz: &serde_json::Value) -> serde_json::Value {
    match biz.get("data") {
        Some(serde_json::Value::String(s)) => {
            serde_json::from_str(s).unwrap_or(serde_json::Value::Null)
        }
        Some(v) => v.clone(),
        None => serde_json::Value::Null,
    }
}

pub fn ureq_err(e: ureq::Error) -> String {
    match e {
        ureq::Error::Status(code, resp) => {
            let body = resp.into_string().unwrap_or_default();
            format!("HTTP {code}: {}", crate::textlog::redact_text(&crate::textlog::truncate(&body, 300)))
        }
        ureq::Error::Transport(t) => format!("网络错误: {t}"),
    }
}

#[cfg(test)]
mod tests {
    use super::{check_business, parse_data_field};

    #[test]
    fn parse_data_field_accepts_object_array_and_json_string() {
        let object = serde_json::json!({"data": {"ok": true}});
        assert_eq!(parse_data_field(&object)["ok"].as_bool(), Some(true));

        let array = serde_json::json!({"data": [1, 2, 3]});
        assert_eq!(parse_data_field(&array).as_array().unwrap().len(), 3);

        let wrapped = serde_json::json!({"data": "{\"records\":[{\"rrid\":7}]}"});
        assert_eq!(
            parse_data_field(&wrapped)["records"][0]["rrid"].as_i64(),
            Some(7)
        );
    }

    #[test]
    fn business_errors_are_rejected_and_success_is_preserved() {
        let ok = serde_json::json!({"error": 10000, "data": {"ok": true}});
        assert!(check_business(&ok).is_ok());
        let bad = serde_json::json!({"error": 10603, "message": "token=secret"});
        let error = check_business(&bad).unwrap_err();
        assert!(error.contains("token=***"));
        assert!(!error.contains("secret"));
    }

    #[test]
    fn auth_failures_map_to_one_marker_and_are_recognised() {
        for code in [401, 10001] {
            let v = serde_json::json!({ "error": code, "message": "expired" });
            let err = super::check_business(&v).unwrap_err();
            assert_eq!(err, super::SESSION_EXPIRED);
            assert!(super::is_session_expired(&err));
        }
        // 其他业务错误不能被误判成登录失效。
        assert!(!super::is_session_expired("业务错误 10603: 限流"));
        assert!(!super::is_session_expired("网络错误: timeout"));
    }

    /// 非认证 4xx（例如地图服务的 403/429）不能当成登录失效去清会话。
    #[test]
    fn only_401_counts_as_expired_session() {
        assert!(super::is_session_expired("HTTP 401: unauthorized"));
        assert!(!super::is_session_expired("HTTP 403: forbidden"));
        assert!(!super::is_session_expired("HTTP 429: too many requests"));
        assert!(!super::is_session_expired("HTTP 500: error"));
    }
}
