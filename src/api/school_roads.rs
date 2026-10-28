//! 按学校范围自动下载 OSM 路网。
//!
//! 特点（与手动导入互补）：
//! * 只按学校围栏（或真实点位）的外接矩形查询，**不发送账号/Token/学校编号**；
//! * 主服务超时或 5xx 时自动切官方 OSM 小范围接口，**不绕过限流**；
//! * 缓存按「学校编号 + 完整围栏」散列隔离，有效期 7 天，可强制刷新；
//! * 下载后先按围栏裁剪验证，围栏内一条路都没有就拒绝缓存。

use route_planner::RoadGraph;
use sha2::{Digest, Sha256};
use std::io::Read;
use std::path::Path;
use std::time::Duration;

const MAX_BYTES: u64 = 24 * 1024 * 1024;
const TTL_MS: i64 = 7 * 24 * 60 * 60 * 1000;
/// Overpass 主实例。官方实例经常满负载返回 502，因此备选多个镜像。
const ENDPOINTS: &[&str] = &[
    "https://overpass-api.de/api/interpreter",
    "https://overpass.kumi.systems/api/interpreter",
    "https://overpass.private.coffee/api/interpreter",
];
/// 最后才用的官方 OSM API：只支持很小的 bbox。
const BACKUP_ENDPOINT: &str = "https://api.openstreetmap.org/api/0.6/map";
const ENDPOINT: &str = ENDPOINTS[0];

/// 围栏必须是有效的闭合环，否则无法确定下载范围。
pub fn validate_fences(fences: &[Vec<(f64, f64)>]) -> Result<(), String> {
    if fences.is_empty() {
        return Err("当前学校没有有效围栏，无法确定路网范围".into());
    }
    for ring in fences {
        if ring.len() < 3
            || ring.iter().any(|&(lat, lon)| {
                !lat.is_finite() || !lon.is_finite() || lat.abs() > 85.0 || lon.abs() > 180.0
            })
        {
            return Err("学校围栏坐标无效".into());
        }
        let (oy, ox) = ring[0];
        let area: f64 = ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .map(|(&(y, x), &(v, u))| (x - ox) * (v - oy) - (u - ox) * (y - oy))
            .sum();
        if area.abs() < 1e-12 {
            return Err("学校围栏面积无效".into());
        }
    }
    Ok(())
}

/// BD-09 → WGS84（数值反解）。
///
/// 直接用正变换迭代逼近：`wgs84_to_bd09` 是解析式，反解没有闭式解，
/// 但迭代几次即可收敛到亚米级。
pub fn to_wgs(lat: f64, lon: f64) -> (f64, f64) {
    let (mut y, mut x) = (lat, lon);
    for _ in 0..16 {
        let (by, bx) = crate::track::geom::wgs84_to_bd09(y, x);
        y += lat - by;
        x += lon - bx;
    }
    (y, x)
}

/// 由围栏构造 Overpass 查询。
fn query(fences: &[Vec<(f64, f64)>]) -> Result<String, String> {
    validate_fences(fences)?;
    let pts: Vec<(f64, f64)> = fences.iter().flatten().copied().collect();
    query_coordinates(&pts, 0.001)
}

fn query_coordinates(points: &[(f64, f64)], margin: f64) -> Result<String, String> {
    if points.is_empty()
        || points.iter().any(|&(a, b)| {
            !a.is_finite() || !b.is_finite() || a.abs() > 85. || b.abs() > 180.
        })
    {
        return Err("缺少有效点位，不能确定路网下载范围".into());
    }
    let (mut south, mut west, mut north, mut east) = (90f64, 180f64, -90f64, -180f64);
    for &(lat, lon) in points {
        let (y, x) = to_wgs(lat, lon);
        south = south.min(y);
        north = north.max(y);
        west = west.min(x);
        east = east.max(x);
    }
    // 跨度异常大时拒绝下载：多半是点位串了城市或围栏数据有问题。
    if north - south > 0.15 || east - west > 0.2 {
        return Err("路网下载范围跨度过大，请确认当前学校和点位后重试".into());
    }
    let bbox = format!(
        "{:.7},{:.7},{:.7},{:.7}",
        south - margin,
        west - margin,
        north + margin,
        east + margin
    );
    Ok(format!(
        "[out:xml][timeout:25];(way[highway]({bbox});way[building]({bbox});\
         relation[building]({bbox}););(._;>;);out body;"
    ))
}

fn parse(bytes: &[u8]) -> Result<RoadGraph, String> {
    if bytes.len() as u64 > MAX_BYTES {
        return Err("路网响应超过 24 MiB".into());
    }
    let xml = std::str::from_utf8(bytes).map_err(|_| "路网不是有效 XML")?;
    if !xml.contains("<osm") || !xml.contains("</osm>") || xml.contains("<remark") {
        return Err("路网服务返回错误或不完整数据，请稍后重试".into());
    }
    let net = route_planner::load_osm(bytes)?;
    if net.graph.edge_count() == 0
        || net.graph.node_weights().any(|n| {
            !n.coord.lon.is_finite()
                || !n.coord.lat.is_finite()
                || n.coord.lon.abs() > 180.0
                || n.coord.lat.abs() > 90.0
        })
    {
        return Err("该区域没有可用的 OSM 道路数据，可手动导入完整路网".into());
    }
    Ok(net)
}

/// Overpass bbox（s,w,n,e）→ 官方 OSM API 的 bbox（w,s,e,n）。
fn official_bbox(query: &str) -> Result<String, String> {
    let raw = query
        .split_once("way[highway](")
        .and_then(|(_, tail)| tail.split_once(')').map(|v| v.0))
        .ok_or("此查询需使用备用搜索")?;
    let values: Vec<f64> = raw
        .split(',')
        .map(str::parse)
        .collect::<Result<_, _>>()
        .map_err(|_| "无效路网范围")?;
    if values.len() != 4 || values.iter().any(|v| !v.is_finite()) {
        return Err("无效路网范围".into());
    }
    let (s, w, n, e) = (values[0], values[1], values[2], values[3]);
    if s < -85. || n > 85. || w < -180. || e > 180. || s >= n || w >= e
        || (n - s) * (e - w) > 0.01
    {
        return Err("备用地图仅支持小范围校园查询".into());
    }
    Ok(format!("{w},{s},{e},{n}"))
}

/// 单次请求的失败分类。
///
/// 用类型而不是字符串嗅探来区分，保证 429 不可能被当成"换个端点再试"的信号。
#[derive(Debug, Clone, PartialEq, Eq)]
enum FetchError {
    /// 传输错误 / 超时 / 408 / 5xx：服务器只是暂时不可用，同一端点退避后可重试。
    Retryable(String),
    /// 429：服务器在限流。必须停手——既不在同一端点重试，也不换镜像。
    RateLimited(String),
    /// 其它 4xx / 响应过大 / 查询非法：重试和换端点都没用。
    Fatal(String),
}

impl FetchError {
    fn message(&self) -> &str {
        match self {
            Self::Retryable(m) | Self::RateLimited(m) | Self::Fatal(m) => m,
        }
    }
}

/// 把一次请求的 `ureq` 错误分成可重试 / 限流 / 致命三类。
fn fetch_error(err: ureq::Error) -> FetchError {
    let msg = format!("自动获取地图失败：{err}");
    match &err {
        ureq::Error::Transport(_) => FetchError::Retryable(msg),
        // 429 是限流：重试或换服务器继续打都是绕过限流，明确拒绝。
        ureq::Error::Status(code, _) if *code == 429 => {
            FetchError::RateLimited(format!("地图服务限流（HTTP 429），请稍后再试：{err}"))
        }
        // 502/503/504 是 Overpass 满载的典型表现，值得退避重试。
        ureq::Error::Status(code, _) if matches!(*code, 408 | 500 | 502 | 503 | 504) => {
            FetchError::Retryable(msg)
        }
        ureq::Error::Status(_, _) => FetchError::Fatal(msg),
    }
}

/// 单个端点上的一次请求。
fn fetch_once(agent: &ureq::Agent, endpoint: &str, query: &str) -> Result<Vec<u8>, FetchError> {
    let request = if endpoint == BACKUP_ENDPOINT {
        let bbox = official_bbox(query).map_err(FetchError::Fatal)?;
        agent
            .get(endpoint)
            .query("bbox", &bbox)
            .set("User-Agent", "NekoSportsWorldTool/0.4.1 (school-road-map)")
            .call()
    } else {
        agent
            .post(endpoint)
            .set("User-Agent", "NekoSportsWorldTool/0.4.1 (school-road-map)")
            .send_form(&[("data", query)])
    };
    let response = request.map_err(fetch_error)?;
    let mut bytes = Vec::new();
    response
        .into_reader()
        .take(MAX_BYTES + 1)
        .read_to_end(&mut bytes)
        .map_err(|e| FetchError::Retryable(format!("读取地图失败：{e}")))?;
    if bytes.len() as u64 > MAX_BYTES {
        return Err(FetchError::Fatal("地图响应超过 24 MiB".into()));
    }
    Ok(bytes)
}

/// 生产环境的退避等待序列（秒）：0 → 3 → 8，总耗时可控在 11 秒左右。
const BACKOFF_WAITS: &[u64] = &[0, 3, 8];

/// 在**同一个**端点上按退避序列重试（Overpass 官方要求）。
///
/// 只有 `Retryable` 才会重试；`RateLimited`（429）由调用方当作全局停止信号；
/// `Fatal` 也不在本端点重试，但允许换端点——各端点能力不同（Overpass 与官方
/// API 的 bbox 限制就不一样），一个端点报 400 不代表请求本身非法。
/// `wait_s` 单独传参，测试可用全 0 序列避免真的 sleep。
fn with_backoff(
    wait_s: &[u64],
    fetch: &mut dyn FnMut() -> Result<Vec<u8>, FetchError>,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<u8>, FetchError> {
    let mut last: Option<FetchError> = None;
    for (attempt, wait) in wait_s.iter().enumerate() {
        if *wait > 0 {
            log(&format!(
                "[road] 地图服务繁忙，{} 秒后第 {} 次重试…",
                wait,
                attempt + 1
            ));
            std::thread::sleep(Duration::from_secs(*wait));
        }
        match fetch() {
            Ok(bytes) => return Ok(bytes),
            Err(e @ (FetchError::RateLimited(_) | FetchError::Fatal(_))) => return Err(e),
            Err(e) => last = Some(e),
        }
    }
    Err(last.unwrap_or_else(|| FetchError::Fatal("未尝试任何地图服务".into())))
}

/// 依次尝试各镜像，最后才用官方 OSM API。
///
/// **只有限流（429）是全局停止信号**：立刻返回，不再打任何端点。
/// 其余失败（5xx/传输错误/4xx）换下一个端点，全部失败才报"均不可用"。
/// 全部端点都报 `Fatal` 时错误消息会点明"请求可能非法"——用户该做的事
/// （换个学校/点位 vs. 过一会儿重试）完全不同，不能笼统报同一句话。
/// `fetch_one` 注入是为了让测试能在不联网、不 sleep 的情况下验证调用次数。
fn fetch_with_failover(
    endpoints: &[&str],
    backup: &str,
    fetch_one: &mut dyn FnMut(&str, &mut dyn FnMut(&str)) -> Result<Vec<u8>, FetchError>,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<u8>, String> {
    let mut failures: Vec<String> = Vec::new();
    let mut all_fatal = true;
    for endpoint in endpoints {
        match fetch_one(endpoint, log) {
            Ok(bytes) => return Ok(bytes),
            Err(FetchError::RateLimited(msg)) => {
                // 换镜像继续打就是绕过限流，这里明确拒绝。
                log(&format!("[road] {endpoint} 限流，停止重试"));
                return Err(msg);
            }
            Err(e @ FetchError::Fatal(_)) => {
                // 记下是 Fatal：排查时要能区分"请求本身有问题"和"这个端点不支持"。
                log(&format!(
                    "[road] {endpoint} 拒绝了本次请求（可能该端点不支持此查询）：{}",
                    e.message()
                ));
                failures.push(format!("{endpoint}: {}", e.message()));
            }
            Err(e) => {
                all_fatal = false;
                log(&format!("[road] {endpoint} 不可用：{}", e.message()));
                failures.push(format!("{endpoint}: {}", e.message()));
            }
        }
    }
    // 所有镜像都不可用：退到官方 OSM API（仅小范围）。
    match fetch_one(backup, log) {
        Ok(bytes) => Ok(bytes),
        Err(FetchError::RateLimited(msg)) => {
            log(&format!("[road] {backup} 限流，停止重试"));
            Err(msg)
        }
        Err(e) => {
            if !matches!(e, FetchError::Fatal(_)) {
                all_fatal = false;
            }
            failures.push(format!("{backup}: {}", e.message()));
            let reason = if all_fatal {
                // 每个端点都认为请求非法：重试没用，得换学校/点位/围栏。
                "所有地图服务都拒绝了本次请求，请求可能非法（请确认当前学校和围栏/点位后重试）"
            } else {
                "所有地图服务均不可用"
            };
            Err(format!("{reason}：{}", failures.join("；")))
        }
    }
}

/// 无日志版本（校园边界搜索等后台任务用）。
pub(crate) fn download(query: &str) -> Result<Vec<u8>, String> {
    download_logged(query, &mut |_| {})
}

/// 带日志的下载（UI 线程用），能把退避过程反馈给用户。
pub fn download_logged(query: &str, log: &mut dyn FnMut(&str)) -> Result<Vec<u8>, String> {
    let agent = ureq::AgentBuilder::new()
        .timeout_connect(Duration::from_secs(10))
        .timeout(Duration::from_secs(60))
        .build();
    fetch_with_failover(
        ENDPOINTS,
        BACKUP_ENDPOINT,
        &mut |endpoint, log| {
            with_backoff(
                BACKOFF_WAITS,
                &mut || fetch_once(&agent, endpoint, query),
                log,
            )
        },
        log,
    )
}

#[derive(serde::Serialize, serde::Deserialize)]
struct Cache {
    key: String,
    loaded_ms: i64,
    xml: String,
    #[serde(default)]
    school: String,
    #[serde(default)]
    name: String,
    #[serde(default)]
    fences: Vec<Vec<(f64, f64)>>,
    #[serde(default)]
    points: Vec<(f64, f64)>,
}

/// 缓存条目（供路网页列表展示）。
#[derive(Clone, Debug)]
pub struct Entry {
    pub key: String,
    pub school: String,
    pub name: String,
    pub loaded_ms: i64,
    pub bytes: u64,
    pub fences: Vec<Vec<(f64, f64)>>,
    pub points: Vec<(f64, f64)>,
}

/// 按「学校编号 + 围栏几何」生成缓存键（约厘米精度，跨 JSON 往返稳定）。
pub fn scope_key(school: &str, fences: &[Vec<(f64, f64)>]) -> String {
    let normalized: Vec<Vec<(i64, i64)>> = fences
        .iter()
        .map(|f| {
            f.iter()
                .map(|&(a, b)| ((a * 1e7).round() as i64, (b * 1e7).round() as i64))
                .collect()
        })
        .collect();
    // 这里的元素是 i64，序列化在结构上不会失败；用 unwrap_or_default 而不是
    // unwrap：后台线程 panic 会让界面的 busy 永久卡死，代价远大于一次哈希落空。
    format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&(school, normalized)).unwrap_or_default())
    )
}

/// [`load_for_route`] 会使用的缓存键（与写缓存时完全一致）。
///
/// 界面在自动获取成功后需要按这个键把缓存导出成文件（见 [`materialize`]），
/// 以便跑步主流程能通过 `config.osm_path` 读到同一份路网。
pub fn route_cache_key(school: &str, fences: &[Vec<(f64, f64)>], points: &[(f64, f64)]) -> String {
    route_scope_key(school, fences, points)
}

fn route_scope_key(school: &str, fences: &[Vec<(f64, f64)>], points: &[(f64, f64)]) -> String {
    if !fences.is_empty() {
        return scope_key(school, fences);
    }
    let normalized: Vec<_> = points
        .iter()
        .map(|&(a, b)| ((a * 1e7).round() as i64, (b * 1e7).round() as i64))
        .collect();
    format!(
        "{:x}",
        Sha256::digest(serde_json::to_vec(&("points-v1", school, normalized)).unwrap_or_default())
    )
}

fn cache_dir() -> std::path::PathBuf {
    crate::platform::data_dir().join("school_roads")
}

fn cache_path(key: &str) -> Result<std::path::PathBuf, String> {
    if key.len() != 64 || !key.bytes().all(|b| b.is_ascii_hexdigit()) {
        return Err("无效缓存标识".into());
    }
    Ok(cache_dir().join(format!("{key}.json")))
}

/// 列出全部缓存（按下载时间倒序）。
pub fn entries() -> Vec<Entry> {
    let Ok(files) = std::fs::read_dir(cache_dir()) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for f in files.flatten() {
        let Some(key) = f
            .path()
            .file_stem()
            .and_then(|s| s.to_str())
            .map(str::to_owned)
        else {
            continue;
        };
        if let Ok((cache, bytes)) = read_cache_record(&key) {
            out.push(cache_entry(&cache, bytes));
        }
    }
    out.sort_by(|a, b| b.loaded_ms.cmp(&a.loaded_ms));
    out
}

fn read_cache_record(key: &str) -> Result<(Cache, u64), String> {
    let path = cache_path(key)?;
    let meta = std::fs::metadata(&path).map_err(|e| e.to_string())?;
    if meta.len() > MAX_BYTES * 2 {
        return Err("缓存过大".into());
    }
    let cache: Cache =
        serde_json::from_slice(&std::fs::read(&path).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    if cache.key != key {
        return Err("缓存标识不一致".into());
    }
    Ok((cache, meta.len()))
}

fn cache_entry(c: &Cache, bytes: u64) -> Entry {
    Entry {
        key: c.key.clone(),
        school: c.school.clone(),
        name: c.name.clone(),
        loaded_ms: c.loaded_ms,
        bytes,
        fences: c.fences.clone(),
        points: c.points.clone(),
    }
}

pub fn read_cache(key: &str) -> Result<(RoadGraph, Entry), String> {
    let (c, bytes) = read_cache_record(key)?;
    Ok((parse(c.xml.as_bytes())?, cache_entry(&c, bytes)))
}

pub fn clear_cache(key: &str) -> Result<(), String> {
    std::fs::remove_file(cache_path(key)?).map_err(|e| e.to_string())
}

/// 导出文件名前缀。完整名带缓存键前 8 位（见 [`materialized_name`]），
/// 这样多个学校的路网可以同时存在而不会互相覆盖。
pub const MATERIALIZED_PREFIX: &str = "school_roads_";

/// 旧版本的固定导出名，仅用于兼容读取。
pub const LEGACY_MATERIALIZED_NAME: &str = "school_roads_active.osm";

/// 某份缓存对应的导出文件名。
pub fn materialized_name(key: &str) -> String {
    let short: String = key.chars().take(8).collect();
    format!("{MATERIALIZED_PREFIX}{short}.osm")
}

/// 把某份缓存导出成可被 `track::generate_road::load_network_path` 读取的 OSM 文件。
///
/// 跑步主流程（`api::flow`）只认 `config.osm_path` 指向的本地文件，而自动获取 /
/// 载入缓存只把 `RoadGraph` 交给界面预览。这里补上「落成文件」这一环，
/// 否则用户在路网页看到路网加载成功、跑步时却静默回退经典算法。
///
/// * 内容是 `Cache.xml` 原文（WGS84 原始 OSM）**不做坐标对齐**——
///   `load_network_path` 会自己转 BD-09，提前转会偏约 950 米。
/// * 先写同目录 `.tmp` 再 rename：中途失败时旧文件保持原样，不会留下半截 XML。
/// * 成功后删掉其它学校的导出文件：同一时刻只会有「当前路网」一份，
///   否则每换一次学校就在数据目录里堆一份 MB 级文件。
pub fn materialize(key: &str) -> Result<std::path::PathBuf, String> {
    let (cache, _) = read_cache_record(key)?;
    materialize_xml(key, &cache.xml)
}

/// `materialize` 的落盘部分，便于测试直接喂 XML 而不必先造缓存记录。
fn materialize_xml(key: &str, xml: &str) -> Result<std::path::PathBuf, String> {
    let dir = crate::platform::data_dir();
    std::fs::create_dir_all(&dir).map_err(|e| format!("无法建立路网导出目录：{e}"))?;
    let target = dir.join(materialized_name(key));
    // 临时名带进程号，避免两个实例同时导出时互相覆盖。
    let tmp = dir.join(format!(
        ".{}.{}.tmp",
        materialized_name(key),
        std::process::id()
    ));
    std::fs::write(&tmp, xml).map_err(|e| format!("导出路网失败：{e}"))?;
    std::fs::rename(&tmp, &target).map_err(|e| {
        let _ = std::fs::remove_file(&tmp);
        format!("导出路网失败：{e}")
    })?;
    prune_materialized(&target);
    Ok(target)
}

/// 删掉除 `keep` 以外的所有导出文件（含旧版固定名）。
///
/// 失败不影响本次导出——最坏只是磁盘上多留几份文件。
fn prune_materialized(keep: &Path) {
    let Some(dir) = keep.parent() else {
        return;
    };
    let Ok(files) = std::fs::read_dir(dir) else {
        return;
    };
    for f in files.flatten() {
        let path = f.path();
        if path == keep || !path.is_file() {
            continue;
        }
        let Some(name) = path.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        // 只清自己产生的文件，不碰数据目录里的 config.json 等。
        let is_legacy = name == LEGACY_MATERIALIZED_NAME;
        let is_generated = name.starts_with(MATERIALIZED_PREFIX)
            && name.ends_with(".osm")
            && name.len() == MATERIALIZED_PREFIX.len() + 8 + 4;
        if is_legacy || is_generated {
            let _ = std::fs::remove_file(&path);
        }
    }
}

/// 指定缓存对应的导出文件（若存在）。
pub fn materialized_path_for(key: &str) -> Option<std::path::PathBuf> {
    let path = crate::platform::data_dir().join(materialized_name(key));
    path.is_file().then_some(path)
}

/// 当前已导出的路网文件（若存在）。
///
/// 优先返回任意一份带 key 前缀的导出；都没有时回退旧版固定名，
/// 保证升级前导出过的用户不会忽然找不到文件。
pub fn materialized_path() -> Option<std::path::PathBuf> {
    let dir = crate::platform::data_dir();
    let legacy = dir.join(LEGACY_MATERIALIZED_NAME);
    if let Ok(files) = std::fs::read_dir(&dir) {
        for f in files.flatten() {
            let path = f.path();
            let name = path.file_name().and_then(|s| s.to_str()).unwrap_or_default();
            if name.starts_with(MATERIALIZED_PREFIX)
                && name.ends_with(".osm")
                && path.is_file()
            {
                return Some(path);
            }
        }
    }
    legacy.is_file().then_some(legacy)
}

/// 回填缓存的学校名/点位（下载时可能还不知道）。
pub fn save_context(school: &str, name: &str, fences: &[Vec<(f64, f64)>], points: &[(f64, f64)]) {
    let key = route_scope_key(school, fences, points);
    let Ok(path) = cache_path(&key) else {
        return;
    };
    let Ok(bytes) = std::fs::read(&path) else {
        return;
    };
    if let Ok(mut cache) = serde_json::from_slice::<Cache>(&bytes) {
        cache.name = name.into();
        cache.points = points.to_vec();
        if let Ok(bytes) = serde_json::to_vec(&cache) {
            let _ = std::fs::write(path, bytes);
        }
    }
}

/// 测试专用：把私有 [`cached_route_network`] 暴露给同 crate 的测试模块。
///
/// 生产路径不要调用——正式入口是 [`load_for_school`] / [`load_for_route`]。
/// 存在意义是测试能注入假 `fetch`，否则「缓存未写入」这类断言必须真联网。
#[cfg(test)]
pub(crate) fn cached_route_network_for_test(
    dir: &Path,
    school: &str,
    fences: &[Vec<(f64, f64)>],
    points: &[(f64, f64)],
    refresh: bool,
    now: i64,
    fetch: impl FnOnce(&str) -> Result<Vec<u8>, String>,
    log: &mut dyn FnMut(&str),
) -> Result<RoadGraph, String> {
    cached_route_network(dir, school, fences, points, refresh, now, fetch, log)
}

fn cached_route_network(
    dir: &Path,
    school: &str,
    fences: &[Vec<(f64, f64)>],
    points: &[(f64, f64)],
    refresh: bool,
    now: i64,
    fetch: impl FnOnce(&str) -> Result<Vec<u8>, String>,
    log: &mut dyn FnMut(&str),
) -> Result<RoadGraph, String> {
    if school.trim().is_empty() {
        return Err("登录信息缺少学校编号，请重新登录".into());
    }
    let q = if fences.is_empty() {
        query_coordinates(points, 0.002)?
    } else {
        query(fences)?
    };
    let key = route_scope_key(school, fences, points);
    let file = dir.join(format!("{key}.json"));
    if !refresh
        && std::fs::metadata(&file)
            .map(|m| m.len() < MAX_BYTES * 2)
            .unwrap_or(false)
    {
        if let Ok(text) = std::fs::read_to_string(&file) {
            if let Ok(cache) = serde_json::from_str::<Cache>(&text) {
                if cache.key == key && (0..TTL_MS).contains(&(now - cache.loaded_ms)) {
                    if let Ok(net) = parse(cache.xml.as_bytes()) {
                        log("[road] 使用当前学校对应的路网缓存");
                        return Ok(net);
                    }
                }
            }
        }
    }
    log("[road] 正在下载当前学校区域的 OpenStreetMap 路网…");
    let bytes = fetch(&q)?;
    let net = parse(&bytes)?;
    // 围栏内没有路时拒绝缓存：否则下次会拿一个空路网继续跑。
    if !fences.is_empty() {
        let mut aligned = net.clone();
        crate::track::generate_road::align_network(&mut aligned);
        if crate::track::generate_road::apply_fences(&aligned, fences)?.graph.edge_count() == 0 {
            return Err("下载的路网在学校围栏内没有可用道路".into());
        }
    }
    let cache = Cache {
        key,
        loaded_ms: now,
        xml: String::from_utf8(bytes).map_err(|e| e.to_string())?,
        school: school.into(),
        name: String::new(),
        fences: fences.to_vec(),
        points: points.to_vec(),
    };
    // 缓存是可选能力：只读目录下不能因为写缓存失败而让本次下载作废。
    let saved = (|| -> Result<(), String> {
        std::fs::create_dir_all(dir).map_err(|e| e.to_string())?;
        std::fs::write(&file, serde_json::to_vec(&cache).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        Ok(())
    })();
    if let Err(e) = saved {
        log(&format!("[road] 本次路网可用，但缓存未保存：{e}"));
    }
    Ok(net)
}

/// 按学校围栏自动获取路网（`path` 非空时直接用本地文件）。
pub fn load_for_school(
    path: &str,
    school: &str,
    fences: &[Vec<(f64, f64)>],
    refresh: bool,
    control: &crate::task_control::Control,
    log: &mut dyn FnMut(&str),
) -> Result<RoadGraph, String> {
    control.check()?;
    if !path.trim().is_empty() {
        let net = crate::track::generate_road::load_network_path(path)?;
        control.check()?;
        return Ok(net);
    }
    validate_fences(fences)?;
    cached_route_network(
        &cache_dir(),
        school,
        fences,
        &[],
        refresh,
        crate::crypto::envelope::now_ms(),
        |q| {
            control.check()?;
            // 闭包不能同时借用 log（cached_route_network 已经持有它），
            // 因此把退避进度收集到局部 Vec，下载结束后再回放。
            let mut lines: Vec<String> = Vec::new();
            let bytes = download_logged(q, &mut |l| lines.push(l.to_string()))?;
            for l in lines {
                eprintln!("{l}");
            }
            control.check()?;
            Ok(bytes)
        },
        log,
    )
}

/// 没有服务端围栏时，按真实打卡点的外接范围下载（该范围**不是**学校边界）。
pub fn load_for_route(
    path: &str,
    school: &str,
    fences: &[Vec<(f64, f64)>],
    points: &[(f64, f64)],
    refresh: bool,
    control: &crate::task_control::Control,
    log: &mut dyn FnMut(&str),
) -> Result<RoadGraph, String> {
    if !path.trim().is_empty() || !fences.is_empty() {
        return load_for_school(path, school, fences, refresh, control, log);
    }
    control.check()?;
    log("[road] 无学校围栏：按真实点位获取附近道路；下载范围不是学校边界");
    cached_route_network(
        &cache_dir(),
        school,
        fences,
        points,
        refresh,
        crate::crypto::envelope::now_ms(),
        |q| {
            control.check()?;
            // 闭包不能同时借用 log（cached_route_network 已经持有它），
            // 因此把退避进度收集到局部 Vec，下载结束后再回放。
            let mut lines: Vec<String> = Vec::new();
            let bytes = download_logged(q, &mut |l| lines.push(l.to_string()))?;
            for l in lines {
                eprintln!("{l}");
            }
            control.check()?;
            Ok(bytes)
        },
        log,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_query_converts_coordinate_order_and_rejects_bad_extents() {
        assert_eq!(
            official_bbox("way[highway](23.20,112.57,23.22,112.59)").unwrap(),
            "112.57,23.2,112.59,23.22"
        );
        for q in [
            "way[name=test]",
            "way[highway](0,0,5,5)",
            "way[highway](NaN,0,1,1)",
            "way[highway](2,0,1,1)",
        ] {
            assert!(official_bbox(q).is_err(), "{q} 应被拒绝");
        }
    }

    /// `fetch_error` 的分类是整个限流逻辑的根，必须逐类验证。
    #[test]
    fn http_status_errors_are_classified_by_code() {
        let status = |code: u16| {
            ureq::Error::Status(code, ureq::Response::new(code, "status", "").unwrap())
        };
        assert!(matches!(
            fetch_error(status(429)),
            FetchError::RateLimited(_)
        ));
        for code in [408, 500, 502, 503, 504] {
            assert!(
                matches!(fetch_error(status(code)), FetchError::Retryable(_)),
                "HTTP {code} 应可重试"
            );
        }
        for code in [400, 401, 403, 404] {
            assert!(
                matches!(fetch_error(status(code)), FetchError::Fatal(_)),
                "HTTP {code} 不应重试"
            );
        }
        // ureq 的传输错误（DNS/连接中断等）永远值得重试。
        let transport: ureq::Error =
            std::io::Error::new(std::io::ErrorKind::ConnectionRefused, "boom").into();
        assert!(matches!(fetch_error(transport), FetchError::Retryable(_)));
    }

    /// 429 是全局停止信号：同端点不重试，也不换镜像。
    #[test]
    fn rate_limit_stops_the_whole_download_at_the_first_endpoint() {
        let mut attempts = 0;
        let mut endpoints: Vec<String> = vec![];
        let err = fetch_with_failover(
            ENDPOINTS,
            BACKUP_ENDPOINT,
            &mut |endpoint, log| {
                endpoints.push(endpoint.to_owned());
                with_backoff(
                    &[0, 0, 0],
                    &mut || {
                        attempts += 1;
                        Err(FetchError::RateLimited("HTTP 429".into()))
                    },
                    log,
                )
            },
            &mut |_| {},
        )
        .unwrap_err();
        assert_eq!(attempts, 1, "429 后不得在同一端点上退避重试");
        assert_eq!(
            endpoints,
            vec![ENDPOINT.to_owned()],
            "429 后不得换镜像或退到官方 API"
        );
        assert!(err.contains("429"));
    }

    /// 502 是 Overpass 满载：先在同一个端点上退避重试，用尽才换下一个端点。
    #[test]
    fn server_errors_retry_on_the_same_endpoint_before_switching() {
        // 记录每一次真实请求（含同端点退避重试），而不只是每个端点被进入一次。
        let mut attempts: Vec<String> = vec![];
        let bytes = fetch_with_failover(
            ENDPOINTS,
            BACKUP_ENDPOINT,
            &mut |endpoint, log| {
                // 主镜像始终 502；第二个镜像第一次 502、第二次成功。
                let mut hit: u32 = 0;
                with_backoff(
                    &[0, 0, 0],
                    &mut || {
                        attempts.push(endpoint.to_owned());
                        hit += 1;
                        if endpoint == ENDPOINT || hit == 1 {
                            Err(FetchError::Retryable("HTTP 502".into()))
                        } else {
                            Ok(b"<osm/>".to_vec())
                        }
                    },
                    log,
                )
            },
            &mut |_| {},
        )
        .unwrap();
        assert_eq!(bytes, b"<osm/>");
        assert_eq!(
            attempts,
            [ENDPOINT, ENDPOINT, ENDPOINT, ENDPOINTS[1], ENDPOINTS[1]]
                .map(str::to_owned)
                .to_vec(),
            "主镜像应重试 3 次后才换；第二个镜像第一次失败、第二次成功"
        );
    }

    /// 传输错误在所有端点上都会退避重试，且不会把结局变成"不可重试"。
    #[test]
    fn transport_errors_are_retried_and_stay_retryable() {
        let mut attempts = 0;
        let err = with_backoff(
            &[0, 0, 0],
            &mut || {
                attempts += 1;
                Err(FetchError::Retryable("连接中断".into()))
            },
            &mut |_| {},
        )
        .unwrap_err();
        assert_eq!(attempts, 3);
        assert!(matches!(err, FetchError::Retryable(_)));
    }

    #[test]
    fn primary_success_never_calls_other_endpoints() {
        let mut calls = vec![];
        let bytes = fetch_with_failover(
            ENDPOINTS,
            BACKUP_ENDPOINT,
            &mut |endpoint, _| {
                calls.push(endpoint.to_owned());
                Ok(b"<osm/>".to_vec())
            },
            &mut |_| {},
        )
        .unwrap();
        assert_eq!(bytes, b"<osm/>");
        assert_eq!(calls, vec![ENDPOINT]);
    }

    /// 所有端点都报 Fatal 时，错误消息要点明"请求可能非法"，而不是笼统的"均不可用"。
    #[test]
    fn all_fatal_failures_say_the_request_may_be_invalid() {
        let err = fetch_with_failover(
            ENDPOINTS,
            BACKUP_ENDPOINT,
            &mut |_, _| Err(FetchError::Fatal("HTTP 400".into())),
            &mut |_| {},
        )
        .unwrap_err();
        assert!(err.contains("请求可能非法"), "{err}");

        // 只要有一个端点是暂时性故障，就还是"均不可用"——用户该做的是稍后重试。
        let err = fetch_with_failover(
            ENDPOINTS,
            BACKUP_ENDPOINT,
            &mut |endpoint, _| {
                if endpoint == BACKUP_ENDPOINT {
                    Err(FetchError::Retryable("连接超时".into()))
                } else {
                    Err(FetchError::Fatal("HTTP 400".into()))
                }
            },
            &mut |_| {},
        )
        .unwrap_err();
        assert!(err.contains("均不可用"), "{err}");
        assert!(!err.contains("请求可能非法"), "{err}");
    }

    /// 所有镜像都失败后才退到官方 API；每个端点只试一次，不做循环重试。
    #[test]
    fn backup_is_only_tried_after_every_mirror_fails() {
        let mut calls = vec![];
        let err = fetch_with_failover(
            ENDPOINTS,
            BACKUP_ENDPOINT,
            &mut |endpoint, _| {
                calls.push(endpoint.to_owned());
                Err(FetchError::Fatal("HTTP 400".into()))
            },
            &mut |_| {},
        )
        .unwrap_err();
        let mut expected: Vec<String> = ENDPOINTS.iter().map(|e| e.to_string()).collect();
        expected.push(BACKUP_ENDPOINT.to_owned());
        assert_eq!(calls, expected);
        assert!(err.contains(ENDPOINT) && err.contains(BACKUP_ENDPOINT));
    }

    #[test]
    fn invalid_fences_are_rejected() {
        assert!(validate_fences(&[]).is_err());
        assert!(validate_fences(&[vec![(39.9, 116.4), (39.9, 116.4), (39.9, 116.4)]]).is_err());
    }

    #[test]
    fn scope_key_is_stable_and_depends_on_geometry() {
        let a = vec![vec![(39.9, 116.4), (39.9, 116.5), (40.0, 116.5), (40.0, 116.4)]];
        let b = vec![vec![(39.9, 116.4), (39.9, 116.6), (40.0, 116.6), (40.0, 116.4)]];
        assert_eq!(scope_key("1001", &a), scope_key("1001", &a));
        assert_ne!(scope_key("1001", &a), scope_key("1001", &b));
        assert_ne!(scope_key("1001", &a), scope_key("1002", &a));
    }

    /// 范围过大的查询必须在下载前被拒绝，避免误拉整个城市。
    #[test]
    fn oversized_extent_is_rejected_before_download() {
        let wide = vec![(39.0, 116.0), (40.0, 118.0)];
        assert!(query_coordinates(&wide, 0.001).is_err());
    }

    /// 临时数据目录：保证测试不碰真实用户的 school_roads 缓存。
    ///
    /// 返回的守卫在 Drop 时清除 `TEST_DATA_DIR` 并删目录，断言失败也不会污染后续测试。
    struct TempDataDir(std::path::PathBuf);
    impl TempDataDir {
        fn new(tag: &str) -> Self {
            let dir = std::env::temp_dir().join(format!("school-roads-{tag}-{}", uuid::Uuid::new_v4()));
            std::fs::create_dir_all(&dir).unwrap();
            crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = Some(dir.clone()));
            Self(dir)
        }
    }
    impl Drop for TempDataDir {
        fn drop(&mut self) {
            crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = None);
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// 缓存目录不存在（从未下载过）时，列表必须是空的而不是报错。
    #[test]
    fn entries_returns_empty_when_cache_dir_is_missing() {
        let _dir = TempDataDir::new("missing");
        assert!(!cache_dir().exists());
        assert!(entries().is_empty());
    }

    /// 缓存目录存在但没有文件时同样是空的。
    #[test]
    fn entries_returns_empty_when_cache_dir_is_empty() {
        let _dir = TempDataDir::new("empty");
        std::fs::create_dir_all(cache_dir()).unwrap();
        assert!(entries().is_empty());
    }

    /// 经 `materialize()` 落盘的缓存必须能被跑步主流程原样读出。
    ///
    /// 这是 N1 的回归：自动获取 / 载入缓存只把 RoadGraph 交给预览，
    /// 而 `api::flow` 只认 `config.osm_path` 指向的文件。落盘格式（WGS84 原始
    /// XML，不预先对齐）必须与 `load_network_path` 的预期一致。
    #[test]
    fn materialize_writes_a_file_the_run_flow_can_load() {
        let _dir = TempDataDir::new("materialize");
        let xml = std::fs::read_to_string(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("crates/route-planner/tests/fixtures/dinghu-roads.osm"),
        )
        .unwrap();
        let key = "a".repeat(64);
        let cache = Cache {
            key: key.clone(),
            loaded_ms: crate::crypto::envelope::now_ms(),
            xml: xml.clone(),
            school: "1001".into(),
            name: String::new(),
            fences: vec![],
            points: vec![],
        };
        std::fs::create_dir_all(cache_dir()).unwrap();
        std::fs::write(
            cache_path(&key).unwrap(),
            serde_json::to_vec(&cache).unwrap(),
        )
        .unwrap();

        let path = materialize(&key).expect("导出应成功");
        assert_eq!(
            path.file_name().unwrap().to_str().unwrap(),
            materialized_name(&key)
        );
        assert!(path.is_file(), "导出文件不存在: {}", path.display());
        // 内容必须是原始 XML 逐字节一致：任何重写都会破坏 OSM 结构。
        assert_eq!(std::fs::read_to_string(&path).unwrap(), xml);

        // 走真实入口：flow 用 load_network_path 读 config.osm_path。
        let net = crate::track::generate_road::load_network_path(path.to_str().unwrap())
            .expect("load_network_path 应能加载导出文件");
        assert!(net.graph.edge_count() > 0, "导出的路网没有边");
        assert_eq!(materialized_path_for(&key).as_deref(), Some(path.as_path()));
    }

    /// 原子写：导出中途失败时，上一次成功导出的文件必须原样保留。
    ///
    /// 用「临时文件位置被同名目录占住」制造写失败——比权限位更可移植。
    #[test]
    fn failed_materialize_leaves_the_previous_file_intact() {
        let _dir = TempDataDir::new("atomic");
        let key = "b".repeat(64);
        let first = materialize_xml(&key, "<osm><way id=\"1\"/></osm>").expect("首次导出应成功");
        let before = std::fs::read_to_string(&first).unwrap();

        // 让临时路径写不进去：该路径上放一个目录。
        let blocker = crate::platform::data_dir().join(format!(
            ".{}.{}.tmp",
            materialized_name(&key),
            std::process::id()
        ));
        std::fs::create_dir_all(&blocker).unwrap();
        let err = materialize_xml(&key, "<osm><way id=\"2\"/></osm>").unwrap_err();
        assert!(err.contains("导出"), "错误信息应说明导出失败: {err}");

        assert_eq!(
            std::fs::read_to_string(&first).unwrap(),
            before,
            "导出失败后旧文件被改动"
        );
        // 失败路径不应留下 `.tmp` 目录之外的碎片。
        let _ = std::fs::remove_dir_all(&blocker);
        assert!(first.is_file());
    }

    /// 取消必须在**发出任何网络请求之前**生效：否则用户点了取消还得等下载结束。
    #[test]
    fn cancelled_download_fails_before_touching_the_network() {
        let mut log = |_: &str| {};
        let control = crate::task_control::Control::default();
        control.cancel();
        // 有围栏走 load_for_school，无围栏走 load_for_route；两条入口都要拦住。
        let fences = vec![vec![(31.5, 104.6), (31.5, 104.7), (31.6, 104.7)]];
        // RoadGraph 没有 Debug，不能 unwrap_err()；用 err() 取出错误串。
        let e = match load_for_school("", "1001", &fences, false, &control, &mut log) {
            Err(e) => e,
            Ok(_) => panic!("已取消仍继续下载（围栏路径）"),
        };
        assert!(e.contains("取消"), "取消错误信息不符: {e}");

        let points = [(31.526444, 104.675651)];
        let e = match load_for_route("", "1001", &[], &points, false, &control, &mut log) {
            Err(e) => e,
            Ok(_) => panic!("已取消仍继续下载（点位路径）"),
        };
        assert!(e.contains("取消"), "取消错误信息不符: {e}");
    }

    /// 非法坐标（NaN/Inf）不能让缓存键计算 panic。
    ///
    /// 后台线程一旦 panic，界面拿不到结果，`busy` 会永久为真、按钮全部禁用。
    #[test]
    fn cache_keys_tolerate_non_finite_coordinates() {
        let bad_fences = vec![vec![
            (f64::NAN, 104.6),
            (31.5, f64::INFINITY),
            (31.6, 104.7),
        ]];
        let bad_points = [(f64::NAN, 104.675), (31.5, f64::NEG_INFINITY)];
        // 只要不 panic 即可；结果是否稳定可复现同样要保证。
        let a = route_scope_key("1001", &bad_fences, &[]);
        let b = route_scope_key("1001", &bad_fences, &[]);
        assert_eq!(a, b, "非有限坐标下缓存键应先稳定");
        assert_eq!(a.len(), 64);
        let p = route_scope_key("1001", &[], &bad_points);
        assert_eq!(p.len(), 64);
    }

    /// A 校导出后切到 B 校：B 的文件在、A 的文件被清掉，不会堆满数据目录。
    ///
    /// 这是多校覆盖场景的回归：固定文件名会让 B 校跑步时用上 A 校的路网。
    #[test]
    fn switching_schools_replaces_the_exported_file() {
        let _dir = TempDataDir::new("multi-school");
        let key_a = "c".repeat(64);
        let key_b = "d".repeat(64);
        assert_ne!(materialized_name(&key_a), materialized_name(&key_b));

        let a = materialize_xml(&key_a, "<osm><way id=\"1\"/></osm>").unwrap();
        assert!(a.is_file());
        // 旧版本的固定名文件也要被清掉，否则 materialized_path 会先撞上它。
        let legacy = crate::platform::data_dir().join(LEGACY_MATERIALIZED_NAME);
        std::fs::write(&legacy, "<osm/>").unwrap();

        let b = materialize_xml(&key_b, "<osm><way id=\"2\"/></osm>").unwrap();
        assert!(b.is_file(), "B 校导出文件不存在");
        assert!(!a.exists(), "A 校导出文件未被清理");
        assert!(!legacy.exists(), "旧版固定名文件未被清理");
        assert_eq!(
            std::fs::read_to_string(&b).unwrap(),
            "<osm><way id=\"2\"/></osm>"
        );
        assert_eq!(materialized_path_for(&key_a), None);
        assert_eq!(materialized_path_for(&key_b).as_deref(), Some(b.as_path()));
        assert_eq!(materialized_path().as_deref(), Some(b.as_path()));
    }

    // ───────── 围栏分支的缓存行为（真实 OSM 夹具） ─────────
    //
    // 夹具 `dinghu-roads.osm` 是 **WGS84**（道路范围 lat 23.1973–23.2155、
    // lon 112.5744–112.5925）。而 `cached_route_network` 在围栏分支会先把路网
    // `align_network` 成 **BD-09**，再 `apply_fences` 裁剪，所以构造围栏时
    // **必须逐点 `wgs84_to_bd09`**。该区域偏移约 Δlat +0.0033 / Δlon +0.0115
    // （约 367m / 1180m），不转会整块错位、把有路的围栏裁成空图。
    //
    // 每个测试只调用一次 `cached_route_network_for_test`：不靠"跑两次验证命中"，
    // 因此不依赖 TTL 和真实时钟。

    const FIXTURE_ROADS: &str = "dinghu-roads.osm";
    // 夹具道路范围（WGS84）。
    const ROAD_LAT_MIN: f64 = 23.1973;
    const ROAD_LON_MIN: f64 = 112.5744;
    const ROAD_LAT_MAX: f64 = 23.2155;
    const ROAD_LON_MAX: f64 = 112.5925;

    fn fence_fixture_bytes() -> Vec<u8> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("crates/route-planner/tests/fixtures")
            .join(FIXTURE_ROADS);
        std::fs::read(&p).unwrap_or_else(|e| panic!("读取夹具 {} 失败: {e}", p.display()))
    }

    /// WGS84 矩形 → BD-09 的 `(lat, lon)` 围栏环（与对齐后的路网同系）。
    fn bd_fence(lat_min: f64, lon_min: f64, lat_max: f64, lon_max: f64) -> Vec<Vec<(f64, f64)>> {
        let corners = [
            (lat_min, lon_min),
            (lat_min, lon_max),
            (lat_max, lon_max),
            (lat_max, lon_min),
        ];
        vec![corners
            .iter()
            .map(|&(lat, lon)| crate::track::geom::wgs84_to_bd09(lat, lon))
            .collect()]
    }

    /// 围栏内没有道路时：必须报错，且**坏结果不得写入缓存**。
    ///
    /// 否则下次会拿一个空路网继续跑（用户看到"已获取路网"但路线全部失败）。
    #[test]
    fn fence_without_roads_is_rejected_and_not_cached() {
        let dir = TempDataDir::new("fence-no-roads");
        // 与夹具道路范围完全不重叠的约 1km 方块（东南方向约 6km 外的空地）。
        let fences = bd_fence(23.2400, 112.6300, 23.2500, 112.6400);
        let key = route_scope_key("1001", &fences, &[]);
        let path = cache_path(&key).unwrap();

        let mut log = |l: &str| println!("{l}");
        let err = match cached_route_network_for_test(
            &cache_dir(),
            "1001",
            &fences,
            &[],
            false,
            1_700_000_000_000,
            |_q| Ok(fence_fixture_bytes()),
            &mut log,
        ) {
            Ok(net) => panic!(
                "围栏内没有道路时必须报错，实际返回 {} 条边",
                net.graph.edge_count()
            ),
            Err(e) => e,
        };
        println!("围栏内无路的错误：{err}");

        // 零道路时 `apply_fences` 会先返回自己的错误；非零但全在围栏外的边缘
        // 情况下则由 `cached_route_network` 的检查兜底。两者都必须被拒绝。
        assert!(
            err.contains("围栏内没有可用道路") || err.contains("围栏裁剪后没有可用道路"),
            "错误消息应说明围栏内没有道路，实际：{err}"
        );
        assert!(!path.exists(), "坏结果不得写入缓存：{}", path.display());
        assert!(
            entries().is_empty(),
            "缓存列表应为空，实际 {} 条",
            entries().len()
        );
    }

    /// 围栏裁剪后仍有道路时：接受，并且缓存可原样读回。
    #[test]
    fn fence_with_roads_is_accepted_and_cached() {
        let dir = TempDataDir::new("fence-with-roads");
        // 内接矩形：从道路范围各边各收 0.002°（约 220m），保证围栏内部确有道路。
        let fences = bd_fence(
            ROAD_LAT_MIN + 0.002,
            ROAD_LON_MIN + 0.002,
            ROAD_LAT_MAX - 0.002,
            ROAD_LON_MAX - 0.002,
        );
        let key = route_scope_key("1001", &fences, &[]);
        let path = cache_path(&key).unwrap();
        assert!(!path.exists(), "前置条件：缓存不应预先存在");

        let mut log = |l: &str| println!("{l}");
        let net = cached_route_network_for_test(
            &cache_dir(),
            "1001",
            &fences,
            &[],
            false,
            1_700_000_000_000,
            |_q| Ok(fence_fixture_bytes()),
            &mut log,
        )
        .expect("围栏裁剪后仍有道路，应接受");
        println!(
            "返回路网：{} 节点 {} 边",
            net.graph.node_count(),
            net.graph.edge_count()
        );
        assert!(net.graph.edge_count() > 0, "裁剪后应仍有道路");

        assert!(path.is_file(), "接受的结果必须写入缓存：{}", path.display());
        // 缓存必须能按 key 读回，且解析出的路网非空。
        let (cached, bytes) = read_cache_record(&key).expect("缓存应可读回");
        println!("缓存 {} 字节，{} 条边", bytes, cached.xml.len());
        assert_eq!(cached.school, "1001");
        assert_eq!(cached.fences, fences);
        assert!(parse(cached.xml.as_bytes()).unwrap().graph.edge_count() > 0);
    }

    /// 多校 / 多围栏的缓存键互不冲突，且两所学校各自落盘、互不覆盖。
    #[test]
    fn cache_keys_are_isolated_per_school_and_fence() {
        let _dir = TempDataDir::new("fence-keys");
        let fence_a = bd_fence(ROAD_LAT_MIN + 0.002, ROAD_LON_MIN + 0.002,
                               ROAD_LAT_MAX - 0.002, ROAD_LON_MAX - 0.002);
        let fence_b = bd_fence(ROAD_LAT_MIN + 0.004, ROAD_LON_MIN + 0.004,
                               ROAD_LAT_MAX - 0.004, ROAD_LON_MAX - 0.004);

        // 同校 + 不同围栏；同围栏 + 不同学校：都必须换键。
        let a_1001 = route_scope_key("1001", &fence_a, &[]);
        let b_1001 = route_scope_key("1001", &fence_b, &[]);
        let a_1002 = route_scope_key("1002", &fence_a, &[]);
        println!("keys: a/1001={a_1001}  b/1001={b_1001}  a/1002={a_1002}");
        assert_ne!(a_1001, b_1001, "同校不同围栏必须生成不同缓存键");
        assert_ne!(a_1001, a_1002, "同围栏不同学校必须生成不同缓存键");

        // 两校在同一围栏下各跑一次：应留下两份缓存。
        for school in ["1001", "1002"] {
            let mut log = |l: &str| println!("{l}");
            cached_route_network_for_test(
                &cache_dir(),
                school,
                &fence_a,
                &[],
                false,
                1_700_000_000_000,
                |_q| Ok(fence_fixture_bytes()),
                &mut log,
            )
            .unwrap_or_else(|e| panic!("{school} 应能接受该围栏：{e}"));
        }
        let mut files: Vec<String> = entries()
            .into_iter()
            .map(|e| e.school)
            .collect();
        files.sort();
        println!("缓存条目学校：{files:?}");
        assert_eq!(files, vec!["1001".to_string(), "1002".to_string()]);
    }
}

#[cfg(test)]
mod live_tests {
    use super::*;

    /// 真实联网测试：用用户学校点位范围下载一次。
    /// 默认忽略（`cargo test -- --ignored` 才跑），避免 CI 依赖外网。
    #[test]
    #[ignore]
    fn download_real_campus() {
        let pts = vec![
            (31.526444, 104.675651),
            (31.526805, 104.676518),
            (31.527248, 104.676262),
            (31.525924, 104.675929),
            (31.526259, 104.676648),
        ];
        let q = query_coordinates(&pts, 0.002).expect("构造查询");
        println!("查询：{q}");
        let bytes = download(&q).expect("下载应成功");
        println!("下载 {} 字节", bytes.len());
        let net = parse(&bytes).expect("解析应成功");
        println!("路网：{} 节点 {} 边", net.graph.node_count(), net.graph.edge_count());
        assert!(net.graph.edge_count() > 0);
        // 对齐后应覆盖打卡点
        let mut aligned = net.clone();
        crate::track::generate_road::align_network(&mut aligned);
        for &(lat, lon) in &pts {
            let d = aligned
                .nearest_edge(aligned.to_m(route_planner::Coord::new(lon, lat)))
                .map(|(_, _, _, d)| d)
                .unwrap_or(f64::INFINITY);
            println!("点位距路网 {d:.0}m");
        }
    }

    /// N1 端到端：真实下载 → 导出 → 跑步主流程加载 → 点位贴路。
    ///
    /// 走的是和界面完全相同的路径：`load_for_route`（自动获取）→
    /// `route_cache_key` → `materialize` → `load_network_path`
    /// （flow 用的就是它读 `config.osm_path`）。
    /// 默认忽略（需联网）：`cargo test -- --ignored real_download_materializes`。
    #[test]
    #[ignore]
    fn real_download_materializes_into_a_run_loadable_file() {
        let dir = std::env::temp_dir().join(format!(
            "neko-e2e-materialize-{}",
            uuid::Uuid::new_v4()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = Some(dir.clone()));

        let pts = vec![
            (31.526444, 104.675651),
            (31.526805, 104.676518),
            (31.527248, 104.676262),
            (31.525924, 104.675929),
            (31.526259, 104.676648),
        ];
        let school = "e2e";
        let control = crate::task_control::Control::default();
        let mut log = |line: &str| println!("{line}");

        // ① 自动获取（无围栏 → 按真实点位范围下载）
        let net = crate::api::school_roads::load_for_route(
            "", school, &[], &pts, true, &control, &mut log,
        )
        .expect("真实下载应成功");
        assert!(net.graph.edge_count() > 0, "下载的路网没有边");

        // ② 界面算出来并交给 materialize 的那个 key
        let key = route_cache_key(school, &[], &pts);
        println!("缓存键：{key}");

        // ③ 导出
        let path = materialize(&key).expect("导出应成功");
        println!("导出路径：{}", path.display());
        assert_eq!(
            path.file_name().unwrap().to_str().unwrap(),
            materialized_name(&key)
        );
        assert!(path.is_file());
        assert!(path.to_string_lossy().contains(MATERIALIZED_PREFIX));
        assert_eq!(materialized_path_for(&key).as_deref(), Some(path.as_path()));
        // 这就是 msgs.rs 会写进 config.osm_path 的值。
        assert_eq!(materialized_path().as_deref(), Some(path.as_path()));

        // ④ 跑步主流程的读法
        let run_net = crate::track::generate_road::load_network_path(
            path.to_str().unwrap(),
        )
        .expect("load_network_path 应能加载导出文件");
        assert!(run_net.graph.edge_count() > 0, "跑步流程读到的路网没有边");

        // ⑤ 导出的路网（flow 读到的）必须覆盖真实点位。
        // 注意：load_network_path 内部已把 WGS84 对齐成 BD-09，点位本身就是 BD-09。
        for &(lat, lon) in &pts {
            let d = run_net
                .nearest_edge(run_net.to_m(route_planner::Coord::new(lon, lat)))
                .map(|(_, _, _, d)| d)
                .unwrap_or(f64::INFINITY);
            println!("点位 ({lat}, {lon}) 距路网 {d:.1}m");
            assert!(d < 20.0, "点位 ({lat}, {lon}) 距路网 {d:.1}m，超过 20m");
        }

        crate::platform::TEST_DATA_DIR.with(|p| *p.borrow_mut() = None);
        let _ = std::fs::remove_dir_all(dir);
    }
}
