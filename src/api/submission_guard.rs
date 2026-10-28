//! 跑步提交的重复保护：写前回执 + 进程文件锁 + UUID 对账。
//!
//! 目的只有一个：**没弄清上一次提交到底成没成功之前，不要再发一次**。
//! 服务端重复入库会直接算违规，比少跑一次严重得多。
//!
//! * 回执按 `sha256(uid:unid)` 分文件，账号/学校互相隔离；不含密码、Token 或坐标。
//! * 请求体写出并落盘成功后才发提交请求，进程崩在中途也有据可查。
//! * 结果未知（网络中断、响应无法解析、成功响应缺 rrid）时**不重发**，
//!   只读拉取记录列表按 UUID 对账。
//! * 操作系统文件锁阻止 GUI 与 CLI 同时为同一账号提交。

use super::client::ApiClient;
use super::records::RecordRow;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::fs::File;
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

/// 回执目录名（升级时需保留）。
pub const RECEIPT_DIR: &str = "submission_receipts";

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "state", rename_all = "snake_case")]
enum Event {
    /// 即将发送：已落盘，但还没有结果。
    Pending {
        uuid: String,
        start_ms: i64,
        stop_ms: i64,
    },
    /// 服务端明确受理。
    Accepted { uuid: String, rrid: i64 },
    /// 服务端明确拒绝（允许用户稍后手动重试）。
    Rejected { uuid: String },
}

#[derive(Debug, Clone)]
struct Attempt {
    uuid: String,
    start_ms: i64,
    stop_ms: i64,
    /// None = 结果未知；Some(None) = 明确拒绝；Some(Some(id)) = 已受理。
    outcome: Option<Option<i64>>,
}

impl Attempt {
    fn overlaps(&self, start_ms: i64, stop_ms: i64) -> bool {
        self.start_ms < stop_ms && start_ms < self.stop_ms
    }
}

/// 一个账号 + 学校维度的提交保护句柄。
pub struct Guard {
    /// 持锁句柄：只用于锁，读写走独立句柄。
    _lock: File,
    path: PathBuf,
    attempts: Vec<Attempt>,
}

impl Guard {
    /// 打开当前账号的保护记录并取得独占锁。
    pub fn open(uid: i64, unid: i64) -> Result<Self, String> {
        Self::open_in(&crate::platform::data_dir(), uid, unid)
    }

    fn open_in(root: &Path, uid: i64, unid: i64) -> Result<Self, String> {
        if uid <= 0 || unid <= 0 {
            return Err("账号或学校编号无效，无法启用重复提交保护".into());
        }
        let dir = root.join(RECEIPT_DIR);
        std::fs::create_dir_all(&dir).map_err(|e| format!("无法建立提交保护目录：{e}"))?;
        let name = format!("{:x}.jsonl", Sha256::digest(format!("{uid}:{unid}")));
        let path = dir.join(name);
        // 锁句柄与读写句柄分开：Windows 上 LockFileEx 的独占区间会阻塞同一
        // 句柄（甚至其他句柄）的读写，锁和 I/O 混用会让 reload() 直接失败。
        // 使用 Windows 原生共享模式打开（不与 std 的默认共享模式纠缠）。
        let lock = open_lock_handle(&path)?;
        lock_exclusive(&lock)?;
        let mut guard = Self {
            _lock: lock,
            path,
            attempts: Vec::new(),
        };
        guard.reload()?;
        Ok(guard)
    }

    fn reload(&mut self) -> Result<(), String> {
        self.attempts.clear();
        let file = File::open(&self.path).map_err(|e| format!("无法读取提交保护记录：{e}"))?;
        for (line_no, line) in BufReader::new(file).lines().enumerate() {
            let line = line.map_err(|e| format!("提交保护记录读取失败：{e}"))?;
            if line.trim().is_empty() {
                continue;
            }
            let event: Event = serde_json::from_str(&line).map_err(|e| {
                format!(
                    "提交保护记录第 {} 行损坏（{e}）。为避免重复提交已停止；请人工核对 {} 后处理",
                    line_no + 1,
                    self.path.display()
                )
            })?;
            self.apply(event);
        }
        Ok(())
    }

    fn apply(&mut self, event: Event) {
        match event {
            Event::Pending {
                uuid,
                start_ms,
                stop_ms,
            } => {
                if !self.attempts.iter().any(|a| a.uuid == uuid) {
                    self.attempts.push(Attempt {
                        uuid,
                        start_ms,
                        stop_ms,
                        outcome: None,
                    });
                }
            }
            Event::Accepted { uuid, rrid } => {
                if let Some(a) = self.attempts.iter_mut().find(|a| a.uuid == uuid) {
                    a.outcome = Some(Some(rrid));
                }
            }
            Event::Rejected { uuid } => {
                if let Some(a) = self.attempts.iter_mut().find(|a| a.uuid == uuid) {
                    a.outcome = Some(None);
                }
            }
        }
    }

    fn append(&mut self, event: Event) -> Result<(), String> {
        let mut line = serde_json::to_string(&event)
            .map_err(|e| format!("无法序列化提交保护记录：{e}"))?;
        line.push('\n');
        let mut file = File::options()
            .append(true)
            .open(&self.path)
            .map_err(|e| format!("无法写入提交保护记录：{e}"))?;
        file.write_all(line.as_bytes())
            .and_then(|_| file.sync_all())
            .map_err(|e| format!("提交保护记录落盘失败，已停止提交：{e}"))?;
        // 落盘成功后才更新内存状态：写入失败时不留下"已发送"的错觉。
        self.apply(event);
        Ok(())
    }

    /// 是否还有结果未知的提交。
    pub fn has_pending(&self) -> bool {
        self.attempts.iter().any(|a| a.outcome.is_none())
    }

    /// 提交前检查：已受理或结果未知的时间段一律拒绝。
    pub fn check(&self, start_ms: i64, stop_ms: i64) -> Result<(), String> {
        for a in &self.attempts {
            if !a.overlaps(start_ms, stop_ms) {
                continue;
            }
            match a.outcome {
                Some(Some(rrid)) => {
                    return Err(format!(
                        "该时间段已有成功记录 rrid={rrid}（uuid={}），不能重复提交；如需重新跑请更换开始时间",
                        a.uuid
                    ))
                }
                None => {
                    return Err(format!(
                        "存在结果未知的提交 uuid={}（{}~{}）。请先在运动记录中刷新核对，确认未入库后再处理",
                        a.uuid, a.start_ms, a.stop_ms
                    ))
                }
                Some(None) => {}
            }
        }
        Ok(())
    }

    /// 落盘 Pending 回执（fsync 成功后返回，之后才允许发送请求）。
    pub fn begin(&mut self, uuid: &str, start_ms: i64, stop_ms: i64) -> Result<(), String> {
        self.append(Event::Pending {
            uuid: uuid.to_string(),
            start_ms,
            stop_ms,
        })
    }

    /// 服务端明确受理。
    pub fn accepted(&mut self, uuid: &str, rrid: i64) -> Result<(), String> {
        self.append(Event::Accepted {
            uuid: uuid.to_string(),
            rrid,
        })
    }

    /// 服务端明确拒绝。
    pub fn rejected(&mut self, uuid: &str) -> Result<(), String> {
        self.append(Event::Rejected {
            uuid: uuid.to_string(),
        })
    }

    /// 用只读拉到的记录列表对账，返回新确认的 rrid。
    pub fn reconcile(&mut self, rows: &[RecordRow]) -> Result<Vec<i64>, String> {
        let pending: Vec<String> = self
            .attempts
            .iter()
            .filter(|a| a.outcome.is_none())
            .map(|a| a.uuid.to_uppercase())
            .collect();
        let mut matches = Vec::new();
        for row in rows {
            if row.uuid.is_empty() || row.rrid <= 0 {
                continue;
            }
            let uuid = row.uuid.to_uppercase();
            if pending.contains(&uuid) {
                matches.push((uuid, row.rrid));
            }
        }
        // 先收集再逐条落盘：落盘失败时不会留下"内存已确认、磁盘未记录"的中间态。
        for (uuid, rrid) in &matches {
            self.accepted(uuid, *rrid)?;
        }
        Ok(matches.into_iter().map(|(_, rrid)| rrid).collect())
    }
}

/// 打开仅用于加锁的句柄。
#[cfg(windows)]
fn open_lock_handle(path: &Path) -> Result<File, String> {
    use std::os::windows::io::FromRawHandle;
    #[link(name = "kernel32")]
    extern "system" {
        fn CreateFileW(
            name: *const u16,
            access: u32,
            share: u32,
            security: *mut std::ffi::c_void,
            creation: u32,
            flags: u32,
            template: *mut std::ffi::c_void,
        ) -> *mut std::ffi::c_void;
    }
    const GENERIC_READ: u32 = 0x8000_0000;
    const GENERIC_WRITE: u32 = 0x4000_0000;
    const FILE_SHARE_READ: u32 = 0x0000_0001;
    const FILE_SHARE_WRITE: u32 = 0x0000_0002;
    const OPEN_ALWAYS: u32 = 4;
    let wide: Vec<u16> = path
        .to_string_lossy()
        .encode_utf16()
        .chain(std::iter::once(0))
        .collect();
    // SAFETY: 路径 UTF-16 以 NUL 结尾且在整个调用期间存活。
    let handle = unsafe {
        CreateFileW(
            wide.as_ptr(),
            GENERIC_READ | GENERIC_WRITE,
            FILE_SHARE_READ | FILE_SHARE_WRITE,
            std::ptr::null_mut(),
            OPEN_ALWAYS,
            0,
            std::ptr::null_mut(),
        )
    };
    if handle as isize == -1 {
        return Err(format!(
            "无法打开提交保护记录：{}",
            std::io::Error::last_os_error()
        ));
    }
    // SAFETY: handle 是刚创建成功的有效句柄，所有权转交 File。
    Ok(unsafe { File::from_raw_handle(handle) })
}

#[cfg(not(windows))]
fn open_lock_handle(path: &Path) -> Result<File, String> {
    File::options()
        .read(true)
        .write(true)
        .create(true)
        .truncate(false)
        .open(path)
        .map_err(|e| format!("无法打开提交保护记录：{e}"))
}

#[cfg(windows)]
fn lock_exclusive(file: &File) -> Result<(), String> {
    use std::os::windows::io::AsRawHandle;
    // FFI 声明放在函数内，避免 Android/非 Windows 目标引入额外依赖。
    #[link(name = "kernel32")]
    extern "system" {
        fn LockFileEx(
            handle: *mut std::ffi::c_void,
            flags: u32,
            reserved: u32,
            low: u32,
            high: u32,
            overlapped: *mut Overlapped,
        ) -> i32;
    }
    /// OVERLAPPED 需要指针宽度对齐；用 usiz 字段保证布局与对齐。
    #[repr(C)]
    struct Overlapped {
        internal: usize,
        internal_high: usize,
        offset: u64,
        event: usize,
    }
    const LOCKFILE_FAIL_IMMEDIATELY: u32 = 0x0000_0001;
    const LOCKFILE_EXCLUSIVE_LOCK: u32 = 0x0000_0002;
    // 只锁最后一个字节：锁住整文件区间会让同进程内其他句柄的读取失败，
    // 而"末字节"锁对追加写在语义上等价（谁拿到它谁独占该账号）。
    let mut overlapped = Overlapped {
        internal: 0,
        internal_high: 0,
        offset: u64::MAX - 1,
        event: 0,
    };
    // SAFETY: 句柄来自存活的 File；overlapped 是栈上对齐的有效结构。
    let ok = unsafe {
        LockFileEx(
            file.as_raw_handle() as *mut std::ffi::c_void,
            LOCKFILE_EXCLUSIVE_LOCK | LOCKFILE_FAIL_IMMEDIATELY,
            0,
            1,
            0,
            &mut overlapped,
        )
    };
    if ok == 0 {
        return Err("另一个程序正在为同一账号提交，请等待其结束".into());
    }
    Ok(())
}

#[cfg(unix)]
fn lock_exclusive(file: &File) -> Result<(), String> {
    use std::os::unix::io::AsRawFd;
    #[link(name = "c")]
    extern "C" {
        fn flock(fd: i32, operation: i32) -> i32;
    }
    const LOCK_EX: i32 = 2;
    const LOCK_NB: i32 = 4;
    // SAFETY: fd 来自存活的 File，调用期间不会关闭。
    if unsafe { flock(file.as_raw_fd(), LOCK_EX | LOCK_NB) } != 0 {
        return Err("另一个程序正在为同一账号提交，请等待其结束".into());
    }
    Ok(())
}

#[cfg(not(any(windows, unix)))]
fn lock_exclusive(_file: &File) -> Result<(), String> {
    Ok(())
}

/// 只读核对：登录状态下拉取记录列表并对账。
///
/// 查不到相同 UUID **不等于**提交失败——服务端可能延迟入库。此时保持未知状态，
/// 由用户稍后再核对，绝不自动补发。
pub fn reconcile_with_server(
    client: &mut ApiClient,
    guard: &mut Guard,
    log: &mut dyn FnMut(&str),
) -> Result<Vec<i64>, String> {
    if !guard.has_pending() {
        return Ok(Vec::new());
    }
    let rows = super::records::fetch_records(client)?;
    let found = guard.reconcile(&rows)?;
    if found.is_empty() {
        log("⚠ [guard] 记录列表中没有未决提交的 UUID；这不代表失败，请稍后再核对，程序不会自动重发");
    } else {
        for rrid in &found {
            log(&format!("√ [guard] 未决提交已确认入库 rrid={rrid}"));
        }
    }
    Ok(found)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn temp_root(tag: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "nekoguard-{tag}-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn row(uuid: &str, rrid: i64) -> RecordRow {
        RecordRow {
            rrid,
            total_dis: 2000.0,
            total_time: 600,
            start_time: 0,
            complete: true,
            avg_step_freq: 170,
            calorie: 100,
            avg_power: 50,
            total_steps: 1700,
            uuid: uuid.into(),
        }
    }

    #[test]
    fn accepted_interval_cannot_be_repeated() {
        let root = temp_root("accepted");
        let mut g = Guard::open_in(&root, 1, 2).unwrap();
        g.begin("UUID-A", 1000, 2000).unwrap();
        g.accepted("UUID-A", 88).unwrap();
        let err = g.check(1500, 2500).unwrap_err();
        assert!(err.contains("88"));
        // 不重叠的时间段可以提交。
        assert!(g.check(3000, 4000).is_ok());
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn unknown_outcome_blocks_until_reconciled() {
        let root = temp_root("unknown");
        let mut g = Guard::open_in(&root, 1, 2).unwrap();
        g.begin("UUID-B", 1000, 2000).unwrap();
        // 重开也解不开未知状态。
        drop(g);
        let mut g = Guard::open_in(&root, 1, 2).unwrap();
        assert!(g.has_pending());
        assert!(g.check(1000, 2000).is_err());
        // 只读对账后发现入库，才转为已知。
        assert_eq!(g.reconcile(&[row("uuid-b", 91)]).unwrap(), vec![91]);
        assert!(!g.has_pending());
        assert!(g.check(1000, 2000).unwrap_err().contains("91"));
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn reconcile_ignores_unrelated_records() {
        let root = temp_root("unrelated");
        let mut g = Guard::open_in(&root, 1, 2).unwrap();
        g.begin("UUID-C", 1000, 2000).unwrap();
        assert!(g.reconcile(&[row("different", 7)]).unwrap().is_empty());
        assert!(g.has_pending());
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn rejection_allows_a_later_manual_retry() {
        let root = temp_root("rejected");
        let mut g = Guard::open_in(&root, 1, 2).unwrap();
        g.begin("UUID-D", 1000, 2000).unwrap();
        g.rejected("UUID-D").unwrap();
        assert!(g.check(1000, 2000).is_ok());
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn accounts_are_isolated() {
        let root = temp_root("isolated");
        let mut a = Guard::open_in(&root, 1, 2).unwrap();
        a.begin("UUID-E", 1000, 2000).unwrap();
        let b = Guard::open_in(&root, 3, 2).unwrap();
        assert!(!b.has_pending());
        assert!(b.check(1000, 2000).is_ok());
        let _ = std::fs::remove_dir_all(root);
    }

    #[test]
    fn corrupt_receipt_stops_submission() {
        let root = temp_root("corrupt");
        let path = root.join(RECEIPT_DIR);
        std::fs::create_dir_all(&path).unwrap();
        let name = format!("{:x}.jsonl", Sha256::digest("1:2"));
        std::fs::write(path.join(name), "{not json}\n").unwrap();
        assert!(Guard::open_in(&root, 1, 2).is_err());
        let _ = std::fs::remove_dir_all(root);
    }
}
