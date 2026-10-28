//! 长任务的取消状态机。
//!
//! 关键约束：**取消与进入提交必须互斥**。用户点"取消"和程序发出成绩提交是
//! 两个并发事件，谁先赢必须确定，不能出现"界面显示已取消、实际上成绩已发出"。
//!
//! 三态（AtomicU8 一次 CAS 完成迁移）：
//! * `0` 可取消的准备阶段
//! * `1` 已取消（不可逆，不会再去提交）
//! * `2` 已进入提交（不可取消，网络请求可能已发出）

use std::sync::atomic::{AtomicU8, Ordering};

const PREPARING: u8 = 0;
const CANCELLED: u8 = 1;
const SUBMITTING: u8 = 2;

#[derive(Default)]
pub struct Control(AtomicU8);

impl Control {
    /// 请求取消。已经进入提交时返回 false（取消无效）。
    pub fn cancel(&self) -> bool {
        self.0
            .compare_exchange(PREPARING, CANCELLED, Ordering::SeqCst, Ordering::SeqCst)
            .is_ok()
    }

    /// 在可取消阶段检查取消状态；被取消时返回 Err 由调用方终止。
    pub fn check(&self) -> Result<(), String> {
        match self.0.load(Ordering::SeqCst) {
            CANCELLED => Err(
                "操作已取消（已发出的网络请求可能仍在结束中，结果会按无效丢弃）".into(),
            ),
            _ => Ok(()),
        }
    }

    /// 关闭取消窗口，进入不可逆的提交阶段。
    pub fn begin_submit(&self) -> Result<(), String> {
        self.0
            .compare_exchange(PREPARING, SUBMITTING, Ordering::SeqCst, Ordering::SeqCst)
            .map(|_| ())
            .map_err(|_| "操作已取消，未进入记录提交".into())
    }

    /// 当前是否仍处于可取消阶段（界面据此决定按钮是否可用）。
    pub fn cancellable(&self) -> bool {
        self.0.load(Ordering::SeqCst) == PREPARING
    }

    /// 是否已进入提交（用于 UI 提示"正在提交，无法取消"）。
    pub fn is_submitting(&self) -> bool {
        self.0.load(Ordering::SeqCst) == SUBMITTING
    }
}

#[cfg(test)]
mod tests {
    use super::Control;

    #[test]
    fn cancellation_and_submission_are_mutually_exclusive() {
        let c = Control::default();
        assert!(c.cancel());
        assert!(c.check().is_err());
        assert!(c.begin_submit().is_err());

        let c = Control::default();
        assert!(c.begin_submit().is_ok());
        assert!(!c.cancel());
        assert!(c.check().is_ok());
        assert!(c.is_submitting());
    }

    #[test]
    fn repeated_cancel_is_idempotent_but_only_first_wins() {
        let c = Control::default();
        assert!(c.cancellable());
        assert!(c.cancel());
        assert!(!c.cancel());
        assert!(!c.cancellable());
    }
}
