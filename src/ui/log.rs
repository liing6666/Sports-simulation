//! 运行日志：彩色等宽滚动，自动滚底，上限 2000 行。
//! 入站行的 √/×/⚠ 前缀仅用于着色，落盘展示时剥离。

use super::theme;
use crate::textlog::{classify, redact_text, LogKind};
use egui::{Color32, RichText, ScrollArea};
use std::collections::VecDeque;

const MAX_LINES: usize = 2000;

#[derive(Debug, Clone)]
pub struct LogLine {
    pub text: String,
    pub kind: LogKind,
}

impl LogLine {
    fn color(&self) -> Color32 {
        match self.kind {
            LogKind::Plain => theme::plain(),
            LogKind::Ok => theme::ok(),
            LogKind::Err => theme::err(),
            LogKind::Warn => theme::warn(),
        }
    }
}

#[derive(Default)]
pub struct LogStore {
    lines: VecDeque<LogLine>,
}

impl LogStore {
    pub fn push(&mut self, msg: &str) {
        for line in msg.lines() {
            let t = line.trim();
            if t.is_empty() {
                continue;
            }
            if self.lines.len() >= MAX_LINES {
                self.lines.pop_front();
            }
            let (kind, text) = classify(t);
            let text = redact_text(text);
            // 同时落盘：卡住时用户能直接把日志文件发出来定位，
            // 不必依赖"运行日志"页的截图。
            append_to_file(&text);
            self.lines.push_back(LogLine { text, kind });
        }
    }

    #[allow(dead_code)]
    pub fn clear(&mut self) {
        self.lines.clear();
    }

    pub fn len(&self) -> usize {
        self.lines.len()
    }

    pub fn render(&mut self, ui: &mut egui::Ui) {
        ScrollArea::vertical()
            .stick_to_bottom(true)
            .auto_shrink([false, false])
            .show(ui, |ui| {
                for line in &self.lines {
                    ui.label(
                        RichText::new(&line.text)
                            .monospace()
                            .size(12.5)
                            .color(line.color()),
                    );
                }
            });
    }
}

/// 把一行日志追加到 `data_dir/run.log`。
///
/// 写入失败静默忽略——日志落盘是诊断辅助，不能因为磁盘问题影响主流程。
/// 文件超过 2 MiB 时截断重写，避免无限增长。
fn append_to_file(text: &str) {
    use std::io::Write;
    let path = crate::platform::data_dir().join("run.log");
    if std::fs::metadata(&path).map(|m| m.len() > 2 * 1024 * 1024).unwrap_or(false) {
        let _ = std::fs::remove_file(&path);
    }
    let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&path)
    else {
        return;
    };
    let stamp = chrono::Local::now().format("%H:%M:%S%.3f");
    let _ = writeln!(f, "[{stamp}] {text}");
}
