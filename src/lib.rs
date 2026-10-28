//! Shared desktop and Android application.

/// 正式构建私有；测试构建公开，供集成测试驱动 `api::school_roads`。
#[cfg(not(test))]
mod api;
#[cfg(test)]
pub mod api;

mod amap;
pub mod cli;
mod crypto;
/// 单任务的取消/提交互斥状态机（长流程使用）。
pub mod task_control;
/// 白名单脱敏诊断报告。
pub mod diagnostics;
pub mod location;
pub mod platform;
mod relaunch;
/// 错误串 → 分类 + 建议动作。
pub mod road_status;
pub mod template;
mod textlog;
mod track;
mod update;

#[cfg(feature = "gui")]
mod ui;

#[cfg(all(target_os = "android", feature = "android"))]
mod android;

#[cfg(feature = "gui")]
pub fn run_gui(options: eframe::NativeOptions) -> eframe::Result<()> {
    eframe::run_native(
        "NekoSportsWorldTool",
        options,
        Box::new(|cc| {
            #[cfg(target_os = "android")]
            android::set_context(&cc.egui_ctx);
            Ok(Box::new(ui::App::new(cc)))
        }),
    )
}
