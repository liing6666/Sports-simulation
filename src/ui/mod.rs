//! UI 层：App 状态机 + 顶栏 + 标签页 + 底部状态栏。
//!
//! 后台任务与消息处理在独立 impl App 块中。
//!
//! 线程模型：所有网络/长任务 spawn thread + mpsc channel<String> 回 UI，
//! 带 200ms 轮询重绘。带内消息协议：
//! __IP_DONE__ / __LOGIN_DONE__ / __RUN_DONE__ / __AI_DONE__ / __RECORDS__ / __AI_LIST__
//! __SEMESTER__ / __CHEAT__ / __RANK__ / __USER__ / __AI_RECORDS__

pub mod about;
pub mod ai;
pub mod data;
pub mod device;
pub mod fonts;
pub mod jobs;
pub mod log;
pub mod map;
pub mod mobile;
pub mod msgs;
pub mod osm;
pub mod profiles;
pub mod records;
pub mod run;
pub mod theme;
pub mod user;

use crate::api::model::{self, Config, HeaderIdentity, Session};
use eframe::egui;
use std::sync::mpsc::{Receiver, Sender};
use std::sync::Arc;

pub const IP: &str = "__IP_DONE__";
pub const LOGIN_DONE: &str = "__LOGIN_DONE__";
pub const RUN_DONE: &str = "__RUN_DONE__";
pub const AI_DONE: &str = "__AI_DONE__";
pub const RECORDS: &str = "__RECORDS__";
pub const AI_LIST: &str = "__AI_LIST__";
pub const AI_DETAIL: &str = "__AI_DETAIL__";
pub const SEMESTER: &str = "__SEMESTER__";
pub const CHEAT: &str = "__CHEAT__";
pub const RANK: &str = "__RANK__";
pub const USER: &str = "__USER__";
pub const AI_RECORDS: &str = "__AI_RECORDS__";
pub const RUN_DETAIL: &str = "__RUN_DETAIL__";
pub const UPDATE_CHK: &str = "__UPDATE_CHK__";
pub const UPDATE_PROG: &str = "__UPDATE_PROG__";
pub const UPDATE_DONE: &str = "__UPDATE_DONE__";
pub const FENCE_DONE: &str = "__FENCE_DONE__";
pub const POINTS_DONE: &str = "__POINTS_DONE__";
pub const CUSTOM_ROUTE_DONE: &str = "__CUSTOM_ROUTE_DONE__";

/// 提交结果弹窗。
pub struct PopupInfo {
    pub title: String,
    pub lines: Vec<String>,
}

/// 顶层 App。
pub struct App {
    pub tx: Sender<String>,
    pub rx: Receiver<String>,
    pub log: log::LogStore,
    pub font_loaded: Option<String>,

    pub ip: String,
    pub identity: HeaderIdentity,
    pub device_buf: HeaderIdentity,
    pub session: Option<Session>,
    pub config: Config,

    pub username: String,
    pub password: String,
    pub remember: bool,
    pub login_busy: bool,
    pub run_busy: bool,
    /// 当前跑步任务的取消句柄；提交流程进入提交阶段后取消会失效。
    pub run_control: std::sync::Arc<crate::task_control::Control>,
    pub ai_busy: bool,
    pub records_busy: bool,
    pub data_busy: bool,
    pub user_busy: bool,
    pub status: String,
    pub popup: Option<PopupInfo>,
    pub ai_confirm: Option<ai::AiBatchPlan>,

    pub tab: usize,
    pub run_page: run::RunPage,
    pub ai_page: ai::AiPage,
    pub records_page: records::RecordsPage,
    pub data_page: data::DataPage,
    pub user_page: user::UserPage,
    pub profiles_page: profiles::ProfilesPage,
    pub device_page: device::DevicePage,
    pub update: about::UpdateUi,

    /// 已加载的 OSM 路网（真实道路路由 + 地图显示共用）。
    pub network: Option<Arc<route_planner::RoadGraph>>,
    pub osm_page: osm::OsmPage,
    /// 路网加载结果 + 该路网对应的 `config.osm_path`。
    ///
    /// 跑步主流程（`api::flow`）只读 `config.osm_path` 指向的文件；自动获取和
    /// 载入缓存并不经过用户选路径，必须把导出的文件路径带回来写进配置。
    net_tx: Sender<Result<osm::NetLoaded, String>>,
    net_rx: Receiver<Result<osm::NetLoaded, String>>,
    /// 校园边界搜索结果回传通道。
    campus_tx: Sender<Result<Vec<route_planner::Campus>, String>>,
    campus_rx: Receiver<Result<Vec<route_planner::Campus>, String>>,
}

impl eframe::App for App {
    #[cfg(target_os = "android")]
    fn clear_color(&self, visuals: &egui::Visuals) -> [f32; 4] {
        visuals.panel_fill.to_normalized_gamma_f32()
    }

    fn raw_input_hook(&mut self, ctx: &egui::Context, input: &mut egui::RawInput) {
        crate::platform::apply_safe_area(ctx, input);
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_messages();
        #[cfg(target_os = "android")]
        self.poll_device_info();
        crate::platform::set_keep_screen_on(self.run_busy || self.ai_busy || self.login_busy);
        ctx.request_repaint_after(std::time::Duration::from_millis(200));

        // ── 顶栏 ────────────────────────────────────────────────
        egui::TopBottomPanel::top("top_bar")
            .frame(
                egui::Frame::none()
                    .fill(theme::WHITE)
                    .inner_margin(egui::Margin::symmetric(theme::GAP_LG, theme::GAP_XS))
                    .stroke(egui::Stroke::new(1.0_f32, theme::BORDER)),
            )
            .show(ctx, |ui| {
            let compact = mobile::compact_ui(ui);
            let logged = self.session.is_some();
            let btn_text = if logged { "登出" } else { "登录" };
            let mut login_clicked = false;
            let draw_connection = |ui: &mut egui::Ui| {
                let ip = if self.ip.is_empty() {
                    "获取中…".to_owned()
                } else {
                    self.ip.clone()
                };
                // 紧凑宽度下设备号另起一行，避免顶栏被挤成两行。
                let aux = if compact {
                    format!("IP：{ip}")
                } else {
                    format!("IP：{ip}   设备：{}", self.identity.device_id)
                };
                let aux = egui::RichText::new(aux).small().color(theme::text_dim());
                ui.label(aux);
                match &self.session {
                    Some(s) => {
                        theme::capsule(ui, &format!("● {}", s.name), theme::ok());
                        ui.label(
                            egui::RichText::new(format!("uid={}", s.uid))
                                .small()
                                .color(theme::text_dim()),
                        );
                    }
                    None => {
                        theme::capsule(ui, "● 未登录", theme::err());
                    }
                }
                if self.ip == "失败" {
                    ui.colored_label(theme::warn(), "IP 获取失败");
                }
            };
            if compact {
                ui.horizontal_wrapped(draw_connection);
                ui.label(
                    egui::RichText::new(format!("设备：{}", self.identity.device_id))
                        .small()
                        .color(theme::text_dim()),
                );
                mobile::row(ui, |ui| {
                    ui.label(egui::RichText::new("账号").small().color(theme::text_dim()));
                    mobile::text_edit(
                        ui,
                        "login_username",
                        &mut self.username,
                        crate::platform::InputKind::Text,
                        ui.available_width().min(200.0),
                    );
                });
                mobile::row(ui, |ui| {
                    ui.label(egui::RichText::new("密码").small().color(theme::text_dim()));
                    mobile::text_edit(
                        ui,
                        "login_password",
                        &mut self.password,
                        crate::platform::InputKind::Password,
                        ui.available_width().min(200.0),
                    );
                });
                mobile::row(ui, |ui| {
                    ui.checkbox(&mut self.remember, "记住密码");
                    login_clicked = ui
                        .add(theme::primary_btn_enabled(btn_text, !self.login_busy))
                        .clicked();
                });
            } else {
                // 桌面端固定两行：上行状态（IP/设备/账号），下行登录操作（输入靠左、按钮靠右）。
                ui.horizontal_wrapped(draw_connection);
                ui.add_space(theme::GAP_XS);
                ui.horizontal_wrapped(|ui| {
                    ui.label(egui::RichText::new("账号").small().color(theme::text_dim()));
                    mobile::text_edit(
                        ui,
                        "login_username",
                        &mut self.username,
                        crate::platform::InputKind::Text,
                        180.0,
                    );
                    ui.label(egui::RichText::new("密码").small().color(theme::text_dim()));
                    mobile::text_edit(
                        ui,
                        "login_password",
                        &mut self.password,
                        crate::platform::InputKind::Password,
                        160.0,
                    );
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        login_clicked = ui
                            .add(theme::primary_btn_enabled(btn_text, !self.login_busy))
                            .clicked();
                        ui.checkbox(&mut self.remember, "记住密码");
                    });
                });
            }
            if login_clicked {
                if logged {
                    self.do_logout();
                } else {
                    self.do_login();
                }
            }
        });

        // ── 底部状态栏 ──────────────────────────────────────────
        egui::TopBottomPanel::bottom("status_bar")
            .frame(
                egui::Frame::none()
                    .fill(theme::WHITE)
                    .inner_margin(egui::Margin::symmetric(theme::GAP_LG, theme::GAP_XS))
                    .stroke(egui::Stroke::new(1.0_f32, theme::BORDER)),
            )
            .show(ctx, |ui| {
            // 状态栏统一小字，整行同一字号。
            let small = |text: String| egui::RichText::new(text).small().color(theme::text_dim());
            // 窄屏时右半区另起一行：`right_to_left` 嵌在 `horizontal_wrapped` 里
            // 会在同一位置叠字（截图实测 460/360 下「检查更新」与状态文字重合）。
            let compact_bar = mobile::compact_ui(ui);
            let mut check_update_clicked = false;
            ui.horizontal_wrapped(|ui| {
                ui.label(small(format!("日志 {} 行", self.log.len())));
                ui.separator();
                if let Some(s) = &self.data_page.semester {
                    ui.label(small(format!(
                        "学期 {}：有效 {}/{} 次",
                        s.sname, s.semester_valid_count, s.semester_count
                    )));
                }
                if let Some(c) = &self.data_page.cheat {
                    if c.is_clean() {
                        ui.label(
                            egui::RichText::new("自查正常")
                                .small()
                                .color(theme::ok()),
                        );
                    } else {
                        ui.label(
                            egui::RichText::new(format!("已标记：{}", c.self_brief()))
                                .small()
                                .color(theme::err()),
                        );
                    }
                }
                ui.separator();
                ui.label(small(self.status.clone()));
                // 宽屏：右半区靠右同排；窄屏：另起一行左对齐，避免叠字。
                if !compact_bar {
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if draw_right_side(ui, small, &self.update) {
                            check_update_clicked = true;
                        }
                    });
                }
            });
            if compact_bar {
                ui.horizontal_wrapped(|ui| {
                    if draw_right_side(ui, small, &self.update) {
                        check_update_clicked = true;
                    }
                });
            }
            if check_update_clicked {
                self.check_update(true);
            }
        });

        // ── 标签页 ──────────────────────────────────────────────
        egui::CentralPanel::default()
            .frame(
                egui::Frame::none()
                    .fill(theme::PAGE)
                    .inner_margin(egui::Margin {
                        left: theme::GAP_XS,
                        right: theme::GAP_XS,
                        top: theme::GAP_SM,
                        bottom: theme::GAP_SM,
                    }),
            )
            .show(ctx, |ui| {
            theme::card(ui, |ui| {
                // 标签卡横跨内容区，避免只包住标签本身的宽度。
                ui.set_width(ui.available_width());
                ui.add_space(theme::GAP_XS);
                mobile::tab_bar(ui, &mut self.tab);
                ui.add_space(theme::GAP_XS);
            });
            ui.add_space(theme::GAP_SM);
            match self.tab {
                0 => self.draw_run(ui),
                1 => self.draw_ai(ui),
                2 => self.draw_records(ui),
                3 => self.draw_data(ui),
                4 => self.draw_user(ui),
                5 => self.draw_device(ui),
                6 => self.log.render(ui),
                7 => self.draw_osm_page(ui),
                8 => self.draw_profiles(ui),
                _ => self.draw_about(ui),
            }
        });

        if let Some(p) = &self.popup {
            let mut open = true;
            let mut close = false;
            egui::Window::new(egui::RichText::new(&p.title).strong())
                .open(&mut open)
                .collapsible(false)
                .resizable(false)
                .anchor(egui::Align2::CENTER_CENTER, [0.0, 0.0])
                .order(egui::Order::Foreground)
                .show(ctx, |ui| {
                    ui.set_max_width((ctx.screen_rect().width() - 32.0).min(520.0));
                    for line in &p.lines {
                        ui.label(line);
                    }
                    ui.add_space(6.0);
                    if ui.button("关闭").clicked() {
                        close = true;
                    }
                });
            if !open || close {
                self.popup = None;
            }
        }
        self.draw_update_windows(ctx);
        crate::platform::sync_clipboard(ctx);
    }
}

impl App {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let (tx, rx) = std::sync::mpsc::channel();
        let (net_tx, net_rx) = std::sync::mpsc::channel();
        let (campus_tx, campus_rx) = std::sync::mpsc::channel();
        let font_loaded = fonts::install(&cc.egui_ctx);
        theme::apply(&cc.egui_ctx);
        let identity = model::load_identity();
        let device_buf = identity.clone();
        let session = model::load_session();
        let config = model::load_config();
        let username = config.username.clone();
        let password = config.password.clone();
        let remember = config.remember;

        let mut app = Self {
            tx: tx.clone(),
            rx,
            log: log::LogStore::default(),
            font_loaded,
            ip: String::new(),
            identity,
            device_buf,
            session: session.is_logged_in().then_some(session),
            config: config.clone(),
            username,
            password,
            remember,
            login_busy: false,
            run_busy: false,
            run_control: std::sync::Arc::new(crate::task_control::Control::default()),
            ai_busy: false,
            records_busy: false,
            data_busy: false,
            user_busy: false,
            status: "就绪".into(),
            popup: None,
            ai_confirm: None,
            tab: 0,
            run_page: run::RunPage {
                dist_min: config.dist_min,
                dist_max: config.dist_max,
                pace_min: config.pace_min,
                pace_max: config.pace_max,
                manual_altitude: config
                    .manual_altitude_range
                    .map(|r| format!("{}-{}", r.min_m, r.max_m))
                    .or_else(|| config.manual_altitude.map(|v| v.to_string()))
                    .unwrap_or_default(),
                start_mode: 0,
                days_ago: 0,
                hour: 12,
                minute: 0,
                face_check: config.face_check,
                plan: None,
                route_mode: crate::track::generate_road::RouteMode::from_str(&config.route_mode),
                ordinary_pick: config.ordinary_pick as i64,
                track_spec: config.track_spec,
                custom_track_length: match config.track_spec {
                    crate::track::stadium::TrackSpec::Custom { total_m } => total_m.to_string(),
                    _ => "400".into(),
                },
                track_color_mode: config.track_color_mode,
                map: map::MapState::default(),
                preview: None,
                preview_stale: true,
                map_fitted: false,
                custom_origin: config.custom_origin.clone(),
                custom_destination: config.custom_destination.clone(),
                custom_loop: config.custom_loop,
                custom_laps: config.custom_laps,
                amap_key: config.amap_key.clone(),
                custom_preview: None,
                custom_status: String::new(),
                preview_progress: 1.0,
            },
            ai_page: ai::AiPage {
                days: 1,
                per_day: 1,
                ..Default::default()
            },
            records_page: records::RecordsPage::default(),
            data_page: data::DataPage::default(),
            user_page: user::UserPage::default(),
            profiles_page: profiles::ProfilesPage::default(),
            device_page: device::DevicePage::default(),
            update: about::UpdateUi::default(),
            network: None,
            osm_page: osm::OsmPage {
                path: config.osm_path.clone(),
                ..Default::default()
            },
            net_tx,
            net_rx,
            campus_tx,
            campus_rx,
        };
        if app.font_loaded.is_none() {
            app.log
                .push("未找到中文字体（msyh/simhei/simsun），界面中文可能显示为方块");
        }
        // 上次更新残留的 .old/.new 顺手清掉
        crate::update::cleanup_residue();
        // 启动检查更新（silent 静默 / ask 询问 / off 关闭）
        match app.config.update_check.as_str() {
            "off" => {}
            "ask" => app.update.ask_startup = true,
            _ => app.check_update(false),
        }
        app.fetch_ip();
        #[cfg(target_os = "android")]
        crate::android::request_device_info(true);
        // AI 项目列表先上缓存，网络刷新后覆盖
        app.ai_page.list = crate::api::model::load_ai_sports().unwrap_or_default();
        if app.session.is_some() {
            app.refresh_data_page();
            app.refresh_records();
            app.refresh_user_page();
            app.refresh_ai_list();
            app.refresh_fence();
            app.refresh_points();
        }
        // 已配置 OSM 路网则启动时后台加载
        if !app.config.osm_path.is_empty() {
            let path = app.config.osm_path.clone();
            app.load_osm_async(path);
        }
        app
    }

    /// 后台任务（线程 + 日志 channel）。
    pub fn spawn_job<F>(&self, f: F)
    where
        F: FnOnce(Sender<String>) + Send + 'static,
    {
        let tx = self.tx.clone();
        std::thread::spawn(move || f(tx));
    }

    pub fn logger(tx: Sender<String>) -> impl FnMut(&str) {
        move |s: &str| {
            tx.send(s.to_string()).ok();
        }
    }
}

/// 状态栏右半区（版本 / 检查更新 / 署名）。
///
/// 返回 true 表示用户点了「检查更新」。抽成独立函数是为了让宽屏（右对齐）
/// 和窄屏（另起一行）共用同一份内容，避免两处各写一遍产生漂移。
fn draw_right_side(
    ui: &mut egui::Ui,
    small: impl Fn(String) -> egui::RichText,
    update: &about::UpdateUi,
) -> bool {
    ui.hyperlink_to(
        egui::RichText::new("YanamiNeko")
            .small()
            .color(theme::text_dim()),
        "https://github.com/YanamiNeko",
    );
    ui.label(small("Powered by".into()));
    ui.separator();
    let clicked = ui
        .add_enabled(
            !(update.checking || update.downloading),
            egui::Button::new(
                egui::RichText::new("检查更新")
                    .small()
                    .color(theme::text_dim()),
            )
            .frame(false),
        )
        .clicked();
    ui.label(small(format!("v{}", crate::platform::version_name())));
    clicked
}
