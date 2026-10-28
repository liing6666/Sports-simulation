//! 「路网」导入页：选择 OSM 文件 → 后台解析 → 画布预览 + 统计。

use eframe::egui;
use std::sync::Arc;

use route_planner::{Coord, RoadGraph};

use super::map::{collect_bounds, MapState};
use super::{mobile, theme, App};

#[derive(Default)]
pub struct OsmPage {
    pub path: String,
    pub busy: bool,
    pub status: String,
    pub map: MapState,
    pub fitted: bool,
    /// 当前路网来源：manual（本地文件）/ school（按学校围栏自动下载）/
    /// cache（从「已下载的学校路网」里载入）。
    pub source: String,
    /// 已下载的学校路网缓存列表。
    pub cache: Vec<crate::api::school_roads::Entry>,
    /// 缓存列表是否需要重新读取（首次打开/删除/新下载后置 false）。
    pub cache_loaded: bool,
    /// 强制刷新（忽略 7 天缓存）。
    pub refresh: bool,
    /// 校名搜索关键词。
    pub campus_query: String,
    /// 搜索结果（候选校区）。
    pub campus_items: Vec<route_planner::Campus>,
    /// 搜索/确认状态提示。
    pub campus_status: String,
    /// 搜索进行中。
    pub campus_busy: bool,
    /// 当前路网下载的取消句柄；点「取消」时由界面线程置位。
    ///
    /// 下载可能要跨多个镜像退避，没有它用户只能强杀进程。
    pub osm_control: std::sync::Arc<crate::task_control::Control>,
}

/// 后台加载成功的结果：路网 + 它对应的 `config.osm_path`。
///
/// 跑步主流程只认 `config.osm_path`，所以每次加载都必须明确「该把配置指向哪」。
#[derive(Clone)]
pub struct NetLoaded {
    pub net: Arc<RoadGraph>,
    /// 应写入 `config.osm_path` 的路径。`None` 表示保持现状（旧行为兜底）。
    pub osm_path: Option<std::path::PathBuf>,
}

fn ring_to_pts(ring: &[Coord]) -> Vec<(f64, f64)> {
    ring.iter().map(|c| (c.lat, c.lon)).collect()
}

/// 在后台线程执行一次路网加载，**保证无论成功、报错还是 panic 都会回消息**。
///
/// 界面靠 `net_rx` 收到结果才把 `osm_page.busy` 复位。线程里一旦 panic
/// （索引越界、内存不足等）就不再发送，按钮会永久禁用，只能重启程序。
/// 这里统一兜住 panic 并转成普通错误。
fn spawn_net_job<F>(tx: std::sync::mpsc::Sender<Result<NetLoaded, String>>, job: F)
where
    F: FnOnce() -> Result<NetLoaded, String> + std::panic::UnwindSafe + Send + 'static,
{
    std::thread::spawn(move || {
        let res = match std::panic::catch_unwind(job) {
            Ok(r) => r,
            Err(payload) => {
                let detail = payload
                    .downcast_ref::<&str>()
                    .map(|s| (*s).to_string())
                    .or_else(|| payload.downcast_ref::<String>().cloned())
                    .unwrap_or_else(|| "未知原因".into());
                Err(format!("内部错误：路网加载线程异常退出（{detail}）"))
            }
        };
        let _ = tx.send(res);
    });
}

/// 把内存里的路网导出成跑步主流程能读的文件。
///
/// **导出失败不判定为加载失败**：路网仍能用于地图预览与跑步页预览，只是
/// `api::flow` 会回退经典算法。因此返回 `osm_path = None`，由状态栏明确
/// 警告「无法用于跑步」。若这里返回 Err，用户会连预览都失去，而且错误信息
/// 会变成「路网加载失败」——与真实情况不符。
fn export_for_run(key: &str) -> Option<std::path::PathBuf> {
    match crate::api::school_roads::materialize(key) {
        Ok(path) => Some(path),
        Err(e) => {
            eprintln!("[osm] 路网已加载，但导出给跑步流程失败：{e}");
            None
        }
    }
}

/// 当前锚点对应的已缓存打卡点（BD-09），供自动获取路网确定范围。
///
/// 锚点无效时退回固定坐标只是为了「有个范围可查」——它不会覆盖用户的
/// 定位锚点，下载失败时错误会原样回报（见 road_status）。
fn cached_points_bd(app: &App) -> Vec<(f64, f64)> {
    let anchor = app
        .identity
        .anchor_coordinate()
        .unwrap_or_else(|_| crate::location::Coordinate::new(39.9, 116.4, 0.0).unwrap());
    crate::api::model::load_points_cache_for(anchor)
        .map(|(_ts, pts)| {
            pts.iter()
                .filter_map(|p| Some((p.get("lat")?.as_f64()?, p.get("lon")?.as_f64()?)))
                .collect()
        })
        .unwrap_or_default()
}

/// 缓存下载时间（毫秒时间戳）→ 本地时间 `YYYY-MM-DD HH:MM`。
fn fmt_cache_time(ms: i64) -> String {
    use chrono::TimeZone as _;
    chrono::Local
        .timestamp_millis_opt(ms)
        .single()
        .map(|t| t.format("%Y-%m-%d %H:%M").to_string())
        .unwrap_or_else(|| "时间未知".into())
}

impl App {
    pub fn draw_osm_page(&mut self, ui: &mut egui::Ui) {
        let mut load_now = false;

        // ── 自动获取（按学校围栏）────────────────────────────
        ui.horizontal(|ui| {
            let can_auto = !self.osm_page.busy && self.session.is_some();
            if ui
                .add_enabled(can_auto, theme::primary_btn("自动获取学校路网"))
                .clicked()
            {
                self.auto_load_osm_async();
            }
            ui.checkbox(&mut self.osm_page.refresh, "强制刷新（忽略 7 天缓存）");
            if self.osm_page.busy {
                // 下载可能跨多个镜像退避数分钟；没有这个按钮用户只能强杀进程。
                let control = self.osm_page.osm_control.clone();
                if ui
                    .add_enabled(control.cancellable(), egui::Button::new("取消"))
                    .clicked()
                {
                    control.cancel();
                    self.osm_page.status = "正在取消路网加载…".into();
                }
            }
            if self.session.is_none() {
                ui.label("（需先登录）");
            }
        });
        // ── 校园边界（无服务端围栏时的本地规划约束）──────────
        ui.add_space(6.0);
        ui.separator();
        ui.label(
            egui::RichText::new("校园边界（服务端没有围栏时，作为本地规划约束）")
                .small()
                .color(theme::plain()),
        );
        let mut want_search = false;
        let mut want_near = false;
        ui.horizontal(|ui| {
            ui.label("校名：");
            ui.add_sized(
                [240.0, 24.0],
                egui::TextEdit::singleline(&mut self.osm_page.campus_query)
                    .hint_text("例如：北京大学"),
            );
            if ui
                .add_enabled(
                    !self.osm_page.campus_busy && !self.osm_page.campus_query.trim().is_empty(),
                    egui::Button::new("搜索校园"),
                )
                .clicked()
            {
                want_search = true;
            }
            if self.osm_page.campus_busy {
                ui.label("搜索中…");
            }
        });
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("找不到？用定位锚点按范围找（不依赖 OSM 标签）：")
                    .small()
                    .color(theme::text_dim()),
            );
            if ui
                .add_enabled(!self.osm_page.campus_busy, egui::Button::new("按锚点范围查找"))
                .on_hover_text("取定位锚点周围 800 米内所有闭合区域，可能包含操场/建筑，需目视确认")
                .clicked()
            {
                want_near = true;
            }
        });
        if !self.osm_page.campus_status.is_empty() {
            ui.add(egui::Label::new(&self.osm_page.campus_status).wrap());
        }
        if !self.osm_page.campus_items.is_empty() {
            let mut choose: Option<usize> = None;
            egui::ScrollArea::vertical()
                .max_height(110.0)
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    for (i, c) in self.osm_page.campus_items.iter().enumerate() {
                        let current = self
                            .config
                            .map_boundary
                            .as_ref()
                            .map(|s| s.id == c.id)
                            .unwrap_or(false);
                        let label = format!(
                            "{}{}",
                            if current { "● " } else { "○ " },
                            c.name
                        );
                        if ui.selectable_label(current, label).clicked() {
                            choose = Some(i);
                        }
                    }
                });
            if let Some(i) = choose {
                let campus = self.osm_page.campus_items[i].clone();
                let rev = crate::api::map_boundary::revision(&campus);
                let school = self
                    .session
                    .as_ref()
                    .map(|s| s.unid.clone())
                    .unwrap_or_default();
                self.config.map_boundary = Some(crate::api::map_boundary::Selection {
                    school,
                    id: campus.id.clone(),
                    revision: rev,
                });
                let _ = crate::api::model::save_config(&self.config);
                self.osm_page.campus_status = format!("已确认校园边界：{}", campus.name);
                self.run_page.preview_stale = true;
            }
        }
        if want_search {
            self.search_campus_async();
        }
        if want_near {
            self.search_campus_near_async();
        }
        ui.separator();
        ui.add_space(4.0);

        ui.horizontal(|ui| {
            ui.label("OSM 文件：");
            ui.add_sized(
                [340.0, 26.0],
                egui::TextEdit::singleline(&mut self.osm_page.path)
                    .hint_text("拖入 .osm 文件或填写路径"),
            );
            #[cfg(not(target_os = "android"))]
            if ui.button("浏览…").clicked() {
                if let Some(p) = rfd::FileDialog::new()
                    .add_filter("OSM", &["osm", "xml", "pbf"])
                    .pick_file()
                {
                    self.osm_page.path = p.display().to_string();
                }
            }
            let can_load = !self.osm_page.path.is_empty() && !self.osm_page.busy;
            if ui
                .add_enabled(can_load, theme::primary_btn("加载路网"))
                .clicked()
            {
                load_now = true;
            }
        });
        // 手动下载入口：自动获取失败或需要更大范围时，用户可到 OSM 自己框选导出。
        ui.horizontal(|ui| {
            ui.label(
                egui::RichText::new("没有路网文件？到 OpenStreetMap 框选区域导出 .osm：")
                    .small()
                    .color(theme::text_dim()),
            );
            ui.hyperlink_to(
                egui::RichText::new("https://www.openstreetmap.org")
                    .small()
                    .color(theme::PRIMARY),
                "https://www.openstreetmap.org",
            );
            ui.label(
                egui::RichText::new("（导出 → 导出为 .osm 文件）")
                    .small()
                    .color(theme::text_dim()),
            );
        });

        // 拖拽导入
        let dropped: Vec<String> = ui.ctx().input(|i| {
            i.raw
                .dropped_files
                .iter()
                .filter_map(|f| f.path.as_ref().map(|p| p.display().to_string()))
                .collect()
        });
        if let Some(p) = dropped.into_iter().find(|p| !p.is_empty()) {
            self.osm_page.path = p;
            if !self.osm_page.busy {
                load_now = true;
            }
        }

        if !self.osm_page.status.is_empty() {
            // 失败时会带 "[分类] 原因\n建议：…"，多行需自动换行而不是撑出面板。
            ui.add(egui::Label::new(&self.osm_page.status).wrap());
        }

        // ── 已下载的学校路网缓存 ──────────────────────────────
        if !self.osm_page.cache_loaded {
            self.osm_page.cache = crate::api::school_roads::entries();
            self.osm_page.cache_loaded = true;
        }
        let mut load_key: Option<String> = None;
        let mut delete_key: Option<String> = None;
        egui::CollapsingHeader::new(format!(
            "已下载的学校路网（{}）",
            self.osm_page.cache.len()
        ))
        .id_salt("school_road_cache")
        .default_open(false)
        .show(ui, |ui| {
            if self.osm_page.cache.is_empty() {
                ui.label(
                    egui::RichText::new("还没有下载过学校路网")
                        .small()
                        .color(theme::plain()),
                );
                return;
            }
            egui::ScrollArea::vertical()
                .max_height(140.0)
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    for e in &self.osm_page.cache {
                        mobile::row(ui, |ui| {
                            let school = if e.school.trim().is_empty() {
                                "未知学校".to_string()
                            } else {
                                format!("学校 {}", e.school)
                            };
                            let label = if e.name.trim().is_empty() {
                                school
                            } else {
                                format!("{school} · {}", e.name)
                            };
                            ui.label(label);
                            ui.label(
                                egui::RichText::new(fmt_cache_time(e.loaded_ms))
                                    .color(theme::plain()),
                            );
                            ui.label(format!("{:.0} KB", e.bytes as f64 / 1024.0));
                            let busy = self.osm_page.busy;
                            if ui
                                .add_enabled(!busy, egui::Button::new("载入").small())
                                .clicked()
                            {
                                load_key = Some(e.key.clone());
                            }
                            if ui.small_button("删除").clicked() {
                                delete_key = Some(e.key.clone());
                            }
                        });
                    }
                });
        });
        if let Some(key) = load_key {
            self.load_road_cache_async(key);
        }
        if let Some(key) = delete_key {
            match crate::api::school_roads::clear_cache(&key) {
                Ok(()) => {
                    self.osm_page.status = "已删除学校路网缓存".into();
                    self.osm_page.cache_loaded = false;
                }
                Err(e) => self.osm_page.status = format!("删除缓存失败：{e}"),
            }
        }

        // 统计
        if let Some(net) = &self.network {
            ui.horizontal(|ui| {
                ui.label(format!(
                    "节点 {} · 有向边 {} · 建筑 {}",
                    net.graph.node_count(),
                    net.graph.edge_count(),
                    net.buildings.len()
                ));
                if let Ok(meta) = std::fs::metadata(&self.osm_page.path) {
                    ui.label(format!("· 文件 {:.1} KB", meta.len() as f64 / 1024.0));
                }
            });
        }

        ui.add_space(4.0);
        if self.network.is_some() {
            super::map::legend(
                ui,
                &[
                    ("道路", egui::Color32::from_rgb(190, 200, 196)),
                    ("建筑", egui::Color32::from_rgb(214, 182, 158)),
                ],
            );
        }
        let rect = ui.available_rect_before_wrap();
        let (response, painter) = ui.allocate_painter(
            egui::Vec2::new(rect.width().max(200.0), rect.height().max(220.0)),
            egui::Sense::drag(),
        );
        let canvas = response.rect;

        if let Some(net) = &self.network {
            if !self.osm_page.fitted {
                let mut bounds: Vec<(f64, f64)> = collect_bounds(
                    &net.edges_deg()
                        .iter()
                        .map(|e| ring_to_pts(e))
                        .collect::<Vec<_>>(),
                );
                for b in &net.buildings {
                    bounds.extend(ring_to_pts(b));
                }
                self.osm_page.map.fit(canvas, &bounds);
                self.osm_page.fitted = true;
            }
        }

        painter.rect_filled(
            canvas,
            egui::Rounding::ZERO,
            egui::Color32::from_rgb(250, 252, 251),
        );
        if let Some(net) = &self.network {
            let road_color = egui::Color32::from_rgb(190, 200, 196);
            for e in net.edges_deg() {
                self.osm_page.map.draw_polyline(
                    &painter,
                    canvas,
                    &ring_to_pts(&e),
                    road_color,
                    1.0,
                );
            }
            let bld_color = egui::Color32::from_rgb(214, 182, 158);
            for b in &net.buildings {
                self.osm_page
                    .map
                    .draw_polygon(&painter, canvas, &ring_to_pts(b), bld_color, 1.0);
            }
        }
        // 已确认/候选的校园边界：画在路网之上，便于核对是否覆盖操场。
        // 校园轮廓是 WGS84，路网已对齐到 BD-09，这里先转换再画。
        for (i, campus) in self.osm_page.campus_items.iter().enumerate() {
            let confirmed = self
                .config
                .map_boundary
                .as_ref()
                .map(|s| s.id == campus.id)
                .unwrap_or(false);
            let color = if confirmed {
                egui::Color32::from_rgb(37, 99, 235)
            } else {
                egui::Color32::from_rgb(160, 174, 192)
            };
            let width = if confirmed { 2.5 } else { 1.0 };
            let _ = i;
            for ring in campus.outer.iter().chain(campus.inner.iter()) {
                let pts: Vec<(f64, f64)> = ring
                    .iter()
                    .map(|c| crate::track::geom::wgs84_to_bd09(c.lat, c.lon))
                    .collect();
                self.osm_page
                    .map
                    .draw_polyline(&painter, canvas, &pts, color, width);
            }
        }
        if self.network.is_none() {
            painter.text(
                canvas.center(),
                egui::Align2::CENTER_CENTER,
                "加载 OSM 路网后可预览道路与建筑",
                egui::FontId::proportional(14.0),
                egui::Color32::from_rgb(150, 160, 156),
            );
        }

        self.osm_page.map.interact(ui, canvas);

        if load_now {
            let path = self.osm_page.path.clone();
            self.load_osm_async(path);
        }
    }

    /// 按当前账号的学校围栏自动下载路网（失败时用真实点位范围兜底）。
    ///
    /// 全程不进主线程：下载 + 解析放后台线程，结果经 `net_rx` 回传。
    pub fn auto_load_osm_async(&mut self) {
        let Some(sess) = self.session.clone() else {
            self.osm_page.status = "请先登录：自动获取路网需要学校编号".into();
            return;
        };
        if self.osm_page.busy {
            return;
        }
        self.osm_page.busy = true;
        self.osm_page.source = "school".into();
        self.osm_page.status = "正在获取学校围栏并下载路网…".into();

        let fences = crate::api::model::load_fence_cache().unwrap_or_default();
        let points = cached_points_bd(self);

        let tx = self.net_tx.clone();
        let school = sess.unid.clone();
        let refresh = self.osm_page.refresh;
        // 取消句柄归界面所有：后台线程只持 clone，用户点「取消」能立刻生效。
        let control = std::sync::Arc::new(crate::task_control::Control::default());
        self.osm_page.osm_control = control.clone();
        spawn_net_job(tx, move || {
            let mut net = crate::api::school_roads::load_for_route(
                "",
                &school,
                &fences,
                &points,
                refresh,
                &control,
                &mut |_| {},
            )?;
            // 导出成 `config.osm_path` 指向的文件，跑步主流程（api::flow）才读得到。
            let key = crate::api::school_roads::route_cache_key(&school, &fences, &points);
            let osm_path = export_for_run(&key);
            // 下载得到的是 WGS84 原始路网，必须对齐到 BD-09 才能和打卡点同系。
            crate::track::generate_road::align_network(&mut net);
            Ok(NetLoaded {
                net: std::sync::Arc::new(net),
                osm_path,
            })
        });
    }

    /// 后台搜索校园边界（主用学校名，失败时可用备用 Nominatim）。
    /// 按定位锚点范围查找校园边界（不依赖 OSM 标签）。
    ///
    /// 适合「刚在 OSM 画了边界但没打 amenity 标签」的场景：直接取锚点周围
    /// 所有闭合多边形，让用户目视确认。
    pub fn search_campus_near_async(&mut self) {
        if self.osm_page.campus_busy {
            return;
        }
        let anchor = match self.identity.anchor_coordinate() {
            Ok(a) => a,
            Err(e) => {
                self.osm_page.campus_status = format!("定位锚点无效：{e}（请到设备信息页填写）");
                return;
            }
        };
        if self.identity.has_unconfigured_default_location() {
            self.osm_page.campus_status =
                "请先到设备信息页填写跑步所在城市和定位锚点".into();
            return;
        }
        self.osm_page.campus_busy = true;
        self.osm_page.campus_status = format!(
            "正在查找 ({:.5},{:.5}) 周围 {} 米内的闭合区域…",
            anchor.latitude, anchor.longitude, 800
        );
        let school = self
            .session
            .as_ref()
            .map(|s| s.unid.clone())
            .unwrap_or_default();
        let queue = self.campus_tx.clone();
        std::thread::spawn(move || {
            let control = crate::task_control::Control::default();
            let res = crate::api::map_boundary::lookup_near(&school, anchor, 800.0, &control);
            let _ = queue.send(res);
        });
    }

    pub fn search_campus_async(&mut self) {
        let name = self.osm_page.campus_query.trim().to_string();
        if name.is_empty() {
            return;
        }
        self.osm_page.campus_busy = true;
        self.osm_page.campus_status = "正在搜索校园边界…".into();
        let school = self
            .session
            .as_ref()
            .map(|s| s.unid.clone())
            .unwrap_or_default();
        let queue = self.campus_tx.clone();
        let endpoint = crate::api::map_search::default_endpoint();
        std::thread::spawn(move || {
            let control = crate::task_control::Control::default();
            let res = crate::api::map_boundary::discover_with_backup(
                &school, &name, crate::location::Coordinate::new(0.0, 0.0, 0.0).unwrap(),
                false, &endpoint, &control,
            );
            let _ = queue.send(res);
        });
    }

    /// 从「已下载的学校路网」里载入一份缓存（后台解析，经 `net_rx` 回传）。
    pub fn load_road_cache_async(&mut self, key: String) {
        if self.osm_page.busy {
            return;
        }
        self.osm_page.source = "cache".into();
        self.osm_page.busy = true;
        self.osm_page.status = "正在载入缓存的学校路网…".into();
        let tx = self.net_tx.clone();
        let control = std::sync::Arc::new(crate::task_control::Control::default());
        self.osm_page.osm_control = control.clone();
        spawn_net_job(tx, move || {
            control.check()?;
            let (mut net, _) = crate::api::school_roads::read_cache(&key)?;
            let osm_path = export_for_run(&key);
            // 缓存里存的是 OSM 原始 WGS84 路网；read_cache 只做解析，
            // 这里必须对齐到 BD-09，否则路网会偏出打卡点约 950 米。
            crate::track::generate_road::align_network(&mut net);
            Ok(NetLoaded {
                net: Arc::new(net),
                osm_path,
            })
        });
    }

    /// 后台线程加载本地 OSM 文件，成功后经 `net_rx` 回传。
    pub fn load_osm_async(&mut self, path: String) {
        self.osm_page.source = "manual".into();
        self.osm_page.path = path.clone();
        self.osm_page.busy = true;
        self.osm_page.status = format!("加载中：{path} …");
        self.config.osm_path = path.clone();
        let _ = crate::api::model::save_config(&self.config);
        let tx = self.net_tx.clone();
        spawn_net_job(tx, move || {
            let bytes = std::fs::read(&path).map_err(|e| format!("读取失败: {e}"))?;
            let mut net = route_planner::load_osm(&bytes)?;
            // OSM 是 WGS84，打卡点/围栏是 BD-09。必须在加载时统一坐标系，
            // 否则两者相差约 950 米（纬度 400m + 经度 870m），
            // 地图上环会画到路网外面去。
            crate::track::generate_road::align_network(&mut net);
            // 用户选的路径本身就是 config.osm_path（见 load_osm_async 开头）。
            let osm_path = Some(std::path::PathBuf::from(&path));
            Ok(NetLoaded {
                net: Arc::new(net),
                osm_path,
            })
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    /// 线程 panic 也必须回传一个 Err：否则界面永远收不到结果，
    /// `osm_page.busy` 卡在 true，所有路网按钮永久禁用直到重启。
    #[test]
    fn panicking_job_still_reports_back_so_busy_can_reset() {
        let (tx, rx) = std::sync::mpsc::channel();
        spawn_net_job(tx, || panic!("模拟后台线程崩溃"));
        let res = rx
            .recv_timeout(Duration::from_secs(5))
            .expect("线程 panic 后必须仍有消息回传");
        // NetLoaded 没有 Debug，不能 expect_err()；手动 match 取错误串。
        let err = match res {
            Err(e) => e,
            Ok(_) => panic!("panic 应转成错误而不是成功"),
        };
        assert!(err.contains("内部错误"), "错误信息应标明内部错误: {err}");
        assert!(err.contains("模拟后台线程崩溃"), "应保留 panic 信息: {err}");
    }

    /// 正常路径不能被 catch_unwind 改变行为。
    #[test]
    fn successful_job_reports_ok() {
        let (tx, rx) = std::sync::mpsc::channel();
        spawn_net_job(tx, || {
            Err::<NetLoaded, String>("普通失败".into())
        });
        let err = match rx.recv_timeout(Duration::from_secs(5)).expect("应收到消息") {
            Err(e) => e,
            Ok(_) => panic!("构造的是 Err"),
        };
        assert_eq!(err, "普通失败");
    }
}
