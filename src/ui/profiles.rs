//! 配置档位页：保存/应用/删除多套跑步参数（可绑定账号）。
//!
//! 与"账号"解耦：档位可以只换参数（如「5km 冲量」「操场 400」），也可以顺带
//! 绑定另一个账号。应用档位会把字段写进 `config.json` 并立即落盘，因此其余
//! 页面读到的是同一份配置，无需改动任何既有代码路径。

use super::{mobile, theme, App};
use crate::api::profiles::{ProfileBook, ProfileSlot};
#[cfg(not(target_os = "android"))]
use crate::api::profiles::ProfileExport;
use eframe::egui;

#[derive(Default)]
pub struct ProfilesPage {
    /// 编辑中的档位名。
    pub new_name: String,
    /// 编辑中的账号（可选）。
    pub new_username: String,
    /// 编辑中的密码（可选）。
    pub new_password: String,
    pub new_remember: bool,
    /// 待确认删除的下标。
    pub pending_delete: Option<usize>,
    /// 导出时是否保留密码（默认否：脱敏导出）。
    pub export_with_password: bool,
    /// 上一次操作结果提示。
    pub status: String,
}

impl App {
    pub fn draw_profiles(&mut self, ui: &mut egui::Ui) {
        let mut book = ProfileBook::load();

        ui.add_space(4.0);
        ui.label(
            egui::RichText::new("每个档位保存一组跑步参数，可另绑账号；应用后立即写入 config.json。")
                .small(),
        );
        if !self.profiles_page.status.is_empty() {
            ui.colored_label(theme::plain(), &self.profiles_page.status);
        }
        ui.separator();

        // ── 现有档位列表 ──────────────────────────────────────
        egui::ScrollArea::vertical()
            .max_height(320.0)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                if book.slots.is_empty() {
                    ui.label("还没有档位。下方填名称后点「保存当前配置」。");
                }
                let mut apply: Option<usize> = None;
                let mut delete: Option<usize> = None;
                let mut overwrite: Option<usize> = None;
                // 先克隆出快照，避免在闭包里同时可变/不可变借用 book。
                let slots: Vec<(usize, ProfileSlot, bool)> = book
                    .slots
                    .iter()
                    .enumerate()
                    .map(|(i, s)| (i, s.clone(), book.active == Some(i)))
                    .collect();
                for (index, slot, active) in slots {
                    let slot = &slot;
                    ui.horizontal(|ui| {
                        let title = if slot.name.trim().is_empty() {
                            format!("档位 {}", index + 1)
                        } else {
                            slot.name.clone()
                        };
                        let label = if active {
                            format!("● {title}")
                        } else {
                            format!("○ {title}")
                        };
                        ui.label(egui::RichText::new(label).strong());
                        if !slot.username.is_empty() {
                            ui.colored_label(theme::plain(), format!("账号 {}", mask_user(&slot.username)));
                        }
                        ui.label(
                            egui::RichText::new(slot_summary(slot))
                                .small()
                                .color(theme::plain()),
                        );
                        if !active && ui.small_button("应用").clicked() {
                            apply = Some(index);
                        }
                        if ui.small_button("覆盖保存").clicked() {
                            overwrite = Some(index);
                        }
                        if ui.small_button("删除").clicked() {
                            delete = Some(index);
                        }
                    });
                }
                if let Some(index) = overwrite {
                    let name = book.slots[index].name.clone();
                    book.slots[index] = ProfileBook::capture(&name, &self.config);
                    let _ = book.save();
                    self.profiles_page.status = format!("已用当前配置覆盖「{name}」");
                }
                if let Some(index) = apply {
                    self.apply_profile(&mut book, index);
                }
                if let Some(index) = delete {
                    let name = book.slots[index].name.clone();
                    book.slots.remove(index);
                    if book.active == Some(index) {
                        book.active = None;
                    } else if let Some(active) = book.active {
                        if active > index {
                            book.active = Some(active - 1);
                        }
                    }
                    let _ = book.save();
                    self.profiles_page.status = format!("已删除「{name}」");
                }
            });

        ui.separator();

        // ── 新建档位 ─────────────────────────────────────────
        ui.label("保存当前配置为新档位");
        mobile::row(ui, |ui| {
            if ui
                .add(egui::TextEdit::singleline(&mut self.profiles_page.new_name).hint_text("档位名称"))
                .changed()
            {}
        });
        mobile::row(ui, |ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.profiles_page.new_username)
                    .hint_text("账号（可留空，仅切换参数）"),
            );
        });
        if !self.profiles_page.new_username.trim().is_empty() {
            mobile::row(ui, |ui| {
                ui.add(
                    egui::TextEdit::singleline(&mut self.profiles_page.new_password)
                        .password(true)
                        .hint_text("密码（仅在勾选记住时保存）"),
                );
            });
            mobile::row(ui, |ui| {
                ui.checkbox(&mut self.profiles_page.new_remember, "记住密码");
            });
        }
        mobile::row(ui, |ui| {
            if ui.button("保存当前配置").clicked() {
                let name = self.profiles_page.new_name.trim().to_string();
                let name = if name.is_empty() {
                    format!("档位 {}", book.slots.len() + 1)
                } else {
                    name
                };
                let username = self.profiles_page.new_username.trim().to_string();
                let password = self.profiles_page.new_password.clone();
                let remember = self.profiles_page.new_remember;

                let mut mut_config = self.config.clone();
                if !username.is_empty() {
                    mut_config.username = username;
                    mut_config.password = password;
                    mut_config.remember = remember;
                }
                let slot = ProfileBook::capture(&name, &mut_config);
                book.slots.push(slot);
                let _ = book.save();
                self.profiles_page.new_name.clear();
                self.profiles_page.new_username.clear();
                self.profiles_page.new_password.clear();
                self.profiles_page.new_remember = false;
                self.profiles_page.status = format!("已保存档位「{name}」");
            }
        });

        ui.add_space(8.0);
        ui.separator();

        // ── 导入 / 导出（仅桌面；Android 无文件对话框后端）────────
        #[cfg(not(target_os = "android"))]
        {
            ui.label("备份与迁移");
            mobile::row(ui, |ui| {
                ui.checkbox(
                    &mut self.profiles_page.export_with_password,
                    "导出时包含密码（文件将含明文密码，请妥善保管）",
                );
            });
            mobile::row(ui, |ui| {
                if ui.button("导出档位…").clicked() {
                    self.export_profiles(&book);
                }
                if ui.button("导入档位…").clicked() {
                    self.import_profiles(&mut book);
                }
            });
        }

        ui.add_space(8.0);
        ui.separator();
        ui.label(
            egui::RichText::new(
                "说明：应用档位会覆盖 config.json 里对应的字段；账号那一栏留空的档位不会改动当前登录账号。",
            )
            .small()
            .color(theme::plain()),
        );
    }

    #[cfg(not(target_os = "android"))]
    fn export_profiles(&mut self, book: &ProfileBook) {
        if book.slots.is_empty() {
            self.profiles_page.status = "还没有档位可导出".into();
            return;
        }
        let include_password = self.profiles_page.export_with_password;
        let json = match crate::api::profiles::export_json(book, include_password) {
            Ok(json) => json,
            Err(e) => {
                self.profiles_page.status = format!("导出失败：{e}");
                return;
            }
        };
        let default_name = format!(
            "neko-profiles-{}.json",
            chrono::Local::now().format("%Y%m%d-%H%M%S")
        );
        let Some(path) = rfd::FileDialog::new()
            .add_filter("JSON", &["json"])
            .set_file_name(default_name)
            .save_file()
        else {
            return;
        };
        match std::fs::write(&path, json) {
            Ok(()) => {
                let note = if include_password {
                    "（含明文密码）"
                } else {
                    "（已脱敏，不含密码）"
                };
                self.profiles_page.status =
                    format!("已导出 {} 个档位到 {}{note}", book.slots.len(), path.display());
            }
            Err(e) => self.profiles_page.status = format!("写入文件失败：{e}"),
        }
    }

    #[cfg(not(target_os = "android"))]
    fn import_profiles(&mut self, book: &mut ProfileBook) {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("JSON", &["json"])
            .pick_file()
        else {
            return;
        };
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(e) => {
                self.profiles_page.status = format!("读取文件失败：{e}");
                return;
            }
        };
        let imported = match ProfileExport::read(&text) {
            Ok(slots) => slots,
            Err(e) => {
                self.profiles_page.status = format!("导入失败：{e}");
                return;
            }
        };
        let count = imported.len();
        let added = crate::api::profiles::merge_slots(book, imported);
        if let Err(e) = book.save() {
            self.profiles_page.status = format!("导入后保存失败：{e}");
            return;
        }
        let names = if added.len() <= 3 {
            added.join("、")
        } else {
            let preview: Vec<&str> = added.iter().take(3).map(String::as_str).collect();
            format!("{} 等", preview.join("、"))
        };
        self.profiles_page.status = format!("已导入 {count} 个档位：{names}");
    }

    fn apply_profile(&mut self, book: &mut ProfileBook, index: usize) {
        let slot = book.slots[index].clone();
        let name = slot.name.clone();
        let mut config = self.config.clone();
        ProfileBook::apply_to(&slot, &mut config);
        if let Err(e) = crate::api::model::save_config(&config) {
            self.profiles_page.status = format!("保存配置失败：{e}");
            return;
        }
        self.config = config;
        // 让跑步页从新配置重新读取一次，避免界面显示旧值。
        self.run_page.dist_min = self.config.dist_min;
        self.run_page.dist_max = self.config.dist_max;
        self.run_page.pace_min = self.config.pace_min;
        self.run_page.pace_max = self.config.pace_max;
        self.run_page.face_check = self.config.face_check;
        self.run_page.ordinary_pick = self.config.ordinary_pick as i64;
        self.run_page.route_mode =
            crate::track::generate_road::RouteMode::from_str(&self.config.route_mode);
        self.run_page.track_color_mode = self.config.track_color_mode;
        self.run_page.preview_stale = true;

        if !slot.username.is_empty() {
            self.username = self.config.username.clone();
            self.password = self.config.password.clone();
            self.remember = self.config.remember;
        }

        book.active = Some(index);
        let _ = book.save();

        let re_login = if slot.username.is_empty() {
            String::new()
        } else {
            "（已切换账号信息，请重新登录）".to_string()
        };
        self.profiles_page.status = format!("已应用档位「{name}」{re_login}");
    }
}

fn mask_user(user: &str) -> String {
    crate::textlog::mask(user)
}

fn slot_summary(slot: &ProfileSlot) -> String {
    let route = match slot.route_mode.as_str() {
        "road" => "真实道路",
        "custom" => "自定义",
        _ => "经典",
    };
    let altitude = if slot.manual_altitude_enabled {
        match (slot.manual_altitude, slot.manual_altitude_range) {
            (_, Some(r)) => format!(" 海拔{:.0}-{:.0}m", r.min_m, r.max_m),
            (Some(v), None) => format!(" 海拔{v:.1}m"),
            _ => String::new(),
        }
    } else {
        String::new()
    };
    format!(
        "{}–{} km · {:.0}–{:.0} s/km · {} · 点名 {}{}",
        slot.dist_min, slot.dist_max, slot.pace_min, slot.pace_max, route, slot.ordinary_pick, altitude
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn summary_reports_route_and_altitude() {
        let slot = ProfileSlot {
            dist_min: 2.0,
            dist_max: 3.0,
            pace_min: 330.0,
            pace_max: 380.0,
            route_mode: "road".into(),
            ordinary_pick: 3,
            manual_altitude_enabled: true,
            manual_altitude: Some(17.5),
            ..Default::default()
        };
        let text = slot_summary(&slot);
        assert!(text.contains("真实道路"));
        assert!(text.contains("17.5m"));
        assert!(text.contains("2"));
    }

    #[test]
    fn user_mask_does_not_leak_full_number() {
        let masked = mask_user("13800001234");
        assert_ne!(masked, "13800001234");
    }
}
