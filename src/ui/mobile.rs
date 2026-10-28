//! Shared responsive and Android input helpers.

use crate::platform::InputKind;
use crate::ui::theme;
use eframe::egui;
use std::{
    hash::Hash,
    ops::{Range, RangeInclusive},
};

pub const COMPACT_BREAKPOINT: f32 = 620.0;
pub const TOUCH_HEIGHT: f32 = 44.0;

pub fn is_compact(width: f32) -> bool {
    width <= COMPACT_BREAKPOINT
}

pub fn compact_ui(ui: &egui::Ui) -> bool {
    is_compact(ui.available_width())
}

pub fn row<R>(
    ui: &mut egui::Ui,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    if compact_ui(ui) {
        ui.horizontal_wrapped(add_contents)
    } else {
        ui.horizontal(add_contents)
    }
}

#[cfg(any(target_os = "android", test))]
pub fn parse_i64_in_range(input: &str, range: RangeInclusive<i64>) -> Result<i64, String> {
    let value = input
        .trim()
        .parse::<i64>()
        .map_err(|_| "请输入整数".to_owned())?;
    if range.contains(&value) {
        Ok(value)
    } else {
        Err(format!("请输入 {} 到 {}", range.start(), range.end()))
    }
}

#[cfg(any(target_os = "android", test))]
pub fn parse_f32_in_range(input: &str, range: RangeInclusive<f32>) -> Result<f32, String> {
    let value = input
        .trim()
        .parse::<f32>()
        .map_err(|_| "请输入数字".to_owned())?;
    if value.is_finite() && range.contains(&value) {
        Ok(value)
    } else {
        Err(format!("请输入 {} 到 {}", range.start(), range.end()))
    }
}

#[cfg(target_os = "android")]
fn parse_f64_in_range(input: &str, range: RangeInclusive<f64>) -> Result<f64, String> {
    let value = input
        .trim()
        .parse::<f64>()
        .map_err(|_| "请输入数字".to_owned())?;
    if value.is_finite() && range.contains(&value) {
        Ok(value)
    } else {
        Err(format!("请输入 {} 到 {}", range.start(), range.end()))
    }
}

/// 单个标签：hover 淡底 + 选中底部 2px 高亮条（不用背景块）。
fn tab_item(ui: &mut egui::Ui, size: egui::Vec2, label: &str, selected: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::click());
    if ui.is_rect_visible(rect) {
        let painter = ui.painter();
        if selected {
            painter.rect_filled(rect, egui::Rounding::same(theme::RADIUS_CONTROL), theme::WHITE);
        } else if response.hovered() {
            painter.rect_filled(
                rect,
                egui::Rounding::same(theme::RADIUS_CONTROL),
                theme::PRIMARY_TINT,
            );
        }

        let color = if selected {
            theme::PRIMARY
        } else {
            theme::TEXT_SECONDARY
        };
        let galley = painter.layout_no_wrap(
            label.to_owned(),
            egui::FontId::proportional(theme::FONT_BODY),
            color,
        );
        let text_pos = rect.center() - galley.size() / 2.0;
        painter.galley(text_pos, galley, color);

        if selected {
            let bar_width = (size.x - theme::GAP_SM * 2.0).max(16.0);
            let bar = egui::Rect::from_center_size(
                egui::pos2(rect.center().x, rect.bottom() - 1.0),
                egui::vec2(bar_width, 2.0),
            );
            painter.rect_filled(bar, egui::Rounding::same(1.0), theme::PRIMARY);
        }
    }
    response
}

/// 把 `total` 项尽量均分成 `rows` 行，返回每行的下标区间（前面的行多 1 项）。
fn balanced_chunks(total: usize, rows: usize) -> Vec<Range<usize>> {
    (0..rows)
        .map(|row| {
            let base = total / rows;
            let extra = total % rows;
            let start = row * base + row.min(extra);
            let len = base + usize::from(row < extra);
            start..start + len
        })
        .collect()
}

pub fn tab_bar(ui: &mut egui::Ui, selected: &mut usize) -> Vec<egui::Response> {
    const TABS: [&str; 10] = [
        "跑步",
        "AI运动",
        "运动记录",
        "数据",
        "我的",
        "设备信息",
        "运行日志",
        "路网",
        "配置档位",
        "关于",
    ];
    let compact = compact_ui(ui);
    let mut responses = Vec::with_capacity(TABS.len());
    if compact {
        // 行内间距置 0，按 4 等分。**不设 68px 下限**：窗口窄时下限会把 4 个
        // 格子撑出可用宽度，最右标签被切掉（实测 360px 下「数据」压在边缘外）。
        // 每格再减 1px 吸收取整误差，保证 4 格之和不超过可用宽度。
        let width = ((ui.available_width() / 4.0) - 1.0).max(1.0);
        // 10 个标签均分 4/3/3（4+4+2 会让末行只剩两个、右侧留半排空位）。
        // 显式分行 + 0 间距：依赖自动换行的等宽网格在边框取整边界上不可靠。
        // 显式垂直排布并按总高度预留空间：CentralPanel 里连续三个
        // `ui.horizontal()` 时，后两行会因为没有预分配高度被挤到面板外
        // （截图实测只有第一行可见）。
        let rows = balanced_chunks(TABS.len(), 3);
        // 必须用 vertical 包住多行：直接在 CentralPanel 里连续 ui.horizontal()
        // 时，后续行不会获得垂直空间，会被挤到面板外（实测只有第一行可见）。
        ui.vertical(|ui| {
            ui.spacing_mut().item_spacing.y = theme::GAP_XS;
            for chunk in rows {
                ui.horizontal(|ui| {
                    ui.spacing_mut().item_spacing.x = 0.0;
                    for (offset, label) in TABS[chunk.clone()].iter().enumerate() {
                        let index = chunk.start + offset;
                        let response = tab_item(
                            ui,
                            egui::vec2(width, TOUCH_HEIGHT),
                            label,
                            *selected == index,
                        );
                        if response.clicked() {
                            *selected = index;
                        }
                        responses.push(response);
                    }
                });
            }
        });
        return responses;
    }
    // 桌面端按文字宽度排布，行内换行兜底，窄窗口下也不会溢出。
    ui.horizontal_wrapped(|ui| {
        for (index, label) in TABS.into_iter().enumerate() {
            let text_width = ui
                .painter()
                .layout_no_wrap(
                    label.to_owned(),
                    egui::FontId::proportional(theme::FONT_BODY),
                    theme::TEXT_SECONDARY,
                )
                .size()
                .x;
            let width = text_width + theme::GAP_MD * 2.0;
            let response = tab_item(ui, egui::vec2(width, 30.0), label, *selected == index);
            if response.clicked() {
                *selected = index;
            }
            responses.push(response);
        }
    });
    responses
}

#[cfg(target_os = "android")]
fn input_width(ui: &egui::Ui, desired: f32) -> f32 {
    desired.min(ui.available_width().max(80.0))
}

#[cfg(target_os = "android")]
fn native_text_button(
    ui: &mut egui::Ui,
    id: egui::Id,
    value: &mut String,
    kind: InputKind,
    desired_width: f32,
) -> egui::Response {
    if let Some(edited) = crate::platform::take_edited_text(id.value()) {
        *value = edited;
    }
    let shown = match kind {
        InputKind::Password if value.is_empty() => "点击输入密码".to_owned(),
        InputKind::Password => "●".repeat(value.chars().count().min(16)),
        _ if value.is_empty() => "点击输入".to_owned(),
        _ => value.clone(),
    };
    let response = ui.add_sized(
        [input_width(ui, desired_width), TOUCH_HEIGHT],
        egui::Button::new(shown),
    );
    if response.clicked() {
        crate::platform::edit_text(id.value(), value, kind);
    }
    response
}

pub fn text_edit(
    ui: &mut egui::Ui,
    id_source: impl Hash,
    value: &mut String,
    kind: InputKind,
    desired_width: f32,
) -> egui::Response {
    let id = ui.make_persistent_id(id_source);
    #[cfg(target_os = "android")]
    {
        native_text_button(ui, id, value, kind, desired_width)
    }
    #[cfg(not(target_os = "android"))]
    {
        let mut edit = egui::TextEdit::singleline(value)
            .id(id)
            .desired_width(desired_width);
        if matches!(kind, InputKind::Password) {
            edit = edit.password(true);
        }
        ui.add(edit)
    }
}

#[cfg(target_os = "android")]
fn show_input_error(ui: &mut egui::Ui, id: egui::Id) {
    if let Some(error) = ui.ctx().data(|data| data.get_temp::<String>(id)) {
        ui.colored_label(crate::ui::theme::err(), error);
    }
}

#[cfg(target_os = "android")]
fn set_input_error(ctx: &egui::Context, id: egui::Id, error: Option<String>) {
    ctx.data_mut(|data| match error {
        Some(error) => data.insert_temp(id, error),
        None => data.remove::<String>(id),
    });
}

#[cfg(any(target_os = "android", test))]
fn numeric_control_shell<R>(
    ui: &mut egui::Ui,
    width: f32,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    let width = width.min(ui.max_rect().width().max(0.0));
    ui.allocate_ui_with_layout(
        egui::vec2(width, TOUCH_HEIGHT),
        egui::Layout::top_down(egui::Align::Min),
        |ui| {
            ui.set_min_width(width);
            ui.set_max_width(width);
            add_contents(ui)
        },
    )
}

pub fn drag_i64(
    ui: &mut egui::Ui,
    id_source: impl Hash,
    value: &mut i64,
    range: RangeInclusive<i64>,
    speed: f64,
    prefix: &str,
    suffix: &str,
) -> egui::Response {
    #[cfg(target_os = "android")]
    {
        let _ = speed;
        let id = ui.make_persistent_id(id_source);
        let error_id = id.with("error");
        let inner = numeric_control_shell(ui, 120.0, |ui| {
            if let Some(edited) = crate::platform::take_edited_text(id.value()) {
                match parse_i64_in_range(&edited, range.clone()) {
                    Ok(parsed) => {
                        *value = parsed;
                        set_input_error(ui.ctx(), error_id, None);
                    }
                    Err(error) => set_input_error(ui.ctx(), error_id, Some(error)),
                }
            }
            let shown = format!("{prefix}{value}{suffix}");
            let response = ui.add_sized([120.0, TOUCH_HEIGHT], egui::Button::new(shown));
            if response.clicked() {
                crate::platform::edit_text(id.value(), &value.to_string(), InputKind::Integer);
            }
            show_input_error(ui, error_id);
            response
        });
        inner.inner
    }
    #[cfg(not(target_os = "android"))]
    {
        let _ = id_source;
        ui.add(
            egui::DragValue::new(value)
                .range(range)
                .speed(speed)
                .prefix(prefix)
                .suffix(suffix),
        )
    }
}

pub fn drag_f32(
    ui: &mut egui::Ui,
    id_source: impl Hash,
    value: &mut f32,
    range: RangeInclusive<f32>,
    speed: f64,
    decimals: usize,
    suffix: &str,
) -> egui::Response {
    #[cfg(target_os = "android")]
    {
        let _ = speed;
        let id = ui.make_persistent_id(id_source);
        let error_id = id.with("error");
        let inner = numeric_control_shell(ui, 120.0, |ui| {
            if let Some(edited) = crate::platform::take_edited_text(id.value()) {
                match parse_f32_in_range(&edited, range.clone()) {
                    Ok(parsed) => {
                        *value = parsed;
                        set_input_error(ui.ctx(), error_id, None);
                    }
                    Err(error) => set_input_error(ui.ctx(), error_id, Some(error)),
                }
            }
            let shown = format!("{:.*}{suffix}", decimals, *value);
            let response = ui.add_sized([120.0, TOUCH_HEIGHT], egui::Button::new(shown));
            if response.clicked() {
                crate::platform::edit_text(id.value(), &value.to_string(), InputKind::Decimal);
            }
            show_input_error(ui, error_id);
            response
        });
        inner.inner
    }
    #[cfg(not(target_os = "android"))]
    {
        let _ = id_source;
        let _ = decimals;
        ui.add(
            egui::DragValue::new(value)
                .range(range)
                .speed(speed)
                .suffix(suffix),
        )
    }
}

pub fn drag_f64(
    ui: &mut egui::Ui,
    id_source: impl Hash,
    value: &mut f64,
    range: RangeInclusive<f64>,
    speed: f64,
    decimals: usize,
) -> egui::Response {
    #[cfg(target_os = "android")]
    {
        let _ = speed;
        let id = ui.make_persistent_id(id_source);
        let error_id = id.with("error");
        let inner = numeric_control_shell(ui, 150.0, |ui| {
            if let Some(edited) = crate::platform::take_edited_text(id.value()) {
                match parse_f64_in_range(&edited, range.clone()) {
                    Ok(parsed) => {
                        *value = parsed;
                        set_input_error(ui.ctx(), error_id, None);
                    }
                    Err(error) => set_input_error(ui.ctx(), error_id, Some(error)),
                }
            }
            let shown = format!("{:.*}", decimals, *value);
            let response = ui.add_sized([150.0, TOUCH_HEIGHT], egui::Button::new(shown));
            if response.clicked() {
                crate::platform::edit_text(id.value(), &value.to_string(), InputKind::Decimal);
            }
            show_input_error(ui, error_id);
            response
        });
        inner.inner
    }
    #[cfg(not(target_os = "android"))]
    {
        let _ = id_source;
        let _ = range;
        ui.add(
            egui::DragValue::new(value)
                .speed(speed)
                .fixed_decimals(decimals),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use eframe::egui;

    #[test]
    fn balanced_chunks_split_ten_tabs_into_even_rows() {
        // 10 项分 3 行：4/3/3，末行不会只剩 2 项。
        let rows = balanced_chunks(10, 3);
        assert_eq!(rows, vec![0..4, 4..7, 7..10]);
        // 边界：整除时不产生空行，余数全部落在前面的行。
        assert_eq!(balanced_chunks(9, 3), vec![0..3, 3..6, 6..9]);
        assert_eq!(balanced_chunks(2, 3), vec![0..1, 1..2, 2..2]);
        // 每项只出现一次且顺序不变。
        let all: Vec<usize> = rows.iter().flat_map(|r| r.clone()).collect();
        assert_eq!(all, (0..10).collect::<Vec<_>>());
    }

    #[test]
    fn compact_breakpoint_covers_phone_widths_but_not_desktop() {
        assert!(is_compact(360.0));
        assert!(is_compact(412.0));
        assert!(!is_compact(880.0));
    }

    #[test]
    fn integer_input_rejects_invalid_or_out_of_range_without_a_replacement() {
        assert_eq!(parse_i64_in_range(" 23 ", 0..=23), Ok(23));
        assert!(parse_i64_in_range("24", 0..=23).is_err());
        assert!(parse_i64_in_range("abc", 0..=23).is_err());
    }

    #[test]
    fn decimal_input_rejects_non_finite_and_out_of_range_values() {
        assert_eq!(parse_f32_in_range(" 3.25 ", 0.5..=20.0), Ok(3.25));
        assert!(parse_f32_in_range("NaN", 0.5..=20.0).is_err());
        assert!(parse_f32_in_range("21", 0.5..=20.0).is_err());
    }

    #[test]
    fn tab_bar_stays_inside_offscreen_phone_and_desktop_frames() {
        for width in [360.0, 412.0, 880.0] {
            let ctx = egui::Context::default();
            let mut rects = Vec::new();
            let raw = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(width, 720.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(raw, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    let mut selected = 0;
                    rects = tab_bar(ui, &mut selected)
                        .into_iter()
                        .map(|response| response.rect)
                        .collect();
                });
            });

            assert_eq!(rects.len(), 10, "width={width}");
            assert!(rects
                .iter()
                .all(|rect| rect.is_finite() && rect.width() > 0.0));
            assert!(
                rects.iter().all(|rect| rect.max.x <= width + 0.5),
                "width={width}: {rects:?}"
            );
            if is_compact(width) {
                assert!(
                    rects.iter().all(|rect| rect.height() >= 44.0),
                    "width={width}: {rects:?}"
                );
            }
        }
    }

    #[test]
    fn responsive_parameter_row_wraps_without_invalid_or_overflowing_rectangles() {
        for width in [360.0, 412.0, 880.0] {
            let ctx = egui::Context::default();
            let mut rects = Vec::new();
            let raw = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(width, 720.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(raw, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    row(ui, |ui| {
                        for (index, control_width) in
                            [110.0, 120.0, 90.0, 120.0].into_iter().enumerate()
                        {
                            rects.push(
                                ui.add_sized(
                                    [control_width, TOUCH_HEIGHT],
                                    egui::Button::new(format!("参数 {index}")),
                                )
                                .rect,
                            );
                        }
                    });
                });
            });

            assert_eq!(rects.len(), 4, "width={width}");
            assert!(rects
                .iter()
                .all(|rect| rect.is_finite() && rect.is_positive()));
            assert!(
                rects.iter().all(|rect| rect.max.x <= width + 0.5),
                "width={width}: {rects:?}"
            );
        }
    }

    #[test]
    fn numeric_errors_expand_rows_without_overlapping_following_controls() {
        for width in [320.0, 360.0, 412.0] {
            let ctx = egui::Context::default();
            let mut rectangles = Vec::new();
            let raw = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(width, 720.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(raw, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    row(ui, |ui| {
                        for index in 0..3 {
                            let control = numeric_control_shell(ui, 120.0, |ui| {
                                let button = ui
                                    .add_sized([120.0, TOUCH_HEIGHT], egui::Button::new("360"))
                                    .rect;
                                let error = ui
                                    .colored_label(
                                        egui::Color32::RED,
                                        if index == 0 {
                                            "Please enter a valid value between 10 and 200"
                                        } else {
                                            "请输入 10 到 200"
                                        },
                                    )
                                    .rect;
                                (button, error)
                            });
                            assert!(control.response.rect.contains_rect(control.inner.0));
                            assert!(control.response.rect.contains_rect(control.inner.1));
                            rectangles.push(control.response.rect);
                        }
                    });
                    rectangles.push(ui.button("Next row").rect);
                });
            });
            for (index, rectangle) in rectangles.iter().enumerate() {
                assert!(
                    rectangle.max.x <= width + 0.5,
                    "width={width}: {rectangles:?}"
                );
                for other in &rectangles[index + 1..] {
                    assert!(
                        !rectangle.intersects(*other),
                        "width={width}: {rectangles:?}"
                    );
                }
            }
        }
    }

    #[test]
    fn wrapped_range_controls_reserve_width_before_the_parent_places_them() {
        for width in [360.0, 412.0] {
            let ctx = egui::Context::default();
            let mut rects = Vec::new();
            let raw = egui::RawInput {
                screen_rect: Some(egui::Rect::from_min_size(
                    egui::Pos2::ZERO,
                    egui::vec2(width, 720.0),
                )),
                ..Default::default()
            };
            let _ = ctx.run(raw, |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    row(ui, |ui| {
                        rects.push(ui.label("配速范围（秒/km）：").rect);
                        let first = numeric_control_shell(ui, 120.0, |ui| {
                            ui.add_sized([120.0, TOUCH_HEIGHT], egui::Button::new("360"))
                        });
                        rects.push(first.inner.rect);
                        rects.push(ui.label("至").rect);
                        let second = numeric_control_shell(ui, 120.0, |ui| {
                            ui.add_sized([120.0, TOUCH_HEIGHT], egui::Button::new("480"))
                        });
                        rects.push(second.inner.rect);
                    });
                });
            });

            assert!(rects
                .iter()
                .all(|rect| rect.is_finite() && rect.is_positive()));
            assert!(
                rects.iter().all(|rect| rect.max.x <= width + 0.5),
                "width={width}: {rects:?}"
            );
        }
    }
}
