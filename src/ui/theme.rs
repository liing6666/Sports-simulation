//! 界面主题：设计令牌 + egui 视觉设置 + 常用控件构造器。
//!
//! 视觉语言：白色卡片浮在冷灰底上；主操作深蓝实心、次要操作白底描边、
//! 轻量操作用纯文字。所有颜色 / 间距 / 圆角 / 字号集中在本文件顶部的令牌里，
//! 页面只引用令牌，不写魔法值。
//!
//! 兼容性：`plain()` / `warn()` / `err()` / `ok()` / `primary_btn(&str)` 等
//! 旧函数名和签名保持不变，其他页面无需改动即可沿用。

use eframe::egui;
use egui::{Color32, Stroke};

// ── 颜色令牌 ────────────────────────────────────────────────────

/// 主色：深蓝。
pub const PRIMARY: Color32 = Color32::from_rgb(0x25, 0x63, 0xEB);
#[allow(dead_code)] // 令牌集的一部分：供后续页面直接取用
pub const PRIMARY_HOVER: Color32 = Color32::from_rgb(0x1D, 0x4E, 0xD8);
pub const PRIMARY_ACTIVE: Color32 = Color32::from_rgb(0x1E, 0x40, 0xAF);
/// 主色淡底（选中项、hover 底色）。
pub const PRIMARY_TINT: Color32 = Color32::from_rgb(0xEF, 0xF6, 0xFF);

/// 页面底色。
pub const PAGE: Color32 = Color32::from_rgb(0xF7, 0xF8, 0xFA);
/// 卡片 / 分区底色。
pub const CARD: Color32 = Color32::from_rgb(0xF1, 0xF3, 0xF6);
pub const WHITE: Color32 = Color32::from_rgb(0xFF, 0xFF, 0xFF);
/// 描边。
pub const BORDER: Color32 = Color32::from_rgb(0xE4, 0xE7, 0xEC);

/// 主文字。
pub const TEXT: Color32 = Color32::from_rgb(0x10, 0x18, 0x28);
/// 次文字。
pub const TEXT_SECONDARY: Color32 = Color32::from_rgb(0x47, 0x54, 0x67);
/// 辅助文字（说明、版本号、设备号）。
pub const TEXT_AUX: Color32 = Color32::from_rgb(0x98, 0xA2, 0xB3);

/// 在线 / 正常。
pub const OK: Color32 = Color32::from_rgb(0x12, 0xB7, 0x6A);
pub const WARN: Color32 = Color32::from_rgb(0xB5, 0x47, 0x08);
pub const ERR: Color32 = Color32::from_rgb(0xB4, 0x23, 0x18);

// ── 尺度令牌 ────────────────────────────────────────────────────

/// 间距阶梯：4 / 8 / 12 / 16 / 24。
pub const GAP_XS: f32 = 4.0;
pub const GAP_SM: f32 = 8.0;
pub const GAP_MD: f32 = 12.0;
pub const GAP_LG: f32 = 16.0;
#[allow(dead_code)] // 令牌集的一部分：供后续页面直接取用
pub const GAP_XL: f32 = 24.0;

/// 圆角：控件 6，卡片 / 窗口 10。
pub const RADIUS_CONTROL: f32 = 6.0;
pub const RADIUS_CARD: f32 = 10.0;

/// 字号：标题 16 / 正文 13 / 辅助 11。
pub const FONT_TITLE: f32 = 16.0;
pub const FONT_BODY: f32 = 13.0;
pub const FONT_AUX: f32 = 11.0;

// ── 旧接口（保持名字与签名）────────────────────────────────────

pub fn ok() -> Color32 {
    OK
}
pub fn warn() -> Color32 {
    WARN
}
pub fn err() -> Color32 {
    ERR
}
pub fn accent() -> Color32 {
    PRIMARY
}
/// 次文字色（旧代码里用于说明性文字）。
pub fn plain() -> Color32 {
    TEXT_SECONDARY
}
pub fn text_dim() -> Color32 {
    TEXT_AUX
}
pub fn text() -> Color32 {
    TEXT
}

// ── 按钮 ────────────────────────────────────────────────────────

/// 主操作：深蓝实心白字。
///
/// 注意：`ui.add_enabled(false, ..)` 会把颜色整体向
/// `noninteractive.weak_bg_fill` 淡化，深蓝会被冲成近白。需要禁用态时请用
/// [`primary_btn_enabled`]，它会在禁用时主动换成灰底。
pub fn primary_btn(text: &str) -> egui::Button<'static> {
    primary_btn_enabled(text, true)
}

/// 带禁用态的主操作按钮：禁用时灰底灰字，而不是被 egui 淡化成白块。
pub fn primary_btn_enabled(text: &str, enabled: bool) -> egui::Button<'static> {
    let (fill, color) = if enabled {
        (PRIMARY, WHITE)
    } else {
        (BORDER, TEXT_AUX)
    };
    egui::Button::new(egui::RichText::new(text.to_owned()).color(color).strong())
        .fill(fill)
        .stroke(Stroke::NONE)
        .rounding(RADIUS_CONTROL)
}

/// 次要操作：白底描边。
#[allow(dead_code)] // 三档按钮规格的一部分：现有页面暂未用到
pub fn secondary_btn(text: &str) -> egui::Button<'static> {
    egui::Button::new(egui::RichText::new(text.to_owned()).color(TEXT))
        .fill(WHITE)
        .stroke(Stroke::new(1.0_f32, BORDER))
        .rounding(RADIUS_CONTROL)
}

/// 轻量操作：纯文字（表格行内、状态栏）。
#[allow(dead_code)] // 三档按钮规格的一部分：现有页面暂未用到
pub fn ghost_btn(text: &str) -> egui::Button<'static> {
    egui::Button::new(egui::RichText::new(text.to_owned()).color(PRIMARY)).frame(false)
}

// ── 容器与标签 ──────────────────────────────────────────────────

/// 卡片外观（不渲染），供 `Frame::group` 的替换点直接 `.show(..)`。
/// 分组卡片用 12 内边距，比整页卡片更紧凑。
pub fn card_frame() -> egui::Frame {
    egui::Frame::none()
        .fill(WHITE)
        .stroke(Stroke::new(1.0_f32, BORDER))
        .rounding(RADIUS_CARD)
        .inner_margin(egui::Margin::same(GAP_MD))
}

/// 卡片容器：白底、描边、10 圆角、16 内边距。
pub fn card<R>(
    ui: &mut egui::Ui,
    add_contents: impl FnOnce(&mut egui::Ui) -> R,
) -> egui::InnerResponse<R> {
    card_frame()
        .inner_margin(egui::Margin::same(GAP_LG))
        .show(ui, add_contents)
}

/// 胶囊标签：淡色底 + 同色文字，用于登录状态 / 分段标记。
pub fn capsule(ui: &mut egui::Ui, text: &str, color: Color32) -> egui::Response {
    let font = egui::FontId::proportional(FONT_AUX + 1.0);
    let galley = ui
        .painter()
        .layout_no_wrap(text.to_owned(), font, color);
    let padding = egui::vec2(GAP_SM, GAP_XS * 0.5);
    let size = galley.size() + padding * 2.0;
    let (rect, response) = ui.allocate_exact_size(size, egui::Sense::hover());
    if ui.is_rect_visible(rect) {
        let painter = ui.painter();
        painter.rect(
            rect,
            egui::Rounding::same(size.y / 2.0),
            egui::Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 26),
            Stroke::new(1.0_f32, egui::Color32::from_rgba_unmultiplied(color.r(), color.g(), color.b(), 96)),
        );
        painter.galley(rect.min + padding, galley, color);
    }
    response
}

// ── 视觉设置 ────────────────────────────────────────────────────

/// 应用全局视觉。旧调用点仍用 `theme::apply`。
pub fn apply(ctx: &egui::Context) {
    apply_visuals(ctx);
}

pub fn apply_visuals(ctx: &egui::Context) {
    let mut style = (*ctx.style()).clone();

    style.text_styles.insert(
        egui::TextStyle::Heading,
        egui::FontId::new(FONT_TITLE, egui::FontFamily::Proportional),
    );
    style.text_styles.insert(
        egui::TextStyle::Body,
        egui::FontId::new(FONT_BODY, egui::FontFamily::Proportional),
    );
    style.text_styles.insert(
        egui::TextStyle::Button,
        egui::FontId::new(FONT_BODY, egui::FontFamily::Proportional),
    );
    style.text_styles.insert(
        egui::TextStyle::Small,
        egui::FontId::new(FONT_AUX, egui::FontFamily::Proportional),
    );

    let rounding_control = egui::Rounding::same(RADIUS_CONTROL);
    let v = &mut style.visuals;
    *v = egui::Visuals::light();

    v.panel_fill = PAGE;
    v.window_fill = WHITE;
    v.window_stroke = Stroke::new(1.0_f32, BORDER);
    // 输入框底色比白卡片略深，边界更清楚。
    v.extreme_bg_color = PAGE;
    v.faint_bg_color = CARD;
    v.override_text_color = Some(TEXT);
    v.hyperlink_color = PRIMARY;
    v.selection.bg_fill = PRIMARY;
    v.selection.stroke = Stroke::new(1.0_f32, WHITE);
    v.window_rounding = egui::Rounding::same(RADIUS_CARD);
    v.menu_rounding = egui::Rounding::same(RADIUS_CONTROL);
    // 禁用态淡出的目标色：偏中性灰，避免深蓝按钮被冲成近白。
    v.widgets.noninteractive.bg_fill = CARD;
    v.widgets.noninteractive.weak_bg_fill = Color32::from_rgb(0xE9, 0xEC, 0xF1);
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0_f32, BORDER);
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0_f32, TEXT);
    v.widgets.noninteractive.rounding = rounding_control;

    v.widgets.inactive.bg_fill = WHITE;
    v.widgets.inactive.weak_bg_fill = WHITE;
    v.widgets.inactive.bg_stroke = Stroke::new(1.0_f32, BORDER);
    v.widgets.inactive.fg_stroke = Stroke::new(1.0_f32, TEXT_SECONDARY);
    v.widgets.inactive.expansion = 0.0;
    v.widgets.inactive.rounding = rounding_control;

    v.widgets.hovered.bg_fill = PRIMARY_TINT;
    v.widgets.hovered.weak_bg_fill = PRIMARY_TINT;
    v.widgets.hovered.bg_stroke = Stroke::new(1.0_f32, PRIMARY);
    v.widgets.hovered.fg_stroke = Stroke::new(1.2_f32, TEXT);
    v.widgets.hovered.expansion = 0.0;
    v.widgets.hovered.rounding = rounding_control;

    v.widgets.active.bg_fill = PRIMARY;
    v.widgets.active.weak_bg_fill = PRIMARY;
    v.widgets.active.bg_stroke = Stroke::new(1.0_f32, PRIMARY_ACTIVE);
    v.widgets.active.fg_stroke = Stroke::new(1.2_f32, WHITE);
    v.widgets.active.expansion = 0.0;
    v.widgets.active.rounding = rounding_control;

    v.widgets.open.bg_fill = WHITE;
    v.widgets.open.weak_bg_fill = WHITE;
    v.widgets.open.bg_stroke = Stroke::new(1.0_f32, PRIMARY);
    v.widgets.open.fg_stroke = Stroke::new(1.2_f32, TEXT);
    v.widgets.open.rounding = rounding_control;

    style.spacing.item_spacing = egui::vec2(GAP_SM + 2.0, GAP_SM);
    style.spacing.button_padding = egui::vec2(GAP_MD + 2.0, 7.0);
    style.spacing.menu_margin = egui::Margin::same(GAP_SM);
    style.spacing.interact_size.y = 26.0;
    #[cfg(target_os = "android")]
    {
        style.spacing.interact_size = egui::vec2(44.0, 44.0);
        style.spacing.button_padding = egui::vec2(14.0, 10.0);
    }

    // 页面文字仅作展示：不可选中、不可复制，输入框不受影响
    style.interaction.selectable_labels = false;
    style.interaction.multi_widget_text_select = false;

    ctx.set_style(style);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_visuals_installs_page_and_primary_colors() {
        let ctx = egui::Context::default();
        apply_visuals(&ctx);
        let style = ctx.style();
        assert_eq!(style.visuals.panel_fill, PAGE);
        assert_eq!(style.visuals.selection.bg_fill, PRIMARY);
        assert_eq!(style.visuals.widgets.hovered.bg_stroke.color, PRIMARY);
        // 禁用态淡出目标不能是纯白，否则深蓝主按钮会被冲淡。
        assert_ne!(style.visuals.widgets.noninteractive.weak_bg_fill, WHITE);
    }

    #[test]
    fn capsule_and_card_draw_inside_allocated_space() {
        let ctx = egui::Context::default();
        let mut rects = Vec::new();
        let _ = ctx.run(egui::RawInput::default(), |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                rects.push(capsule(ui, "● 未登录", ERR).rect);
                rects.push(card(ui, |ui| ui.label("内容")).response.rect);
            });
        });
        assert!(rects.iter().all(|r| r.is_finite() && r.is_positive()));
    }
}
