//! Plug-ins panel: the installed filter plug-ins with their parameters (the same tabbed form as
//! their dialogs) and an Apply button, Install/Reload, and the Snap to Anchors toggle — dragged
//! anchors and Bézier direction handles snap live to the other anchor points and handle ends.

use egui::Ui;
use serde_json::{Value, json};
use vectorcraft_plugins::{Kind, registry};

use super::{pstate, set_pstate};
use crate::VectorcraftApp;
use crate::dialogs::form;
use crate::state::Dialog;
use crate::theme::Tokens;
use crate::widgets;

pub const ID: &str = "plugins";

/// The panel's form state for plug-in `id` (egui temp memory): its parameter values.
fn fields(ui: &Ui, id: &str) -> Option<Dialog> {
    pstate::<Option<Dialog>>(ui.ctx(), &format!("plugins-panel-{id}"))
}

fn set_fields(ui: &Ui, id: &str, d: Dialog) {
    set_pstate(ui.ctx(), &format!("plugins-panel-{id}"), Some(d));
}

pub fn show(app: &mut VectorcraftApp, ui: &mut Ui) {
    let t = Tokens::get(ui.ctx());
    // Snap to Anchors: live snapping of dragged anchors and handles to anchor points and
    // Bézier handle ends (ruler guides too; View → Snap to Grid/Pixel take over when on).
    widgets::section_header(ui, "Snapping");
    if widgets::check_tip(
        ui,
        "Snap to Anchors",
        app.ui.view.snap_to_anchors,
        true,
        tl!(
            "Dragged anchor points and Bézier handles snap to the other anchor points and handle ends (and ruler guides), even with Smart Guides and Snap to Point off. Snap to Grid and Snap to Pixel take over while they are on."
        ),
    ) {
        app.ui.view.snap_to_anchors ^= true;
    }
    widgets::divider(ui);
    let plugins: Vec<_> = registry::list().into_iter().filter(|p| p.manifest().kind == Kind::Filter).collect();
    if plugins.is_empty() {
        widgets::dim_label(ui, crate::dialogs::plugin::NO_FILTERS);
    }
    for plugin in plugins {
        let (id, m) = (plugin.id().to_string(), plugin.manifest());
        let open = pstate::<bool>(ui.ctx(), &format!("plugins-open-{id}"));
        if widgets::section_toggle_name(ui, &m.name, open).clicked() {
            set_pstate(ui.ctx(), &format!("plugins-open-{id}"), !open);
        }
        if !open {
            continue;
        }
        ui.label(egui::RichText::new(format!("{} {}", tl!("Version"), m.version)).color(t.text_dim).size(11.0));
        ui.add_space(4.0);
        let mut d = fields(ui, &id).unwrap_or_else(|| Dialog { kind: "plugin-panel".into(), fields: m.defaults() });
        if form::schema_fields(ui, &mut d, &m.params, m.tabs.as_ref()) {
            set_fields(ui, &id, d.clone());
        }
        ui.add_space(6.0);
        if widgets::primary_button(ui, "Apply").clicked() {
            match app.run("plugin.run", json!({"id": id, "params": form::params(&d)})) {
                Ok(_) => set_pstate(ui.ctx(), &format!("plugins-error-{id}"), String::new()),
                Err(e) => set_pstate(ui.ctx(), &format!("plugins-error-{id}"), e),
            }
        }
        let error = pstate::<String>(ui.ctx(), &format!("plugins-error-{id}"));
        if !error.is_empty() {
            ui.add_space(4.0);
            ui.label(egui::RichText::new(error).color(t.warning).size(11.0));
        }
        ui.add_space(4.0);
    }
    widgets::divider(ui);
    ui.horizontal(|ui| {
        if widgets::secondary_button(ui, "Install Plug-in…").clicked() {
            crate::menus::invoke(app, "ui.installPlugin", Value::Null);
        }
        if widgets::secondary_button(ui, "Reload").clicked() {
            app.run("plugin.reload", json!({})).ok();
        }
    });
}

#[cfg(test)]
mod tests {
    use super::*;
    use vectorcraft_engine::Session;

    /// Run the panel for one headless frame with pointer `events`: the texts drawn and their rects.
    fn frame(app: &mut VectorcraftApp, ctx: &egui::Context, events: Vec<egui::Event>) -> Vec<(String, egui::Rect)> {
        fn texts(s: &egui::Shape, out: &mut Vec<(String, egui::Rect)>) {
            match s {
                egui::Shape::Text(t) => out.push((t.galley.text().to_string(), t.visual_bounding_rect())),
                egui::Shape::Vec(v) => v.iter().for_each(|s| texts(s, out)),
                _ => {}
            }
        }
        let mut out = ctx.run_ui(egui::RawInput { events, ..Default::default() }, |ui| show(app, ui));
        out.textures_delta.clear();
        let mut v = vec![];
        out.shapes.iter().for_each(|c| texts(&c.shape, &mut v));
        v
    }

    #[test]
    fn the_panel_shows_and_toggles_snap_to_anchors() {
        let mut app = VectorcraftApp::new(Session::new(), Default::default());
        let ctx = egui::Context::default();
        crate::theme::install_fonts(&ctx);
        crate::theme::apply(&ctx, Default::default());
        assert!(!app.ui.view.snap_to_anchors);
        let texts = frame(&mut app, &ctx, vec![]);
        let find = |texts: &[(String, egui::Rect)], s: &str| texts.iter().find(|(t, _)| t == s).map(|(_, r)| *r);
        let pos = find(&texts, "Snap to Anchors").expect("the checkbox shows").center();
        assert!(find(&texts, "No filter plug-ins installed").is_some(), "{texts:?}");
        // Clicking the checkbox turns live anchor snapping on.
        let button = egui::PointerButton::Primary;
        frame(
            &mut app,
            &ctx,
            vec![egui::Event::PointerMoved(pos), egui::Event::PointerButton { pos, button, pressed: true, modifiers: Default::default() }],
        );
        frame(&mut app, &ctx, vec![egui::Event::PointerButton { pos, button, pressed: false, modifiers: Default::default() }]);
        assert!(app.ui.view.snap_to_anchors);
        assert!(app.view_info().snap_to_anchors, "the flag reaches the tools");
    }
}
