//! Intersection Anchors: a VectorCraft object filter plug-in (plug-in ABI v1, see
//! `docs/plugins.md`).
//!
//! Five modes on the selected paths and compound paths:
//!
//! - **insert**: finds every point where two segments (lines or Bézier curves) cross — between
//!   objects, between subpaths and within one subpath — and inserts an anchor point exactly there,
//!   splitting each crossing segment with de Casteljau so the shapes don't change. With a direct
//!   selection (anchors or segments picked with the Direct Selection tool), only the picked parts
//!   take part, so crossings appear exactly where the user chose to work.
//! - **mark**: adds a small magenta marker on top of each crossing instead of editing the paths,
//!   so the crossings are easy to see (and, with the dialog's live preview, easy to see *before*
//!   committing). It honours the same direct selection as **insert**, so it also answers "which
//!   crossings will be used?". Undo removes the markers in one step.
//! - **snap handles**: rotates every Bézier handle of the selection (or of the direct-selected
//!   anchors) onto the nearest multiple of `angle` degrees (45° by default), keeping its length;
//!   smooth anchors stay smooth. The interactive equivalent — holding Shift while dragging a
//!   handle — is built into the app's Direct Selection, Anchor Point and Pen tools.
//! - **smooth flow**: recomputes the targeted anchors' handles for tangent continuity (G1), so a
//!   chain of Bézier segments flows evenly instead of looking distorted by its old handles; when
//!   the selection also holds a straight line, the curve's tangents follow that line's direction.
//! - **merge anchors**: collapses runs of direct-selected anchors into one anchor and moves every
//!   selected anchor to a common point — its position and handles computed per the `merge` method
//!   (average, geometric centre, or dominant curve direction).
//!
//! Intersections are found by recursive subdivision: each pair of segments is split in half until
//! both halves are flat enough to be treated as lines, whose crossing gives the curve parameters.
//! Crossings that coincide with an existing anchor are skipped, and coincident hits are merged.
//! The host passes the user's direct selection per object as `selectedAnchors` /
//! `selectedSegments`; an object without them is selected whole.
//! Only `serde_json` is used: no imports, no WASI.

use serde_json::{Map, Value, json};

const MANIFEST: &str = r#"{
  "id": "org.vectorcraft.intersect",
  "name": "Intersection Anchors",
  "version": "1.2.0",
  "kind": "filter",
  "author": "VectorCraft contributors",
  "description": "Anchor points at path crossings (insert or mark, honouring direct-selected anchors), handle angle snapping, tangent-continuous smoothing along a line, and merging of selected anchor points.",
  "params": {
    "mode": {"type": "choice", "options": ["insert", "mark", "snap handles", "smooth flow", "merge anchors"], "default": "insert"},
    "marker": {"type": "number", "min": 1, "max": 50, "default": 5},
    "angle": {"type": "int", "min": 1, "max": 180, "default": 45},
    "flow": {"type": "number", "min": 0, "max": 100, "default": 100},
    "merge": {"type": "choice", "options": ["average", "geometric center", "dominant direction"], "default": "average"}
  },
  "tabs": {
    "param": "mode",
    "params": {
      "marker": ["mark"],
      "angle": ["snap handles"],
      "flow": ["smooth flow"],
      "merge": ["merge anchors"]
    },
    "descriptions": {
      "insert": "Inserts an anchor point exactly where two segments or curves cross, without changing the drawn shape. With anchors or segments direct-selected, only crossings between the selected parts are made.",
      "mark": "Drops a magenta marker dot on every crossing instead of editing the paths, so you see each crossing before committing. One undo removes them all.",
      "snap handles": "Rotates Bézier handles onto the nearest multiple of the angle step (45°: 0°, 45°, 90°, …), keeping their lengths. With anchors direct-selected, only those anchors' handles snap.",
      "smooth flow": "Re-fits handles for an even, tangent-continuous flow. Straight segments stay straight, curves merge smoothly into adjoining lines at any angle, and a selected straight line acts as a direction guide. Flow blends from the current handles (0) to fully smoothed (100).",
      "merge anchors": "Merges two or more direct-selected anchor points into one: average (centroid, mean handles), geometric center (bounding-box centre) or dominant direction (aligned with the strongest handle)."
    }
  }
}"#;

/// Two points closer than this (in pt) are the same crossing.
const MERGE_TOL: f64 = 1e-3;
/// A segment half is "flat" when its control points stay within this of its chord (pt).
const FLAT_TOL: f64 = 1e-4;
/// Most crossings reported, and most segments examined, before the filter declines.
const MAX_HITS: usize = 4096;
const MAX_SEGMENTS: usize = 20_000;

#[unsafe(no_mangle)]
pub extern "C" fn vc_abi_version() -> u32 {
    1
}

/// `len << 32 | ptr` of the UTF-8 manifest JSON.
#[unsafe(no_mangle)]
pub extern "C" fn vc_manifest() -> u64 {
    pack(MANIFEST.as_bytes())
}

/// A block of `size` bytes the host writes the input and the parameters into. Never freed: every
/// run gets a fresh instance.
#[unsafe(no_mangle)]
pub extern "C" fn vc_alloc(size: u32) -> u32 {
    let mut block = Vec::<u8>::with_capacity(size as usize);
    let ptr = block.as_mut_ptr();
    std::mem::forget(block);
    ptr as u32
}

/// Runs the filter: `input` holds `{"objects": [...]}`, `params` the parameters. Returns the
/// output JSON as `len << 32 | ptr`, or a negative error code.
///
/// # Safety
/// The host guarantees both blocks came from `vc_alloc` and hold `*_len` bytes.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn vc_run(input: *const u8, input_len: u32, params: *const u8, params_len: u32) -> i64 {
    // SAFETY: see the function's contract.
    let (input, params) = unsafe { (std::slice::from_raw_parts(input, input_len as usize), std::slice::from_raw_parts(params, params_len as usize)) };
    let (Ok(doc), Ok(params)) = (serde_json::from_slice::<Value>(input), serde_json::from_slice::<Value>(params)) else { return -1 };
    output(&run(&doc, &params))
}

/// The whole filter on parsed JSON (also what the native tests call).
pub fn run(doc: &Value, params: &Value) -> Value {
    let Some(objects) = doc["objects"].as_array().filter(|o| !o.is_empty()) else {
        return json!({"error": "Select some paths."});
    };
    match params["mode"].as_str().unwrap_or("insert") {
        "snap handles" => {
            let step = params["angle"].as_f64().unwrap_or(45.0).clamp(1.0, 180.0);
            snap_handles(objects, step)
        }
        "smooth flow" => {
            let flow = params["flow"].as_f64().unwrap_or(100.0).clamp(0.0, 100.0) / 100.0;
            smooth_flow(objects, flow)
        }
        "merge anchors" => merge_anchors(objects, params["merge"].as_str().unwrap_or("average")),
        mode => {
            let marker = params["marker"].as_f64().unwrap_or(5.0).clamp(1.0, 50.0);
            intersections(objects, mode, marker)
        }
    }
}

// --- Geometry ------------------------------------------------------------------------------

type P = [f64; 2];

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn lerp(a: P, b: P, t: f64) -> P {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}
fn dist(a: P, b: P) -> f64 {
    let d = sub(a, b);
    (d[0] * d[0] + d[1] * d[1]).sqrt()
}
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// A cubic Bézier segment; a line is the degenerate cubic whose control points sit on its ends.
#[derive(Clone, Copy)]
struct Cubic {
    p: [P; 4],
}

impl Cubic {
    fn eval(&self, t: f64) -> P {
        let [a, b, c, d] = self.p;
        let (ab, bc, cd) = (lerp(a, b, t), lerp(b, c, t), lerp(c, d, t));
        lerp(lerp(ab, bc, t), lerp(bc, cd, t), t)
    }

    /// de Casteljau split at `t`: the two halves, sharing the point at `t`.
    fn split(&self, t: f64) -> (Cubic, Cubic) {
        let [a, b, c, d] = self.p;
        let (ab, bc, cd) = (lerp(a, b, t), lerp(b, c, t), lerp(c, d, t));
        let (abc, bcd) = (lerp(ab, bc, t), lerp(bc, cd, t));
        let m = lerp(abc, bcd, t);
        (Cubic { p: [a, ab, abc, m] }, Cubic { p: [m, bcd, cd, d] })
    }

    /// The axis-aligned box of the control points (contains the curve).
    fn bbox(&self) -> [f64; 4] {
        let mut b = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
        for q in self.p {
            b = [b[0].min(q[0]), b[1].min(q[1]), b[2].max(q[0]), b[3].max(q[1])];
        }
        b
    }

    /// Both inner control points within `tol` of the chord?
    fn flat(&self, tol: f64) -> bool {
        let [a, b, c, d] = self.p;
        let ch = sub(d, a);
        let len = (ch[0] * ch[0] + ch[1] * ch[1]).sqrt();
        if len < tol {
            return dist(a, b) < tol && dist(a, c) < tol;
        }
        cross(ch, sub(b, a)).abs() / len < tol && cross(ch, sub(c, a)).abs() / len < tol
    }
}

fn boxes_overlap(a: [f64; 4], b: [f64; 4]) -> bool {
    a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3]
}

/// Where segments `a0→a1` and `b0→b1` cross: their parameters, both within `[0, 1]`.
/// Parallel (and collinear-overlapping) segments report nothing.
fn seg_x(a0: P, a1: P, b0: P, b1: P) -> Option<(f64, f64)> {
    let (da, db) = (sub(a1, a0), sub(b1, b0));
    let den = cross(da, db);
    if den.abs() < 1e-12 {
        return None;
    }
    let w = sub(b0, a0);
    let s = cross(w, db) / den;
    let t = cross(w, da) / den;
    ((-1e-9..=1.0 + 1e-9).contains(&s) && (-1e-9..=1.0 + 1e-9).contains(&t)).then(|| (s.clamp(0.0, 1.0), t.clamp(0.0, 1.0)))
}

/// Every crossing of two cubics, by recursive subdivision, as `(ta, tb)` pairs.
fn curve_x(a: &Cubic, b: &Cubic, ta: (f64, f64), tb: (f64, f64), depth: u32, out: &mut Vec<(f64, f64)>) {
    if out.len() >= MAX_HITS || !boxes_overlap(a.bbox(), b.bbox()) {
        return;
    }
    if depth == 0 || (a.flat(FLAT_TOL) && b.flat(FLAT_TOL)) {
        if let Some((s, t)) = seg_x(a.p[0], a.p[3], b.p[0], b.p[3]) {
            out.push((ta.0 + (ta.1 - ta.0) * s, tb.0 + (tb.1 - tb.0) * t));
        }
        return;
    }
    let ((a1, a2), (b1, b2)) = (a.split(0.5), b.split(0.5));
    let (am, bm) = ((ta.0 + ta.1) / 2.0, (tb.0 + tb.1) / 2.0);
    curve_x(&a1, &b1, (ta.0, am), (tb.0, bm), depth - 1, out);
    curve_x(&a1, &b2, (ta.0, am), (bm, tb.1), depth - 1, out);
    curve_x(&a2, &b1, (am, ta.1), (tb.0, bm), depth - 1, out);
    curve_x(&a2, &b2, (am, ta.1), (bm, tb.1), depth - 1, out);
}

// --- The document's paths ------------------------------------------------------------------

/// One anchor as parsed, keeping the original JSON so unknown keys survive.
struct Anchor {
    raw: Map<String, Value>,
    p: P,
    inh: Option<P>,
    outh: Option<P>,
}

struct SubPath {
    closed: bool,
    anchors: Vec<Anchor>,
}

/// One selected object's parsed subpaths, with its index into `objects` and, when the user
/// direct-selected parts of it, those parts (`None`: the object is selected whole).
struct Obj {
    index: usize,
    subs: Vec<SubPath>,
    sel_anchors: Option<Vec<(usize, usize)>>,
    sel_segments: Option<Vec<(usize, usize)>>,
}

impl Obj {
    /// Does segment (`si`, `ai`) of a subpath of `n` anchors take part? A whole-object selection
    /// takes every segment; a direct selection takes the segments picked explicitly and the ones
    /// touching a selected anchor.
    fn seg_selected(&self, si: usize, ai: usize, n: usize) -> bool {
        let (sa, ss) = (self.sel_anchors.as_deref(), self.sel_segments.as_deref());
        if sa.is_none() && ss.is_none() {
            return true;
        }
        ss.is_some_and(|s| s.contains(&(si, ai))) || sa.is_some_and(|s| s.contains(&(si, ai)) || (n > 0 && s.contains(&(si, (ai + 1) % n))))
    }

    /// Is anchor (`si`, `ai`) one the user asked to work on? Without a direct selection, all are.
    fn anchor_selected(&self, si: usize, ai: usize) -> bool {
        self.sel_anchors.as_deref().is_none_or(|s| s.contains(&(si, ai)))
    }
}

fn point(v: &Value) -> Option<P> {
    let (x, y) = (v.get(0)?.as_f64()?, v.get(1)?.as_f64()?);
    (x.is_finite() && y.is_finite()).then_some([x, y])
}

/// `selectedAnchors` / `selectedSegments` on an input object: `[[subpath, index], …]`.
fn sel_refs(v: &Value) -> Option<Vec<(usize, usize)>> {
    let list = v.as_array()?;
    let mut out = Vec::new();
    for r in list {
        let (si, ai) = (r.get(0)?.as_u64()?, r.get(1)?.as_u64()?);
        out.push((si as usize, ai as usize));
    }
    Some(out)
}

/// The objects whose paths parse; ones that don't are passed through untouched.
fn parse(objects: &[Value]) -> Vec<Obj> {
    let mut out = Vec::new();
    for (index, o) in objects.iter().enumerate() {
        let Some(subs) = o["path"]["subpaths"].as_array() else { continue };
        let mut parsed = Vec::new();
        let ok = subs.iter().all(|s| {
            let Some(list) = s["anchors"].as_array() else { return false };
            let mut anchors = Vec::new();
            for a in list {
                let (Some(raw), Some(p)) = (a.as_object(), point(&a["p"])) else { return false };
                anchors.push(Anchor { raw: raw.clone(), p, inh: point(&a["in"]), outh: point(&a["out"]) });
            }
            parsed.push(SubPath { closed: s["closed"].as_bool().unwrap_or(false), anchors });
            true
        });
        if ok {
            out.push(Obj { index, subs: parsed, sel_anchors: sel_refs(&o["selectedAnchors"]), sel_segments: sel_refs(&o["selectedSegments"]) });
        }
    }
    out
}

/// A segment's address: object (index into the parsed list), subpath, first anchor.
type SegId = (usize, usize, usize);

/// Every participating segment of every parsed object, as a degenerate-aware cubic. With a
/// direct selection, only the segments the user picked (or whose anchors they picked) join in.
fn segments(objs: &[Obj]) -> Vec<(SegId, Cubic)> {
    let mut out = Vec::new();
    for (oi, o) in objs.iter().enumerate() {
        for (si, s) in o.subs.iter().enumerate() {
            let n = s.anchors.len();
            let count = if s.closed && n > 1 { n } else { n.saturating_sub(1) };
            for ai in 0..count {
                if !o.seg_selected(si, ai, n) {
                    continue;
                }
                let (a, b) = (&s.anchors[ai], &s.anchors[(ai + 1) % n]);
                let c = Cubic { p: [a.p, a.outh.unwrap_or(a.p), b.inh.unwrap_or(b.p), b.p] };
                if dist(a.p, b.p) > 0.0 || a.outh.is_some() || b.inh.is_some() {
                    out.push(((oi, si, ai), c));
                }
            }
        }
    }
    out
}

// --- Modes ---------------------------------------------------------------------------------

/// Finds the crossings and either inserts anchors (`insert`) or adds markers (`mark`).
fn intersections(objects: &[Value], mode: &str, marker: f64) -> Value {
    let objs = parse(objects);
    let segs = segments(&objs);
    if segs.len() > MAX_SEGMENTS {
        return json!({"error": "The selection is too complex for this filter."});
    }
    // Every crossing: the point, and the parameter on each segment it lies on.
    let mut points: Vec<P> = Vec::new();
    let mut splits: Vec<(SegId, f64)> = Vec::new();
    for i in 0..segs.len() {
        for j in i + 1..segs.len() {
            let ((ida, a), (idb, b)) = (&segs[i], &segs[j]);
            let mut hits = Vec::new();
            curve_x(a, b, (0.0, 1.0), (0.0, 1.0), 24, &mut hits);
            for (ta, tb) in hits {
                let p = a.eval(ta);
                // A crossing at a segment end is an existing anchor: nothing to insert.
                if [a.p[0], a.p[3], b.p[0], b.p[3]].iter().any(|q| dist(*q, p) < MERGE_TOL) {
                    continue;
                }
                if !points.iter().any(|q| dist(*q, p) < MERGE_TOL) {
                    points.push(p);
                }
                splits.push((*ida, ta));
                splits.push((*idb, tb));
                if points.len() >= MAX_HITS {
                    return json!({"error": "Too many crossings; select fewer paths."});
                }
            }
        }
    }
    if points.is_empty() {
        return if objs.iter().any(|o| o.sel_anchors.is_some() || o.sel_segments.is_some()) {
            json!({"error": "No crossings among the direct-selected anchors or segments. Select the parts that cross, or select whole objects."})
        } else {
            json!({"error": "No crossings found in the selection."})
        };
    }
    let mut out: Vec<Value> = Vec::new();
    if mode == "mark" {
        for o in objects {
            out.push(json!({"id": o["id"]}));
        }
        for p in &points {
            out.push(marker_circle(*p, marker / 2.0));
        }
    } else {
        for (oi, o) in objs.iter().enumerate() {
            let subs: Vec<Value> = o
                .subs
                .iter()
                .enumerate()
                .map(|(si, s)| {
                    let anchors = insert_in_subpath(s, &splits, oi, si);
                    json!({"closed": s.closed, "anchors": anchors})
                })
                .collect();
            out.push(json!({"id": objects[o.index]["id"], "path": {"subpaths": subs}}));
        }
        // Objects whose paths didn't parse stay as they are.
        let parsed: Vec<usize> = objs.iter().map(|o| o.index).collect();
        for (i, o) in objects.iter().enumerate() {
            if !parsed.contains(&i) {
                out.push(json!({"id": o["id"]}));
            }
        }
    }
    json!({"objects": out})
}

/// The subpath's anchors with an anchor inserted at every split parameter, as JSON.
fn insert_in_subpath(s: &SubPath, splits: &[(SegId, f64)], oi: usize, si: usize) -> Vec<Value> {
    let n = s.anchors.len();
    let mut raw: Vec<Map<String, Value>> = s.anchors.iter().map(|a| a.raw.clone()).collect();
    let count = if s.closed && n > 1 { n } else { n.saturating_sub(1) };
    // Highest segment first, so earlier anchor indices stay valid while inserting.
    for ai in (0..count).rev() {
        let mut ts: Vec<f64> = splits.iter().filter(|((o, sp, sg), _)| (*o, *sp, *sg) == (oi, si, ai)).map(|(_, t)| *t).collect();
        ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        ts.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
        ts.retain(|t| (1e-6..1.0 - 1e-6).contains(t));
        if ts.is_empty() {
            continue;
        }
        let (a, b) = (&s.anchors[ai], &s.anchors[(ai + 1) % n]);
        let line = a.outh.is_none() && b.inh.is_none();
        let mut cur = Cubic { p: [a.p, a.outh.unwrap_or(a.p), b.inh.unwrap_or(b.p), b.p] };
        let (mut t0, mut inserted) = (0.0, Vec::<Value>::new());
        for &t in &ts {
            let local = ((t - t0) / (1.0 - t0)).clamp(0.0, 1.0);
            let (left, right) = cur.split(local);
            if line {
                inserted.push(anchor_json(left.p[3], None, None, false));
            } else {
                // The piece before this split belongs to the previous anchor's out handle.
                match inserted.last_mut() {
                    Some(last) => update_out(last, left.p[1]),
                    None => set_handle(&mut raw[ai], "out", left.p[1], a.p),
                }
                inserted.push(anchor_json(left.p[3], Some(left.p[2]), Some(right.p[1]), true));
            }
            cur = right;
            t0 = t;
        }
        if !line {
            set_handle(&mut raw[(ai + 1) % n], "in", cur.p[2], b.p);
        }
        // Splice the new anchors in after `ai`.
        let at = ai + 1;
        for (k, v) in inserted.into_iter().enumerate() {
            if let Value::Object(m) = v {
                raw.insert(at + k, m);
            }
        }
    }
    raw.into_iter().map(Value::Object).collect()
}

/// A new anchor as manifest JSON; handles equal to the point are dropped.
fn anchor_json(p: P, inh: Option<P>, outh: Option<P>, smooth: bool) -> Value {
    let mut m = Map::new();
    m.insert("p".into(), json!([p[0], p[1]]));
    if let Some(h) = inh.filter(|h| dist(*h, p) > 1e-9) {
        m.insert("in".into(), json!([h[0], h[1]]));
    }
    if let Some(h) = outh.filter(|h| dist(*h, p) > 1e-9) {
        m.insert("out".into(), json!([h[0], h[1]]));
    }
    if smooth && m.contains_key("in") && m.contains_key("out") {
        m.insert("kind".into(), json!("Smooth"));
    }
    Value::Object(m)
}

/// Writes handle `key` on an existing anchor, dropping it when it collapses onto the anchor.
fn set_handle(a: &mut Map<String, Value>, key: &str, h: P, p: P) {
    if dist(h, p) > 1e-9 {
        a.insert(key.into(), json!([h[0], h[1]]));
    } else {
        a.remove(key);
    }
}

fn update_out(a: &mut Value, h: P) {
    if let Value::Object(m) = a {
        let p = m.get("p").and_then(point).unwrap_or(h);
        set_handle(m, "out", h, p);
    }
}

/// A filled circle marking one crossing, in a colour that contrasts with both the usual black
/// strokes and the blue anchor/selection chrome.
fn marker_circle(c: P, r: f64) -> Value {
    const K: f64 = 0.552_284_749_830_793_4; // kappa: a circle from four cubics
    let k = K * r;
    let a = |p: P, inh: P, outh: P| json!({"p": [p[0], p[1]], "in": [inh[0], inh[1]], "out": [outh[0], outh[1]], "kind": "Smooth"});
    let (x, y) = (c[0], c[1]);
    json!({
        "type": "path",
        "name": "Crossing marker",
        "path": {"subpaths": [{"closed": true, "anchors": [
            a([x + r, y], [x + r, y + k], [x + r, y - k]),
            a([x, y - r], [x + k, y - r], [x - k, y - r]),
            a([x - r, y], [x - r, y - k], [x - r, y + k]),
            a([x, y + r], [x - k, y + r], [x + k, y + r]),
        ]}]},
        "fills": [{"type": "solid", "color": {"model": "rgb", "r": 1.0, "g": 0.0, "b": 0.8}}]
    })
}

/// Rotates every handle of the selection — or, with a direct selection, only the selected
/// anchors' handles — onto its nearest `step`-degree angle, keeping lengths; a smooth anchor's
/// two handles stay collinear (the longer one leads).
fn snap_handles(objects: &[Value], step: f64) -> Value {
    let objs = parse(objects);
    if objs.is_empty() {
        return json!({"error": "Select some paths."});
    }
    let mut out: Vec<Value> = Vec::new();
    let parsed: Vec<usize> = objs.iter().map(|o| o.index).collect();
    for o in &objs {
        let subs: Vec<Value> = o
            .subs
            .iter()
            .enumerate()
            .map(|(si, s)| {
                let anchors: Vec<Value> = s
                    .anchors
                    .iter()
                    .enumerate()
                    .map(|(ai, a)| {
                        let mut raw = a.raw.clone();
                        if !o.anchor_selected(si, ai) {
                            return Value::Object(raw);
                        }
                        let smooth = raw.get("kind").and_then(Value::as_str) == Some("Smooth") && a.inh.is_some() && a.outh.is_some();
                        if smooth {
                            // Snap the longer handle's angle; the other mirrors it.
                            let (inh, outh) = (a.inh.unwrap_or(a.p), a.outh.unwrap_or(a.p));
                            let (li, lo) = (dist(inh, a.p), dist(outh, a.p));
                            let lead = snap_vec(sub(if lo >= li { outh } else { inh }, a.p), step);
                            let (ux, uy) = unit(lead);
                            set_handle(&mut raw, "out", [a.p[0] + ux * lo, a.p[1] + uy * lo], a.p);
                            set_handle(&mut raw, "in", [a.p[0] - ux * li, a.p[1] - uy * li], a.p);
                            if lo < li {
                                // The in handle led: flip so it keeps its own direction.
                                set_handle(&mut raw, "in", [a.p[0] + ux * li, a.p[1] + uy * li], a.p);
                                set_handle(&mut raw, "out", [a.p[0] - ux * lo, a.p[1] - uy * lo], a.p);
                            }
                        } else {
                            if let Some(h) = a.inh {
                                let v = snap_vec(sub(h, a.p), step);
                                set_handle(&mut raw, "in", [a.p[0] + v[0], a.p[1] + v[1]], a.p);
                            }
                            if let Some(h) = a.outh {
                                let v = snap_vec(sub(h, a.p), step);
                                set_handle(&mut raw, "out", [a.p[0] + v[0], a.p[1] + v[1]], a.p);
                            }
                        }
                        Value::Object(raw)
                    })
                    .collect();
                json!({"closed": s.closed, "anchors": anchors})
            })
            .collect();
        out.push(json!({"id": objects[o.index]["id"], "path": {"subpaths": subs}}));
    }
    for (i, o) in objects.iter().enumerate() {
        if !parsed.contains(&i) {
            out.push(json!({"id": o["id"]}));
        }
    }
    json!({"objects": out})
}

/// Smooths the targeted anchors' handles for tangent continuity (G1), so several Bézier segments
/// flow evenly instead of fighting each other's old handles:
///
/// - **Straight segments stay straight** — no handles are pushed into them, so lines are never
///   bent into "jaggy" near-lines.
/// - **An anchor joining a line and a curve takes the line's direction** as its tangent, whatever
///   the orientation (horizontal, vertical, diagonal), so the curve merges optically smoothly
///   into the straight segment instead of kinking at the join.
/// - Elsewhere the tangent comes from the neighbours (Catmull-Rom style), or from a straight
///   guide line selected along with the paths (a plain two-anchor handle-less line), which passes
///   through unchanged.
/// - Handle lengths are `SMOOTH` (≈ 0.39) of each chord — the ratio of a circular arc's cubic,
///   noticeably fuller than the usual ⅓ so the curvature spreads out and the result reads smooth
///   to the eye.
///
/// `flow` (0–1) blends from the current handles to the smoothed ones. With a direct selection,
/// only the selected anchors change.
fn smooth_flow(objects: &[Value], flow: f64) -> Value {
    /// Handle length over chord of a cubic tracing a circular arc through its endpoints
    /// (a quarter circle: r·(4/3)·tan(π/8) over r·√2) — the "optically smooth" fullness.
    const SMOOTH: f64 = 0.390_524;
    let objs = parse(objects);
    if objs.is_empty() {
        return json!({"error": "Select some paths."});
    }
    // A guide: the first plain line (one open two-anchor handle-less subpath), used only when
    // something else is selected too.
    let straight = |o: &Obj| {
        o.subs.len() == 1
            && o.subs.first().is_some_and(|s| !s.closed && s.anchors.len() == 2 && s.anchors.iter().all(|a| a.inh.is_none() && a.outh.is_none()))
    };
    let guide_at = (objs.len() > 1).then(|| objs.iter().position(straight)).flatten();
    let guide_dir: Option<P> = guide_at.and_then(|gi| {
        let s = objs.get(gi)?.subs.first()?;
        let (a, b) = (s.anchors.first()?, s.anchors.get(1)?);
        let (ux, uy) = unit(sub(b.p, a.p));
        (ux != 0.0 || uy != 0.0).then_some([ux, uy])
    });
    let mut out: Vec<Value> = Vec::new();
    let parsed: Vec<usize> = objs.iter().map(|o| o.index).collect();
    for (oi, o) in objs.iter().enumerate() {
        if Some(oi) == guide_at {
            out.push(json!({"id": objects[o.index]["id"]}));
            continue;
        }
        let subs: Vec<Value> = o
            .subs
            .iter()
            .enumerate()
            .map(|(si, s)| {
                let n = s.anchors.len();
                // Straight segments are preserved only where they are deliberate: in a subpath
                // that also has curves. A subpath with no handles at all is a polyline being
                // smoothed into a curve, so everything participates.
                let mixed = s.anchors.iter().any(|a| a.inh.is_some() || a.outh.is_some());
                // The segment from anchor `from` (wrapping) is a straight line: no handles.
                let seg_straight = |from: usize| {
                    let to = (from + 1) % n;
                    mixed && s.anchors.get(from).is_some_and(|a| a.outh.is_none()) && s.anchors.get(to).is_some_and(|a| a.inh.is_none())
                };
                let anchors: Vec<Value> = s
                    .anchors
                    .iter()
                    .enumerate()
                    .map(|(ai, a)| {
                        let mut raw = a.raw.clone();
                        if n < 2 || !o.anchor_selected(si, ai) {
                            return Value::Object(raw);
                        }
                        let prev = if ai > 0 {
                            Some(s.anchors[ai - 1].p)
                        } else if s.closed {
                            Some(s.anchors[n - 1].p)
                        } else {
                            None
                        };
                        let next = if ai + 1 < n {
                            Some(s.anchors[ai + 1].p)
                        } else if s.closed {
                            Some(s.anchors[0].p)
                        } else {
                            None
                        };
                        let prev_straight = prev.is_some() && seg_straight(if ai > 0 { ai - 1 } else { n - 1 });
                        let next_straight = next.is_some() && seg_straight(ai);
                        if prev_straight && next_straight {
                            // A corner between two straight lines: leave it a corner.
                            return Value::Object(raw);
                        }
                        // The tangent through this anchor: its neighbours' chord, or one-sided.
                        let t = match (prev, next) {
                            (Some(q), Some(r)) => sub(r, q),
                            (Some(q), None) => sub(a.p, q),
                            (None, Some(r)) => sub(r, a.p),
                            (None, None) => return Value::Object(raw),
                        };
                        let (mut ux, mut uy) = unit(t);
                        if prev_straight && let Some(q) = prev {
                            // The curve must merge with the incoming straight segment: take the
                            // line's own direction, whatever its angle.
                            (ux, uy) = unit(sub(a.p, q));
                        } else if next_straight && let Some(r) = next {
                            (ux, uy) = unit(sub(r, a.p));
                        } else if let Some(d) = guide_dir {
                            // Follow the guide line: its direction, signed to match the flow.
                            let sgn = if ux * d[0] + uy * d[1] < 0.0 { -1.0 } else { 1.0 };
                            (ux, uy) = (d[0] * sgn, d[1] * sgn);
                        }
                        if ux == 0.0 && uy == 0.0 {
                            return Value::Object(raw);
                        }
                        if let Some(q) = prev
                            && !prev_straight
                        {
                            let len = dist(a.p, q) * SMOOTH;
                            let target = [a.p[0] - ux * len, a.p[1] - uy * len];
                            set_handle(&mut raw, "in", lerp(a.inh.unwrap_or(a.p), target, flow), a.p);
                        }
                        if let Some(r) = next
                            && !next_straight
                        {
                            let len = dist(r, a.p) * SMOOTH;
                            let target = [a.p[0] + ux * len, a.p[1] + uy * len];
                            set_handle(&mut raw, "out", lerp(a.outh.unwrap_or(a.p), target, flow), a.p);
                        }
                        if raw.contains_key("in") && raw.contains_key("out") {
                            raw.insert("kind".into(), json!("Smooth"));
                        }
                        Value::Object(raw)
                    })
                    .collect();
                json!({"closed": s.closed, "anchors": anchors})
            })
            .collect();
        out.push(json!({"id": objects[o.index]["id"], "path": {"subpaths": subs}}));
    }
    for (i, obj) in objects.iter().enumerate() {
        if !parsed.contains(&i) {
            out.push(json!({"id": obj["id"]}));
        }
    }
    json!({"objects": out})
}

/// Merges the direct-selected anchor points: consecutive selected anchors of one subpath collapse
/// into a single anchor, and every merged or lone selected anchor moves to one common point, so
/// anchors across subpaths and objects end up together. The point and the handles come from
/// `method`: `average` (centroid; handles the mean of the merged handle vectors), `geometric
/// center` (bounding-box centre; same handles) or `dominant direction` (centroid; handles turned
/// collinear with the longest selected handle, a smooth point). The surviving anchor keeps the
/// first member's tie to the segment before it and the last member's to the one after.
fn merge_anchors(objects: &[Value], method: &str) -> Value {
    let objs = parse(objects);
    let picked: Vec<(usize, usize, usize)> = objs
        .iter()
        .enumerate()
        .flat_map(|(oi, o)| {
            o.sel_anchors
                .as_deref()
                .unwrap_or(&[])
                .iter()
                .filter(|(si, ai)| o.subs.get(*si).is_some_and(|s| *ai < s.anchors.len()))
                .map(move |&(si, ai)| (oi, si, ai))
        })
        .collect();
    if picked.len() < 2 {
        return json!({"error": "Direct-select two or more anchor points (Direct Selection tool), then run the merge."});
    }
    let pts: Vec<P> = picked.iter().map(|&(oi, si, ai)| objs[oi].subs[si].anchors[ai].p).collect();
    let target = match method {
        "geometric center" => {
            let (mut lo, mut hi) = (pts[0], pts[0]);
            for p in &pts {
                lo = [lo[0].min(p[0]), lo[1].min(p[1])];
                hi = [hi[0].max(p[0]), hi[1].max(p[1])];
            }
            [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]
        }
        _ => {
            let sum = pts.iter().fold([0.0, 0.0], |s, p| [s[0] + p[0], s[1] + p[1]]);
            [sum[0] / pts.len() as f64, sum[1] / pts.len() as f64]
        }
    };
    // The dominant direction: the longest handle among the selected anchors.
    let dominant: Option<P> = picked
        .iter()
        .flat_map(|&(oi, si, ai)| {
            let a = &objs[oi].subs[si].anchors[ai];
            [a.outh.map(|h| sub(h, a.p)), a.inh.map(|h| sub(a.p, h))]
        })
        .flatten()
        .max_by(|a, b| {
            let (la, lb) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1]);
            la.partial_cmp(&lb).unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|v| {
            let (x, y) = unit(v);
            [x, y]
        });
    let mut out: Vec<Value> = Vec::new();
    let parsed: Vec<usize> = objs.iter().map(|o| o.index).collect();
    for (oi, o) in objs.iter().enumerate() {
        if o.sel_anchors.as_deref().is_none_or(|s| s.is_empty()) {
            out.push(json!({"id": objects[o.index]["id"]}));
            continue;
        }
        let subs: Vec<Value> = o
            .subs
            .iter()
            .enumerate()
            .map(|(si, s)| {
                let anchors = merge_in_subpath(s, |ai| picked.contains(&(oi, si, ai)), target, method, dominant);
                json!({"closed": s.closed, "anchors": anchors})
            })
            .collect();
        out.push(json!({"id": objects[o.index]["id"], "path": {"subpaths": subs}}));
    }
    for (i, obj) in objects.iter().enumerate() {
        if !parsed.contains(&i) {
            out.push(json!({"id": obj["id"]}));
        }
    }
    json!({"objects": out})
}

/// The subpath's anchors with each run of consecutive selected anchors merged into one anchor at
/// `target` (runs wrap round a closed subpath). Returns the new anchor list as JSON.
fn merge_in_subpath(s: &SubPath, selected: impl Fn(usize) -> bool, target: P, method: &str, dominant: Option<P>) -> Vec<Value> {
    let n = s.anchors.len();
    let sel: Vec<bool> = (0..n).map(&selected).collect();
    // Each anchor's run id, runs numbered by their first member in walking order; wrapped runs of
    // a closed subpath take the id of their start near the end.
    let mut run = vec![usize::MAX; n];
    let mut runs: Vec<Vec<usize>> = Vec::new();
    let all = sel.iter().all(|&b| b);
    if all && n > 0 {
        runs.push((0..n).collect());
        run.iter_mut().for_each(|r| *r = 0);
    } else {
        for start in 0..n {
            let before = if s.closed { sel[(start + n - 1) % n] } else { start > 0 && sel[start - 1] };
            if !sel[start] || before {
                continue;
            }
            let mut members = Vec::new();
            let mut i = start;
            while sel[i] && members.len() < n {
                members.push(i);
                run[i] = runs.len();
                i = (i + 1) % n;
                if !s.closed && i == 0 {
                    break;
                }
            }
            runs.push(members);
        }
    }
    let mut out = Vec::new();
    let mut emitted = vec![false; runs.len()];
    for (ai, &r) in run.iter().enumerate().take(n) {
        if r == usize::MAX {
            out.push(Value::Object(s.anchors[ai].raw.clone()));
            continue;
        }
        if emitted[r] {
            continue;
        }
        emitted[r] = true;
        out.push(merged_anchor(s, &runs[r], target, method, dominant));
    }
    out
}

/// One anchor standing in for the run `members` (subpath anchor indices, in path order), at
/// `target`, its handles per `method`.
fn merged_anchor(s: &SubPath, members: &[usize], target: P, method: &str, dominant: Option<P>) -> Value {
    let hvec = |get: fn(&Anchor) -> Option<P>| -> Option<P> {
        // The mean of the members' handle vectors (relative to their anchors).
        let vs: Vec<P> = members.iter().filter_map(|&i| get(&s.anchors[i]).map(|h| sub(h, s.anchors[i].p))).collect();
        (!vs.is_empty()).then(|| {
            let sum = vs.iter().fold([0.0, 0.0], |a, v| [a[0] + v[0], a[1] + v[1]]);
            [sum[0] / vs.len() as f64, sum[1] / vs.len() as f64]
        })
    };
    // Keep the tie to the neighbouring segments: in from the first member, out from the last; the
    // averaging methods mean over the run instead.
    let (first, last) = (&s.anchors[members[0]], &s.anchors[members[members.len() - 1]]);
    let (mut inv, mut outv) = if method == "dominant direction" || members.len() == 1 {
        (first.inh.map(|h| sub(h, first.p)), last.outh.map(|h| sub(h, last.p)))
    } else {
        (hvec(|a| a.inh), hvec(|a| a.outh))
    };
    if method == "dominant direction"
        && let Some(d) = dominant
    {
        inv = inv.map(|v| {
            let len = (v[0] * v[0] + v[1] * v[1]).sqrt();
            [-d[0] * len, -d[1] * len]
        });
        outv = outv.map(|v| {
            let len = (v[0] * v[0] + v[1] * v[1]).sqrt();
            [d[0] * len, d[1] * len]
        });
    }
    let smooth = method == "dominant direction" && inv.is_some() && outv.is_some();
    anchor_json(target, inv.map(|v| [target[0] + v[0], target[1] + v[1]]), outv.map(|v| [target[0] + v[0], target[1] + v[1]]), smooth)
}

/// `v` rotated onto the nearest multiple of `step` degrees, length kept — the same rule as the
/// app's own Shift constraint (`vectorcraft_geom::constrain_angle`).
fn snap_vec(v: P, step: f64) -> P {
    let len = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if len < 1e-12 {
        return v;
    }
    let step = step.to_radians();
    let ang = (v[1].atan2(v[0]) / step).round() * step;
    [len * ang.cos(), len * ang.sin()]
}

fn unit(v: P) -> (f64, f64) {
    let len = (v[0] * v[0] + v[1] * v[1]).sqrt();
    if len < 1e-12 { (0.0, 0.0) } else { (v[0] / len, v[1] / len) }
}

/// Keeps `v` serialised in memory for the host to read, as `len << 32 | ptr`.
fn output(v: &Value) -> i64 {
    let Ok(bytes) = serde_json::to_vec(v) else { return -2 };
    let bytes = bytes.leak();
    pack(bytes) as i64
}

fn pack(bytes: &[u8]) -> u64 {
    ((bytes.len() as u64) << 32) | bytes.as_ptr() as u64
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(id: u64, a: P, b: P) -> Value {
        json!({"id": id, "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [a[0], a[1]]}, {"p": [b[0], b[1]]}]}]}})
    }

    fn anchors(out: &Value, obj: usize, sub: usize) -> Vec<Value> {
        out["objects"][obj]["path"]["subpaths"][sub]["anchors"].as_array().cloned().unwrap_or_default()
    }

    /// Two crossing lines get one new anchor each, at the crossing, with no handles.
    #[test]
    fn crossing_lines_gain_an_anchor() {
        let doc = json!({"objects": [line(1, [0.0, 0.0], [100.0, 100.0]), line(2, [0.0, 100.0], [100.0, 0.0])]});
        let out = run(&doc, &json!({"mode": "insert"}));
        for obj in 0..2 {
            let a = anchors(&out, obj, 0);
            assert_eq!(a.len(), 3, "{out}");
            assert_eq!(a[1]["p"], json!([50.0, 50.0]));
            assert!(a[1].get("in").is_none() && a[1].get("out").is_none());
        }
    }

    /// A curve crossing a line is split with de Casteljau: the shape survives (the new anchor is
    /// on the curve) and the new anchor is smooth.
    #[test]
    fn curve_meets_line() {
        // A cubic from (0,50) to (100,50) bulging down to y=87.5 at t=0.5, crossing x=50.
        let curve = json!({"id": 7, "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 50.0], "out": [25.0, 100.0]},
            {"p": [100.0, 50.0], "in": [75.0, 100.0]}]}]}});
        let doc = json!({"objects": [curve, line(8, [50.0, 0.0], [50.0, 200.0])]});
        let out = run(&doc, &json!({"mode": "insert"}));
        let a = anchors(&out, 0, 0);
        assert_eq!(a.len(), 3, "{out}");
        let p = a[1]["p"].as_array().unwrap();
        assert!((p[0].as_f64().unwrap() - 50.0).abs() < 1e-3);
        assert!((p[1].as_f64().unwrap() - 87.5).abs() < 1e-3);
        assert_eq!(a[1]["kind"], json!("Smooth"));
        // The line got its anchor too.
        assert_eq!(anchors(&out, 1, 0).len(), 3);
    }

    /// `mark` leaves the objects alone and adds one marker per crossing, magenta-filled.
    #[test]
    fn mark_adds_markers() {
        let doc = json!({"objects": [line(1, [0.0, 0.0], [100.0, 100.0]), line(2, [0.0, 100.0], [100.0, 0.0])]});
        let out = run(&doc, &json!({"mode": "mark", "marker": 6.0}));
        let objs = out["objects"].as_array().unwrap();
        assert_eq!(objs.len(), 3, "{out}");
        assert_eq!(objs[0], json!({"id": 1}));
        assert_eq!(objs[2]["name"], json!("Crossing marker"));
        assert_eq!(objs[2]["fills"][0]["color"]["model"], json!("rgb"));
    }

    /// Crossings that land on an existing anchor are skipped; parallel lines have none.
    #[test]
    fn no_crossings_declines() {
        let doc = json!({"objects": [line(1, [0.0, 0.0], [100.0, 0.0]), line(2, [0.0, 10.0], [100.0, 10.0])]});
        let out = run(&doc, &json!({"mode": "insert"}));
        assert!(out["error"].is_string(), "{out}");
        // Lines meeting at a shared endpoint: that point is an anchor already.
        let doc = json!({"objects": [line(1, [0.0, 0.0], [50.0, 50.0]), line(2, [50.0, 50.0], [100.0, 0.0])]});
        let out = run(&doc, &json!({"mode": "insert"}));
        assert!(out["error"].is_string(), "{out}");
    }

    /// A self-crossing subpath (a figure of eight) gets anchors at its own crossing.
    #[test]
    fn self_intersection() {
        let doc = json!({"objects": [{"id": 3, "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 0.0]}, {"p": [100.0, 100.0]}, {"p": [100.0, 0.0]}, {"p": [0.0, 100.0]}]}]}}]});
        let out = run(&doc, &json!({"mode": "insert"}));
        let a = anchors(&out, 0, 0);
        assert_eq!(a.len(), 6, "{out}"); // crossing of segment 0 and 2 → one anchor on each
        assert_eq!(a[1]["p"], json!([50.0, 50.0]));
        assert_eq!(a[4]["p"], json!([50.0, 50.0]));
    }

    /// Handle snapping: 45° steps by default, length kept, smooth anchors stay collinear.
    #[test]
    fn snap_handles_to_45() {
        let doc = json!({"objects": [{"id": 4, "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 0.0], "out": [10.0, 1.0]},
            {"p": [100.0, 0.0], "in": [90.0, 2.0], "out": [110.0, -2.0], "kind": "Smooth"},
            {"p": [200.0, 0.0]}]}]}}]});
        let out = run(&doc, &json!({"mode": "snap handles", "angle": 45}));
        let a = anchors(&out, 0, 0);
        // (10, 1) from (0,0): nearest 45° step is 0° → (len, 0).
        let h = a[0]["out"].as_array().unwrap();
        let len = (10.0f64 * 10.0 + 1.0).sqrt();
        assert!((h[0].as_f64().unwrap() - len).abs() < 1e-9 && h[1].as_f64().unwrap().abs() < 1e-9);
        // The smooth anchor's handles stay opposite.
        let (inh, outh) = (a[1]["in"].as_array().unwrap(), a[1]["out"].as_array().unwrap());
        let (iv, ov) = ([inh[0].as_f64().unwrap() - 100.0, inh[1].as_f64().unwrap()], [outh[0].as_f64().unwrap() - 100.0, outh[1].as_f64().unwrap()]);
        assert!(cross(iv, ov).abs() < 1e-9, "{out}");
        assert!(iv[0] * ov[0] + iv[1] * ov[1] < 0.0, "handles must point apart");
    }

    /// The crossing of two curves found by subdivision is accurate to well under a hundredth pt.
    #[test]
    fn curve_curve_accuracy() {
        let a = Cubic { p: [[0.0, 0.0], [50.0, 100.0], [50.0, 100.0], [100.0, 0.0]] };
        let b = Cubic { p: [[0.0, 75.0], [50.0, -25.0], [50.0, -25.0], [100.0, 75.0]] };
        let mut hits = Vec::new();
        curve_x(&a, &b, (0.0, 1.0), (0.0, 1.0), 24, &mut hits);
        assert!(!hits.is_empty());
        for (ta, tb) in hits {
            assert!(dist(a.eval(ta), b.eval(tb)) < 1e-2);
        }
    }

    /// A line with selected anchors, `|sel|` of them: `[[si, ai], …]` attached to the object.
    fn with_sel(mut o: Value, sel: &[[u64; 2]]) -> Value {
        o["selectedAnchors"] = json!(sel);
        o
    }

    /// A direct selection limits `insert` to segments touching the selected anchors, with clear
    /// feedback when the selected parts don't cross.
    #[test]
    fn insert_honours_the_direct_selection() {
        // A zigzag whose first segment crosses the line; only its far anchors are selected, so
        // nothing participates for it and no crossing is found.
        let zig = json!({"id": 1, "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 0.0]}, {"p": [100.0, 100.0]}, {"p": [200.0, 0.0]}]}]}});
        let cross = line(2, [0.0, 100.0], [100.0, 0.0]);
        let doc = json!({"objects": [with_sel(zig.clone(), &[[0, 2]]), cross.clone()]});
        let out = run(&doc, &json!({"mode": "insert"}));
        assert!(out["error"].as_str().unwrap_or("").contains("direct-selected"), "{out}");
        // Selecting an anchor of the crossing segment brings it back in.
        let doc = json!({"objects": [with_sel(zig, &[[0, 0]]), cross]});
        let out = run(&doc, &json!({"mode": "insert"}));
        assert_eq!(anchors(&out, 0, 0).len(), 4, "{out}");
        assert_eq!(anchors(&out, 1, 0).len(), 3);
    }

    /// `snap handles` with a direct selection leaves unselected anchors' handles alone.
    #[test]
    fn snap_handles_honours_the_direct_selection() {
        let doc = json!({"objects": [{"id": 4, "selectedAnchors": [[0, 0]], "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 0.0], "out": [10.0, 1.0]},
            {"p": [100.0, 0.0], "in": [90.0, 2.0]},
            {"p": [200.0, 0.0]}]}]}}]});
        let out = run(&doc, &json!({"mode": "snap handles", "angle": 45}));
        let a = anchors(&out, 0, 0);
        assert!(a[0]["out"][1].as_f64().unwrap().abs() < 1e-9, "selected: snapped");
        assert_eq!(a[1]["in"], json!([90.0, 2.0]), "unselected: untouched");
    }

    /// `smooth flow` gives a polyline tangent-continuous handles; with a straight line selected
    /// too, the tangents follow the line's direction.
    #[test]
    fn smooth_flow_is_g1_and_follows_a_line() {
        let zig = json!({"id": 1, "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 0.0]}, {"p": [100.0, 40.0]}, {"p": [200.0, 0.0]}, {"p": [300.0, 40.0]}]}]}});
        let out = run(&json!({"objects": [zig.clone()]}), &json!({"mode": "smooth flow", "flow": 100}));
        let a = anchors(&out, 0, 0);
        // Inner anchors are smooth: in and out handles collinear through the anchor.
        for i in [1, 2] {
            assert_eq!(a[i]["kind"], json!("Smooth"), "{out}");
            let p = [a[i]["p"][0].as_f64().unwrap(), a[i]["p"][1].as_f64().unwrap()];
            let hi = [a[i]["in"][0].as_f64().unwrap() - p[0], a[i]["in"][1].as_f64().unwrap() - p[1]];
            let ho = [a[i]["out"][0].as_f64().unwrap() - p[0], a[i]["out"][1].as_f64().unwrap() - p[1]];
            assert!(cross(hi, ho).abs() < 1e-9 && hi[0] * ho[0] + hi[1] * ho[1] < 0.0, "G1 at {i}");
        }
        // With a horizontal guide line, every handle is horizontal and the guide is untouched.
        let out = run(&json!({"objects": [zig, line(2, [0.0, 80.0], [300.0, 80.0])]}), &json!({"mode": "smooth flow", "flow": 100}));
        let a = anchors(&out, 0, 0);
        for a in a.iter().take(4) {
            for key in ["in", "out"] {
                if let Some(h) = a.get(key) {
                    let dy = h[1].as_f64().unwrap() - a["p"][1].as_f64().unwrap();
                    assert!(dy.abs() < 1e-9, "handle {key} follows the line: {out}");
                }
            }
        }
        assert_eq!(out["objects"][1], json!({"id": 2}), "the guide line is untouched");
    }

    /// In a path mixing lines and curves, `smooth flow` keeps the straight segments straight and
    /// turns the junction anchor's tangent onto the line's direction — the curve merges with the
    /// line, horizontal or diagonal alike — with handles fuller than the old third of the chord.
    #[test]
    fn smooth_flow_merges_curves_into_straight_segments() {
        for (a0, a1) in [([0.0, 0.0], [100.0, 0.0]), ([0.0, 0.0], [100.0, 100.0])] {
            // A straight first segment, then a curved one.
            let path = json!({"id": 1, "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
                {"p": a0},
                {"p": a1, "out": [a1[0] + 20.0, a1[1] + 30.0]},
                {"p": [a1[0] + 100.0, a1[1] + 100.0], "in": [a1[0] + 80.0, a1[1] + 80.0]}]}]}});
            let out = run(&json!({"objects": [path]}), &json!({"mode": "smooth flow", "flow": 100}));
            let a = anchors(&out, 0, 0);
            // The line stays a line: no handles pushed into the straight segment.
            assert!(a[0].get("out").is_none() && a[1].get("in").is_none(), "straight stays straight: {out}");
            // The junction's out handle continues the line's own direction.
            let dir = unit(sub(a1, a0));
            let h = [a[1]["out"][0].as_f64().unwrap() - a1[0], a[1]["out"][1].as_f64().unwrap() - a1[1]];
            let (hx, hy) = unit(h);
            assert!((hx - dir.0).abs() < 1e-9 && (hy - dir.1).abs() < 1e-9, "merges with the line: {out}");
            // Fuller than the old third of the chord: the circle-arc ratio.
            let chord = dist(a1, [a1[0] + 100.0, a1[1] + 100.0]);
            let len = dist(h, [0.0, 0.0]);
            assert!(len > chord / 3.0 && (len - chord * 0.390_524).abs() < 1e-6, "stronger handles: {len} of {chord}");
        }
    }

    /// `merge anchors` needs a direct selection of two or more anchors, collapses consecutive
    /// selected anchors to the centroid and keeps the run's outer handles' ties.
    #[test]
    fn merge_anchors_average() {
        let doc = json!({"objects": [line(1, [0.0, 0.0], [100.0, 100.0])]});
        let out = run(&doc, &json!({"mode": "merge anchors"}));
        assert!(out["error"].as_str().unwrap_or("").contains("Direct-select"), "{out}");
        // Two inner anchors of a four-anchor path merge to their centroid.
        let path = json!({"id": 1, "selectedAnchors": [[0, 1], [0, 2]], "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 0.0], "out": [10.0, 10.0]},
            {"p": [100.0, 0.0], "in": [90.0, -10.0]},
            {"p": [140.0, 20.0], "out": [160.0, 30.0]},
            {"p": [300.0, 0.0]}]}]}});
        let out = run(&json!({"objects": [path]}), &json!({"mode": "merge anchors", "merge": "average"}));
        let a = anchors(&out, 0, 0);
        assert_eq!(a.len(), 3, "{out}");
        assert_eq!(a[1]["p"], json!([120.0, 10.0]));
        // Mean handle vectors: in = mean([-10,-10]) rel 1 member with in; out = mean([20,10]).
        assert_eq!(a[1]["in"], json!([110.0, 0.0]));
        assert_eq!(a[1]["out"], json!([140.0, 20.0]));
        assert_eq!(a[0]["p"], json!([0.0, 0.0]), "unselected anchors stay");
    }

    /// Anchors across two objects move to one common point (geometric centre), merging the gap.
    #[test]
    fn merge_anchors_across_objects_geometric_center() {
        let a = with_sel(line(1, [0.0, 0.0], [100.0, 0.0]), &[[0, 1]]);
        let b = with_sel(line(2, [120.0, 40.0], [200.0, 40.0]), &[[0, 0]]);
        let out = run(&json!({"objects": [a, b]}), &json!({"mode": "merge anchors", "merge": "geometric center"}));
        assert_eq!(anchors(&out, 0, 0)[1]["p"], json!([110.0, 20.0]), "{out}");
        assert_eq!(anchors(&out, 1, 0)[0]["p"], json!([110.0, 20.0]));
    }

    /// `dominant direction` turns the merged point's handles collinear with the longest selected
    /// handle, making it smooth.
    #[test]
    fn merge_anchors_dominant_direction() {
        let path = json!({"id": 1, "selectedAnchors": [[0, 1], [0, 2]], "type": "path", "path": {"subpaths": [{"closed": false, "anchors": [
            {"p": [0.0, 0.0]},
            {"p": [100.0, 0.0], "in": [60.0, 0.0]},
            {"p": [140.0, 20.0], "out": [150.0, 30.0]},
            {"p": [300.0, 0.0]}]}]}});
        let out = run(&json!({"objects": [path]}), &json!({"mode": "merge anchors", "merge": "dominant direction"}));
        let a = anchors(&out, 0, 0);
        assert_eq!(a.len(), 3, "{out}");
        assert_eq!(a[1]["kind"], json!("Smooth"));
        let p = [a[1]["p"][0].as_f64().unwrap(), a[1]["p"][1].as_f64().unwrap()];
        let hi = [a[1]["in"][0].as_f64().unwrap() - p[0], a[1]["in"][1].as_f64().unwrap() - p[1]];
        let ho = [a[1]["out"][0].as_f64().unwrap() - p[0], a[1]["out"][1].as_f64().unwrap() - p[1]];
        // The dominant handle is the in handle of anchor 1 (length 40, along -x → direction +x).
        assert!(cross(hi, ho).abs() < 1e-9 && hi[0] * ho[0] + hi[1] * ho[1] < 0.0, "collinear: {out}");
        assert!((dist(hi, [0.0, 0.0]) - 40.0).abs() < 1e-9, "in length kept");
    }
}
