//! Intersection Anchors: a VectorCraft object filter plug-in (plug-in ABI v1, see
//! `docs/plugins.md`).
//!
//! Three modes on the selected paths and compound paths:
//!
//! - **insert**: finds every point where two segments (lines or Bézier curves) cross — between
//!   objects, between subpaths and within one subpath — and inserts an anchor point exactly there,
//!   splitting each crossing segment with de Casteljau so the shapes don't change.
//! - **mark**: adds a small magenta marker on top of each crossing instead of editing the paths,
//!   so the crossings are easy to see (and, with the dialog's live preview, easy to see *before*
//!   committing). Undo removes the markers in one step.
//! - **snap handles**: rotates every Bézier handle of the selection onto the nearest multiple of
//!   `angle` degrees (45° by default), keeping its length; smooth anchors stay smooth. The
//!   interactive equivalent — holding Shift while dragging a handle — is built into the app's
//!   Direct Selection, Anchor Point and Pen tools, which constrain to 45° steps natively.
//!
//! Intersections are found by recursive subdivision: each pair of segments is split in half until
//! both halves are flat enough to be treated as lines, whose crossing gives the curve parameters.
//! Crossings that coincide with an existing anchor are skipped, and coincident hits are merged.
//! Only `serde_json` is used: no imports, no WASI.

use serde_json::{Map, Value, json};

const MANIFEST: &str = r#"{
  "id": "org.vectorcraft.intersect",
  "name": "Intersection Anchors",
  "version": "1.0.0",
  "kind": "filter",
  "author": "VectorCraft contributors",
  "description": "Inserts anchor points where the selected paths cross, marks the crossings, or snaps Bezier handles to angle steps.",
  "params": {
    "mode": {"type": "choice", "options": ["insert", "mark", "snap handles"], "default": "insert"},
    "marker": {"type": "number", "min": 1, "max": 50, "default": 5},
    "angle": {"type": "int", "min": 1, "max": 180, "default": 45}
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
    let mode = params["mode"].as_str().unwrap_or("insert");
    if mode == "snap handles" {
        let step = params["angle"].as_f64().unwrap_or(45.0).clamp(1.0, 180.0);
        return snap_handles(objects, step);
    }
    let marker = params["marker"].as_f64().unwrap_or(5.0).clamp(1.0, 50.0);
    intersections(objects, mode, marker)
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

/// One selected object's parsed subpaths, with its index into `objects`.
struct Obj {
    index: usize,
    subs: Vec<SubPath>,
}

fn point(v: &Value) -> Option<P> {
    let (x, y) = (v.get(0)?.as_f64()?, v.get(1)?.as_f64()?);
    (x.is_finite() && y.is_finite()).then_some([x, y])
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
            out.push(Obj { index, subs: parsed });
        }
    }
    out
}

/// A segment's address: object (index into the parsed list), subpath, first anchor.
type SegId = (usize, usize, usize);

/// Every segment of every parsed object, as a degenerate-aware cubic.
fn segments(objs: &[Obj]) -> Vec<(SegId, Cubic)> {
    let mut out = Vec::new();
    for (oi, o) in objs.iter().enumerate() {
        for (si, s) in o.subs.iter().enumerate() {
            let n = s.anchors.len();
            let count = if s.closed && n > 1 { n } else { n.saturating_sub(1) };
            for ai in 0..count {
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
        return json!({"error": "No crossings found in the selection."});
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

/// Rotates every handle of the selection onto its nearest `step`-degree angle, keeping lengths;
/// a smooth anchor's two handles stay collinear (the longer one leads).
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
            .map(|s| {
                let anchors: Vec<Value> = s
                    .anchors
                    .iter()
                    .map(|a| {
                        let mut raw = a.raw.clone();
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
}
