# Intersection Anchors — plan

Working notes for this plug-in, kept up to date so other developers (or AI agents) can pick the
work up. The requirements came in as: (1) create an anchor point where two segments/curves cross,
(2) highlight crossings in a contrasting colour before the user commits, (3) Shift-snap Bézier
handle/anchor drags to 45° steps like line rotation.

## 1. Investigation of the host (done)

- Plug-in API: **ABI v1 WebAssembly object filters / live effects only** (`docs/plugins.md`,
  host in `crates/plugins`, commands in `crates/engine/src/cmd/plugin.rs`). Explicitly *not in
  v1*: tool plug-ins (pointer events, canvas overlays), panel plug-ins. Modules must have **no
  imports**; every run gets a fresh instance; filters run on the selection as JSON and return the
  new object state; filters with parameters get a generated dialog **with live preview**.
- The app already Shift-snaps handle drags natively: `tools/src/guides.rs` (`HandleSnap::snap`,
  `Leave::segment`) uses `vectorcraft_geom::constrain_angle_from(v, 45.0, cx.constrain_angle)`
  in the Direct Selection, Anchor Point and Pen tools.
- Path JSON shape: `{"subpaths": [{"closed", "anchors": [{"p", "in"?, "out"?, "kind"?}]}]}`,
  points in pt, y down. New objects default to no fills/strokes; `fills` on a new object paints it.

## 2. Assumptions (stated before implementation)

- **A1 — no tool plug-in**: the interactive parts (hover highlight, drag snapping) cannot be a
  plug-in. Workarounds: the dialog's live preview shows crossings/anchors before OK ("mark" and
  "insert" preview), and interactive Shift-snapping is documented as already built into the app.
  A batch "snap handles" mode covers the non-interactive remainder.
- **A2 — marker colour**: magenta `rgb(1, 0, 0.8)` contrasts with default black strokes and the
  blue selection/anchor chrome. Markers are ordinary filled paths (the output format has no
  "overlay" concept); undo removes them.
- **A3 — crossing = transversal intersection point**: collinear overlaps have no single point and
  are ignored; hits at existing anchors are skipped (nothing to insert there).
- **A4 — snapping base angle 0°**: the host does not pass the Constrain Angle preference to
  plug-ins, so batch snapping counts steps from 0° (the app's interactive snap honours the
  preference).
- **A5 — delivery**: the task asked for a new repo; this sandbox can only push to the current
  repository, so the plug-in lives as a self-contained, non-workspace crate at
  `crates/plugins/intersect` (same pattern as `crates/plugins/example`). It can be moved to its
  own repo verbatim: it depends only on `serde_json`.

## 3. Design (done)

- One filter plug-in, id `org.vectorcraft.intersect`, three modes (`mode` choice param):
  `insert`, `mark`, `snap handles`; `marker` size (pt) and `angle` step (deg) params.
- Intersection method (documented choice): pairwise recursive de Casteljau subdivision.
  Segment → cubic (lines degenerate); prune by control-point bounding boxes; split both halves
  until flat (≤ 1e-4 pt from chord, depth ≤ 24); intersect chords; map parameters back; merge
  hits within 1e-3 pt; skip hits on existing anchors. O(n² · log) over segments, capped at
  20,000 segments / 4,096 hits (the wasm fuel budget is the backstop).
- Insertion: per segment, sort split parameters, split the cubic cumulatively (renormalising t),
  insert `Smooth` anchors with the split handles; straight segments get handle-less anchors; the
  neighbouring anchors' `out`/`in` handles are shortened to the split pieces. Segments are
  processed highest-index first so earlier indices stay valid while splicing.
- Snapping: rotate each handle vector onto `round(angle / step) * step`, length kept; for a
  `Smooth` anchor the longer handle leads and the other mirrors it, so smoothness survives.

## 4. Status

- [x] Investigate plug-in API and existing snapping (§1)
- [x] Geometry: cubic eval/split/bbox/flatness, segment and curve intersection
- [x] `insert`, `mark`, `snap handles` modes; input parsing that preserves unknown anchor keys
- [x] Native unit tests (7: lines, curve–line, self-intersection, markers, declines, snapping,
      curve–curve accuracy) — `cargo test`
- [x] wasm build (`build.sh`) and end-to-end check through the real host sandbox
      (`vectorcraft_plugins::Plugin::load` + `run`): load, insert and mark verified
- [x] README (install, usage, method, limitations)

## 5. Possible follow-ups (for the next agent/developer)

- A committed test fixture + host-side test in `crates/plugins/tests` (like `desaturate.wasm`),
  if the maintainers want this sample covered by `cargo xtask ci`.
- An "insert and mark" combined mode, or a `tolerance` parameter for near-misses.
- Collinear-overlap handling (report the overlap's endpoints).
- If/when the host grows tool plug-ins (ABI v2?), move hover-highlighting and click-to-insert
  into a real tool; until then the native Anchor Point tool plus this filter is the path.
- Upstream idea: pass `constrain_angle` (the preference) in `_context` so plug-ins can match the
  app's snapping base angle (A4).
