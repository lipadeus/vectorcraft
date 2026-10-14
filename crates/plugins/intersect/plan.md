# Intersection Anchors — plan

Working notes for this plug-in, kept up to date so other developers (or AI agents) can pick the
work up. The requirements came in as: (1) create an anchor point where two segments/curves cross,
(2) highlight crossings in a contrasting colour before the user commits, (3) Shift-snap Bézier
handle/anchor drags to 45° steps like line rotation. A second round added: (4) curves that follow
a line evenly (tangent continuity, no distorted multi-segment Béziers), (5) honouring the user's
direct selection of anchors/segments for intersections, (6) merging selected anchor points
(difference/centroid/handle-average analysis, several strategies), with the UI/UX separated per
function ("tabs").

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
- The host's `Selection` (`crates/doc/src/selection.rs`) tracks direct-selected anchors and
  segments per object (`anchors`, `segments`: `(subpath, index)` sets), but the plug-in input
  originally carried only whole objects. **Host extended (round 2):**
  `vectorcraft_plugins::objects::encode_input_selected` now attaches `selectedAnchors` /
  `selectedSegments` (sorted `[subpath, index]` pairs) to each direct-selected object, and
  `cmd/plugin.rs` passes the live selection. Additive and backwards compatible (absent = whole
  object selected); documented in `docs/plugins.md` and covered by
  `tests_plugins::filter_input_carries_the_direct_selection`.

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
- **A6 — selection in the input (round 2)**: ABI v1 input had no direct-selection information, so
  the host was extended additively (see §1). Older plug-ins ignore the new keys; this plug-in
  treats an object without them as fully selected.
- **A7 — no tabs in generated dialogs**: ABI v1 dialogs are a flat parameter list; there is no
  tab widget. Workaround: the `mode` dropdown is the "tab selector", each mode reads only its own
  parameters (others are ignored), and the README documents each mode in its own section with a
  mode→parameter table.
- **A8 — "follow a line" = guide object**: a plain two-anchor straight line in the selection is
  the direction guide for `smooth flow` (and passes through unchanged); anything else is
  smoothed. Chosen over a numeric angle parameter so users point at the line they drew.
- **A9 — merge needs a direct selection**: merging all anchors of a whole-selected object is
  never what the user means, so `merge anchors` requires ≥ 2 direct-selected anchors and
  otherwise declines with instructions (the "clear feedback" UX requirement).

## 3. Design (done)

- One filter plug-in, id `org.vectorcraft.intersect`, five modes (`mode` choice param):
  `insert`, `mark`, `snap handles`, `smooth flow`, `merge anchors`; `marker` size (pt), `angle`
  step (deg), `flow` blend (%) and `merge` strategy params.
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
  With a direct selection, only selected anchors' handles are touched.
- Selection semantics (round 2): a segment *participates* in `insert`/`mark` when its object has
  no direct selection, the segment is in `selectedSegments`, or either endpoint anchor is in
  `selectedAnchors`; crossings need both segments participating. An anchor is *selected* for
  `snap handles`/`smooth flow` under the same rule; `merge anchors` uses only explicitly
  direct-selected anchors. When a direct selection restricted the search and nothing crossed,
  the decline message says so (UX feedback).
- `smooth flow`: per anchor, tangent = normalised `next − prev` (Catmull-Rom; one-sided at open
  ends), handles placed on the tangent at ⅓ of the chord to each neighbour, anchor positions
  unchanged → G1 continuity. `flow` (0–100 %) lerps original → smoothed handles. A plain
  two-anchor straight line among the parsed objects (when more than one object is selected) is
  the *guide*: tangents become the guide's direction (sign-matched by dot product with the
  Catmull-Rom tangent), and the guide passes through untouched.
- `merge anchors`: group direct-selected anchors; per subpath, runs of consecutive selected
  anchors (with cyclic wrap on closed subpaths) collapse into one anchor, lone selected anchors
  move; all survivors land on the shared *target*: centroid (`average`, `dominant direction`) or
  bounding-box centre (`geometric center`) of all selected anchors across objects. Handles:
  mean of the members' handle vectors (`average`/`geometric center`), or kept lengths re-aimed
  along ± the longest selected handle's unit vector with kind `Smooth` (`dominant direction`).

## 4. Status

- [x] Investigate plug-in API and existing snapping (§1)
- [x] Geometry: cubic eval/split/bbox/flatness, segment and curve intersection
- [x] `insert`, `mark`, `snap handles` modes; input parsing that preserves unknown anchor keys
- [x] Native unit tests — `cargo test` (13: lines, curve–line, self-intersection, markers,
      declines, snapping, curve–curve accuracy, selection-restricted insert and snap, smooth
      flow G1 + guide line, merge average/geometric-center/dominant-direction)
- [x] wasm build (`build.sh`) and end-to-end check through the real host sandbox
      (`vectorcraft_plugins::Plugin::load` + `run`): load, insert, mark, smooth flow and merge
      verified
- [x] README (install, per-mode "tab" sections, method, limitations)
- [x] Round 2: host `selectedAnchors`/`selectedSegments` input extension (+ engine test,
      `docs/plugins.md`), selection-aware insert/mark/snap, `smooth flow`, `merge anchors`

## 5. Possible follow-ups (for the next agent/developer)

- A committed test fixture + host-side test in `crates/plugins/tests` (like `desaturate.wasm`),
  if the maintainers want this sample covered by `cargo xtask ci`.
- An "insert and mark" combined mode, or a `tolerance` parameter for near-misses.
- Collinear-overlap handling (report the overlap's endpoints).
- If/when the host grows tool plug-ins (ABI v2?), move hover-highlighting and click-to-insert
  into a real tool; until then the native Anchor Point tool plus this filter is the path.
- Upstream idea: pass `constrain_angle` (the preference) in `_context` so plug-ins can match the
  app's snapping base angle (A4).
- `merge anchors` could optionally re-fit the neighbours' handles to preserve the merged region's
  shape exactly (least-squares cubic fit of the removed run); today the neighbours keep their
  handles and `smooth flow` is the manual fix-up.
- `smooth flow` could accept a curved guide (project tangents onto the nearest point's tangent)
  and a numeric angle alternative when no guide line is selected.
- G2 (curvature) continuity as a stronger option for `smooth flow`.
