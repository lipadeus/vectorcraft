# Intersection Anchors — a VectorCraft plug-in

A WebAssembly object-filter plug-in (plug-in ABI v1, see [`docs/plugins.md`](../../../docs/plugins.md))
for working with crossing lines and Bézier curves. It has five tools, shown in the dialog as a
**row of real tabs** (the manifest's `tabs` layout): clicking a tab picks the tool, shows that
tab's own description and then only that tool's input fields, so nothing gets mixed up. **Each
tab is documented in its own section below.**

| Tab | What it does | Its parameters |
|---|---|---|
| [insert](#tab-insert) | Anchor points at crossings | — |
| [mark](#tab-mark) | Magenta markers on crossings | `marker` |
| [snap handles](#tab-snap-handles) | Quantise handle angles | `angle` |
| [smooth flow](#tab-smooth-flow) | Even, tangent-continuous curves that merge with straight lines | `flow` |
| [merge anchors](#tab-merge-anchors) | Merge selected anchor points into one | `merge` |

## Installing

1. Build the module (needs Rust with the wasm target):

   ```sh
   rustup target add wasm32-unknown-unknown
   crates/plugins/intersect/build.sh
   ```

   This produces `target/wasm32-unknown-unknown/release/vectorcraft_plugin_intersect.wasm`.

2. Install it in VectorCraft, any of:
   - **File › Open** the `.wasm` file (works on the web build too);
   - **Object › Plug-ins › Install Plug-in…**;
   - the `plugin.install` command: `{"path": "/…/vectorcraft_plugin_intersect.wasm"}`;
   - drop it into the *Additional Plug-ins Folder* (Edit › Preferences › Performance & Storage)
     to have it installed at every start.

## Using it

Select what you care about and run **Object › Plug-ins › Intersection Anchors**. The dialog shows
the parameters with a **live preview**, so you see the result on the canvas *before* you click
OK — that preview is how "show what will be affected before the change" works within the plug-in
API. Everything runs as one undo step.

### Your selection matters

The plug-in respects the **direct selection** (Direct Selection tool). Every object reaches the
plug-in with its direct-selected anchors and segments attached:

- **Whole object selected** (Selection tool, or no anchors picked) — the whole object
  participates.
- **Segments selected** — only those segments participate.
- **Anchor points selected** — the anchors themselves participate (snap handles, merge anchors),
  and every segment touching a selected anchor participates (insert, mark, smooth flow).

So to create an anchor at one specific crossing out of many, direct-select the two crossing
segments (or an anchor of each) and run `insert`: only crossings between participating segments
are made. If the parts you selected don't cross anything, the dialog says so explicitly instead
of silently doing nothing.

---

### Tab: insert

Finds every point where two **participating** segments cross (line–line, line–curve, curve–curve;
between objects, between subpaths, and a subpath crossing itself) and inserts an anchor point
exactly at each crossing. Curves are split with de Casteljau, so the drawn shape does not
change — only new, editable anchors appear. The live preview shows the new anchors before OK.

### Tab: mark

Instead of editing the paths, drops a small **magenta** marker dot on top of each crossing
(diameter: the `marker` parameter, 1–50 pt, default 5). Magenta contrasts with the usual black
strokes and with the app's blue anchor and selection chrome, so markers are easy to tell apart
from ordinary control points. One undo step removes them all. Honours the direct selection the
same way as `insert`, so you can preview exactly the crossings that `insert` would use.

### Tab: snap handles

Rotates Bézier handles onto the nearest multiple of the `angle` parameter (45° by default: 0°,
45°, 90°, 135°, 180°, …), keeping each handle's length. Smooth anchors stay smooth: their two
handles stay collinear. With anchor points direct-selected, **only those anchors'** handles are
snapped; otherwise the whole selection's.

**Interactive Shift-snapping:** dragging a handle **while holding Shift already snaps to 45°
steps natively** in VectorCraft's Direct Selection, Anchor Point and Pen tools (the same
`constrain_angle` logic that constrains rotated lines, honouring the General › Constrain Angle
preference). A plug-in cannot change pointer behaviour; this mode is the batch companion.

### Tab: smooth flow

Re-fits the handles of the selected curves so the path flows evenly — no kinks, no bulges from
mismatched handles, giving **G1 (tangent) continuity** through every interior anchor. The anchors
themselves don't move, so the path stays where you drew it.

- **Straight segments stay straight.** In a path mixing lines and curves, no handles are pushed
  into the handle-less segments, so lines are never bent into jagged near-lines. (A path with no
  handles at all is a polyline being smoothed, so there everything participates.)
- **Curves merge into lines.** An anchor joining a straight segment and a curve takes the line's
  own direction as its tangent — horizontal, vertical or diagonal alike — so the curve continues
  the line optically smoothly instead of kinking at the join.
- Elsewhere the tangent comes from the neighbours (a Catmull-Rom tangent), and the handle
  lengths are ≈ 0.39 of each chord — the ratio of a circular arc's cubic, noticeably fuller
  than the usual ⅓, so the curvature spreads out and the result reads smooth to the eye.

**Following a line:** select a straight line (a plain two-anchor segment) *together with* the
curves, and the curves' tangents are turned to follow the line's direction — the way to make a
run of Bézier segments ride evenly along a drawn direction. The line itself is never modified.

`flow` (0–100 %, default 100) blends between the original handles (0) and the fully smoothed
ones (100), with live preview. With anchors direct-selected, only those anchors are re-fitted.

### Tab: merge anchors

Merges direct-selected anchor points into one. **Requires** two or more anchors picked with the
Direct Selection tool — with anything less the dialog explains exactly what to select, so you
always get feedback instead of a silent no-op.

- Consecutive selected anchors on the same subpath collapse into a single anchor (the segments
  between them disappear); selections on different subpaths or objects each move their anchor to
  the same shared point, closing gaps between endpoints.
- The surviving anchor's position and handles come from the `merge` method:

| `merge` method | New position | New handles |
|---|---|---|
| `average` | Centroid (mean) of the selected anchors | Mean of the members' handle vectors |
| `geometric center` | Centre of the selected anchors' bounding box | Mean of the members' handle vectors |
| `dominant direction` | Centroid | Aligned with the longest selected handle, lengths kept — the merged anchor becomes `Smooth` |

The differences between the merged points' positions and handles are averaged (or, for
`dominant direction`, projected onto the strongest handle's axis), so the merged point gets a
stable position *and* a reasonable curve behaviour; neighbouring anchors are untouched, which
keeps the rest of the curve's shape.

## How crossings are found

Each path segment is treated as a cubic Bézier (a straight segment is the degenerate cubic on its
chord). For every pair of segments whose bounding boxes overlap, both are recursively halved with
de Casteljau until each half is flat (its control points within 10⁻⁴ pt of its chord) or a depth
of 24 is reached; the flat halves are intersected as line segments and the parameters mapped back
to the original curves. Hits closer than 10⁻³ pt are merged, and a hit that lands on an existing
anchor (shared endpoints, segments meeting at a point) is skipped.

## Limitations and known issues

- **No tool plug-ins in ABI v1**: a plug-in cannot receive pointer events, draw canvas overlays or
  alter drag behaviour. Hover-highlighting a crossing before clicking is therefore approximated by
  the dialog's live preview (`mark` mode), and interactive snapping is left to the app's built-in
  Shift constraint.
- Collinear overlapping segments (lines lying on top of each other) have no single crossing point
  and are ignored.
- Tangential touches may be reported as a crossing when the curves actually meet within the
  flatness tolerance.
- Only paths and compound paths are processed (the host never hands a filter type, images or
  symbols). Caps: 20,000 segments and 4,096 crossings per run.
- Markers are plain filled paths: they don't follow the geometry afterwards and are meant to be
  deleted (or undone) when you're done looking.
- `merge anchors` keeps the neighbours' handles as they are; after a large merge the adjoining
  segments can change shape near the merged point (run `smooth flow` after it to even them out).
- `smooth flow`'s guide must be a plain two-anchor straight line; polylines and curves are
  smoothed, not used as guides.

## Development

Native unit tests (geometry, insertion, snapping, selection handling, smoothing, merging):
`cargo test` in this folder. The crate is not a member of the VectorCraft workspace. `plan.md`
records the design plan, assumptions and possible follow-ups for other developers or agents
extending this plug-in.
