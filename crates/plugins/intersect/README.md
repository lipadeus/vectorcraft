# Intersection Anchors — a VectorCraft plug-in

A WebAssembly object-filter plug-in (plug-in ABI v1, see [`docs/plugins.md`](../../../docs/plugins.md))
for working with crossing lines and Bézier curves:

- **insert** — finds every point where two segments of the selection cross (line–line, line–curve,
  curve–curve; between objects, between subpaths, and a subpath crossing itself) and inserts an
  anchor point exactly at each crossing. Curves are split with de Casteljau, so the drawn shape
  does not change — only new, editable anchors appear.
- **mark** — instead of editing the paths, drops a small **magenta** marker dot on top of each
  crossing. Magenta contrasts with the usual black strokes and with the app's blue anchor and
  selection chrome, so markers are easy to tell apart from ordinary control points. One undo step
  removes them all.
- **snap handles** — rotates every Bézier handle of the selection onto the nearest multiple of
  the `angle` parameter (45° by default: 0°, 45°, 90°, 135°, 180°, …), keeping each handle's
  length. Smooth anchors stay smooth: their two handles stay collinear.

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

Select the paths you care about and run **Object › Plug-ins › Intersection Anchors**. The dialog
shows the parameters with a **live preview**, so with `mark` (or `insert`) you see every crossing
highlighted on the canvas *before* you click OK — that preview is how "show the crossing before
the anchor is created" works within the plug-in API.

| Parameter | Meaning |
|---|---|
| `mode` | `insert` (anchors at crossings), `mark` (magenta markers only), `snap handles` (quantise handle angles). |
| `marker` | Marker diameter in pt (`mark` mode), 1–50, default 5. |
| `angle` | Angle step in degrees (`snap handles` mode), 1–180, default 45. |

Everything runs as one undo step.

### Interactive Shift-snapping

Dragging a handle **while holding Shift already snaps to 45° steps natively** in VectorCraft's
Direct Selection, Anchor Point and Pen tools (the same `constrain_angle` logic that constrains
rotated lines, honouring the General › Constrain Angle preference). This plug-in does not — and
cannot — change pointer behaviour; its `snap handles` mode is the batch companion: it cleans up
all the selection's handles onto the angle grid in one go.

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

## Development

Native unit tests (geometry, insertion, snapping): `cargo test` in this folder. The crate is not a
member of the VectorCraft workspace. `plan.md` records the design plan, assumptions and possible
follow-ups for other developers or agents extending this plug-in.
