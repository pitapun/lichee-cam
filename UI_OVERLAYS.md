# UI Overlays

The web UI has two separate video tabs:

- Live tab
- Zone tab

They intentionally show different overlays.

## Live Tab

Live tab should show:

- video,
- detection bounding boxes.

Live tab should not show:

- filter-zone polygons,
- detection-zone crop boxes.

Reason: filter zones and crop boxes cover the image and make normal live viewing
hard.

## Zone Tab

Zone tab should show:

- filter-zone polygons,
- detection-zone crop boxes,
- drag handles for detection zones,
- active follower zone in orange.

Zone tab is the editor. It is the only place where zones should be visible by
default.

## Filter Zones

Filter zones are polygon rules stored in `cfg.zones`.

They are drawn only on Zone tab.

Drawing rules:

- low alpha fill via `hexToRgba(col, 0.04)`,
- solid stroke,
- disabled zones do not use a black mask,
- disabled zones get a small `disabled` label.

Do not use `#rgb` plus alpha suffix such as `col + "10"` for canvas fills. Some
browser/canvas combinations render it incorrectly.

## Detection Zones

Detection zones are square YOLO crop regions stored in `cfg.detection_zones`.

They are drawn only on Zone tab.

Drawing rules:

- normal zone: yellow,
- active follower zone: orange,
- dragging zone: brighter yellow,
- low alpha fill,
- solid stroke.

Dashed/high-alpha detection zones were removed because they looked like they
were flashing.

## Flicker Prevention

`syncCvs()` must not assign `canvas.width` or `canvas.height` every animation
frame.

Changing canvas dimensions clears the canvas. If dimensions are reassigned on
every requestAnimationFrame, overlays appear to flicker.

Correct behavior:

- compute rendered video rect every frame,
- only update `cvs.width` when the numeric width changes,
- only update `cvs.height` when the numeric height changes,
- only update style fields when the string value changes.

## Detection Bounding Boxes

Detection boxes are retained briefly by `latestBoxes`.

Purpose:

- avoid one-frame flash behavior,
- keep track boxes visually stable while detector output arrives asynchronously.

