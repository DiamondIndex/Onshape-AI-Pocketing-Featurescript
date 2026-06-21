# Auto Pocket — Web App / Algorithm Playground

A browser-based playground for the plate-pocketing algorithm. It runs the **same
logic** that the Onshape FeatureScript uses, but with an instant render/tune loop
(no Onshape deploy cycle). Once a design matches, the algorithm is ported to
FeatureScript (`../autoPocket.fs`).

The goal: load a plate (DXF), tune the triangulated rib/pocket pattern live
against a reference image, then generate the equivalent FeatureScript.

## Files

| File | Role | Ported to FeatureScript? |
|---|---|---|
| `pocket-core.js` | The pocketing algorithm: hole filtering, declustering, optional lattice gap-fill, Delaunay (Bowyer–Watson), in-material triangle keep, inset + filleted pocket loops. Written in a deliberately FS-portable style (plain arrays/numbers/maps, deterministic, no libraries). | **Yes** — this is the part that becomes FeatureScript. |
| `dxf.js` | Minimal DXF reader → `{ outline, inners, holes }`. | No (web input only). |
| `index.html` | UI: canvas render, live parameter sliders, reference-image overlay, DXF + image upload, sample plate, parameter export. | No (web UI only). |

## Run it

It's static files, but `file://` blocks some loads, so serve the folder:

```bash
# from the repo root
python -m http.server 8765 --directory webapp
# then open http://localhost:8765
```

(There's also a `.claude/launch.json` config named `pocket` for the Claude
preview tooling.)

## Parameters (mirror the FeatureScript inputs)

| Control | Meaning |
|---|---|
| Triangle size | Target pocket/triangle scale (mm). Also drives `maxEdge`. |
| Min hole Ø | Ignore holes smaller than this diameter (skip pilot holes). |
| Decluster | Drop holes closer than this to an already-kept (larger) hole, so tight bolt clusters don't shatter the mesh. |
| Max edge ×s | Longest rib allowed = factor × triangle size (spans sparse regions; ribs over open holes are dropped by the in-material test). |
| Rib width | Wall thickness left between pockets (mm). |
| Corner fillet | Pocket corner radius (mm). |
| Edge margin | Keep-out from the plate boundary / cutouts (mm). |
| Fill sparse gaps | Optional background lattice (Steiner points) for sparse areas. **Off** = pure hole-to-hole (vertices on holes, like a hand-pocketed plate). |

## Workflow

1. **Load a plate** — upload a DXF (outline polyline + circles), or use the sample.
2. **Overlay the reference** image and align (opacity / scale / X / Y).
3. **Tune** the sliders until the rib/pocket pattern matches.
4. **Generate FeatureScript params** (and, ultimately, port `pocket-core.js` into
   `../autoPocket.fs`).

## FeatureScript portability notes

`pocket-core.js` avoids anything that doesn't exist in FeatureScript: no external
libraries, no randomness, no classes/closures; only arrays, numbers and
maps-as-objects, with simple `for`/`while` loops and modest (≈O(n²)) complexity to
respect FeatureScript's regen time limits. Each function has a direct FS analogue.
