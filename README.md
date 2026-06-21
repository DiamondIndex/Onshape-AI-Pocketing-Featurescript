# Onshape AI Pocketing FeatureScript

A custom Onshape **FeatureScript** that automatically lays out triangulated
lightening ribs ("pocketing") on FTC / FRC plates. Click a plate face and it
finds the holes, connects their centers with an optimized rib network, and
draws the result as a sketch you can extrude-cut.

This automates the tedious manual step of drawing rib lines between stress
points for weight savings while keeping the structure strong and visually
balanced.

## Web app (algorithm playground)

A browser-based playground for developing and tuning the pocketing algorithm with
an instant render loop (no Onshape deploy cycle), then porting it to this
FeatureScript, lives in its own section: [`webapp/`](webapp/) — see
[`webapp/README.md`](webapp/README.md).

## How it works

1. **Select a planar face.** The feature reads every through-hole on it
   (detected as full circular edges lying in the face plane).
2. **Optional edge anchoring.** Points are sampled along the plate's outer
   edges so ribs run out to the perimeter, like good hand-pocketed plates.
3. **Delaunay triangulation** (Bowyer–Watson) of all those points produces the
   triangular web — this is what gives the clean, evenly-sized triangular
   pockets seen in well-done FTC/FRC plates.
4. Triangle edges are drawn as **rib centerlines** in a sketch.
5. **Optional pocket profiles**: hole bosses (a material ring around each hole)
   plus rib edges offset by the rib width. These overlapping curves split the
   face into closed regions — select the open triangle interiors with
   **Extrude → Remove** to cut the pockets and keep the ribs.

## Installation

1. In Onshape, create a new **Feature Studio** in any document (or your
   public Custom Features document).
2. Onshape auto-fills the first two lines (`FeatureScript <n>;` and the
   `import` with `version "<n>.0"`). Paste the contents of
   [`autoPocket.fs`](autoPocket.fs) below those two lines, **or** paste the
   whole file and change `2588` to the version number Onshape generated.
3. **Commit** the Feature Studio.
4. In a Part Studio, **+ Add custom features** → pick **Auto Pocket**.

## Parameters

| Parameter | Purpose |
|---|---|
| **Plate face** | The planar face to pocket. |
| **Ignore holes smaller than (diameter)** | Skip small pilot/fastener holes so they don't anchor ribs. `0` = use all holes. |
| **Anchor ribs to plate edges** | Sample the outer edges so ribs reach the perimeter. |
| **Edge anchor points per side** | More samples = ribs hug the boundary more closely. |
| **Limit rib length** | Prune overly long ribs (helps on concave/L-shaped plates). `0` = no limit. |
| **Draw reference circles at holes** | Adds construction-style circles at each hole center. |
| **Also generate pocket profiles to cut** | Emits a second sketch with bosses + offset rib edges ready for Extrude → Remove. |
| **Rib width** | Width of the kept ribs (only when generating pockets). |
| **Material ring around holes (boss)** | Extra material kept around each hole. |

## Current limitations (v1)

- **Hole detection** assumes holes are full circular edges in the face plane.
  Slotted/non-circular holes and fully round plates aren't classified yet.
- **Concave plates**: plain Delaunay spans the convex hull, so concave regions
  (L-shapes, trapezoids) can get ribs crossing empty space — use *Limit rib
  length* to prune them. Constrained triangulation is a planned improvement.
- **Symmetry is not yet enforced.** The layout follows hole positions; mirror
  your holes for a symmetric result.
- Pocket profiles are generated for review/region-selection; the actual cut is
  done manually with Extrude → Remove (auto-cut is a planned option).

## Roadmap

- Constrained Delaunay that respects concave boundaries
- Enforced symmetry detection / mirroring
- One-click auto extrude-cut of pocket regions
- Rib-width-aware fillets at junctions
