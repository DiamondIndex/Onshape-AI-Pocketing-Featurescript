FeatureScript 2985;
import(path : "onshape/std/geometry.fs", version : "2985.0");

// ============================================================================
// AUTO POCKET  —  triangulated lightening for FTC / FRC plates
//
// This is a direct port of the web playground algorithm (webapp/pocket-core.js).
// Same pipeline, so both produce the same result:
//   1. read holes (centre + radius) and the plate outline from a planar face
//   2. MERGE very-close holes into one vertex at their centroid (+ rib the pair)
//   3. optionally fill sparse regions with an even lattice (Steiner points)
//   4. sample the outer rim so ribs reach the perimeter
//   5. Delaunay triangulate (Bowyer-Watson); keep in-material triangles
//   6. ribs = kept triangle edges + cluster ribs
//   7. OUTPUT: a SKETCH only -- the rib centrelines (same as the web-app ribs)
//      plus a reference circle (boss) around every hole. No solid cut is done;
//      use the sketch to drive your own extrude / Lighten.
//
// Coordinates are handled in millimetres internally (plain numbers).
// ============================================================================

// ----- parameter bounds -----------------------------------------------------

export const TRI_BOUNDS = { (meter) : [0.005, 0.030, 0.5],
        (centimeter) : 3.0, (millimeter) : 30.0, (inch) : 1.2 } as LengthBoundSpec;

export const MERGE_BOUNDS = { (meter) : [0.0, 0.012, 0.2],
        (centimeter) : 1.2, (millimeter) : 12.0, (inch) : 0.5 } as LengthBoundSpec;

export const RIB_BOUNDS = { (meter) : [1e-4, 0.006, 0.1],
        (centimeter) : 0.6, (millimeter) : 6.0, (inch) : 0.236 } as LengthBoundSpec;

export const FILLET_BOUNDS = { (meter) : [0.0, 0.002, 0.05],
        (centimeter) : 0.2, (millimeter) : 2.0, (inch) : 0.0787 } as LengthBoundSpec;

export const HOLEMAT_BOUNDS = { (meter) : [0.0, 0.003, 0.05],
        (centimeter) : 0.3, (millimeter) : 3.0, (inch) : 0.12 } as LengthBoundSpec;

export const MARGIN_BOUNDS = { (meter) : [0.0, 0.006, 0.2],
        (centimeter) : 0.6, (millimeter) : 6.0, (inch) : 0.25 } as LengthBoundSpec;

export const MINHOLE_BOUNDS = { (meter) : [0.0, 0.0, 5.0],
        (centimeter) : 0.0, (millimeter) : 0.0, (inch) : 0.0 } as LengthBoundSpec;

export const MAXEDGE_BOUNDS = { (unitless) : [1.4, 2.8, 6.0] } as RealBoundSpec;

export const ANGLE_BOUNDS = { (degree) : [0.0, 25.0, 80.0],
        (radian) : 0.4363323129985824 } as AngleBoundSpec;

export const POCKET_BOUNDS = { (meter) : [0.0, 0.004, 0.05],
        (centimeter) : 0.4, (millimeter) : 4.0, (inch) : 0.16 } as LengthBoundSpec;

export const VERT_BOUNDS = { (degree) : [0.0, 15.0, 45.0],
        (radian) : 0.2617993877991494 } as AngleBoundSpec;

export const REFINE_BOUNDS = { (unitless) : [1.0, 1.5, 4.0] } as RealBoundSpec;

// ----- feature --------------------------------------------------------------

annotation { "Feature Type Name" : "Auto Pocket" }
export const autoPocket = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Plate face",
                     "Filter" : EntityType.FACE && GeometryType.PLANE,
                     "MaxNumberOfPicks" : 1 }
        definition.face is Query;

        annotation { "Name" : "Triangle size" }
        isLength(definition.triangleSize, TRI_BOUNDS);

        annotation { "Name" : "Merge holes closer than" }
        isLength(definition.mergeDist, MERGE_BOUNDS);

        annotation { "Name" : "Material around holes" }
        isLength(definition.holeMargin, HOLEMAT_BOUNDS);

        annotation { "Name" : "Margin from plate edge" }
        isLength(definition.edgeMargin, MARGIN_BOUNDS);

        annotation { "Name" : "Ignore holes smaller than (diameter)" }
        isLength(definition.minHoleDiameter, MINHOLE_BOUNDS);

        annotation { "Name" : "Max rib length (x triangle size)" }
        isReal(definition.maxEdgeFactor, MAXEDGE_BOUNDS);

        annotation { "Name" : "Even spacing (fill sparse gaps)" }
        definition.evenSpacing is boolean;

        annotation { "Name" : "Minimum angle between ribs" }
        isAngle(definition.minRibAngle, ANGLE_BOUNDS);

        annotation { "Name" : "Remove pockets smaller than" }
        isLength(definition.minPocket, POCKET_BOUNDS);

        annotation { "Name" : "Avoid ribs within this angle of vertical" }
        isAngle(definition.vertTol, VERT_BOUNDS);

        annotation { "Name" : "Subdivide pockets larger than (x triangle size)" }
        isReal(definition.maxPocketFactor, REFINE_BOUNDS);

        annotation { "Name" : "Only triangular pockets" }
        definition.onlyTriangles is boolean;

        annotation { "Name" : "Equilateral triangles only" }
        definition.equilateral is boolean;

        annotation { "Name" : "Voronoi cells (looks)" }
        definition.voronoi is boolean;
    }
    {
        if (size(evaluateQuery(context, definition.face)) != 1)
            throw regenError("Select exactly one planar face.", ["face"]);

        const plane = evPlane(context, { "face" : definition.face });
        const s = definition.triangleSize / millimeter;
        const mergeDist = definition.mergeDist / millimeter;
        const marginMm = definition.edgeMargin / millimeter;
        const minR = (definition.minHoleDiameter / 2) / millimeter;
        const maxEdge = definition.maxEdgeFactor * s;
        const sinVert = sin(definition.vertTol);   // a rib is "vertical" if |dx|/len < sinVert

        // ----- 1. read holes + outline ------------------------------------
        var holePts = [];
        var holeRadii = [];
        var polylines = [];
        for (var edge in evaluateQuery(context, qLoopEdges(definition.face)))
        {
            const cdef = try silent(evCurveDefinition(context, { "edge" : edge }));
            var isHole = false;
            if (cdef != undefined && cdef is Circle)
            {
                const circ = 2 * PI * cdef.radius;
                const len = try silent(evLength(context, { "entities" : edge }));
                const full = (len != undefined) && (abs(len - circ) < 0.02 * circ);
                const c = cdef.coordSystem.origin;
                const onPlane = abs(dot(c - plane.origin, plane.normal)) < 1e-5 * meter;
                const axial = abs(dot(normalize(cdef.coordSystem.zAxis), plane.normal)) > 0.99;
                if (full && onPlane && axial && (cdef.radius / millimeter) >= minR)
                {
                    const p2d = worldToPlane(plane, c);
                    holePts = append(holePts, [p2d[0] / millimeter, p2d[1] / millimeter]);
                    holeRadii = append(holeRadii, cdef.radius / millimeter);
                    isHole = true;
                }
            }
            if (!isHole)
            {
                var pl = [];
                for (var i = 0; i < 6; i += 1)
                {
                    const ln = try silent(evEdgeTangentLine(context, { "edge" : edge, "parameter" : i / 5 }));
                    if (ln != undefined)
                    {
                        const p2d = worldToPlane(plane, ln.origin);
                        pl = append(pl, [p2d[0] / millimeter, p2d[1] / millimeter]);
                    }
                }
                if (size(pl) >= 2)
                    polylines = append(polylines, pl);
            }
        }

        const loops = buildLoops(polylines);
        if (size(loops) == 0)
            throw regenError("Could not read the plate outline.", ["face"]);
        var outerIdx = 0; var maxA = -1;
        for (var i = 0; i < size(loops); i += 1)
        {
            const ar = polygonArea(loops[i]);
            if (ar > maxA) { maxA = ar; outerIdx = i; }
        }
        const poly = loops[outerIdx];
        var inners = [];
        for (var i = 0; i < size(loops); i += 1)
            if (i != outerIdx && polygonArea(loops[i]) > (0.3 * s) * (0.3 * s))
                inners = append(inners, loops[i]);

        // ----- 2. dedupe + filter holes -----------------------------------
        var holes = [];
        var hRad = [];
        for (var i = 0; i < size(holePts); i += 1)
        {
            var dup = false;
            for (var j = 0; j < size(holes); j += 1)
                if (pointDistance(holePts[i], holes[j]) < 1.0) { dup = true; break; }
            if (!dup) { holes = append(holes, holePts[i]); hRad = append(hRad, holeRadii[i]); }
        }

        // ===== VORONOI MODE (looks) =======================================
        // Ribs run along Voronoi cell walls (dual of the Delaunay): connect the
        // circumcenters of triangles sharing a Delaunay edge, then clip to the
        // plate material. Produces the organic cellular lightening look.
        if (definition.voronoi)
        {
            var bMinX = poly[0][0]; var bMaxX = poly[0][0];
            var bMinY = poly[0][1]; var bMaxY = poly[0][1];
            for (var pp in poly)
            {
                bMinX = min(bMinX, pp[0]); bMaxX = max(bMaxX, pp[0]);
                bMinY = min(bMinY, pp[1]); bMaxY = max(bMaxY, pp[1]);
            }
            const jit = 0.32;
            const gcx = (bMinX + bMaxX) / 2; const gcy = (bMinY + bMaxY) / 2;

            // sites = square lattice ROTATED 45 deg (walls run ~45/135 deg, off
            // horizontal/vertical) + hash jitter. Holes are NOT sites, so they
            // land on cell walls (supported) instead of floating in cell centres.
            var sites = [];
            const diag = sqrt((bMaxX - bMinX) * (bMaxX - bMinX) + (bMaxY - bMinY) * (bMaxY - bMinY));
            const NN = ceil(diag / s) + 2;
            var hidx = 0;
            for (var gi = -NN; gi <= NN; gi += 1)
                for (var gj = -NN; gj <= NN; gj += 1)
                {
                    hidx += 1;
                    const lx = gi * s; const ly = gj * s;
                    const rx = (lx - ly) * 0.7071068; const ry = (lx + ly) * 0.7071068;
                    const e1 = sin((hidx * 12.9898 + 7.13) * radian) * 43758.5453;
                    const e2 = sin(((hidx + 9173) * 12.9898 + 7.13) * radian) * 43758.5453;
                    const j1 = e1 - floor(e1); const j2 = e2 - floor(e2);
                    const q = [gcx + rx + (j1 - 0.5) * 2 * s * jit, gcy + ry + (j2 - 0.5) * 2 * s * jit];
                    if (q[0] < bMinX || q[0] > bMaxX || q[1] < bMinY || q[1] > bMaxY) continue;
                    if (inMaterial(poly, inners, q) && distToPolygon(poly, q) >= marginMm
                            && distToInners(inners, q) >= marginMm)
                    {
                        var clear = true;
                        for (var sp in sites) if (pointDistance(q, sp) < 0.7 * s) { clear = false; break; }
                        if (clear)
                            for (var hh = 0; hh < size(holes); hh += 1)
                                if (pointDistance(q, holes[hh]) < 0.45 * s) { clear = false; break; }
                        if (clear) sites = append(sites, q);
                    }
                }
            const nReal = size(sites);

            // guard ring far outside so real cells are all finite
            const grad = max(bMaxX - bMinX, bMaxY - bMinY) * 2 + 200;
            for (var g = 0; g < 16; g += 1)
            {
                const ga = (g / 16 * 2 * PI) * radian;
                sites = append(sites, [gcx + cos(ga) * grad, gcy + sin(ga) * grad]);
            }

            // Delaunay -> circumcenters -> wall per shared Delaunay edge
            const vtris = bowyerWatson(sites);
            var cc = [];
            for (var t = 0; t < size(vtris); t += 1)
                cc = append(cc, apCircumcenter(sites[vtris[t][0]], sites[vtris[t][1]], sites[vtris[t][2]]));
            var emap = {};
            for (var t = 0; t < size(vtris); t += 1)
            {
                const tr = vtris[t];
                const te = [[tr[0], tr[1]], [tr[1], tr[2]], [tr[2], tr[0]]];
                for (var e = 0; e < 3; e += 1)
                {
                    const a = min(te[e][0], te[e][1]); const b = max(te[e][0], te[e][1]);
                    const k = a ~ "_" ~ b;
                    if (emap[k] == undefined) emap[k] = [];
                    emap[k] = append(emap[k], t);
                }
            }

            // clip each wall to material (outline / inners / hole keep-outs)
            var ribs = [];
            for (var key in keys(emap))
            {
                const arr = emap[key];
                if (size(arr) != 2) continue;
                const w0 = cc[arr[0]]; const w1 = cc[arr[1]];
                var ts = [0.0, 1.0];
                for (var i = 0; i < size(poly); i += 1)
                {
                    const t = apSegT(w0, w1, poly[i], poly[(i + 1) % size(poly)]);
                    if (t >= 0) ts = append(ts, t);
                }
                for (var ki = 0; ki < size(inners); ki += 1)
                {
                    const inn = inners[ki];
                    for (var i = 0; i < size(inn); i += 1)
                    {
                        const t = apSegT(w0, w1, inn[i], inn[(i + 1) % size(inn)]);
                        if (t >= 0) ts = append(ts, t);
                    }
                }
                ts = apSortNums(ts);
                for (var m = 0; m < size(ts) - 1; m += 1)
                {
                    const ta = ts[m]; const tb = ts[m + 1];
                    if (tb - ta < 1e-4) continue;
                    const tm = (ta + tb) / 2;
                    const pm = [w0[0] + (w1[0] - w0[0]) * tm, w0[1] + (w1[1] - w0[1]) * tm];
                    if (inMaterial(poly, inners, pm))
                    {
                        const pa = [w0[0] + (w1[0] - w0[0]) * ta, w0[1] + (w1[1] - w0[1]) * ta];
                        const pb = [w0[0] + (w1[0] - w0[0]) * tb, w0[1] + (w1[1] - w0[1]) * tb];
                        ribs = append(ribs, [pa, pb]);
                    }
                }
            }

            // one connected network (Part Lightening needs a single part)
            ribs = apConnectSegs(ribs);

            // support every hole: if no rib already runs through it, add a short
            // strut to the nearest wall (the part inside the hole is cut away).
            for (var hh = 0; hh < size(holes); hh += 1)
            {
                const hc = holes[hh];
                var best = 1e18; var bp = hc; var found = false;
                for (var i = 0; i < size(ribs); i += 1)
                {
                    const pt = apClosestOnSeg(hc, ribs[i][0], ribs[i][1]);
                    const dd = pointDistance(hc, pt);
                    if (dd < best) { best = dd; bp = pt; found = true; }
                }
                if (found && best > hRad[hh] + 0.5) ribs = append(ribs, [hc, bp]);
            }

            const vSketch = newSketchOnPlane(context, id + "ribs", { "sketchPlane" : plane });
            var ri = 0;
            for (var rib in ribs)
            {
                const a = rib[0]; const b = rib[1];
                if (pointDistance(a, b) < 1e-6) continue;
                skLineSegment(vSketch, "rib" ~ ri, {
                            "start" : vector(a[0] * millimeter, a[1] * millimeter),
                            "end"   : vector(b[0] * millimeter, b[1] * millimeter) });
                ri += 1;
            }
            skSolve(vSketch);
            reportFeatureInfo(context, id, "Voronoi: " ~ nReal ~ " cells, " ~ ri ~ " rib segments.");
            return;
        }

        // ----- 3. merge very-close holes (union-find) ---------------------
        var parent = [];
        for (var i = 0; i < size(holes); i += 1) parent = append(parent, i);
        if (mergeDist > 0)
        {
            for (var a = 0; a < size(holes); a += 1)
                for (var b = a + 1; b < size(holes); b += 1)
                    if (pointDistance(holes[a], holes[b]) < mergeDist)
                    {
                        const ra = ufFind(parent, a);
                        const rb = ufFind(parent, b);
                        parent[ra] = rb;
                    }
        }
        var groupKeys = [];
        var groupMembers = {};
        for (var i = 0; i < size(holes); i += 1)
        {
            const r = ufFind(parent, i) ~ "";
            if (groupMembers[r] == undefined) { groupMembers[r] = []; groupKeys = append(groupKeys, r); }
            groupMembers[r] = append(groupMembers[r], i);
        }

        var nodePts = [];           // triangulation vertices
        var kept = [];              // same, for lattice clearance test
        var clusterRibs = [];       // [ [memberPt, centroid], ... ]
        for (var gi = 0; gi < size(groupKeys); gi += 1)
        {
            const mem = groupMembers[groupKeys[gi]];
            var cx = 0; var cy = 0;
            for (var m = 0; m < size(mem); m += 1) { cx += holes[mem[m]][0]; cy += holes[mem[m]][1]; }
            const cpt = [cx / size(mem), cy / size(mem)];
            if (!inMaterial(poly, inners, cpt)) continue;
            nodePts = append(nodePts, cpt);
            kept = append(kept, cpt);
            if (size(mem) >= 2)
                for (var m = 0; m < size(mem); m += 1)
                    clusterRibs = append(clusterRibs, [holes[mem[m]], cpt]);
        }

        // "Equilateral triangles only" ignores the (irregularly placed) holes as
        // vertices and triangulates a pure regular lattice instead.
        if (definition.equilateral)
        {
            nodePts = []; kept = []; clusterRibs = [];
        }

        // ----- 4. even-spacing lattice fill (optional) --------------------
        if (definition.evenSpacing || definition.equilateral)
        {
            const rowH = s * sqrt(3) / 2;
            var bMinX = poly[0][0]; var bMaxX = poly[0][0];
            var bMinY = poly[0][1]; var bMaxY = poly[0][1];
            for (var p in poly)
            {
                bMinX = min(bMinX, p[0]); bMaxX = max(bMaxX, p[0]);
                bMinY = min(bMinY, p[1]); bMaxY = max(bMaxY, p[1]);
            }
            var row = 0; var y = bMinY;
            while (y <= bMaxY)
            {
                var x = bMinX + ((row % 2 == 0) ? 0 : s / 2);
                while (x <= bMaxX)
                {
                    const q = [x, y];
                    if (inMaterial(poly, inners, q) && distToPolygon(poly, q) >= marginMm
                            && distToInners(inners, q) >= marginMm)
                    {
                        var clear = true;
                        for (var k in kept) if (pointDistance(q, k) < 0.85 * s) { clear = false; break; }
                        if (clear) { nodePts = append(nodePts, q); kept = append(kept, q); }
                    }
                    x += s;
                }
                y += rowH; row += 1;
            }
        }

        // ----- 5. rim sampling (inset INWARD off the boundary) ------------
        // Rib ends that sit exactly on the plate edge go degenerate when Part
        // Lightening thickens them ("failed to finalize and boolean"), so pull
        // each rim point a little inward (toward the plate centroid) so ribs end
        // inside the material and merge cleanly with the perimeter wall.
        var cenX = 0; var cenY = 0;
        for (var p in poly) { cenX += p[0]; cenY += p[1]; }
        cenX = cenX / size(poly); cenY = cenY / size(poly);
        // Pull rim points only slightly off the edge: enough to avoid the
        // exact-edge boolean degeneracy, but small enough that rib ends still
        // land inside the Part-Lightening perimeter wall and merge with it
        // (a large inset leaves the rib short of the wall -> thin spike pocket).
        const rimInset = max(4.5, min(marginMm, 0.1 * s));
        var rim = [];
        var acc = s;
        for (var i = 0; i < size(poly); i += 1)
        {
            acc += pointDistance(poly[i], poly[(i + 1) % size(poly)]);
            if (acc >= s)
            {
                const rp = poly[i];
                const dx = cenX - rp[0]; const dy = cenY - rp[1];
                const dl = sqrt(dx * dx + dy * dy);
                var ip = rp;
                if (dl > 1e-6)
                {
                    const cand = [rp[0] + dx / dl * rimInset, rp[1] + dy / dl * rimInset];
                    if (inMaterial(poly, inners, cand)) ip = cand;
                }
                rim = append(rim, ip);
                acc = 0;
            }
        }
        rim = dedupePoints(rim, 0.4 * s);

        const rimStart = size(nodePts);
        const rimEnd = rimStart + size(rim);   // pts[rimStart .. rimEnd-1] are rim points
        var pts = concatenateArrays([nodePts, rim]);
        if (size(pts) < 3)
            throw regenError("Plate too small for this triangle size.", ["triangleSize"]);

        // ----- 5b. subdivide large pockets (Steiner refinement) -----------
        // A big pocket left by few ribs reads as one large rounded blob once
        // Part Lightening fillets it, and leaves the plate centre too open. Drop
        // an organic vertex into any oversized triangle and re-triangulate, so it
        // splits into similar-size pockets and the open area fills with diagonal
        // ribs (which the no-vertical rule keeps off-vertical).
        const refThresh = definition.maxPocketFactor * s;
        var rpass = 0;
        while (rpass < 6)
        {
            rpass += 1;
            const tg = bowyerWatson(pts);
            var addPts = [];
            for (var tri in tg)
            {
                const A = pts[tri[0]]; const B = pts[tri[1]]; const C = pts[tri[2]];
                const longest = max(max(pointDistance(A, B), pointDistance(B, C)), pointDistance(C, A));
                if (longest <= refThresh) continue;
                const cc = [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3];
                if (!inMaterial(poly, inners, cc)) continue;
                if (distToPolygon(poly, cc) < marginMm || distToInners(inners, cc) < marginMm) continue;
                var clear = true;
                for (var pp in pts) if (pointDistance(cc, pp) < 0.6 * s) { clear = false; break; }
                if (clear) for (var ap in addPts) if (pointDistance(cc, ap) < 0.6 * s) { clear = false; break; }
                if (clear) addPts = append(addPts, cc);
            }
            if (size(addPts) == 0) break;
            pts = concatenateArrays([pts, addPts]);
        }

        // Removed Delaunay edges are pooled so the network can be reconnected
        // later using only real (non-crossing) edges -- never arbitrary struts.
        // Priority: 0 = acute prune, 1 = vertical, 2 = tiny-pocket, 3 = rim-to-rim
        // (all re-added last, only if needed, so the dropped pockets stay dropped).
        var removed = {};
        var removedPri = {};

        // ----- 6. Delaunay + keep in-material triangle edges --------------
        const triangles = bowyerWatson(pts);
        var edgeIdx = {};                      // "lo_hi" -> [lo, hi]
        for (var tri in triangles)
        {
            const A = pts[tri[0]]; const B = pts[tri[1]]; const C = pts[tri[2]];
            if (pointDistance(A, B) > maxEdge || pointDistance(B, C) > maxEdge
                    || pointDistance(C, A) > maxEdge) continue;
            const mAB = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2];
            const mBC = [(B[0] + C[0]) / 2, (B[1] + C[1]) / 2];
            const mCA = [(C[0] + A[0]) / 2, (C[1] + A[1]) / 2];
            const cen = [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3];
            if (!inMaterial(poly, inners, cen) || !inMaterial(poly, inners, mAB)
                    || !inMaterial(poly, inners, mBC) || !inMaterial(poly, inners, mCA)) continue;
            const e = [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]];
            for (var k = 0; k < 3; k += 1)
            {
                const lo = min(e[k][0], e[k][1]);
                const hi = max(e[k][0], e[k][1]);
                if (!definition.onlyTriangles
                        && lo >= rimStart && lo < rimEnd && hi >= rimStart && hi < rimEnd)
                {   // rim-to-rim rib runs along the boundary and traps thin band
                    // pockets against the wall -- pool it, don't draw it.
                    removed[lo ~ "_" ~ hi] = [lo, hi]; removedPri[lo ~ "_" ~ hi] = 3;
                    continue;
                }
                edgeIdx[lo ~ "_" ~ hi] = [lo, hi];
            }
        }

        // ----- 6-merge: dissolve tiny pockets ------------------------------
        // A pocket whose incircle is smaller than this is a sliver; drop the
        // shortest rib bounding it so it merges into its neighbour (the user's
        // "remove the strut that makes the pocket smaller than it should be").
        const minPocketR = definition.minPocket / millimeter;
        if (minPocketR > 0 && !definition.onlyTriangles)
        {
            var removeKeys = {};
            for (var tri in triangles)
            {
                const A = pts[tri[0]]; const B = pts[tri[1]]; const C = pts[tri[2]];
                const cen = [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3];
                if (!inMaterial(poly, inners, cen)) continue;
                const la = pointDistance(B, C); const lb = pointDistance(C, A); const lc = pointDistance(A, B);
                const per = la + lb + lc;
                if (per < 1e-6) continue;
                const area2 = abs((B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]));
                if (area2 / per >= minPocketR) continue;   // incircle radius = area / semiperimeter
                var p0 = tri[0]; var p1 = tri[1]; var sl = lc;
                if (la < sl) { p0 = tri[1]; p1 = tri[2]; sl = la; }
                if (lb < sl) { p0 = tri[2]; p1 = tri[0]; sl = lb; }
                removeKeys[min(p0, p1) ~ "_" ~ max(p0, p1)] = true;
            }
            var edgeT = {};
            for (var key in keys(edgeIdx))
            {
                if (removeKeys[key] == undefined) edgeT[key] = edgeIdx[key];
                else { removed[key] = edgeIdx[key]; removedPri[key] = 2; }
            }
            edgeIdx = edgeT;
        }

        // ----- 6a. thin out acute / redundant ribs ------------------------
        // The starburst of ribs at dense holes meet at tiny angles; when Part
        // Lightening thickens them they self-collide ("failed to finalize and
        // boolean"). At each vertex keep a well-spread set of ribs (shortest
        // first) and drop any whose direction is within the minimum angle of one
        // already kept.
        const cosMin = cos(definition.minRibAngle);
        if (definition.minRibAngle > 0 * degree && !definition.onlyTriangles)
        {
            var incident = {};
            for (var key in keys(edgeIdx))
            {
                const e = edgeIdx[key];
                const k0 = e[0] ~ ""; const k1 = e[1] ~ "";
                if (incident[k0] == undefined) incident[k0] = [];
                if (incident[k1] == undefined) incident[k1] = [];
                incident[k0] = append(incident[k0], key);
                incident[k1] = append(incident[k1], key);
            }
            var prunedE = {};
            for (var v = 0; v < size(pts); v += 1)
            {
                const vk = v ~ "";
                if (incident[vk] == undefined) continue;
                var live = [];
                for (var ii = 0; ii < size(incident[vk]); ii += 1)
                {
                    const key = incident[vk][ii];
                    if (prunedE[key] != undefined) continue;
                    const e = edgeIdx[key];
                    const other = (e[0] == v) ? e[1] : e[0];
                    const len = pointDistance(pts[v], pts[other]);
                    if (len < 1e-6) continue;
                    live = append(live, [key, (pts[other][0] - pts[v][0]) / len,
                                              (pts[other][1] - pts[v][1]) / len, len]);
                }
                live = sortByLenAsc(live);
                var keptU = [];
                for (var li = 0; li < size(live); li += 1)
                {
                    const ux = live[li][1]; const uy = live[li][2];
                    var ok = true;
                    for (var ka = 0; ka < size(keptU); ka += 1)
                        if (ux * keptU[ka][0] + uy * keptU[ka][1] > cosMin) { ok = false; break; }
                    if (ok) keptU = append(keptU, [ux, uy]);
                    else prunedE[live[li][0]] = true;
                }
            }
            var edge2 = {};
            for (var key in keys(edgeIdx))
            {
                if (prunedE[key] == undefined) edge2[key] = edgeIdx[key];
                else { removed[key] = edgeIdx[key]; removedPri[key] = 0; }
            }
            edgeIdx = edge2;
        }

        // ----- 6a2. drop (almost) vertical ribs ---------------------------
        // No vertical / near-vertical lines in the pocketing.
        if (sinVert > 0 && !definition.onlyTriangles)
        {
            var edgeV = {};
            for (var key in keys(edgeIdx))
            {
                const e = edgeIdx[key];
                const dx = pts[e[1]][0] - pts[e[0]][0];
                const dy = pts[e[1]][1] - pts[e[0]][1];
                const len = sqrt(dx * dx + dy * dy);
                if (len > 1e-6 && abs(dx) / len < sinVert)
                { removed[key] = e; removedPri[key] = 1; continue; }   // pool near-vertical
                edgeV[key] = e;
            }
            edgeIdx = edgeV;
        }

        // ----- 6b. reconnect into ONE part with real edges only -----------
        // Part Lightening fails ("results in more than one part") if the ribs
        // form disconnected islands. Reconnect ONLY by re-adding pooled Delaunay
        // edges (which never cross other ribs and always meet at a point) --
        // acute first, vertical next, tiny-pocket last. This keeps the whole
        // network point-to-point with no rib-on-rib T-junctions.
        var par2 = [];
        for (var i = 0; i < size(pts); i += 1) par2 = append(par2, i);
        var inc = [];
        for (var i = 0; i < size(pts); i += 1) inc = append(inc, false);
        for (var key in keys(edgeIdx))
        {
            const ed = edgeIdx[key];
            inc[ed[0]] = true; inc[ed[1]] = true;
            par2[ufFind(par2, ed[0])] = ufFind(par2, ed[1]);
        }
        var pool = [];
        for (var key in keys(removed))
        {
            const e = removed[key];
            pool = append(pool, [key, removedPri[key], pointDistance(pts[e[0]], pts[e[1]])]);
        }
        pool = sortPool(pool);
        for (var pi = 0; pi < size(pool); pi += 1)
        {
            const key = pool[pi][0]; const e = removed[key];
            if (ufFind(par2, e[0]) != ufFind(par2, e[1]))
            {
                edgeIdx[key] = e;
                par2[ufFind(par2, e[0])] = ufFind(par2, e[1]);
            }
        }

        var ribs = [];
        for (var key in keys(edgeIdx))
        {
            const ed = edgeIdx[key];
            ribs = append(ribs, [pts[ed[0]], pts[ed[1]]]);
        }
        for (var cr in clusterRibs)
        {
            const dx = cr[1][0] - cr[0][0]; const dy = cr[1][1] - cr[0][1];
            const len = sqrt(dx * dx + dy * dy);
            if (sinVert > 0 && len > 1e-6 && abs(dx) / len < sinVert) continue;   // no vertical cluster ribs
            ribs = append(ribs, cr);
        }

        // ----- 7. draw the rib network as a SKETCH only (no solid cut) ----
        // Exactly the web-app ribs: kept Delaunay edges + cluster ribs, drawn as
        // centrelines, plus a reference circle (boss) around each hole.
        const ribSketch = newSketchOnPlane(context, id + "ribs", { "sketchPlane" : plane });
        var ri = 0;
        for (var rib in ribs)
        {
            const a = rib[0]; const b = rib[1];
            if (pointDistance(a, b) < 1e-6) continue;
            skLineSegment(ribSketch, "rib" ~ ri, {
                        "start" : vector(a[0] * millimeter, a[1] * millimeter),
                        "end"   : vector(b[0] * millimeter, b[1] * millimeter) });
            ri += 1;
        }
        skSolve(ribSketch);

        reportFeatureInfo(context, id, size(ribs) ~ " ribs, " ~ size(holes) ~ " holes, "
                ~ size(clusterRibs) ~ " merge ribs.");
    });

// ============================================================================
// Helpers  (ports of the corresponding webapp/pocket-core.js functions)
// ============================================================================

function pointDistance(a is array, b is array) returns number
{
    const dx = a[0] - b[0]; const dy = a[1] - b[1];
    return sqrt(dx * dx + dy * dy);
}

function polygonArea(poly is array) returns number
{
    var a = 0; const n = size(poly);
    for (var i = 0; i < n; i += 1)
    {
        const j = (i + 1) % n;
        a += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1];
    }
    return abs(a) / 2;
}

function pointInPolygon(poly is array, p is array) returns boolean
{
    var inside = false; const n = size(poly);
    var j = n - 1;
    for (var i = 0; i < n; i += 1)
    {
        const xi = poly[i][0]; const yi = poly[i][1];
        const xj = poly[j][0]; const yj = poly[j][1];
        if (((yi > p[1]) != (yj > p[1])) && (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi))
            inside = !inside;
        j = i;
    }
    return inside;
}

function inMaterial(poly is array, inners is array, p is array) returns boolean
{
    if (!pointInPolygon(poly, p)) return false;
    for (var k = 0; k < size(inners); k += 1)
        if (pointInPolygon(inners[k], p)) return false;
    return true;
}

function distToSegment(p is array, a is array, b is array) returns number
{
    const vx = b[0] - a[0]; const vy = b[1] - a[1];
    const wx = p[0] - a[0]; const wy = p[1] - a[1];
    const c1 = vx * wx + vy * wy;
    if (c1 <= 0) return pointDistance(p, a);
    const c2 = vx * vx + vy * vy;
    if (c2 <= c1) return pointDistance(p, b);
    const t = c1 / c2;
    return pointDistance(p, [a[0] + t * vx, a[1] + t * vy]);
}

function distToPolygon(poly is array, p is array) returns number
{
    var best = 1e18; const n = size(poly);
    for (var i = 0; i < n; i += 1)
    {
        const d = distToSegment(p, poly[i], poly[(i + 1) % n]);
        if (d < best) best = d;
    }
    return best;
}

function distToInners(inners is array, p is array) returns number
{
    var best = 1e18;
    for (var k = 0; k < size(inners); k += 1)
    {
        const d = distToPolygon(inners[k], p);
        if (d < best) best = d;
    }
    return best;
}

function dedupePoints(arr is array, tol is number) returns array
{
    var out = [];
    for (var i = 0; i < size(arr); i += 1)
    {
        var dup = false;
        for (var j = 0; j < size(out); j += 1)
            if (pointDistance(arr[i], out[j]) < tol) { dup = true; break; }
        if (!dup) out = append(out, arr[i]);
    }
    return out;
}

function sortByLenAsc(arr is array) returns array
{
    var a = arr;
    for (var i = 0; i < size(a); i += 1)
    {
        var mi = i;
        for (var j = i + 1; j < size(a); j += 1)
            if (a[j][3] < a[mi][3]) mi = j;
        if (mi != i) { const t = a[i]; a[i] = a[mi]; a[mi] = t; }
    }
    return a;
}

function apCircumcenter(A is array, B is array, C is array) returns array
{
    const d = 2 * (A[0] * (B[1] - C[1]) + B[0] * (C[1] - A[1]) + C[0] * (A[1] - B[1]));
    if (abs(d) < 1e-9)
        return [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3];
    const a2 = A[0] * A[0] + A[1] * A[1];
    const b2 = B[0] * B[0] + B[1] * B[1];
    const c2 = C[0] * C[0] + C[1] * C[1];
    const ux = (a2 * (B[1] - C[1]) + b2 * (C[1] - A[1]) + c2 * (A[1] - B[1])) / d;
    const uy = (a2 * (C[0] - B[0]) + b2 * (A[0] - C[0]) + c2 * (B[0] - A[0])) / d;
    return [ux, uy];
}

// intersection parameter t in [0,1] along p0->p1 where it crosses q0-q1, else -1
function apSegT(p0 is array, p1 is array, q0 is array, q1 is array) returns number
{
    const r0 = p1[0] - p0[0]; const r1 = p1[1] - p0[1];
    const s0 = q1[0] - q0[0]; const s1 = q1[1] - q0[1];
    const den = r0 * s1 - r1 * s0;
    if (abs(den) < 1e-12) return -1;
    const qpx = q0[0] - p0[0]; const qpy = q0[1] - p0[1];
    const t = (qpx * s1 - qpy * s0) / den;
    const u = (qpx * r1 - qpy * r0) / den;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return t;
    return -1;
}

function apSegCircleT(p0 is array, p1 is array, c is array, rr) returns array
{
    var res = [];
    const dx = p1[0] - p0[0]; const dy = p1[1] - p0[1];
    const a = dx * dx + dy * dy;
    if (a < 1e-12) return res;
    const fx = p0[0] - c[0]; const fy = p0[1] - c[1];
    const b = 2 * (fx * dx + fy * dy);
    const cc = fx * fx + fy * fy - rr * rr;
    var disc = b * b - 4 * a * cc;
    if (disc < 0) return res;
    disc = sqrt(disc);
    const t1 = (-b - disc) / (2 * a); const t2 = (-b + disc) / (2 * a);
    if (t1 > 0 && t1 < 1) res = append(res, t1);
    if (t2 > 0 && t2 < 1) res = append(res, t2);
    return res;
}

function apClosestOnSeg(p is array, a is array, b is array) returns array
{
    const vx = b[0] - a[0]; const vy = b[1] - a[1];
    const c1 = vx * (p[0] - a[0]) + vy * (p[1] - a[1]);
    if (c1 <= 0) return a;
    const c2 = vx * vx + vy * vy;
    if (c2 <= c1) return b;
    const t = c1 / c2;
    return [a[0] + t * vx, a[1] + t * vy];
}

// merge disconnected wall islands into ONE network with shortest links
function apConnectSegs(segs is array) returns array
{
    var nodes = []; var key2 = {}; var pairs = [];
    for (var i = 0; i < size(segs); i += 1)
    {
        var ids = [];
        for (var e = 0; e < 2; e += 1)
        {
            const pt = segs[i][e];
            const kk = round(pt[0] / 0.8) ~ "_" ~ round(pt[1] / 0.8);
            if (key2[kk] == undefined) { key2[kk] = size(nodes); nodes = append(nodes, pt); }
            ids = append(ids, key2[kk]);
        }
        pairs = append(pairs, ids);
    }
    var par = [];
    for (var i = 0; i < size(nodes); i += 1) par = append(par, i);
    for (var i = 0; i < size(pairs); i += 1) par[ufFind(par, pairs[i][0])] = ufFind(par, pairs[i][1]);
    var out = segs;
    var guard = 0;
    while (guard < 400)
    {
        guard += 1;
        var roots = {}; var nr = 0;
        for (var i = 0; i < size(nodes); i += 1)
        {
            const r = ufFind(par, i) ~ "";
            if (roots[r] == undefined) { roots[r] = true; nr += 1; }
        }
        if (nr <= 1) break;
        var best = 1e18; var bi = -1; var bj = -1;
        for (var i = 0; i < size(nodes); i += 1)
            for (var j = i + 1; j < size(nodes); j += 1)
            {
                if (ufFind(par, i) == ufFind(par, j)) continue;
                const dd = pointDistance(nodes[i], nodes[j]);
                if (dd < best) { best = dd; bi = i; bj = j; }
            }
        if (bi < 0) break;
        out = append(out, [nodes[bi], nodes[bj]]);
        par[ufFind(par, bi)] = ufFind(par, bj);
    }
    return out;
}

function apSortNums(arr is array) returns array
{
    var a = arr;
    for (var i = 0; i < size(a); i += 1)
    {
        var mi = i;
        for (var j = i + 1; j < size(a); j += 1) if (a[j] < a[mi]) mi = j;
        if (mi != i) { const t = a[i]; a[i] = a[mi]; a[mi] = t; }
    }
    return a;
}

function sortPool(arr is array) returns array
{
    var a = arr;
    for (var i = 0; i < size(a); i += 1)
    {
        var mi = i;
        for (var j = i + 1; j < size(a); j += 1)
        {
            const better = (a[j][1] < a[mi][1]) || (a[j][1] == a[mi][1] && a[j][2] < a[mi][2]);
            if (better) mi = j;
        }
        if (mi != i) { const t = a[i]; a[i] = a[mi]; a[mi] = t; }
    }
    return a;
}

function ufFind(parent is array, a) returns number
{
    var x = a;
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
}

// ---- Delaunay (Bowyer-Watson) ---------------------------------------------

function circumContains(ax, ay, bx, by, cx, cy, px, py) returns boolean
{
    const adx = ax - px; const ady = ay - py;
    const bdx = bx - px; const bdy = by - py;
    const cdx = cx - px; const cdy = cy - py;
    const d = (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
            - (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady)
            + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
    const ori = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (ori > 0) return d > 0;
    return d < 0;
}

function bowyerWatson(pts is array) returns array
{
    const n = size(pts);
    if (n < 3) return [];
    var minX = pts[0][0]; var minY = pts[0][1]; var maxX = pts[0][0]; var maxY = pts[0][1];
    for (var i = 1; i < n; i += 1)
    {
        minX = min(minX, pts[i][0]); minY = min(minY, pts[i][1]);
        maxX = max(maxX, pts[i][0]); maxY = max(maxY, pts[i][1]);
    }
    const dmax = max(maxX - minX, maxY - minY) * 20 + 100;
    const midx = (minX + maxX) / 2; const midy = (minY + maxY) / 2;
    var work = pts;
    work = append(work, [midx - dmax, midy - dmax]);
    work = append(work, [midx + dmax, midy - dmax]);
    work = append(work, [midx, midy + dmax]);
    const s0 = n; const s1 = n + 1; const s2 = n + 2;

    var tris = [[s0, s1, s2]];
    for (var ip = 0; ip < n; ip += 1)
    {
        const px = work[ip][0]; const py = work[ip][1];
        var bad = [];
        for (var t = 0; t < size(tris); t += 1)
        {
            const tr = tris[t];
            if (circumContains(work[tr[0]][0], work[tr[0]][1], work[tr[1]][0], work[tr[1]][1],
                               work[tr[2]][0], work[tr[2]][1], px, py))
                bad = append(bad, t);
        }
        var edges = [];
        for (var bi = 0; bi < size(bad); bi += 1)
        {
            const tr = tris[bad[bi]];
            const te = [[tr[0], tr[1]], [tr[1], tr[2]], [tr[2], tr[0]]];
            for (var ei = 0; ei < 3; ei += 1)
            {
                const a = te[ei][0]; const b = te[ei][1];
                var shared = false;
                for (var bj = 0; bj < size(bad); bj += 1)
                {
                    if (bj == bi) continue;
                    const tr2 = tris[bad[bj]];
                    const t2 = [[tr2[0], tr2[1]], [tr2[1], tr2[2]], [tr2[2], tr2[0]]];
                    for (var fj = 0; fj < 3; fj += 1)
                    {
                        const c = t2[fj][0]; const dd = t2[fj][1];
                        if ((a == c && b == dd) || (a == dd && b == c)) shared = true;
                    }
                }
                if (!shared) edges = append(edges, [a, b]);
            }
        }
        // remove bad triangles, highest index first
        for (var bi = size(bad) - 1; bi >= 0; bi -= 1)
            tris = apRemoveAt(tris, bad[bi]);
        for (var ee in edges)
            tris = append(tris, [ee[0], ee[1], ip]);
    }
    var out = [];
    for (var t = 0; t < size(tris); t += 1)
    {
        const tr = tris[t];
        if (tr[0] >= s0 || tr[1] >= s0 || tr[2] >= s0) continue;
        out = append(out, tr);
    }
    return out;
}

function apRemoveAt(arr is array, idx is number) returns array
{
    var out = [];
    for (var i = 0; i < size(arr); i += 1)
        if (i != idx) out = append(out, arr[i]);
    return out;
}

// ---- chain edge polylines into closed loops (outline + cutouts) -----------

function buildLoops(polylines is array) returns array
{
    var segs = [];
    for (var pl in polylines)
        for (var i = 0; i + 1 < size(pl); i += 1)
            segs = append(segs, [pl[i], pl[i + 1]]);
    const tol = 0.1;
    var used = [];
    for (var i = 0; i < size(segs); i += 1) used = append(used, false);
    var loops = [];
    for (var s = 0; s < size(segs); s += 1)
    {
        if (used[s]) continue;
        used[s] = true;
        var loop = [segs[s][0], segs[s][1]];
        var advanced = true;
        while (advanced)
        {
            advanced = false;
            const endP = loop[size(loop) - 1];
            for (var t = 0; t < size(segs); t += 1)
            {
                if (used[t]) continue;
                if (pointDistance(segs[t][0], endP) < tol) { loop = append(loop, segs[t][1]); used[t] = true; advanced = true; break; }
                if (pointDistance(segs[t][1], endP) < tol) { loop = append(loop, segs[t][0]); used[t] = true; advanced = true; break; }
            }
        }
        if (size(loop) >= 4 && pointDistance(loop[0], loop[size(loop) - 1]) < tol)
        {
            var trimmed = [];
            for (var ti = 0; ti < size(loop) - 1; ti += 1) trimmed = append(trimmed, loop[ti]);
            loops = append(loops, trimmed);
        }
    }
    return loops;
}
