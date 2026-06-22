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
//   7. keep a material ring (boss) around EVERY hole of any size
//   8. CUT: extrude {ribs ∪ bosses} as a tool and INTERSECT it with the plate,
//      so only the ribs + hole-rings survive (holes preserved, nothing floats)
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
    }
    {
        if (size(evaluateQuery(context, definition.face)) != 1)
            throw regenError("Select exactly one planar face.", ["face"]);

        const plane = evPlane(context, { "face" : definition.face });
        const s = definition.triangleSize / millimeter;
        const mergeDist = definition.mergeDist / millimeter;
        const holeMargin = definition.holeMargin / millimeter;
        const marginMm = definition.edgeMargin / millimeter;
        const minR = (definition.minHoleDiameter / 2) / millimeter;
        const maxEdge = definition.maxEdgeFactor * s;

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

        // ----- 4. even-spacing lattice fill (optional) --------------------
        if (definition.evenSpacing)
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

        // ----- 5. rim sampling --------------------------------------------
        var rim = [];
        var acc = s;
        for (var i = 0; i < size(poly); i += 1)
        {
            acc += pointDistance(poly[i], poly[(i + 1) % size(poly)]);
            if (acc >= s) { rim = append(rim, poly[i]); acc = 0; }
        }
        rim = dedupePoints(rim, 0.4 * s);

        const pts = concatenateArrays([nodePts, rim]);
        if (size(pts) < 3)
            throw regenError("Plate too small for this triangle size.", ["triangleSize"]);

        // ----- 6. Delaunay + keep in-material triangle edges --------------
        const triangles = bowyerWatson(pts);
        var ribSet = {};
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
                ribSet[lo ~ "_" ~ hi] = [pts[lo], pts[hi]];
            }
        }
        var ribs = [];
        for (var key in keys(ribSet)) ribs = append(ribs, ribSet[key]);
        for (var cr in clusterRibs) ribs = append(ribs, cr);

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
        for (var i = 0; i < size(holes); i += 1)
            skCircle(ribSketch, "boss" ~ i, {
                        "center" : vector(holes[i][0] * millimeter, holes[i][1] * millimeter),
                        "radius" : (hRad[i] + holeMargin) * millimeter });
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
