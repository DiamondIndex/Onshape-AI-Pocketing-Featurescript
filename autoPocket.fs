FeatureScript 2985;
import(path : "onshape/std/geometry.fs", version : "2985.0");

// AutoPocket  -  Uniform triangular lightening for FTC / FRC plates
//
// Instead of triangulating the (irregularly spaced) holes -- which gives
// uneven pockets -- this lays a REGULAR equilateral-triangle lattice over the
// plate, snaps lattice nodes onto the holes, clips everything to the plate
// outline, and triangulates. The result is an even web of same-size triangles
// with every hole supported from all sides, like a hand-pocketed plate.

// ===== Parameter bounds ====================================================

export const CELL_BOUNDS = {
            (meter)      : [0.005, 0.028, 0.5],
            (centimeter) : 2.8,
            (millimeter) : 28.0,
            (inch)       : 1.1
        } as LengthBoundSpec;

export const MARGIN_BOUNDS = {
            (meter)      : [0.0, 0.006, 0.2],
            (centimeter) : 0.6,
            (millimeter) : 6.0,
            (inch)       : 0.25
        } as LengthBoundSpec;

export const RIB_WIDTH_BOUNDS = {
            (meter)      : [1e-4, 0.004, 0.1],
            (centimeter) : 0.4,
            (millimeter) : 4.0,
            (inch)       : 0.15
        } as LengthBoundSpec;

export const BOSS_BOUNDS = {
            (meter)      : [0.0, 0.004, 0.1],
            (centimeter) : 0.4,
            (millimeter) : 4.0,
            (inch)       : 0.15
        } as LengthBoundSpec;

export const NONNEG_LENGTH_BOUNDS = {
            (meter)      : [0.0, 0.0, 5.0],
            (centimeter) : 0.0,
            (millimeter) : 0.0,
            (inch)       : 0.0
        } as LengthBoundSpec;

// ===== Feature =============================================================

annotation { "Feature Type Name" : "Auto Pocket",
             "Feature Type Description" : "Lays a uniform triangular lightening lattice over a plate face." }
export const autoPocket = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Plate face",
                     "Filter" : EntityType.FACE && GeometryType.PLANE,
                     "MaxNumberOfPicks" : 1 }
        definition.face is Query;

        annotation { "Name" : "Triangle size" }
        isLength(definition.cellSize, CELL_BOUNDS);

        annotation { "Name" : "Margin from plate edge" }
        isLength(definition.edgeMargin, MARGIN_BOUNDS);

        annotation { "Name" : "Ignore holes smaller than (diameter)" }
        isLength(definition.minHoleDiameter, NONNEG_LENGTH_BOUNDS);

        annotation { "Name" : "Snap lattice to holes" }
        definition.snapToHoles is boolean;

        annotation { "Name" : "Draw reference circles at holes" }
        definition.drawHoleCircles is boolean;

        annotation { "Name" : "Also generate pocket profiles to cut" }
        definition.generatePockets is boolean;

        if (definition.generatePockets)
        {
            annotation { "Name" : "Rib width" }
            isLength(definition.ribWidth, RIB_WIDTH_BOUNDS);

            annotation { "Name" : "Material ring around holes (boss)" }
            isLength(definition.bossOffset, BOSS_BOUNDS);
        }
    }
    {
        if (size(evaluateQuery(context, definition.face)) != 1)
            throw regenError("Select exactly one planar face.", ["face"]);

        const plane = evPlane(context, { "face" : definition.face });
        const minRadiusMm = (definition.minHoleDiameter / 2) / millimeter;
        const s = definition.cellSize / millimeter;          // triangle edge length, mm
        const marginMm = definition.edgeMargin / millimeter;

        // ----- 1. Read holes and the plate outline ----------------------------
        var holePts = [];
        var holeRadii = [];
        var polylines = [];   // boundary edges, each as an ordered list of [x,y]

        for (var edge in evaluateQuery(context, qLoopEdges(definition.face)))
        {
            const cdef = try silent(evCurveDefinition(context, { "edge" : edge }));
            var isHole = false;

            if (cdef != undefined && cdef is Circle)
            {
                const circ = 2 * PI * cdef.radius;
                const len = try silent(evLength(context, { "entities" : edge }));
                const fullCircle = (len != undefined) && (abs(len - circ) < 0.02 * circ);
                const c = cdef.coordSystem.origin;
                const onPlane = abs(dot(c - plane.origin, plane.normal)) < 1e-5 * meter;
                const axisAligned = abs(dot(normalize(cdef.coordSystem.zAxis), plane.normal)) > 0.99;
                if (fullCircle && onPlane && axisAligned && (cdef.radius / millimeter) >= minRadiusMm)
                {
                    const p2d = worldToPlane(plane, c);
                    holePts   = append(holePts, [p2d[0] / millimeter, p2d[1] / millimeter]);
                    holeRadii = append(holeRadii, cdef.radius / millimeter);
                    isHole = true;
                }
            }

            if (!isHole)
            {
                // Sample this boundary edge into an ordered polyline.
                var pl = [];
                for (var i = 0; i < 6; i += 1)
                {
                    const t = i / 5;
                    const ln = try silent(evEdgeTangentLine(context, { "edge" : edge, "parameter" : t }));
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

        if (size(polylines) < 2)
            throw regenError("Could not read the plate outline from this face.", ["face"]);

        // Chain the boundary edges into one closed outline polygon (mm).
        const poly = buildBoundaryPolygon(polylines);
        if (size(poly) < 3)
            throw regenError("Could not form a closed plate outline.", ["face"]);

        // ----- 2. Merge holes that are too close (avoids micro-pockets) -------
        const holeMerge = 0.5 * s;
        var mHolePts = [];
        var mHoleRadii = [];
        for (var i = 0; i < size(holePts); i += 1)
        {
            var keep = true;
            for (var j = 0; j < size(mHolePts); j += 1)
            {
                if (pointDistance(holePts[i], mHolePts[j]) < holeMerge) { keep = false; break; }
            }
            if (keep) { mHolePts = append(mHolePts, holePts[i]); mHoleRadii = append(mHoleRadii, holeRadii[i]); }
        }

        // ----- 3. Generate the equilateral-triangle lattice inside the plate --
        var bMinX = poly[0][0]; var bMaxX = poly[0][0];
        var bMinY = poly[0][1]; var bMaxY = poly[0][1];
        for (var p in poly)
        {
            bMinX = min(bMinX, p[0]); bMaxX = max(bMaxX, p[0]);
            bMinY = min(bMinY, p[1]); bMaxY = max(bMaxY, p[1]);
        }
        const rowH = s * sqrt(3) / 2;
        const holeClear = 0.55 * s;

        var latticePts = [];
        var row = 0;
        var y = bMinY;
        while (y <= bMaxY)
        {
            const xoff = (row % 2 == 0) ? 0 : (s / 2);
            var x = bMinX + xoff;
            while (x <= bMaxX)
            {
                const q = [x, y];
                if (pointInPolygon(poly, q) && distToPolygon(poly, q) >= marginMm)
                {
                    var clearOfHoles = true;
                    if (definition.snapToHoles)
                    {
                        for (var h in mHolePts)
                        {
                            if (pointDistance(q, h) < holeClear) { clearOfHoles = false; break; }
                        }
                    }
                    if (clearOfHoles)
                        latticePts = append(latticePts, q);
                }
                x += s;
            }
            y += rowH;
            row += 1;
        }

        // ----- 4. Sample the outline at ~s spacing so the rim triangulates ----
        var rimPts = [];
        var acc = s; // force first point
        for (var i = 0; i < size(poly); i += 1)
        {
            const a = poly[i];
            const b = poly[(i + 1) % size(poly)];
            acc += pointDistance(a, b);
            if (acc >= s) { rimPts = append(rimPts, a); acc = 0; }
        }
        rimPts = dedupePoints(rimPts, 0.25 * s);

        // ----- 5. Assemble nodes: holes first, then lattice + rim -------------
        const holeNodes = (definition.snapToHoles) ? mHolePts : [];
        const holeNodeRadii = (definition.snapToHoles) ? mHoleRadii : [];
        const nHoleNodes = size(holeNodes);
        const pts = concatenateArrays([holeNodes, latticePts, rimPts]);
        if (size(pts) < 3)
            throw regenError("Plate too small for this triangle size; reduce 'Triangle size'.", ["cellSize"]);

        // ----- 6. Triangulate and keep only interior lattice edges ------------
        const triangles = bowyerWatson(pts);
        const maxEdge = 1.8 * s;
        var keepSet = {};
        for (var tri in triangles)
        {
            const triEdges = [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]];
            for (var e in triEdges)
            {
                const a = min(e[0], e[1]);
                const b = max(e[0], e[1]);
                const pa = pts[a];
                const pb = pts[b];
                const L = pointDistance(pa, pb);
                if (L < 0.001 || L > maxEdge)
                    continue;
                const mid = [(pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2];
                if (!pointInPolygon(poly, mid))
                    continue;
                keepSet[a ~ "_" ~ b] = [a, b];
            }
        }
        var keptEdges = [];
        for (var key in keys(keepSet))
            keptEdges = append(keptEdges, keepSet[key]);

        // ----- 7. Draw rib centerlines ----------------------------------------
        const ribSketch = newSketchOnPlane(context, id + "ribs", { "sketchPlane" : plane });
        var ribIndex = 0;
        for (var e in keptEdges)
        {
            const pa = pts[e[0]];
            const pb = pts[e[1]];
            skLineSegment(ribSketch, "rib" ~ ribIndex, {
                        "start" : vector(pa[0] * millimeter, pa[1] * millimeter),
                        "end"   : vector(pb[0] * millimeter, pb[1] * millimeter)
                    });
            ribIndex += 1;
        }

        if (definition.drawHoleCircles)
        {
            for (var i = 0; i < nHoleNodes; i += 1)
            {
                skCircle(ribSketch, "holeRef" ~ i, {
                            "center" : vector(holeNodes[i][0] * millimeter, holeNodes[i][1] * millimeter),
                            "radius" : holeNodeRadii[i] * millimeter
                        });
            }
        }
        skSolve(ribSketch);

        // ----- 8. Optional pocket profiles (bosses + offset rib edges) --------
        if (definition.generatePockets)
        {
            const pocketSketch = newSketchOnPlane(context, id + "pockets", { "sketchPlane" : plane });
            const halfW = (definition.ribWidth / millimeter) / 2;
            const bossExtra = definition.bossOffset / millimeter;

            for (var i = 0; i < nHoleNodes; i += 1)
            {
                skCircle(pocketSketch, "boss" ~ i, {
                            "center" : vector(holeNodes[i][0] * millimeter, holeNodes[i][1] * millimeter),
                            "radius" : (holeNodeRadii[i] + bossExtra) * millimeter
                        });
            }

            var ofs = 0;
            for (var e in keptEdges)
            {
                const pa = pts[e[0]];
                const pb = pts[e[1]];
                const dx = pb[0] - pa[0];
                const dy = pb[1] - pa[1];
                const len = sqrt(dx * dx + dy * dy);
                if (len < 0.001)
                    continue;
                const nx = -dy / len;
                const ny =  dx / len;
                for (var sgn in [halfW, -halfW])
                {
                    skLineSegment(pocketSketch, "edge" ~ ofs, {
                                "start" : vector((pa[0] + nx * sgn) * millimeter, (pa[1] + ny * sgn) * millimeter),
                                "end"   : vector((pb[0] + nx * sgn) * millimeter, (pb[1] + ny * sgn) * millimeter)
                            });
                    ofs += 1;
                }
            }
            skSolve(pocketSketch);
        }

        reportFeatureInfo(context, id, size(keptEdges) ~ " struts on a " ~ round(s) ~ " mm triangle lattice.");
    });

// ===== Helper functions ====================================================

function pointDistance(a is array, b is array) returns number
{
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    return sqrt(dx * dx + dy * dy);
}

function dedupePoints(pts is array, tol is number) returns array
{
    var out = [];
    for (var p in pts)
    {
        var dup = false;
        for (var q in out)
        {
            if (pointDistance(p, q) < tol) { dup = true; break; }
        }
        if (!dup)
            out = append(out, p);
    }
    return out;
}

function pointInPolygon(poly is array, p is array) returns boolean
{
    var inside = false;
    const n = size(poly);
    var j = n - 1;
    for (var i = 0; i < n; i += 1)
    {
        const yi = poly[i][1];
        const yj = poly[j][1];
        if ((yi > p[1]) != (yj > p[1]))
        {
            const xcross = (poly[j][0] - poly[i][0]) * (p[1] - yi) / (yj - yi) + poly[i][0];
            if (p[0] < xcross)
                inside = !inside;
        }
        j = i;
    }
    return inside;
}

function distPointToSegment(p is array, a is array, b is array) returns number
{
    const vx = b[0] - a[0];
    const vy = b[1] - a[1];
    const wx = p[0] - a[0];
    const wy = p[1] - a[1];
    const c1 = vx * wx + vy * wy;
    if (c1 <= 0)
        return pointDistance(p, a);
    const c2 = vx * vx + vy * vy;
    if (c2 <= c1)
        return pointDistance(p, b);
    const t = c1 / c2;
    return pointDistance(p, [a[0] + t * vx, a[1] + t * vy]);
}

function distToPolygon(poly is array, p is array) returns number
{
    var best = 1e18;
    const n = size(poly);
    var j = n - 1;
    for (var i = 0; i < n; i += 1)
    {
        best = min(best, distPointToSegment(p, poly[j], poly[i]));
        j = i;
    }
    return best;
}

function polygonArea(poly is array) returns number
{
    var a = 0;
    const n = size(poly);
    var j = n - 1;
    for (var i = 0; i < n; i += 1)
    {
        a += poly[j][0] * poly[i][1] - poly[i][0] * poly[j][1];
        j = i;
    }
    return abs(a) / 2;
}

// Chain edge polylines end-to-end into closed loops; return the largest (outer).
function buildBoundaryPolygon(polylines is array) returns array
{
    const tol = 0.5;
    var used = [];
    for (var i = 0; i < size(polylines); i += 1)
        used = append(used, false);

    var bestLoop = [];
    var bestArea = -1;
    for (var startI = 0; startI < size(polylines); startI += 1)
    {
        if (used[startI])
            continue;
        var loop = polylines[startI];
        used[startI] = true;
        var changed = true;
        while (changed)
        {
            changed = false;
            const endP = loop[size(loop) - 1];
            for (var k = 0; k < size(polylines); k += 1)
            {
                if (used[k])
                    continue;
                const pl = polylines[k];
                if (pointDistance(endP, pl[0]) < tol)
                {
                    for (var m = 1; m < size(pl); m += 1)
                        loop = append(loop, pl[m]);
                    used[k] = true; changed = true; break;
                }
                else if (pointDistance(endP, pl[size(pl) - 1]) < tol)
                {
                    for (var m = size(pl) - 2; m >= 0; m -= 1)
                        loop = append(loop, pl[m]);
                    used[k] = true; changed = true; break;
                }
            }
        }
        const area = polygonArea(loop);
        if (area > bestArea) { bestArea = area; bestLoop = loop; }
    }
    return bestLoop;
}

// Delaunay triangulation (Bowyer-Watson). In: array of [x,y]. Out: array of [i,j,k].
function inCircumcircle(p is array, a is array, b is array, c is array) returns boolean
{
    const ax = a[0] - p[0]; const ay = a[1] - p[1];
    const bx = b[0] - p[0]; const by = b[1] - p[1];
    const cx = c[0] - p[0]; const cy = c[1] - p[1];

    var det = (ax * ax + ay * ay) * (bx * cy - cx * by)
            - (bx * bx + by * by) * (ax * cy - cx * ay)
            + (cx * cx + cy * cy) * (ax * by - bx * ay);

    const orient = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    if (orient < 0)
        det = -det;
    return det > 0;
}

function sameUndirectedEdge(e1 is array, e2 is array) returns boolean
{
    return (e1[0] == e2[0] && e1[1] == e2[1]) || (e1[0] == e2[1] && e1[1] == e2[0]);
}

function bowyerWatson(inputPts is array) returns array
{
    const n = size(inputPts);

    var minX = inputPts[0][0]; var maxX = inputPts[0][0];
    var minY = inputPts[0][1]; var maxY = inputPts[0][1];
    for (var p in inputPts)
    {
        minX = min(minX, p[0]); maxX = max(maxX, p[0]);
        minY = min(minY, p[1]); maxY = max(maxY, p[1]);
    }
    const dmax = max(maxX - minX, maxY - minY) * 10 + 1;
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;

    const verts = concatenateArrays([inputPts, [
                [midX - 2 * dmax, midY - dmax],
                [midX,            midY + 2 * dmax],
                [midX + 2 * dmax, midY - dmax]
            ]]);

    var triangles = [[n, n + 1, n + 2]];

    for (var i = 0; i < n; i += 1)
    {
        const p = verts[i];

        var badTriangles = [];
        for (var t in triangles)
        {
            if (inCircumcircle(p, verts[t[0]], verts[t[1]], verts[t[2]]))
                badTriangles = append(badTriangles, t);
        }

        var polygon = [];
        for (var t in badTriangles)
        {
            const tEdges = [[t[0], t[1]], [t[1], t[2]], [t[2], t[0]]];
            for (var e in tEdges)
            {
                var shared = false;
                for (var t2 in badTriangles)
                {
                    if (t2 == t)
                        continue;
                    const t2Edges = [[t2[0], t2[1]], [t2[1], t2[2]], [t2[2], t2[0]]];
                    for (var e2 in t2Edges)
                    {
                        if (sameUndirectedEdge(e, e2)) { shared = true; break; }
                    }
                    if (shared) break;
                }
                if (!shared)
                    polygon = append(polygon, e);
            }
        }

        var remaining = [];
        for (var t in triangles)
        {
            var isBad = false;
            for (var bt in badTriangles)
            {
                if (bt == t) { isBad = true; break; }
            }
            if (!isBad)
                remaining = append(remaining, t);
        }
        triangles = remaining;

        for (var e in polygon)
            triangles = append(triangles, [e[0], e[1], i]);
    }

    var result = [];
    for (var t in triangles)
    {
        if (t[0] < n && t[1] < n && t[2] < n)
            result = append(result, t);
    }
    return result;
}
