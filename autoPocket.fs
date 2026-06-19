FeatureScript 2985;
import(path : "onshape/std/geometry.fs", version : "2985.0");

// AutoPocket  -  Automatic rib-pocketing for FTC / FRC plates
//
// Click a planar plate face and this feature finds every through-hole,
// (optionally) anchors to the plate edges, builds a Delaunay triangulation
// of those points, then THINS it into a clean lightening pattern:
//   - holes that sit closer together than "Merge holes closer than" are
//     collapsed to a single strut anchor (kills tiny clustered pockets), and
//   - each anchor keeps only its N shortest struts ("Max struts per hole"),
//     instead of connecting to every neighbour.
// The result is a coarse, organic web of large pockets with ~3 struts/hole.

// ===== Parameter bounds ====================================================

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

export const SPACING_BOUNDS = {
            (meter)      : [0.0, 0.025, 5.0],
            (centimeter) : 2.5,
            (millimeter) : 25.0,
            (inch)       : 1.0
        } as LengthBoundSpec;

export const EDGE_SAMPLE_BOUNDS = {
            (unitless) : [0, 1, 10]
        } as IntegerBoundSpec;

export const STRUT_BOUNDS = {
            (unitless) : [2, 3, 8]
        } as IntegerBoundSpec;

// ===== Feature =============================================================

annotation { "Feature Type Name" : "Auto Pocket",
             "Feature Type Description" : "Draws a thinned triangulated rib network between hole centers on a plate face." }
export const autoPocket = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Plate face",
                     "Filter" : EntityType.FACE && GeometryType.PLANE,
                     "MaxNumberOfPicks" : 1 }
        definition.face is Query;

        annotation { "Name" : "Ignore holes smaller than (diameter)" }
        isLength(definition.minHoleDiameter, NONNEG_LENGTH_BOUNDS);

        annotation { "Name" : "Merge holes closer than" }
        isLength(definition.minHoleSpacing, SPACING_BOUNDS);

        annotation { "Name" : "Max struts per hole" }
        isInteger(definition.maxStrutsPerHole, STRUT_BOUNDS);

        annotation { "Name" : "Anchor ribs to plate edges", "Default" : true }
        definition.includeBoundary is boolean;

        if (definition.includeBoundary)
        {
            annotation { "Name" : "Edge anchor points per side" }
            isInteger(definition.boundarySamples, EDGE_SAMPLE_BOUNDS);
        }

        annotation { "Name" : "Limit rib length (0 = no limit)" }
        isLength(definition.maxRibLength, NONNEG_LENGTH_BOUNDS);

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

        // ----- 1. Collect hole centers and boundary anchor points (mm) --------
        var holePts = [];      // [x, y]
        var holeRadii = [];    // mm, index-aligned with holePts
        var boundaryPts = [];  // [x, y]

        for (var edge in evaluateQuery(context, qLoopEdges(definition.face)))
        {
            const cdef = try silent(evCurveDefinition(context, { "edge" : edge }));
            var classifiedAsHole = false;

            if (cdef != undefined && cdef is Circle)
            {
                const circumference = 2 * PI * cdef.radius;
                const len = try silent(evLength(context, { "entities" : edge }));
                const isFullCircle = (len != undefined) && (abs(len - circumference) < 0.02 * circumference);
                const center = cdef.coordSystem.origin;
                const onPlane = abs(dot(center - plane.origin, plane.normal)) < 1e-5 * meter;
                const axisAligned = abs(dot(normalize(cdef.coordSystem.zAxis), plane.normal)) > 0.99;

                if (isFullCircle && onPlane && axisAligned && (cdef.radius / millimeter) >= minRadiusMm)
                {
                    const p2d = worldToPlane(plane, center);
                    holePts   = append(holePts, [p2d[0] / millimeter, p2d[1] / millimeter]);
                    holeRadii = append(holeRadii, cdef.radius / millimeter);
                    classifiedAsHole = true;
                }
            }

            if (!classifiedAsHole && definition.includeBoundary)
            {
                const count = definition.boundarySamples + 2;
                for (var i = 0; i < count; i += 1)
                {
                    const t = i / (count - 1);
                    const ln = try silent(evEdgeTangentLine(context, { "edge" : edge, "parameter" : t }));
                    if (ln != undefined)
                    {
                        const p2d = worldToPlane(plane, ln.origin);
                        boundaryPts = append(boundaryPts, [p2d[0] / millimeter, p2d[1] / millimeter]);
                    }
                }
            }
        }

        // ----- 2. Thin the strut anchors --------------------------------------
        // Holes closer than "Merge holes closer than" collapse to one anchor, so
        // dense grids of fastener holes don't each spawn struts (-> big pockets).
        const spacingMm = definition.minHoleSpacing / millimeter;
        var nodePts = [];
        var nodeRadii = [];
        for (var i = 0; i < size(holePts); i += 1)
        {
            var keep = true;
            if (spacingMm > 0.001)
            {
                for (var j = 0; j < size(nodePts); j += 1)
                {
                    if (pointDistance(holePts[i], nodePts[j]) < spacingMm) { keep = false; break; }
                }
            }
            if (keep)
            {
                nodePts = append(nodePts, holePts[i]);
                nodeRadii = append(nodeRadii, holeRadii[i]);
            }
        }
        holePts = nodePts;
        holeRadii = nodeRadii;

        boundaryPts = dedupePoints(boundaryPts, 0.05);

        const nHoles = size(holePts);
        if (nHoles == 0)
            throw regenError("No through-holes found on this face.", ["face"]);

        const pts = concatenateArrays([holePts, boundaryPts]);
        if (size(pts) < 2)
            throw regenError("Need at least 2 anchor points to draw a rib.", ["face"]);

        // ----- 3. Triangulate -------------------------------------------------
        var edgeSet = {};
        if (size(pts) == 2)
        {
            edgeSet["0_1"] = [0, 1];
        }
        else
        {
            const triangles = bowyerWatson(pts);
            for (var tri in triangles)
            {
                const triEdges = [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]];
                for (var e in triEdges)
                {
                    const a = min(e[0], e[1]);
                    const b = max(e[0], e[1]);
                    edgeSet[a ~ "_" ~ b] = [a, b];
                }
            }
        }

        // ----- 4. Thin edges: each hole keeps only its N shortest struts ------
        const maxRibMm = definition.maxRibLength / millimeter;
        const maxStruts = definition.maxStrutsPerHole;
        const totalPts = size(pts);
        var keepSet = {};
        for (var n = 0; n < totalPts; n += 1)
        {
            // Gather this node's candidate struts (must touch a hole, not too long).
            var inc = [];
            for (var key in keys(edgeSet))
            {
                const e = edgeSet[key];
                if (e[0] != n && e[1] != n)
                    continue;
                const other = (e[0] == n) ? e[1] : e[0];
                if (n >= nHoles && other >= nHoles)   // skip boundary-to-boundary
                    continue;
                const L = pointDistance(pts[n], pts[other]);
                if (L < 0.001)
                    continue;
                if (maxRibMm > 0.001 && L > maxRibMm)
                    continue;
                inc = append(inc, [other, L]);
            }
            inc = sortPairsByLength(inc);
            // Limit struts only for holes; boundary anchors keep all their links.
            const limit = (n < nHoles) ? min(maxStruts, size(inc)) : size(inc);
            for (var i = 0; i < limit; i += 1)
            {
                const other = inc[i][0];
                const aa = min(n, other);
                const bb = max(n, other);
                keepSet[aa ~ "_" ~ bb] = [aa, bb];
            }
        }
        var keptEdges = [];
        for (var key in keys(keepSet))
            keptEdges = append(keptEdges, keepSet[key]);

        // ----- 5. Draw rib centerlines ----------------------------------------
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
            for (var i = 0; i < nHoles; i += 1)
            {
                skCircle(ribSketch, "holeRef" ~ i, {
                            "center" : vector(holePts[i][0] * millimeter, holePts[i][1] * millimeter),
                            "radius" : holeRadii[i] * millimeter
                        });
            }
        }
        skSolve(ribSketch);

        // ----- 6. Optional pocket profiles (bosses + offset rib edges) --------
        if (definition.generatePockets)
        {
            const pocketSketch = newSketchOnPlane(context, id + "pockets", { "sketchPlane" : plane });
            const halfW = (definition.ribWidth / millimeter) / 2;
            const bossExtra = definition.bossOffset / millimeter;

            for (var i = 0; i < nHoles; i += 1)
            {
                skCircle(pocketSketch, "boss" ~ i, {
                            "center" : vector(holePts[i][0] * millimeter, holePts[i][1] * millimeter),
                            "radius" : (holeRadii[i] + bossExtra) * millimeter
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
                for (var s in [halfW, -halfW])
                {
                    skLineSegment(pocketSketch, "edge" ~ ofs, {
                                "start" : vector((pa[0] + nx * s) * millimeter, (pa[1] + ny * s) * millimeter),
                                "end"   : vector((pb[0] + nx * s) * millimeter, (pb[1] + ny * s) * millimeter)
                            });
                    ofs += 1;
                }
            }
            skSolve(pocketSketch);
        }

        reportFeatureInfo(context, id, nHoles ~ " anchors, " ~ size(keptEdges) ~ " struts drawn.");
    });

// ===== Helper functions ====================================================

function pointDistance(a is array, b is array) returns number
{
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    return sqrt(dx * dx + dy * dy);
}

function sortPairsByLength(arr is array) returns array
{
    var a = arr;
    for (var i = 0; i < size(a); i += 1)
    {
        var mi = i;
        for (var j = i + 1; j < size(a); j += 1)
        {
            if (a[j][1] < a[mi][1])
                mi = j;
        }
        if (mi != i)
        {
            const tmp = a[i];
            a[i] = a[mi];
            a[mi] = tmp;
        }
    }
    return a;
}

function dedupePoints(pts is array, tol is number) returns array
{
    var out = [];
    for (var p in pts)
    {
        var duplicate = false;
        for (var q in out)
        {
            if (pointDistance(p, q) < tol) { duplicate = true; break; }
        }
        if (!duplicate)
            out = append(out, p);
    }
    return out;
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
