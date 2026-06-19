// AutoPocket  -  Automatic rib-pocketing for FTC / FRC plates
//
// Click a planar plate face and this feature finds every through-hole,
// (optionally) anchors to the plate edges, builds a Delaunay triangulation
// of those points, and draws the triangle edges as rib centerlines in a
// sketch -- the classic triangulated lightening pattern.
//
// NOTE ON THE VERSION LINE BELOW:
//   When you create a new Feature Studio, Onshape auto-fills the first two
//   lines (FeatureScript <n>; and the import with version "<n>.0") with the
//   version current to your document. If you paste this whole file, just
//   replace 2588 with whatever number Onshape generated for you, or paste
//   everything from "// ===== Parameter bounds" downward below the import
//   lines Onshape created.

FeatureScript 2588;
import(path : "onshape/std/geometry.fs", version : "2588.0");

// ===== Parameter bounds ====================================================

export const RIB_WIDTH_BOUNDS = {
            (meter)      : [1e-4, 0.003, 0.1],
            (centimeter) : 0.3,
            (millimeter) : 3.0,
            (inch)       : 0.125
        } as LengthBoundSpec;

export const BOSS_BOUNDS = {
            (meter)      : [0.0, 0.003, 0.1],
            (centimeter) : 0.3,
            (millimeter) : 3.0,
            (inch)       : 0.125
        } as LengthBoundSpec;

export const NONNEG_LENGTH_BOUNDS = {
            (meter)      : [0.0, 0.0, 5.0],
            (centimeter) : 0.0,
            (millimeter) : 0.0,
            (inch)       : 0.0
        } as LengthBoundSpec;

export const SAMPLE_BOUNDS = {
            (unitless) : [0, 1, 10]
        } as IntegerBoundSpec;

// ===== Feature definition ==================================================

annotation { "Feature Type Name" : "Auto Pocket",
             "Feature Type Description" : "Draws triangulated rib lines between hole centers on a plate face." }
export const autoPocket = defineFeature(function(context is Context, id is Id, definition is map)
    precondition
    {
        annotation { "Name" : "Plate face",
                     "Filter" : EntityType.FACE && GeometryType.PLANE,
                     "MaxNumberOfPicks" : 1 }
        definition.face is Query;

        annotation { "Name" : "Ignore holes smaller than (diameter)" }
        isLength(definition.minHoleDiameter, NONNEG_LENGTH_BOUNDS);

        annotation { "Name" : "Anchor ribs to plate edges" }
        definition.includeBoundary is boolean;

        if (definition.includeBoundary)
        {
            annotation { "Name" : "Edge anchor points per side" }
            isInteger(definition.boundarySamples, SAMPLE_BOUNDS);
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

        // ----- 1. Collect hole centers and boundary anchor points (numbers, mm)
        var holePts   = [];   // [x, y] in mm
        var holeRadii = [];   // mm, index-aligned with holePts
        var boundaryPts = []; // [x, y] in mm

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

            // Anything that is not a hole is treated as plate boundary; sample it.
            if (!classifiedAsHole && definition.includeBoundary)
            {
                const count = definition.boundarySamples + 2; // endpoints + interior samples
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

        boundaryPts = dedupePoints(boundaryPts, 0.05); // collapse shared vertices (0.05 mm)

        const nHoles = size(holePts);
        if (nHoles == 0)
            throw regenError("No through-holes found on this face. (Holes are detected as full circular edges lying in the face.)", ["face"]);

        const pts = concatenateArrays([holePts, boundaryPts]);
        if (size(pts) < 2)
            throw regenError("Need at least 2 anchor points (holes and/or plate edges) to draw a rib.", ["face"]);

        // ----- 2. Triangulate -------------------------------------------------
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

        // ----- 3. Filter edges: keep ribs that touch a hole and aren't too long
        const maxRibMm = definition.maxRibLength / millimeter;
        var keptEdges = [];
        for (var key in keys(edgeSet))
        {
            const e = edgeSet[key];
            const a = e[0];
            const b = e[1];
            // Skip edges that run between two boundary points (they sit on the plate edge).
            if (a >= nHoles && b >= nHoles)
                continue;
            const L = pointDistance(pts[a], pts[b]);
            if (L < 0.001)
                continue;
            if (maxRibMm > 0.001 && L > maxRibMm)
                continue;
            keptEdges = append(keptEdges, [a, b]);
        }

        // ----- 4. Draw rib centerlines ----------------------------------------
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

        // ----- 5. Optional: pocket profiles (bosses + offset rib edges) -------
        // These overlapping curves divide the face into closed regions you can
        // select with Extrude (Remove) -- the open triangle interiors are the
        // pockets, the ribs/bosses are the material you keep.
        if (definition.generatePockets)
        {
            const pocketSketch = newSketchOnPlane(context, id + "pockets", { "sketchPlane" : plane });
            const halfW   = (definition.ribWidth / millimeter) / 2;
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
                const nx = -dy / len; // unit normal to the rib
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

        reportFeatureInfo(context, id, nHoles ~ " holes detected, " ~ size(keptEdges) ~ " ribs drawn.");
    });

// ===== Geometry helpers ====================================================

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
        var duplicate = false;
        for (var q in out)
        {
            if (pointDistance(p, q) < tol)
            {
                duplicate = true;
                break;
            }
        }
        if (!duplicate)
            out = append(out, p);
    }
    return out;
}

// ===== Delaunay triangulation (Bowyer-Watson) ==============================
// Input:  array of [x, y] numbers.
// Output: array of triangles, each an array of 3 indices into the input.

function inCircumcircle(p is array, a is array, b is array, c is array) returns boolean
{
    const ax = a[0] - p[0];
    const ay = a[1] - p[1];
    const bx = b[0] - p[0];
    const by = b[1] - p[1];
    const cx = c[0] - p[0];
    const cy = c[1] - p[1];

    var det = (ax * ax + ay * ay) * (bx * cy - cx * by)
            - (bx * bx + by * by) * (ax * cy - cx * ay)
            + (cx * cx + cy * cy) * (ax * by - bx * ay);

    // Sign depends on the winding of a, b, c; normalize for CCW.
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

    // Bounding box -> super-triangle that comfortably contains every point.
    var minX = inputPts[0][0];
    var maxX = inputPts[0][0];
    var minY = inputPts[0][1];
    var maxY = inputPts[0][1];
    for (var p in inputPts)
    {
        minX = min(minX, p[0]);
        maxX = max(maxX, p[0]);
        minY = min(minY, p[1]);
        maxY = max(maxY, p[1]);
    }
    const dmax = max(maxX - minX, maxY - minY) * 10 + 1;
    const midX = (minX + maxX) / 2;
    const midY = (minY + maxY) / 2;

    const verts = concatenateArrays([inputPts, [
                [midX - 2 * dmax, midY - dmax],
                [midX,            midY + 2 * dmax],
                [midX + 2 * dmax, midY - dmax]
            ]]);

    var triangles = [[n, n + 1, n + 2]]; // start with the super-triangle

    for (var i = 0; i < n; i += 1)
    {
        const p = verts[i];

        // Triangles whose circumcircle contains p are invalid.
        var badTriangles = [];
        for (var t in triangles)
        {
            if (inCircumcircle(p, verts[t[0]], verts[t[1]], verts[t[2]]))
                badTriangles = append(badTriangles, t);
        }

        // Boundary of the cavity = edges belonging to exactly one bad triangle.
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
                        if (sameUndirectedEdge(e, e2))
                        {
                            shared = true;
                            break;
                        }
                    }
                    if (shared)
                        break;
                }
                if (!shared)
                    polygon = append(polygon, e);
            }
        }

        // Drop bad triangles.
        var remaining = [];
        for (var t in triangles)
        {
            var isBad = false;
            for (var bt in badTriangles)
            {
                if (bt == t)
                {
                    isBad = true;
                    break;
                }
            }
            if (!isBad)
                remaining = append(remaining, t);
        }
        triangles = remaining;

        // Re-triangulate the cavity against the new point.
        for (var e in polygon)
            triangles = append(triangles, [e[0], e[1], i]);
    }

    // Discard any triangle still touching the super-triangle vertices.
    var result = [];
    for (var t in triangles)
    {
        if (t[0] < n && t[1] < n && t[2] < n)
            result = append(result, t);
    }
    return result;
}
