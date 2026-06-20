FeatureScript 2985;
import(path : "onshape/std/geometry.fs", version : "2985.0");

// AutoPocket  -  Uniform triangular lightening for FTC / FRC plates
//
// Lays a regular equilateral-triangle lattice over the plate and snaps the
// nearest lattice node onto each well-spaced hole. Holes packed closer than the
// triangle size are not made grid vertices; instead each such "floating" hole
// is tied into the network with short support struts to its nearest neighbours
// (chaining rows of holes and anchoring to nearby vertices). Non-circular slots
// get a reinforced band.

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

export const SLOT_BOUNDS = {
            (meter)      : [0.0, 0.012, 0.2],
            (centimeter) : 1.2,
            (millimeter) : 12.0,
            (inch)       : 0.5
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

export const ANGLE_BOUNDS = {
            (degree) : [0.0, 30.0, 90.0],
            (radian) : 0.5235987755982988
        } as AngleBoundSpec;

export const MERGE_BOUNDS = {
            (meter)      : [0.0, 0.006, 0.1],
            (centimeter) : 0.6,
            (millimeter) : 6.0,
            (inch)       : 0.25
        } as LengthBoundSpec;

export const RIB_EDGE_BOUNDS = {
            (meter)      : [0.0, 0.005, 0.1],
            (centimeter) : 0.5,
            (millimeter) : 5.0,
            (inch)       : 0.2
        } as LengthBoundSpec;

export const REROUTE_BOUNDS = {
            (meter)      : [0.0, 0.03, 0.3],
            (centimeter) : 3.0,
            (millimeter) : 30.0,
            (inch)       : 1.2
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

        annotation { "Name" : "Triangle side length" }
        isLength(definition.cellSize, CELL_BOUNDS);

        annotation { "Name" : "Minimum angle between ribs" }
        isAngle(definition.minAngle, ANGLE_BOUNDS);

        annotation { "Name" : "Merge ribs closer than" }
        isLength(definition.mergeDist, MERGE_BOUNDS);

        annotation { "Name" : "Rib-to-edge margin" }
        isLength(definition.ribEdgeMargin, RIB_EDGE_BOUNDS);

        annotation { "Name" : "Reroute search radius" }
        isLength(definition.rerouteRadius, REROUTE_BOUNDS);

        annotation { "Name" : "Margin from plate edge" }
        isLength(definition.edgeMargin, MARGIN_BOUNDS);

        annotation { "Name" : "Material band around slots" }
        isLength(definition.slotMargin, SLOT_BOUNDS);

        annotation { "Name" : "Ignore holes smaller than (diameter)" }
        isLength(definition.minHoleDiameter, NONNEG_LENGTH_BOUNDS);

        annotation { "Name" : "Triangulate holes directly (match plate)" }
        definition.holesDirect is boolean;

        annotation { "Name" : "Fill sparse gaps with lattice" }
        definition.gapFill is boolean;

        annotation { "Name" : "Snap lattice to holes", "Default" : true }
        definition.snapToHoles is boolean;

        annotation { "Name" : "Force uniform triangles (test)" }
        definition.forceTriangles is boolean;

        annotation { "Name" : "Force equal-size triangles (test)" }
        definition.equalSize is boolean;

        annotation { "Name" : "Force equilateral triangles (test)" }
        definition.equilateral is boolean;

        annotation { "Name" : "Draw reference circles at holes" }
        definition.drawHoleCircles is boolean;

        annotation { "Name" : "Material around holes" }
        isLength(definition.bossOffset, BOSS_BOUNDS);

        annotation { "Name" : "Also generate pocket profiles to cut" }
        definition.generatePockets is boolean;

        annotation { "Name" : "Cut pockets (rib walls + remove)" }
        definition.autoCut is boolean;

        if (definition.generatePockets || definition.autoCut)
        {
            annotation { "Name" : "Rib width / wall thickness" }
            isLength(definition.ribWidth, RIB_WIDTH_BOUNDS);

            annotation { "Name" : "Pocket corner fillet" }
            isLength(definition.pocketFillet, RIB_EDGE_BOUNDS);
        }
    }
    {
        if (size(evaluateQuery(context, definition.face)) != 1)
            throw regenError("Select exactly one planar face.", ["face"]);

        const plane = evPlane(context, { "face" : definition.face });
        const minRadiusMm = (definition.minHoleDiameter / 2) / millimeter;
        const s = definition.cellSize / millimeter;
        const marginMm = definition.edgeMargin / millimeter;
        const slotMarginMm = definition.slotMargin / millimeter;

        // ----- 1. Read holes and outline edges --------------------------------
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

        // ----- 2. Outer outline + inner cutouts (slots) -----------------------
        const loops = buildLoops(polylines);
        if (size(loops) == 0)
            throw regenError("Could not read the plate outline from this face.", ["face"]);
        var outerIdx = 0;
        var maxA = -1;
        for (var i = 0; i < size(loops); i += 1)
        {
            const ar = polygonArea(loops[i]);
            if (ar > maxA) { maxA = ar; outerIdx = i; }
        }
        const poly = loops[outerIdx];
        var innerLoops = [];
        for (var i = 0; i < size(loops); i += 1)
        {
            if (i != outerIdx && polygonArea(loops[i]) > (0.3 * s) * (0.3 * s))
                innerLoops = append(innerLoops, loops[i]);
        }

        // ----- 3. Drop duplicate holes ----------------------------------------
        var dHolePts = [];
        var dHoleRadii = [];
        for (var i = 0; i < size(holePts); i += 1)
        {
            var dup = false;
            for (var j = 0; j < size(dHolePts); j += 1)
            {
                if (pointDistance(holePts[i], dHolePts[j]) < 1.0) { dup = true; break; }
            }
            if (!dup) { dHolePts = append(dHolePts, holePts[i]); dHoleRadii = append(dHoleRadii, holeRadii[i]); }
        }

        // ----- 4-6. Triangulation nodes ---------------------------------------
        var nodePts = [];
        var nodeIsHole = [];
        var nodeRadius = [];
        var floatPts = [];
        var floatRadii = [];

        if (definition.holesDirect)
        {
            // Match a hand-pocketed plate: the holes themselves ARE the rib
            // vertices (no background lattice). Ribs run hole-to-hole.
            //
            // TUNING/LEARNING: a real plate has tight clusters of fastener holes
            // (bolt rows, the ear) that must NOT each become a triangle vertex,
            // or the triangulation shatters into slivers. So we decluster: take
            // holes largest-first and keep one only if it is at least ~0.55*s
            // from every hole already kept. This gives the clean, evenly spaced
            // vertex set seen in the reference image while still covering the
            // whole plate.
            const dropDist = 0.55 * s;
            var hOrder = [];
            for (var i = 0; i < size(dHolePts); i += 1)
                hOrder = append(hOrder, [i, dHoleRadii[i]]);
            hOrder = sortByValueDesc(hOrder);
            var keptHoles = [];
            for (var oi = 0; oi < size(hOrder); oi += 1)
            {
                const hi = hOrder[oi][0];
                const hp = dHolePts[hi];
                if (!inMaterial(poly, innerLoops, hp))
                    continue;
                var skip = false;
                for (var kp in keptHoles)
                {
                    if (pointDistance(hp, kp) < dropDist) { skip = true; break; }
                }
                if (skip)
                    continue;
                keptHoles = append(keptHoles, hp);
                nodePts = append(nodePts, hp);
                nodeIsHole = append(nodeIsHole, true);
                nodeRadius = append(nodeRadius, dHoleRadii[hi]);
            }

            // OPTIONAL gap fill (default OFF). LEARNING: the reference plate is a
            // HOLE-TO-HOLE triangulation -- the vertices ARE the holes, giving
            // coarse organic triangles. A background lattice fill turns it into a
            // fine uniform mesh that does NOT look like the reference, so this is
            // only enabled when explicitly requested.
            if (definition.gapFill)
            {
            const rowH = s * sqrt(3) / 2;
            var lbMinX = poly[0][0]; var lbMaxX = poly[0][0];
            var lbMinY = poly[0][1]; var lbMaxY = poly[0][1];
            for (var p in poly)
            {
                lbMinX = min(lbMinX, p[0]); lbMaxX = max(lbMaxX, p[0]);
                lbMinY = min(lbMinY, p[1]); lbMaxY = max(lbMaxY, p[1]);
            }
            var lrow = 0;
            var ly = lbMinY;
            while (ly <= lbMaxY)
            {
                const lxoff = (lrow % 2 == 0) ? 0 : (s / 2);
                var lx = lbMinX + lxoff;
                while (lx <= lbMaxX)
                {
                    const lq = [lx, ly];
                    if (inMaterial(poly, innerLoops, lq)
                            && distToPolygon(poly, lq) >= marginMm
                            && distToInners(innerLoops, lq) >= slotMarginMm)
                    {
                        var clear = true;
                        for (var kp in keptHoles)
                        {
                            if (pointDistance(lq, kp) < 0.85 * s) { clear = false; break; }
                        }
                        if (clear)
                        {
                            nodePts = append(nodePts, lq);
                            nodeIsHole = append(nodeIsHole, false);
                            nodeRadius = append(nodeRadius, 0);
                        }
                    }
                    lx += s;
                }
                ly += rowH;
                lrow += 1;
            }
            }
        }
        else
        {
        // ----- 4. Full regular equilateral lattice inside the material --------
        var bMinX = poly[0][0]; var bMaxX = poly[0][0];
        var bMinY = poly[0][1]; var bMaxY = poly[0][1];
        for (var p in poly)
        {
            bMinX = min(bMinX, p[0]); bMaxX = max(bMaxX, p[0]);
            bMinY = min(bMinY, p[1]); bMaxY = max(bMaxY, p[1]);
        }
        const rowH = s * sqrt(3) / 2;

        var gridPts = [];
        var row = 0;
        var y = bMinY;
        while (y <= bMaxY)
        {
            const xoff = (row % 2 == 0) ? 0 : (s / 2);
            var x = bMinX + xoff;
            while (x <= bMaxX)
            {
                const q = [x, y];
                if (inMaterial(poly, innerLoops, q)
                        && distToPolygon(poly, q) >= marginMm
                        && distToInners(innerLoops, q) >= slotMarginMm)
                    gridPts = append(gridPts, q);
                x += s;
            }
            y += rowH;
            row += 1;
        }

        // ----- 5. Snap well-spaced holes; collect the rest as "floating" ------
        var gridIsHole = [];
        var gridRadius = [];
        for (var i = 0; i < size(gridPts); i += 1)
        {
            gridIsHole = append(gridIsHole, false);
            gridRadius = append(gridRadius, 0);
        }
        floatPts = [];
        floatRadii = [];
        if (definition.snapToHoles && !definition.equalSize && !definition.equilateral)
        {
            const snapMax = 1.3 * s;
            const dropDist = 0.7 * s;

            var order = [];
            for (var i = 0; i < size(dHolePts); i += 1)
                order = append(order, [i, dHoleRadii[i]]);
            order = sortByValueDesc(order);

            var keptHolePos = [];
            for (var oi = 0; oi < size(order); oi += 1)
            {
                const hi = order[oi][0];
                const hp = dHolePts[hi];

                var skip = false;
                for (var kp in keptHolePos)
                {
                    if (pointDistance(hp, kp) < dropDist) { skip = true; break; }
                }
                if (skip)
                {
                    floatPts = append(floatPts, hp);
                    floatRadii = append(floatRadii, dHoleRadii[hi]);
                    continue;
                }

                var bestG = -1;
                var bestD = snapMax;
                for (var g = 0; g < size(gridPts); g += 1)
                {
                    if (gridIsHole[g])
                        continue;
                    const d = pointDistance(hp, gridPts[g]);
                    if (d < bestD) { bestD = d; bestG = g; }
                }
                if (bestG >= 0)
                {
                    gridPts[bestG] = hp;
                    gridIsHole[bestG] = true;
                    gridRadius[bestG] = dHoleRadii[hi];
                    keptHolePos = append(keptHolePos, hp);
                }
                else
                {
                    floatPts = append(floatPts, hp);
                    floatRadii = append(floatRadii, dHoleRadii[hi]);
                }
            }
        }

        // ----- 6. Drop free lattice nodes too close to a snapped hole ---------
        var holePos = [];
        for (var g = 0; g < size(gridPts); g += 1)
            if (gridIsHole[g])
                holePos = append(holePos, gridPts[g]);
        const sliver = 0.45 * s;

        nodePts = [];
        nodeIsHole = [];
        nodeRadius = [];
        for (var g = 0; g < size(gridPts); g += 1)
        {
            if (gridIsHole[g])
            {
                nodePts = append(nodePts, gridPts[g]);
                nodeIsHole = append(nodeIsHole, true);
                nodeRadius = append(nodeRadius, gridRadius[g]);
            }
            else
            {
                var tooClose = false;
                for (var hp in holePos)
                {
                    if (pointDistance(gridPts[g], hp) < sliver) { tooClose = true; break; }
                }
                if (!tooClose)
                {
                    nodePts = append(nodePts, gridPts[g]);
                    nodeIsHole = append(nodeIsHole, false);
                    nodeRadius = append(nodeRadius, 0);
                }
            }
        }
        }

        // ----- 7. Sample the outer rim at ~s spacing --------------------------
        var rimPts = [];
        var acc = s;
        for (var i = 0; i < size(poly); i += 1)
        {
            const a = poly[i];
            const b = poly[(i + 1) % size(poly)];
            acc += pointDistance(a, b);
            if (acc >= s) { rimPts = append(rimPts, a); acc = 0; }
        }
        rimPts = dedupePoints(rimPts, 0.4 * s);

        const nNodes = size(nodePts);
        const pts = concatenateArrays([nodePts, rimPts]);
        if (size(pts) < 3)
            throw regenError("Plate too small for this triangle size; reduce 'Triangle side length'.", ["cellSize"]);

        // ----- 8. Triangulate, clipping to the material -----------------------
        const triangles = bowyerWatson(pts);
        const maxEdge = 2.8 * s;   // TUNING: span sparse hole regions for full-plate coverage (ribs crossing big holes are dropped by the in-material midpoint test)
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
                if (!inMaterial(poly, innerLoops, mid))
                    continue;
                keepSet[a ~ "_" ~ b] = [a, b];
            }
        }

        // ----- 9. Support struts for floating holes ---------------------------
        // Each floating hole connects to its nearest neighbour(s) -- vertices OR
        // other floating holes -- within a short range, so rows of holes chain
        // together and anchor to the lattice (like hand-drawn support struts).
        const drawPts = concatenateArrays([pts, floatPts]);
        const maxConnect = 1.35 * s;
        for (var f = 0; f < size(floatPts); f += 1)
        {
            const fi = size(pts) + f;
            const fp = floatPts[f];

            var n1 = -1;
            var d1 = maxConnect;
            for (var k = 0; k < size(drawPts); k += 1)
            {
                if (k == fi)
                    continue;
                const d = pointDistance(fp, drawPts[k]);
                if (d < d1) { d1 = d; n1 = k; }
            }
            if (n1 < 0)
                continue;
            keepSet[min(fi, n1) ~ "_" ~ max(fi, n1)] = [min(fi, n1), max(fi, n1)];

            // a second strut on the far side, so support comes from both sides
            const v1x = drawPts[n1][0] - fp[0];
            const v1y = drawPts[n1][1] - fp[1];
            var n2 = -1;
            var d2 = maxConnect;
            for (var k = 0; k < size(drawPts); k += 1)
            {
                if (k == fi || k == n1)
                    continue;
                const wx = drawPts[k][0] - fp[0];
                const wy = drawPts[k][1] - fp[1];
                if (v1x * wx + v1y * wy < 0)
                {
                    const d = pointDistance(fp, drawPts[k]);
                    if (d < d2) { d2 = d; n2 = k; }
                }
            }
            if (n2 >= 0)
                keepSet[min(fi, n2) ~ "_" ~ max(fi, n2)] = [min(fi, n2), max(fi, n2)];
        }

        // ----- 9b. Any hole with <= 2 struts gets a 3rd on its empty side -----
        const totalDraw = size(drawPts);
        var deg = [];
        var sumx = [];
        var sumy = [];
        for (var i = 0; i < totalDraw; i += 1)
        {
            deg = append(deg, 0);
            sumx = append(sumx, 0);
            sumy = append(sumy, 0);
        }
        for (var key in keys(keepSet))
        {
            const e = keepSet[key];
            const a = e[0];
            const b = e[1];
            const dxab = drawPts[b][0] - drawPts[a][0];
            const dyab = drawPts[b][1] - drawPts[a][1];
            const L = sqrt(dxab * dxab + dyab * dyab);
            if (L < 0.001)
                continue;
            deg[a] += 1; deg[b] += 1;
            sumx[a] += dxab / L; sumy[a] += dyab / L;       // unit strut directions
            sumx[b] += -dxab / L; sumy[b] += -dyab / L;
        }

        // Prune the sharper of any two ribs that meet closer than the min angle.
        const minAngleRad = definition.minAngle / radian;
        var pruned = {};
        for (var v = 0; v < totalDraw; v += 1)
        {
            var inc = [];
            for (var key in keys(keepSet))
            {
                if (pruned[key] != undefined)
                    continue;
                const e = keepSet[key];
                var other = -1;
                if (e[0] == v)
                    other = e[1];
                else if (e[1] == v)
                    other = e[0];
                else
                    continue;
                const dx = drawPts[other][0] - drawPts[v][0];
                const dy = drawPts[other][1] - drawPts[v][1];
                const len = sqrt(dx * dx + dy * dy);
                if (len < 0.001)
                    continue;
                inc = append(inc, [other, atan2(dy, dx) / radian, len, key, dx / len, dy / len]);
            }
            if (size(inc) < 2)
                continue;
            inc = sortByAngleAsc(inc);
            for (var i = 0; i < size(inc); i += 1)
            {
                const j = (i + 1) % size(inc);
                if (pruned[inc[i][3]] != undefined || pruned[inc[j][3]] != undefined)
                    continue;
                var gap = inc[j][1] - inc[i][1];
                if (j == 0)
                    gap = gap + 2 * PI;
                if (gap >= minAngleRad)
                    continue;
                const rm = (inc[i][2] >= inc[j][2]) ? inc[i] : inc[j];     // drop the longer rib
                const rmOther = rm[0];
                if (deg[v] - 1 < 2 || deg[rmOther] - 1 < 2)
                    continue;                                              // keep the network connected
                pruned[rm[3]] = true;
                deg[v] -= 1; deg[rmOther] -= 1;
                sumx[v] -= rm[4]; sumy[v] -= rm[5];
                sumx[rmOther] += rm[4]; sumy[rmOther] += rm[5];
            }
        }

        var holeIdx = [];
        for (var i = 0; i < nNodes; i += 1)
            if (nodeIsHole[i])
                holeIdx = append(holeIdx, i);
        for (var f = 0; f < size(floatPts); f += 1)
            holeIdx = append(holeIdx, size(pts) + f);

        const reach = 1.6 * s;
        for (var hidx in holeIdx)
        {
            if (deg[hidx] >= 3)
                continue;
            const hp = drawPts[hidx];
            // empty side = opposite the resultant of existing struts
            var ex = -sumx[hidx];
            var ey = -sumy[hidx];
            const emag = sqrt(ex * ex + ey * ey);
            const useDir = emag > 0.1;
            if (useDir) { ex = ex / emag; ey = ey / emag; }

            var bn = -1;
            var bd = reach;
            for (var k = 0; k < totalDraw; k += 1)
            {
                if (k == hidx)
                    continue;
                const akey = min(hidx, k) ~ "_" ~ max(hidx, k);
                if (keepSet[akey] != undefined)
                    continue;
                const wx = drawPts[k][0] - hp[0];
                const wy = drawPts[k][1] - hp[1];
                if (useDir && (ex * wx + ey * wy) <= 0)
                    continue;               // only the empty side
                const mid = [(hp[0] + drawPts[k][0]) / 2, (hp[1] + drawPts[k][1]) / 2];
                if (!inMaterial(poly, innerLoops, mid))
                    continue;
                const d = sqrt(wx * wx + wy * wy);
                if (d < bd) { bd = d; bn = k; }
            }
            if (bn >= 0)
            {
                keepSet[min(hidx, bn) ~ "_" ~ max(hidx, bn)] = [min(hidx, bn), max(hidx, bn)];
                deg[hidx] += 1;
                deg[bn] += 1;
            }
        }

        var keptEdges = [];
        for (var key in keys(keepSet))
            if (pruned[key] == undefined)
                keptEdges = append(keptEdges, keepSet[key]);

        // ===== Geometric cleanup of the rib network ===========================
        const mergeDistMm = definition.mergeDist / millimeter;
        const ribEdgeMm = definition.ribEdgeMargin / millimeter;
        const rerouteMm = definition.rerouteRadius / millimeter;

        // Classify each node: "outer" (near plate boundary) and "hole".
        var outerNode = [];
        var holeNode = [];
        for (var i = 0; i < totalDraw; i += 1)
        {
            outerNode = append(outerNode, distToPolygon(poly, drawPts[i]) <= ribEdgeMm);
            var h = false;
            if (i < nNodes) h = nodeIsHole[i];
            else if (i >= size(pts)) h = true;        // floating holes live past pts
            holeNode = append(holeNode, h);
        }

        // Working edge map + removed-set + live degree.
        var E = {};
        for (var e in keptEdges)
            E[e[0] ~ "_" ~ e[1]] = [e[0], e[1]];
        var gone = {};
        var dg = [];
        for (var i = 0; i < totalDraw; i += 1)
            dg = append(dg, 0);
        for (var key in keys(E)) { const e = E[key]; dg[e[0]] += 1; dg[e[1]] += 1; }

        // Rule 3 + 4: drop edge-hugging ribs and outer-to-outer ribs.
        for (var key in keys(E))
        {
            const e = E[key];
            const a = e[0]; const b = e[1];
            var drop = (outerNode[a] && outerNode[b]);            // rule 4
            if (!drop)
            {
                var maxd = 0;
                for (var sgmt = 0; sgmt <= 8; sgmt += 1)
                {
                    const t = sgmt / 8;
                    const px = drawPts[a][0] + t * (drawPts[b][0] - drawPts[a][0]);
                    const py = drawPts[a][1] + t * (drawPts[b][1] - drawPts[a][1]);
                    const dd = distToPolygon(poly, [px, py]);
                    if (dd > maxd) maxd = dd;
                }
                if (maxd <= ribEdgeMm) drop = true;               // rule 3 (whole rib hugs edge)
            }
            if (drop && dg[a] > 1 && dg[b] > 1)                   // keep every node connected
            {
                gone[key] = true;
                dg[a] -= 1; dg[b] -= 1;
            }
        }

        // Rule 2: merge near-parallel ribs that run very close (drop the shorter).
        var ekeys = [];
        for (var key in keys(E))
            if (gone[key] == undefined) ekeys = append(ekeys, key);
        for (var i = 0; i < size(ekeys); i += 1)
        {
            if (gone[ekeys[i]] != undefined) continue;
            const e1 = E[ekeys[i]];
            const a1 = drawPts[e1[0]]; const b1 = drawPts[e1[1]];
            const l1 = pointDistance(a1, b1);
            if (l1 < 0.001) continue;
            const u1x = (b1[0] - a1[0]) / l1; const u1y = (b1[1] - a1[1]) / l1;
            for (var j = i + 1; j < size(ekeys); j += 1)
            {
                if (gone[ekeys[j]] != undefined) continue;
                const e2 = E[ekeys[j]];
                if (e1[0] == e2[0] || e1[0] == e2[1] || e1[1] == e2[0] || e1[1] == e2[1]) continue;
                const a2 = drawPts[e2[0]]; const b2 = drawPts[e2[1]];
                const l2 = pointDistance(a2, b2);
                if (l2 < 0.001) continue;
                const u2x = (b2[0] - a2[0]) / l2; const u2y = (b2[1] - a2[1]) / l2;
                if (abs(u1x * u2x + u1y * u2y) < 0.966) continue;          // not near-parallel (~15 deg)
                if (segMinDist(a1, b1, a2, b2) > mergeDistMm) continue;    // not close
                const rmk = (l1 >= l2) ? ekeys[j] : ekeys[i];             // drop the shorter
                const rme = E[rmk];
                if (dg[rme[0]] > 1 && dg[rme[1]] > 1)
                {
                    gone[rmk] = true;
                    dg[rme[0]] -= 1; dg[rme[1]] -= 1;
                    if (rmk == ekeys[i]) break;
                }
            }
        }

        // Rule 1: resolve rib-on-rib crossings -> reroute longer rib to nearest
        // valid node (holes first), else drop it. Iterate until clean or capped.
        var crossPass = 0;
        var keepFixing = true;
        while (keepFixing && crossPass < 8)
        {
            keepFixing = false;
            crossPass += 1;
            var live = [];
            for (var key in keys(E))
                if (gone[key] == undefined) live = append(live, [E[key][0], E[key][1], key]);
            for (var i = 0; i < size(live); i += 1)
            {
                if (keepFixing) break;
                for (var j = i + 1; j < size(live); j += 1)
                {
                    const e1 = live[i]; const e2 = live[j];
                    if (e1[0] == e2[0] || e1[0] == e2[1] || e1[1] == e2[0] || e1[1] == e2[1]) continue;
                    if (!segmentsCross(drawPts[e1[0]], drawPts[e1[1]], drawPts[e2[0]], drawPts[e2[1]])) continue;
                    const len1 = pointDistance(drawPts[e1[0]], drawPts[e1[1]]);
                    const len2 = pointDistance(drawPts[e2[0]], drawPts[e2[1]]);
                    const rr = (len1 >= len2) ? e1 : e2;
                    const keepEnd = rr[0];
                    const oldKey = rr[2];
                    var bestT = -1;
                    var bestD = rerouteMm;
                    for (var phase = 0; phase < 2; phase += 1)        // 0: holes only, 1: any node
                    {
                        for (var k = 0; k < totalDraw; k += 1)
                        {
                            if (k == keepEnd) continue;
                            if (phase == 0 && !holeNode[k]) continue;
                            const ck = min(keepEnd, k) ~ "_" ~ max(keepEnd, k);
                            if (E[ck] != undefined && gone[ck] == undefined) continue;
                            const dcand = pointDistance(drawPts[keepEnd], drawPts[k]);
                            if (dcand >= bestD || dcand < 0.001) continue;
                            const midc = [(drawPts[keepEnd][0] + drawPts[k][0]) / 2, (drawPts[keepEnd][1] + drawPts[k][1]) / 2];
                            if (!inMaterial(poly, innerLoops, midc)) continue;
                            var crosses = false;
                            for (var m = 0; m < size(live); m += 1)
                            {
                                const le = live[m];
                                if (le[2] == oldKey) continue;
                                if (le[0] == keepEnd || le[1] == keepEnd || le[0] == k || le[1] == k) continue;
                                if (segmentsCross(drawPts[keepEnd], drawPts[k], drawPts[le[0]], drawPts[le[1]])) { crosses = true; break; }
                            }
                            if (crosses) continue;
                            bestD = dcand; bestT = k;
                        }
                        if (bestT >= 0) break;
                    }
                    gone[oldKey] = true;
                    if (bestT >= 0)
                        E[min(keepEnd, bestT) ~ "_" ~ max(keepEnd, bestT)] = [min(keepEnd, bestT), max(keepEnd, bestT)];
                    keepFixing = true;
                    break;
                }
            }
        }

        keptEdges = [];
        for (var key in keys(E))
            if (gone[key] == undefined)
                keptEdges = append(keptEdges, E[key]);

        // Test mode: force every pocket to a uniform triangle by using only the
        // raw Delaunay triangulation (skips support struts, merges, reroutes).
        if (definition.forceTriangles || definition.equalSize)
        {
            var triSet = {};
            for (var tri in triangles)
            {
                const tE = [[tri[0], tri[1]], [tri[1], tri[2]], [tri[2], tri[0]]];
                for (var e in tE)
                {
                    const a = min(e[0], e[1]);
                    const b = max(e[0], e[1]);
                    const pa = pts[a]; const pb = pts[b];
                    const L = pointDistance(pa, pb);
                    if (L < 0.001 || L > maxEdge) continue;
                    if (outerNode[a] && outerNode[b]) continue;   // skip ribs lying on the plate edge
                    const mid = [(pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2];
                    if (!inMaterial(poly, innerLoops, mid)) continue;
                    triSet[a ~ "_" ~ b] = [a, b];
                }
            }
            keptEdges = [];
            for (var key in keys(triSet))
                keptEdges = append(keptEdges, triSet[key]);
        }

        // Equilateral test: connect only exact grid-neighbour pairs (60 deg).
        if (definition.equilateral)
        {
            var eqSet = {};
            for (var i = 0; i < nNodes; i += 1)
            {
                for (var j = i + 1; j < nNodes; j += 1)
                {
                    const L = pointDistance(nodePts[i], nodePts[j]);
                    if (L < 0.55 * s || L > 1.15 * s) continue;     // the 6 lattice neighbours (~s)
                    if (outerNode[i] && outerNode[j]) continue;
                    const mid = [(nodePts[i][0] + nodePts[j][0]) / 2, (nodePts[i][1] + nodePts[j][1]) / 2];
                    if (!inMaterial(poly, innerLoops, mid)) continue;
                    eqSet[i ~ "_" ~ j] = [i, j];
                }
            }
            keptEdges = [];
            for (var key in keys(eqSet))
                keptEdges = append(keptEdges, eqSet[key]);
        }

        // ----- 10. Draw rib centerlines ---------------------------------------
        const ribSketch = newSketchOnPlane(context, id + "ribs", { "sketchPlane" : plane });
        var ribIndex = 0;
        for (var e in keptEdges)
        {
            const pa = drawPts[e[0]];
            const pb = drawPts[e[1]];
            skLineSegment(ribSketch, "rib" ~ ribIndex, {
                        "start" : vector(pa[0] * millimeter, pa[1] * millimeter),
                        "end"   : vector(pb[0] * millimeter, pb[1] * millimeter)
                    });
            ribIndex += 1;
        }

        if (definition.drawHoleCircles)
        {
            for (var i = 0; i < nNodes; i += 1)
            {
                if (nodeIsHole[i])
                {
                    skCircle(ribSketch, "holeRef" ~ i, {
                                "center" : vector(nodePts[i][0] * millimeter, nodePts[i][1] * millimeter),
                                "radius" : nodeRadius[i] * millimeter
                            });
                }
            }
        }

        skSolve(ribSketch);

        // ----- 11. Optional pocket profiles -----------------------------------
        if (definition.generatePockets)
        {
            const pocketSketch = newSketchOnPlane(context, id + "pockets", { "sketchPlane" : plane });
            const halfW = (definition.ribWidth / millimeter) / 2;
            const bossExtra = definition.bossOffset / millimeter;

            for (var i = 0; i < nNodes; i += 1)
            {
                if (nodeIsHole[i])
                {
                    skCircle(pocketSketch, "boss" ~ i, {
                                "center" : vector(nodePts[i][0] * millimeter, nodePts[i][1] * millimeter),
                                "radius" : (nodeRadius[i] + bossExtra) * millimeter
                            });
                }
            }
            for (var f = 0; f < size(floatPts); f += 1)
            {
                skCircle(pocketSketch, "fboss" ~ f, {
                            "center" : vector(floatPts[f][0] * millimeter, floatPts[f][1] * millimeter),
                            "radius" : (floatRadii[f] + bossExtra) * millimeter
                        });
            }

            var ofs = 0;
            for (var e in keptEdges)
            {
                const pa = drawPts[e[0]];
                const pb = drawPts[e[1]];
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

        // ----- 11b. Auto-cut pockets (inset each triangle, remove) ------------
        // Each in-material Delaunay triangle is a pocket. Insetting every edge by
        // half the rib width leaves a full rib (wall) between neighbouring
        // pockets; extrude-remove the insets through the plate.
        if (definition.autoCut)
        {
            const d = (definition.ribWidth / millimeter) / 2;
            const cutSketch = newSketchOnPlane(context, id + "pocketcut", { "sketchPlane" : plane });
            var pcount = 0;
            for (var tri in triangles)
            {
                const A = pts[tri[0]];
                const B = pts[tri[1]];
                const C = pts[tri[2]];
                const eAB = pointDistance(A, B);
                const eBC = pointDistance(B, C);
                const eCA = pointDistance(C, A);
                if (eAB > maxEdge || eBC > maxEdge || eCA > maxEdge)
                    continue;
                const mAB = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2];
                const mBC = [(B[0] + C[0]) / 2, (B[1] + C[1]) / 2];
                const mCA = [(C[0] + A[0]) / 2, (C[1] + A[1]) / 2];
                const cen = [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3];
                if (!inMaterial(poly, innerLoops, cen)
                        || !inMaterial(poly, innerLoops, mAB)
                        || !inMaterial(poly, innerLoops, mBC)
                        || !inMaterial(poly, innerLoops, mCA))
                    continue;
                const per = eAB + eBC + eCA;
                if (per < 0.001)
                    continue;
                const Ix = (eBC * A[0] + eCA * B[0] + eAB * C[0]) / per;
                const Iy = (eBC * A[1] + eCA * B[1] + eAB * C[1]) / per;
                const area2 = abs((B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]));
                const inrad = area2 / per;                  // = 2*area / perimeter
                if (inrad <= d + 0.4)
                    continue;                               // too small to leave a pocket
                const f = 1 - d / inrad;
                const V = [[Ix + (A[0] - Ix) * f, Iy + (A[1] - Iy) * f],
                           [Ix + (B[0] - Ix) * f, Iy + (B[1] - Iy) * f],
                           [Ix + (C[0] - Ix) * f, Iy + (C[1] - Iy) * f]];
                for (var k = 0; k < 3; k += 1)
                {
                    const p1 = V[k];
                    const p2 = V[(k + 1) % 3];
                    skLineSegment(cutSketch, "pc" ~ pcount ~ "_" ~ k, {
                                "start" : vector(p1[0] * millimeter, p1[1] * millimeter),
                                "end"   : vector(p2[0] * millimeter, p2[1] * millimeter)
                            });
                }
                pcount += 1;
            }
            skSolve(cutSketch);
            const cutRegions = qSketchRegion(id + "pocketcut");
            if (pcount > 0 && size(evaluateQuery(context, cutRegions)) > 0)
            {
                opExtrude(context, id + "cutExtrude", {
                            "entities" : cutRegions,
                            "direction" : plane.normal,
                            "endBound" : BoundingType.BLIND,
                            "endDepth" : 50 * millimeter,
                            "startBound" : BoundingType.BLIND,
                            "startDepth" : 50 * millimeter,
                            "operationType" : NewBodyOperationType.NEW
                        });
                opBoolean(context, id + "cutBoolean", {
                            "tools" : qCreatedBy(id + "cutExtrude", EntityType.BODY),
                            "targets" : qOwnerBody(definition.face),
                            "operationType" : BooleanOperationType.SUBTRACTION
                        });
            }
        }

        reportFeatureInfo(context, id, size(keptEdges) ~ " struts, " ~ size(floatPts) ~ " floating holes supported.");
    });

// ===== Helper functions ====================================================

function pointDistance(a is array, b is array) returns number
{
    const dx = a[0] - b[0];
    const dy = a[1] - b[1];
    return sqrt(dx * dx + dy * dy);
}

function sortByValueDesc(arr is array) returns array
{
    var a = arr;
    for (var i = 0; i < size(a); i += 1)
    {
        var mi = i;
        for (var j = i + 1; j < size(a); j += 1)
        {
            if (a[j][1] > a[mi][1])
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

function sortByAngleAsc(arr is array) returns array
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

function inMaterial(outer is array, inners is array, p is array) returns boolean
{
    if (!pointInPolygon(outer, p))
        return false;
    for (var inr in inners)
    {
        if (pointInPolygon(inr, p))
            return false;
    }
    return true;
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

function distToInners(inners is array, p is array) returns number
{
    var best = 1e18;
    for (var inr in inners)
        best = min(best, distToPolygon(inr, p));
    return best;
}

function ccw2(a is array, b is array, c is array) returns number
{
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

// Strict 2D segment crossing; shared endpoints (collinear touch) do not count.
function segmentsCross(p1 is array, p2 is array, p3 is array, p4 is array) returns boolean
{
    const d1 = ccw2(p3, p4, p1);
    const d2 = ccw2(p3, p4, p2);
    const d3 = ccw2(p1, p2, p3);
    const d4 = ccw2(p1, p2, p4);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
           ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

function segMinDist(a is array, b is array, c is array, d is array) returns number
{
    if (segmentsCross(a, b, c, d))
        return 0;
    return min(min(distPointToSegment(a, c, d), distPointToSegment(b, c, d)),
               min(distPointToSegment(c, a, b), distPointToSegment(d, a, b)));
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

function buildLoops(polylines is array) returns array
{
    const tol = 0.5;
    var used = [];
    for (var i = 0; i < size(polylines); i += 1)
        used = append(used, false);

    var loops = [];
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
        loops = append(loops, loop);
    }
    return loops;
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
