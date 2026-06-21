/* ============================================================================
 * dxf.js  —  Minimal DXF reader for plate geometry.
 *
 * Extracts:
 *   - CIRCLE        -> holes {x,y,r}
 *   - LWPOLYLINE    -> closed loops (outline / inner cutouts)
 *   - LINE / ARC    -> segments, chained into loops
 *   - POLYLINE/VERTEX (old style) -> loops
 *
 * Returns { outline:[[x,y]...], inners:[[[x,y]...]...], holes:[{x,y,r}...],
 *           bbox:{minX,minY,maxX,maxY}, warnings:[...] }
 * Units are whatever the DXF uses (assumed mm; UI can scale).
 * ==========================================================================*/

function dxfParse(text) {
    // tokenize into [code(int), value(string)] pairs
    var lines = text.split(/\r\n|\r|\n/);
    var pairs = [];
    for (var i = 0; i + 1 < lines.length; i += 2) {
        var code = parseInt(lines[i].trim(), 10);
        var val = lines[i + 1];
        if (!isNaN(code)) pairs.push([code, val]);
    }

    var holes = [];
    var segments = [];   // {a:[x,y], b:[x,y]}
    var closedLoops = []; // [[x,y],...]
    var warnings = [];

    var idx = 0;
    function findEntities() {
        for (var p = 0; p < pairs.length; p += 1) {
            if (pairs[p][0] === 2 && pairs[p][1].trim() === "ENTITIES") return p + 1;
        }
        return 0;
    }
    idx = findEntities();

    function sampleArc(cx, cy, r, a0deg, a1deg) {
        var a0 = a0deg * Math.PI / 180, a1 = a1deg * Math.PI / 180;
        var da = a1 - a0;
        while (da < 0) da += 2 * Math.PI;
        var n = Math.max(2, Math.ceil(da / (Math.PI / 18))); // ~10deg steps
        var pts = [];
        for (var k = 0; k <= n; k += 1) {
            var a = a0 + da * (k / n);
            pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]);
        }
        return pts;
    }

    var p = idx;
    while (p < pairs.length) {
        if (pairs[p][0] !== 0) { p += 1; continue; }
        var ent = pairs[p][1].trim();
        p += 1;
        // collect this entity's group codes until next 0
        var g = {};         // single-valued
        var xs = [], ys = []; // for LWPOLYLINE / POLYLINE vertices
        var bulge = [];
        var closed = false;
        while (p < pairs.length && pairs[p][0] !== 0) {
            var c = pairs[p][0], v = pairs[p][1];
            if (ent === "LWPOLYLINE") {
                if (c === 10) xs.push(parseFloat(v));
                else if (c === 20) ys.push(parseFloat(v));
                else if (c === 70) closed = (parseInt(v, 10) & 1) === 1;
            } else {
                if (c === 10) g.x10 = parseFloat(v);
                else if (c === 20) g.y20 = parseFloat(v);
                else if (c === 11) g.x11 = parseFloat(v);
                else if (c === 21) g.y21 = parseFloat(v);
                else if (c === 40) g.r40 = parseFloat(v);
                else if (c === 50) g.a50 = parseFloat(v);
                else if (c === 51) g.a51 = parseFloat(v);
                else if (c === 70) g.f70 = parseInt(v, 10);
            }
            p += 1;
        }

        if (ent === "CIRCLE") {
            holes.push({ x: g.x10, y: g.y20, r: g.r40 });
        } else if (ent === "LINE") {
            segments.push({ a: [g.x10, g.y20], b: [g.x11, g.y21] });
        } else if (ent === "ARC") {
            var arcPts = sampleArc(g.x10, g.y20, g.r40, g.a50, g.a51);
            for (var ai = 0; ai + 1 < arcPts.length; ai += 1) {
                segments.push({ a: arcPts[ai], b: arcPts[ai + 1] });
            }
        } else if (ent === "LWPOLYLINE") {
            var loop = [];
            for (var vi = 0; vi < xs.length; vi += 1) loop.push([xs[vi], ys[vi]]);
            if (loop.length >= 3) {
                if (closed) closedLoops.push(loop);
                else {
                    for (var li = 0; li + 1 < loop.length; li += 1)
                        segments.push({ a: loop[li], b: loop[li + 1] });
                }
            }
        }
    }

    // chain free segments into loops
    var chained = dxfChainLoops(segments);
    var allLoops = closedLoops.concat(chained);

    if (allLoops.length === 0) {
        warnings.push("No closed boundary found in DXF (need a closed outline polyline or chained lines).");
    }

    // pick outline = largest area loop; the rest = inner cutouts
    var outline = [];
    var inners = [];
    var maxA = -1, outIdx = -1;
    for (var i2 = 0; i2 < allLoops.length; i2 += 1) {
        var ar = dxfArea(allLoops[i2]);
        if (ar > maxA) { maxA = ar; outIdx = i2; }
    }
    if (outIdx >= 0) {
        outline = allLoops[outIdx];
        for (var i3 = 0; i3 < allLoops.length; i3 += 1) {
            if (i3 === outIdx) continue;
            if (dxfArea(allLoops[i3]) > 25) inners.push(allLoops[i3]); // ignore tiny
        }
    }

    var bbox = dxfBBox(outline.length ? outline : (holes.length ? holes.map(function (h) { return [h.x, h.y]; }) : [[0, 0]]));
    return { outline: outline, inners: inners, holes: holes, bbox: bbox, warnings: warnings };
}

function dxfArea(poly) {
    var a = 0, n = poly.length;
    for (var i = 0; i < n; i += 1) { var j = (i + 1) % n; a += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1]; }
    return Math.abs(a) / 2;
}

function dxfBBox(pts) {
    var minX = pts[0][0], minY = pts[0][1], maxX = pts[0][0], maxY = pts[0][1];
    for (var i = 1; i < pts.length; i += 1) {
        if (pts[i][0] < minX) minX = pts[i][0];
        if (pts[i][1] < minY) minY = pts[i][1];
        if (pts[i][0] > maxX) maxX = pts[i][0];
        if (pts[i][1] > maxY) maxY = pts[i][1];
    }
    return { minX: minX, minY: minY, maxX: maxX, maxY: maxY };
}

function dxfChainLoops(segments) {
    var tol = 0.05;
    var used = [];
    for (var i = 0; i < segments.length; i += 1) used.push(false);
    function near(a, b) { var dx = a[0] - b[0], dy = a[1] - b[1]; return dx * dx + dy * dy < tol * tol; }
    var loops = [];
    for (var s = 0; s < segments.length; s += 1) {
        if (used[s]) continue;
        used[s] = true;
        var loop = [segments[s].a, segments[s].b];
        var advanced = true;
        while (advanced) {
            advanced = false;
            var endP = loop[loop.length - 1];
            for (var t = 0; t < segments.length; t += 1) {
                if (used[t]) continue;
                if (near(segments[t].a, endP)) { loop.push(segments[t].b); used[t] = true; advanced = true; break; }
                if (near(segments[t].b, endP)) { loop.push(segments[t].a); used[t] = true; advanced = true; break; }
            }
        }
        // closed if last meets first
        if (loop.length >= 4 && near(loop[0], loop[loop.length - 1])) {
            loop.pop();
            loops.push(loop);
        }
    }
    return loops;
}

if (typeof window !== "undefined") { window.dxfParse = dxfParse; }
