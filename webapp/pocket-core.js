/* ============================================================================
 * pocket-core.js  —  Pure 2D pocketing algorithm (FS-PORTABLE)
 *
 * Everything here is written so it maps almost line-for-line to Onshape
 * FeatureScript: plain arrays, numbers and objects-as-maps only, no external
 * libraries, deterministic, no closures/classes. Points are [x, y] in MILLIMETRES.
 *
 * Public entry point:  pcGenerate(plate, params) -> result
 *   plate  = { outline: [[x,y],...],            // outer boundary, CCW or CW
 *              inners:  [ [[x,y],...], ... ],     // inner cutouts (slots), may be []
 *              holes:   [ {x, y, r}, ... ] }      // circular holes (r = radius mm)
 *   params = { triangleSize, minHoleDia, declusterDist, maxEdgeFactor,
 *              ribWidth, pocketFillet, edgeMargin, gapFill }   (all mm / numbers)
 *   result = { vertices: [[x,y],...],            // triangulation vertices
 *              edges:    [[i,j],...],             // rib centrelines (indices)
 *              triangles:[[i,j,k],...],           // kept triangles (pockets)
 *              pockets:  [ [[x,y],...], ... ],    // inset (filleted) pocket loops
 *              info: {...} }
 * ==========================================================================*/

// ---- basic geometry -------------------------------------------------------

function pcDist(a, b) {
    var dx = a[0] - b[0];
    var dy = a[1] - b[1];
    return Math.sqrt(dx * dx + dy * dy);
}

function pcPolygonArea(poly) {
    var a = 0;
    var n = poly.length;
    for (var i = 0; i < n; i += 1) {
        var j = (i + 1) % n;
        a += poly[i][0] * poly[j][1] - poly[j][0] * poly[i][1];
    }
    return Math.abs(a) / 2;
}

function pcPointInPolygon(poly, p) {
    // ray casting
    var inside = false;
    var n = poly.length;
    for (var i = 0, j = n - 1; i < n; j = i, i += 1) {
        var xi = poly[i][0], yi = poly[i][1];
        var xj = poly[j][0], yj = poly[j][1];
        var intersect = ((yi > p[1]) !== (yj > p[1])) &&
            (p[0] < (xj - xi) * (p[1] - yi) / (yj - yi) + xi);
        if (intersect) inside = !inside;
    }
    return inside;
}

function pcInMaterial(poly, inners, p) {
    if (!pcPointInPolygon(poly, p)) return false;
    for (var k = 0; k < inners.length; k += 1) {
        if (pcPointInPolygon(inners[k], p)) return false;
    }
    return true;
}

function pcDistToSegment(p, a, b) {
    var vx = b[0] - a[0], vy = b[1] - a[1];
    var wx = p[0] - a[0], wy = p[1] - a[1];
    var c1 = vx * wx + vy * wy;
    if (c1 <= 0) return pcDist(p, a);
    var c2 = vx * vx + vy * vy;
    if (c2 <= c1) return pcDist(p, b);
    var t = c1 / c2;
    return pcDist(p, [a[0] + t * vx, a[1] + t * vy]);
}

function pcDistToPolygon(poly, p) {
    var best = 1e18;
    var n = poly.length;
    for (var i = 0; i < n; i += 1) {
        var d = pcDistToSegment(p, poly[i], poly[(i + 1) % n]);
        if (d < best) best = d;
    }
    return best;
}

function pcDistToInners(inners, p) {
    var best = 1e18;
    for (var k = 0; k < inners.length; k += 1) {
        var d = pcDistToPolygon(inners[k], p);
        if (d < best) best = d;
    }
    return best;
}

// descending sort of [index, value] pairs by value (selection sort = FS-friendly)
function pcSortByValueDesc(arr) {
    var a = arr.slice();
    for (var i = 0; i < a.length; i += 1) {
        var mi = i;
        for (var j = i + 1; j < a.length; j += 1) {
            if (a[j][1] > a[mi][1]) mi = j;
        }
        if (mi !== i) { var t = a[i]; a[i] = a[mi]; a[mi] = t; }
    }
    return a;
}

// ---- Delaunay (Bowyer-Watson), same approach as the .fs ------------------

function pcCircumcircleContains(ax, ay, bx, by, cx, cy, px, py) {
    var adx = ax - px, ady = ay - py;
    var bdx = bx - px, bdy = by - py;
    var cdx = cx - px, cdy = cy - py;
    var d = (adx * adx + ady * ady) * (bdx * cdy - cdx * bdy)
          - (bdx * bdx + bdy * bdy) * (adx * cdy - cdx * ady)
          + (cdx * cdx + cdy * cdy) * (adx * bdy - bdx * ady);
    // orientation of triangle a,b,c
    var ori = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
    if (ori > 0) return d > 0;
    return d < 0;
}

function pcBowyerWatson(pts) {
    var n = pts.length;
    if (n < 3) return [];
    // super-triangle
    var minX = pts[0][0], minY = pts[0][1], maxX = pts[0][0], maxY = pts[0][1];
    for (var i = 1; i < n; i += 1) {
        if (pts[i][0] < minX) minX = pts[i][0];
        if (pts[i][1] < minY) minY = pts[i][1];
        if (pts[i][0] > maxX) maxX = pts[i][0];
        if (pts[i][1] > maxY) maxY = pts[i][1];
    }
    var dx = maxX - minX, dy = maxY - minY;
    var dmax = (dx > dy ? dx : dy) * 20 + 100;
    var midx = (minX + maxX) / 2, midy = (minY + maxY) / 2;
    var work = pts.slice();
    work.push([midx - dmax, midy - dmax]);
    work.push([midx + dmax, midy - dmax]);
    work.push([midx, midy + dmax]);
    var s0 = n, s1 = n + 1, s2 = n + 2;

    var tris = [[s0, s1, s2]];

    for (var ip = 0; ip < n; ip += 1) {
        var px = work[ip][0], py = work[ip][1];
        var bad = [];
        for (var t = 0; t < tris.length; t += 1) {
            var tr = tris[t];
            if (pcCircumcircleContains(
                    work[tr[0]][0], work[tr[0]][1],
                    work[tr[1]][0], work[tr[1]][1],
                    work[tr[2]][0], work[tr[2]][1], px, py)) {
                bad.push(t);
            }
        }
        // boundary of polygonal hole
        var edges = [];
        for (var bi = 0; bi < bad.length; bi += 1) {
            var tr2 = tris[bad[bi]];
            var te = [[tr2[0], tr2[1]], [tr2[1], tr2[2]], [tr2[2], tr2[0]]];
            for (var e = 0; e < 3; e += 1) {
                var a = te[e][0], b = te[e][1];
                var shared = false;
                for (var bj = 0; bj < bad.length; bj += 1) {
                    if (bj === bi) continue;
                    var tr3 = tris[bad[bj]];
                    var t3 = [[tr3[0], tr3[1]], [tr3[1], tr3[2]], [tr3[2], tr3[0]]];
                    for (var f = 0; f < 3; f += 1) {
                        var c = t3[f][0], dd = t3[f][1];
                        if ((a === c && b === dd) || (a === dd && b === c)) { shared = true; }
                    }
                }
                if (!shared) edges.push([a, b]);
            }
        }
        // remove bad triangles (high index first)
        bad.sort(function (x, y) { return y - x; });
        for (var rb = 0; rb < bad.length; rb += 1) tris.splice(bad[rb], 1);
        // re-triangulate hole
        for (var ee = 0; ee < edges.length; ee += 1) {
            tris.push([edges[ee][0], edges[ee][1], ip]);
        }
    }
    // drop triangles touching super-triangle
    var out = [];
    for (var to = 0; to < tris.length; to += 1) {
        var tt = tris[to];
        if (tt[0] >= s0 || tt[1] >= s0 || tt[2] >= s0) continue;
        out.push([tt[0], tt[1], tt[2]]);
    }
    return out;
}

// ---- rim sampling ---------------------------------------------------------

function pcDedupePoints(arr, tol) {
    var out = [];
    for (var i = 0; i < arr.length; i += 1) {
        var dup = false;
        for (var j = 0; j < out.length; j += 1) {
            if (pcDist(arr[i], out[j]) < tol) { dup = true; break; }
        }
        if (!dup) out.push(arr[i]);
    }
    return out;
}

// ---- inset + fillet a triangle into a pocket loop -------------------------
// Returns a list of points approximating the rounded inset triangle, or null
// if it is too small. Fillet arcs are flattened into short segments (the web
// preview uses these directly; the FS port emits real arcs).

function pcRoundedInsetTriangle(A, B, C, d, R) {
    var eAB = pcDist(A, B), eBC = pcDist(B, C), eCA = pcDist(C, A);
    var per = eAB + eBC + eCA;
    if (per < 0.001) return null;
    var Ix = (eBC * A[0] + eCA * B[0] + eAB * C[0]) / per;
    var Iy = (eBC * A[1] + eCA * B[1] + eAB * C[1]) / per;
    var area2 = Math.abs((B[0] - A[0]) * (C[1] - A[1]) - (C[0] - A[0]) * (B[1] - A[1]));
    var inrad = area2 / per;
    if (inrad <= d + 0.4) return null;
    var f = 1 - d / inrad;
    var V = [
        [Ix + (A[0] - Ix) * f, Iy + (A[1] - Iy) * f],
        [Ix + (B[0] - Ix) * f, Iy + (B[1] - Iy) * f],
        [Ix + (C[0] - Ix) * f, Iy + (C[1] - Iy) * f]
    ];
    if (R <= 0.01) return V;  // sharp

    var loop = [];
    for (var k = 0; k < 3; k += 1) {
        var Vk = V[k];
        var Vp = V[(k + 2) % 3];
        var Vn = V[(k + 1) % 3];
        var up = [Vp[0] - Vk[0], Vp[1] - Vk[1]];
        var un = [Vn[0] - Vk[0], Vn[1] - Vk[1]];
        var lp = Math.sqrt(up[0] * up[0] + up[1] * up[1]);
        var ln = Math.sqrt(un[0] * un[0] + un[1] * un[1]);
        if (lp < 1e-6 || ln < 1e-6) { loop.push(Vk); continue; }
        up = [up[0] / lp, up[1] / lp];
        un = [un[0] / ln, un[1] / ln];
        var cosA = up[0] * un[0] + up[1] * un[1];
        if (cosA > 1) cosA = 1; if (cosA < -1) cosA = -1;
        var ang = Math.acos(cosA);
        var halfA = ang / 2;
        var tt = R / Math.tan(halfA);
        var maxT = 0.45 * (lp < ln ? lp : ln);
        var Reff = R;
        if (tt > maxT) { tt = maxT; Reff = tt * Math.tan(halfA); }
        var Tin = [Vk[0] + up[0] * tt, Vk[1] + up[1] * tt];
        var Tout = [Vk[0] + un[0] * tt, Vk[1] + un[1] * tt];
        var bis = [up[0] + un[0], up[1] + un[1]];
        var lb = Math.sqrt(bis[0] * bis[0] + bis[1] * bis[1]);
        if (lb < 1e-6) { loop.push(Vk); continue; }
        bis = [bis[0] / lb, bis[1] / lb];
        var cdist = Reff / Math.sin(halfA);
        var Cc = [Vk[0] + bis[0] * cdist, Vk[1] + bis[1] * cdist];
        // sweep arc from Tin to Tout around Cc (short way), flattened
        var a0 = Math.atan2(Tin[1] - Cc[1], Tin[0] - Cc[0]);
        var a1 = Math.atan2(Tout[1] - Cc[1], Tout[0] - Cc[0]);
        var da = a1 - a0;
        while (da > Math.PI) da -= 2 * Math.PI;
        while (da < -Math.PI) da += 2 * Math.PI;
        var steps = 6;
        for (var sidx = 0; sidx <= steps; sidx += 1) {
            var aa = a0 + da * (sidx / steps);
            loop.push([Cc[0] + Reff * Math.cos(aa), Cc[1] + Reff * Math.sin(aa)]);
        }
    }
    return loop;
}

// ---- main -----------------------------------------------------------------

function pcGenerate(plate, params) {
    var s = params.triangleSize;
    var minR = (params.minHoleDia || 0) / 2;
    var dropDist = params.declusterDist;
    // For uniform (similar-area) triangles, vertices must be roughly one
    // triangle apart everywhere. When "even spacing" is on, thin any holes
    // packed closer than ~0.8*s so the hole vertices match the lattice spacing.
    if (params.gapFill && dropDist < 0.8 * s) dropDist = 0.8 * s;
    var maxEdge = (params.maxEdgeFactor || 2.8) * s;
    var marginMm = params.edgeMargin || 6;
    var d = params.ribWidth / 2;
    var R = params.pocketFillet || 0;
    var holeMargin = params.holeMargin || 0;   // solid material kept around EVERY hole edge

    var poly = plate.outline;
    var inners = plate.inners || [];

    // 1. filter + dedupe holes
    var holes = [];
    for (var i = 0; i < plate.holes.length; i += 1) {
        var h = plate.holes[i];
        if (h.r < minR) continue;
        var dup = false;
        for (var j = 0; j < holes.length; j += 1) {
            if (pcDist([h.x, h.y], [holes[j].x, holes[j].y]) < 1.0) { dup = true; break; }
        }
        if (!dup) holes.push(h);
    }

    // 2. decluster: keep largest-first, drop any hole within dropDist of a kept one
    var order = [];
    for (var k = 0; k < holes.length; k += 1) order.push([k, holes[k].r]);
    order = pcSortByValueDesc(order);
    var nodePts = [];
    var nodeIsHole = [];
    var nodeR = [];
    var kept = [];
    var holeIsVertex = [];
    for (var hv = 0; hv < holes.length; hv += 1) holeIsVertex.push(false);
    // Holes are prioritised FIRST (added before any lattice fill) so the
    // triangulation is anchored on real holes; declustering only thins holes
    // that are packed closer than dropDist (those get a support strut later so
    // they are still attached to the network).
    for (var oi = 0; oi < order.length; oi += 1) {
        var hi = order[oi][0];
        var hp = [holes[hi].x, holes[hi].y];
        if (!pcInMaterial(poly, inners, hp)) continue;
        var skip = false;
        for (var kp = 0; kp < kept.length; kp += 1) {
            if (pcDist(hp, kept[kp]) < dropDist) { skip = true; break; }
        }
        if (skip) continue;
        kept.push(hp);
        holeIsVertex[hi] = true;
        nodePts.push(hp); nodeIsHole.push(true); nodeR.push(holes[hi].r);
    }

    // 2b. optional lattice gap-fill (default off)
    if (params.gapFill) {
        var rowH = s * Math.sqrt(3) / 2;
        var bMinX = poly[0][0], bMaxX = poly[0][0], bMinY = poly[0][1], bMaxY = poly[0][1];
        for (var p2 = 0; p2 < poly.length; p2 += 1) {
            if (poly[p2][0] < bMinX) bMinX = poly[p2][0];
            if (poly[p2][0] > bMaxX) bMaxX = poly[p2][0];
            if (poly[p2][1] < bMinY) bMinY = poly[p2][1];
            if (poly[p2][1] > bMaxY) bMaxY = poly[p2][1];
        }
        var row = 0;
        for (var y = bMinY; y <= bMaxY; y += rowH) {
            var xoff = (row % 2 === 0) ? 0 : s / 2;
            for (var x = bMinX + xoff; x <= bMaxX; x += s) {
                var q = [x, y];
                if (pcInMaterial(poly, inners, q) && pcDistToPolygon(poly, q) >= marginMm
                        && pcDistToInners(inners, q) >= marginMm) {
                    var clear = true;
                    for (var kk = 0; kk < kept.length; kk += 1) {
                        if (pcDist(q, kept[kk]) < 0.85 * s) { clear = false; break; }
                    }
                    if (clear) { nodePts.push(q); nodeIsHole.push(false); nodeR.push(0); kept.push(q); }
                }
            }
            row += 1;
        }
    }

    // 3. rim sampling so ribs reach the perimeter
    var rim = [];
    var acc = s;
    for (var ri = 0; ri < poly.length; ri += 1) {
        var a = poly[ri], b = poly[(ri + 1) % poly.length];
        acc += pcDist(a, b);
        if (acc >= s) { rim.push(a); acc = 0; }
    }
    rim = pcDedupePoints(rim, 0.4 * s);

    var nNodes = nodePts.length;
    var pts = nodePts.concat(rim);

    // 4. triangulate + keep in-material triangles
    var triangles = pcBowyerWatson(pts);
    var keepTris = [];
    var edgeSet = {};
    var pockets = [];
    for (var ti = 0; ti < triangles.length; ti += 1) {
        var tr = triangles[ti];
        var A = pts[tr[0]], B = pts[tr[1]], C = pts[tr[2]];
        var eAB = pcDist(A, B), eBC = pcDist(B, C), eCA = pcDist(C, A);
        if (eAB > maxEdge || eBC > maxEdge || eCA > maxEdge) continue;
        var mAB = [(A[0] + B[0]) / 2, (A[1] + B[1]) / 2];
        var mBC = [(B[0] + C[0]) / 2, (B[1] + C[1]) / 2];
        var mCA = [(C[0] + A[0]) / 2, (C[1] + A[1]) / 2];
        var cen = [(A[0] + B[0] + C[0]) / 3, (A[1] + B[1] + C[1]) / 3];
        if (!pcInMaterial(poly, inners, cen) || !pcInMaterial(poly, inners, mAB)
                || !pcInMaterial(poly, inners, mBC) || !pcInMaterial(poly, inners, mCA)) continue;
        keepTris.push(tr);
        var ee2 = [[tr[0], tr[1]], [tr[1], tr[2]], [tr[2], tr[0]]];
        for (var x2 = 0; x2 < 3; x2 += 1) {
            var aa = Math.min(ee2[x2][0], ee2[x2][1]);
            var bb = Math.max(ee2[x2][0], ee2[x2][1]);
            edgeSet[aa + "_" + bb] = [aa, bb];
        }
        var loop = pcRoundedInsetTriangle(A, B, C, d, R);
        if (loop) pockets.push(loop);
    }

    var edges = [];
    for (var key in edgeSet) { if (edgeSet.hasOwnProperty(key)) edges.push(edgeSet[key]); }

    // 5. material ring (boss) around EVERY hole of any size, kept BEFORE ribs.
    var bosses = [];
    for (var bi = 0; bi < holes.length; bi += 1) {
        bosses.push({ x: holes[bi].x, y: holes[bi].y, r: holes[bi].r, R: holes[bi].r + holeMargin });
    }

    // 6. guarantee every hole is attached: any hole that did NOT become a
    //    triangulation vertex (declustered) gets a support strut to the nearest
    //    vertex, so its boss can never become a floating island of material.
    var struts = [];
    for (var si = 0; si < holes.length; si += 1) {
        if (holeIsVertex[si]) continue;
        var hpS = [holes[si].x, holes[si].y];
        if (!pcInMaterial(poly, inners, hpS)) continue;
        var best = -1, bestD = 1e18;
        for (var vi = 0; vi < nNodes; vi += 1) {
            var dd2 = pcDist(hpS, nodePts[vi]);
            if (dd2 < bestD) { bestD = dd2; best = vi; }
        }
        if (best >= 0) struts.push([hpS, nodePts[best]]);
    }

    return {
        vertices: pts,
        nNodes: nNodes,
        nodeIsHole: nodeIsHole,
        nodeR: nodeR,
        edges: edges,
        triangles: keepTris,
        pockets: pockets,
        bosses: bosses,
        struts: struts,
        info: { vertices: pts.length, holes: holes.length, holesUsed: nNodes,
                struts: struts.length, ribs: edges.length, pockets: pockets.length }
    };
}

// expose for both browser (window) and node-ish
if (typeof window !== "undefined") {
    window.pcGenerate = pcGenerate;
}
