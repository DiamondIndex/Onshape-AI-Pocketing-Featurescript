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

// union-find root (path-halving), FS-friendly
function pcUFFind(parent, a) {
    while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; }
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
    // Holes within mergeDist of each other are "very close": they merge into a
    // single vertex at the cluster centroid and are ribbed to it (a close pair
    // becomes one midpoint vertex with a rib joining the two holes). 0 = never
    // merge (every hole is its own vertex). No hole is ever simply dropped.
    var mergeDist = params.declusterDist;
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

    // 2. merge very-close holes into clusters (union-find on pairwise distance).
    var parent = [];
    for (var pf = 0; pf < holes.length; pf += 1) parent.push(pf);
    if (mergeDist > 0) {
        for (var ci = 0; ci < holes.length; ci += 1) {
            for (var cj = ci + 1; cj < holes.length; cj += 1) {
                if (pcDist([holes[ci].x, holes[ci].y], [holes[cj].x, holes[cj].y]) < mergeDist) {
                    parent[pcUFFind(parent, ci)] = pcUFFind(parent, cj);
                }
            }
        }
    }
    // group holes by cluster root; one vertex per cluster at its centroid.
    var groups = {};
    for (var gi = 0; gi < holes.length; gi += 1) {
        var root = pcUFFind(parent, gi);
        if (!groups[root]) groups[root] = [];
        groups[root].push(gi);
    }
    var nodePts = [];
    var nodeIsHole = [];
    var nodeR = [];
    var kept = [];
    var clusterRibs = [];   // ribs joining merged holes to their cluster vertex
    for (var gk in groups) {
        if (!groups.hasOwnProperty(gk)) continue;
        var mem = groups[gk];
        var cx = 0, cy = 0, rmax = 0;
        for (var mm = 0; mm < mem.length; mm += 1) {
            cx += holes[mem[mm]].x; cy += holes[mem[mm]].y;
            if (holes[mem[mm]].r > rmax) rmax = holes[mem[mm]].r;
        }
        cx /= mem.length; cy /= mem.length;
        var cpt = [cx, cy];
        if (!pcInMaterial(poly, inners, cpt)) continue;
        nodePts.push(cpt); nodeIsHole.push(true); nodeR.push(rmax); kept.push(cpt);
        // a multi-hole cluster: rib each member hole to the centroid vertex
        // (a 2-hole cluster => the two ribs form the single hole-to-hole rib
        //  through the midpoint).
        if (mem.length >= 2) {
            for (var mr = 0; mr < mem.length; mr += 1) {
                clusterRibs.push([[holes[mem[mr]].x, holes[mem[mr]].y], cpt]);
            }
        }
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

    // 6. attachment ribs = the cluster ribs joining each merged hole to its
    //    centroid vertex, so every merged hole stays tied into the network.
    var struts = clusterRibs;

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

/* ============================================================================
 * VORONOI pocketing  —  the "looks" version.
 * Ribs run along Voronoi cell walls (the dual of the Delaunay we already build):
 * each Delaunay triangle has a circumcenter, and the wall between two adjacent
 * sites is the segment joining the circumcenters of the two triangles sharing
 * that Delaunay edge. Cells are clipped to the plate material.
 * ==========================================================================*/

function pcCircumcenter(A, B, C) {
    var ax = A[0], ay = A[1], bx = B[0], by = B[1], cx = C[0], cy = C[1];
    var d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (Math.abs(d) < 1e-12) return [(ax + bx + cx) / 3, (ay + by + cy) / 3];
    var a2 = ax * ax + ay * ay, b2 = bx * bx + by * by, c2 = cx * cx + cy * cy;
    var ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d;
    var uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d;
    return [ux, uy];
}

// intersection parameter t in [0,1] along p0->p1 where it crosses q0-q1, else -1
function pcSegInterT(p0, p1, q0, q1) {
    var r0 = p1[0] - p0[0], r1 = p1[1] - p0[1];
    var s0 = q1[0] - q0[0], s1 = q1[1] - q0[1];
    var den = r0 * s1 - r1 * s0;
    if (Math.abs(den) < 1e-12) return -1;
    var qpx = q0[0] - p0[0], qpy = q0[1] - p0[1];
    var t = (qpx * s1 - qpy * s0) / den;
    var u = (qpx * r1 - qpy * r0) / den;
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) return t;
    return -1;
}

function pcSegCircleT(p0, p1, c, rr, out) {
    var dx = p1[0] - p0[0], dy = p1[1] - p0[1];
    var fx = p0[0] - c[0], fy = p0[1] - c[1];
    var a = dx * dx + dy * dy;
    if (a < 1e-12) return;
    var b = 2 * (fx * dx + fy * dy);
    var cc = fx * fx + fy * fy - rr * rr;
    var disc = b * b - 4 * a * cc;
    if (disc < 0) return;
    disc = Math.sqrt(disc);
    var t1 = (-b - disc) / (2 * a), t2 = (-b + disc) / (2 * a);
    if (t1 > 0 && t1 < 1) out.push(t1);
    if (t2 > 0 && t2 < 1) out.push(t2);
}

function pcVoroMaterialOK(p, poly, inners, holes, holeMargin) {
    if (!pcInMaterial(poly, inners, p)) return false;
    for (var i = 0; i < holes.length; i += 1) {
        if (pcDist(p, [holes[i].x, holes[i].y]) < holes[i].r + holeMargin) return false;
    }
    return true;
}

// clip a rib segment to material: cut at outline / inner / hole-keepout crossings,
// keep sub-segments whose midpoint is solid material outside every hole.
function pcClipVoroSeg(p0, p1, poly, inners, holes, holeMargin) {
    var ts = [0, 1];
    for (var i = 0; i < poly.length; i += 1) {
        var t = pcSegInterT(p0, p1, poly[i], poly[(i + 1) % poly.length]);
        if (t >= 0) ts.push(t);
    }
    for (var k = 0; k < inners.length; k += 1) {
        var inn = inners[k];
        for (var j = 0; j < inn.length; j += 1) {
            var t2 = pcSegInterT(p0, p1, inn[j], inn[(j + 1) % inn.length]);
            if (t2 >= 0) ts.push(t2);
        }
    }
    for (var h = 0; h < holes.length; h += 1) {
        pcSegCircleT(p0, p1, [holes[h].x, holes[h].y], holes[h].r + holeMargin, ts);
    }
    ts.sort(function (a, b) { return a - b; });
    var out = [];
    for (var m = 0; m < ts.length - 1; m += 1) {
        var ta = ts[m], tb = ts[m + 1];
        if (tb - ta < 1e-4) continue;
        var tm = (ta + tb) / 2;
        var pm = [p0[0] + (p1[0] - p0[0]) * tm, p0[1] + (p1[1] - p0[1]) * tm];
        if (pcVoroMaterialOK(pm, poly, inners, holes, holeMargin)) {
            out.push([[p0[0] + (p1[0] - p0[0]) * ta, p0[1] + (p1[1] - p0[1]) * ta],
                      [p0[0] + (p1[0] - p0[0]) * tb, p0[1] + (p1[1] - p0[1]) * tb]]);
        }
    }
    return out;
}

function pcClosestOnSeg(p, a, b) {
    var vx = b[0] - a[0], vy = b[1] - a[1];
    var wx = p[0] - a[0], wy = p[1] - a[1];
    var c1 = vx * wx + vy * wy;
    if (c1 <= 0) return a;
    var c2 = vx * vx + vy * vy;
    if (c2 <= c1) return b;
    var t = c1 / c2;
    return [a[0] + t * vx, a[1] + t * vy];
}

// merge any disconnected wall islands into ONE network (shortest links).
function pcConnectVoroSegs(segs) {
    var nodes = [], key2idx = {};
    function nidx(p) {
        var k = Math.round(p[0] / 0.8) + "_" + Math.round(p[1] / 0.8);
        if (key2idx[k] === undefined) { key2idx[k] = nodes.length; nodes.push(p); }
        return key2idx[k];
    }
    var pairs = [];
    for (var i = 0; i < segs.length; i += 1) pairs.push([nidx(segs[i][0]), nidx(segs[i][1])]);
    var parent = [];
    for (var i = 0; i < nodes.length; i += 1) parent.push(i);
    function find(a) { while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; }
    for (var i = 0; i < pairs.length; i += 1) parent[find(pairs[i][0])] = find(pairs[i][1]);
    var out = segs.slice();
    var guard = 0;
    while (guard++ < 400) {
        var roots = {};
        for (var i = 0; i < nodes.length; i += 1) roots[find(i)] = true;
        if (Object.keys(roots).length <= 1) break;
        var best = 1e18, bi = -1, bj = -1;
        for (var i = 0; i < nodes.length; i += 1) {
            for (var j = i + 1; j < nodes.length; j += 1) {
                if (find(i) === find(j)) continue;
                var dd = pcDist(nodes[i], nodes[j]);
                if (dd < best) { best = dd; bi = i; bj = j; }
            }
        }
        if (bi < 0) break;
        out.push([nodes[bi], nodes[bj]]);
        parent[find(bi)] = find(bj);
    }
    return out;
}

function pcGenerateVoronoi(plate, params) {
    var poly = plate.outline;
    var inners = plate.inners || [];
    var s = params.triangleSize;
    var minR = (params.minHoleDia || 0) / 2;
    var marginMm = params.edgeMargin || 6;
    var jitter = (params.jitter === undefined) ? 0.32 : params.jitter;
    var relax = (params.relax === undefined) ? 0 : params.relax;
    // deterministic hash-based "random" in [0,1)
    function hash(i) { var x = Math.sin(i * 12.9898 + 7.13) * 43758.5453; return x - Math.floor(x); }

    var holes = [];
    for (var i = 0; i < plate.holes.length; i += 1) {
        if (plate.holes[i].r < minR) continue;
        holes.push(plate.holes[i]);
    }

    var minX = poly[0][0], maxX = poly[0][0], minY = poly[0][1], maxY = poly[0][1];
    for (var p = 1; p < poly.length; p += 1) {
        if (poly[p][0] < minX) minX = poly[p][0];
        if (poly[p][0] > maxX) maxX = poly[p][0];
        if (poly[p][1] < minY) minY = poly[p][1];
        if (poly[p][1] > maxY) maxY = poly[p][1];
    }
    var cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
    var rad = Math.max(maxX - minX, maxY - minY) * 2 + 200;
    function guardRing() {
        var g = [];
        for (var k = 0; k < 16; k += 1) { var a = k / 16 * 2 * Math.PI; g.push([cx + Math.cos(a) * rad, cy + Math.sin(a) * rad]); }
        return g;
    }

    // 1. sites = jittered lattice ONLY (holes are NOT sites, so they can land on
    //    cell walls / junctions instead of floating in cell centres). Keep sites
    //    off the holes so holes fall on the boundaries between cells.
    var sites = [];
    // square lattice ROTATED 45 deg -> Voronoi walls run at ~45/135 deg
    // (maximally far from horizontal and vertical), then jittered for variation.
    var cxg = (minX + maxX) / 2, cyg = (minY + maxY) / 2;
    var diag = Math.sqrt((maxX - minX) * (maxX - minX) + (maxY - minY) * (maxY - minY));
    var N = Math.ceil(diag / s) + 2, hidx = 0;
    for (var gi = -N; gi <= N; gi += 1) {
        for (var gj = -N; gj <= N; gj += 1) {
            hidx += 1;
            var lx = gi * s, ly = gj * s;
            var rx = (lx - ly) * 0.7071068, ry = (lx + ly) * 0.7071068;
            var q = [cxg + rx + (hash(hidx) - 0.5) * 2 * s * jitter,
                     cyg + ry + (hash(hidx + 9173) - 0.5) * 2 * s * jitter];
            if (q[0] < minX || q[0] > maxX || q[1] < minY || q[1] > maxY) continue;
            if (pcInMaterial(poly, inners, q) && pcDistToPolygon(poly, q) >= marginMm
                    && pcDistToInners(inners, q) >= marginMm) {
                var clear = true;
                for (var k = 0; k < sites.length; k += 1) { if (pcDist(q, sites[k]) < 0.7 * s) { clear = false; break; } }
                if (clear) for (var h = 0; h < holes.length; h += 1) { if (pcDist(q, [holes[h].x, holes[h].y]) < 0.45 * s) { clear = false; break; } }
                if (clear) sites.push(q);
            }
        }
    }
    var nReal = sites.length;

    // 2. Lloyd relaxation -> organic blue-noise cells (no straight rows/columns)
    for (var it = 0; it < relax; it += 1) {
        var all0 = sites.concat(guardRing());
        var tg = pcBowyerWatson(all0);
        var ax = [], ay = [], cn = [];
        for (var i = 0; i < nReal; i += 1) { ax.push(0); ay.push(0); cn.push(0); }
        for (var t = 0; t < tg.length; t += 1) {
            var tr = tg[t];
            var c = pcCircumcenter(all0[tr[0]], all0[tr[1]], all0[tr[2]]);
            for (var j = 0; j < 3; j += 1) { var vi = tr[j]; if (vi < nReal) { ax[vi] += c[0]; ay[vi] += c[1]; cn[vi] += 1; } }
        }
        for (var i = 0; i < nReal; i += 1) {
            if (cn[i] === 0) continue;
            var np = [ax[i] / cn[i], ay[i] / cn[i]];
            if (pcInMaterial(poly, inners, np) && pcDistToPolygon(poly, np) >= marginMm * 0.5) sites[i] = np;
        }
    }

    // 3. final Voronoi: circumcenters -> wall per shared Delaunay edge
    var all = sites.concat(guardRing());
    var tris = pcBowyerWatson(all);
    var cc = [];
    for (var t = 0; t < tris.length; t += 1) cc.push(pcCircumcenter(all[tris[t][0]], all[tris[t][1]], all[tris[t][2]]));
    var emap = {};
    for (var t = 0; t < tris.length; t += 1) {
        var tr = tris[t];
        var te = [[tr[0], tr[1]], [tr[1], tr[2]], [tr[2], tr[0]]];
        for (var e = 0; e < 3; e += 1) {
            var a = Math.min(te[e][0], te[e][1]), b = Math.max(te[e][0], te[e][1]);
            var key = a + "_" + b;
            if (!emap[key]) emap[key] = [];
            emap[key].push(t);
        }
    }

    // 4. walls clipped to BOUNDARY ONLY (not holes) -> ribs run through holes
    var segs = [];
    for (var key in emap) {
        if (!emap.hasOwnProperty(key)) continue;
        var ts2 = emap[key];
        if (ts2.length !== 2) continue;
        var clipped = pcClipVoroSeg(cc[ts2[0]], cc[ts2[1]], poly, inners, [], 0);
        for (var c = 0; c < clipped.length; c += 1) segs.push(clipped[c]);
    }

    // 5. one connected network (bridge islands)
    segs = pcConnectVoroSegs(segs);

    // 6. support every hole: if no rib passes through it, add a short strut to
    //    the nearest wall (the part inside the hole is cut away, leaving a rib
    //    from the hole edge to the web).
    var struts = 0;
    for (var h = 0; h < holes.length; h += 1) {
        var hc = [holes[h].x, holes[h].y];
        var best = 1e18, bp = null;
        for (var i = 0; i < segs.length; i += 1) {
            var pt = pcClosestOnSeg(hc, segs[i][0], segs[i][1]);
            var dd = pcDist(hc, pt);
            if (dd < best) { best = dd; bp = pt; }
        }
        if (bp && best > holes[h].r + 0.5) { segs.push([hc, bp]); struts += 1; }
    }

    return {
        segs: segs,
        sites: sites.slice(0, nReal),
        holes: holes,
        vertices: [], nNodes: 0, edges: [], triangles: [],
        pockets: [], bosses: [], struts: [],
        info: { holes: holes.length, holesUsed: nReal, struts: struts,
                ribs: segs.length, pockets: 0 }
    };
}

// expose for both browser (window) and node-ish
if (typeof window !== "undefined") {
    window.pcGenerate = pcGenerate;
    window.pcGenerateVoronoi = pcGenerateVoronoi;
}
