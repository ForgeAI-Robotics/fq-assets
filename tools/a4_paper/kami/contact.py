"""Self contact between layers of the sheet: vertex against triangle, with Coulomb friction.

Detection runs once per frame: every triangle is inserted into a spatial hash (cells 1.3 L wide), padded by how far its
vertices can move in a frame (their speed relative to the sheet's mean, capped at 2 L); each vertex then keeps up to
`SLOTS` nearby triangles that are not its grid neighbours, with frozen barycentric coordinates and the side it is on
(remembered from the previous frame while the layers are close, so a layer cannot switch sides by passing through).
The solve runs every substep, sequentially per sheet (Gauss-Seidel over vertices): each contact keeps the vertex at least
`layer_spacing` from the triangle's plane on its side, and friction removes relative tangential motion up to mu times the
normal correction. Overflowing the hash or a non-finite position is counted in `overflow` (the contact pass then misses
contacts rather than inventing them).
"""
import warp as wp

from ._layout import P_MU_PAPER, SCAN_CHUNK

vec8i = wp.types.vector(length=8, dtype=wp.int32)
vec8f = wp.types.vector(length=8, dtype=wp.float32)


@wp.func
def cell_hash(x: int, y: int, z: int, mask: int):
    return ((x * 73856093) ^ (y * 19349663) ^ (z * 83492791)) & mask


@wp.func
def cell_interval(lo: float, hi: float, inv_cell: float, max_span: int):
    """Integer cell range [a, b] covering [lo, hi], or valid = 0 for non-finite or too wide input."""
    if not wp.isfinite(lo) or not wp.isfinite(hi) or lo > hi:
        return int(0), int(-1), int(0)
    first = wp.floor(lo * inv_cell)
    last = wp.floor(hi * inv_cell)
    if not wp.isfinite(first) or not wp.isfinite(last) or first < -536870000.0 or last > 536870000.0:
        return int(0), int(-1), int(0)
    a = int(first)
    b = int(last)
    if b < a or b - a > max_span:
        return int(0), int(-1), int(0)
    return a, b, int(1)


@wp.func
def tri_cells(pos: wp.array2d(dtype=wp.vec3), tris: wp.array2d(dtype=int), k: int, b: int, e: float, icx: float, icz: float):
    a = pos[tris[k, 0], b]
    bb = pos[tris[k, 1], b]
    c = pos[tris[k, 2], b]
    for axis in range(3):
        if not wp.isfinite(a[axis]) or not wp.isfinite(bb[axis]) or not wp.isfinite(c[axis]) or not wp.isfinite(e):
            return int(0), int(-1), int(0), int(-1), int(0), int(-1), int(0)
    lo = wp.min(wp.min(a, bb), c) - wp.vec3(e)
    hi = wp.max(wp.max(a, bb), c) + wp.vec3(e)
    x0, x1, vx = cell_interval(lo[0], hi[0], icx, 8)
    y0, y1, vy = cell_interval(lo[1], hi[1], icx, 8)
    z0, z1, vz = cell_interval(lo[2], hi[2], icz, 12)
    return x0, x1, y0, y1, z0, z1, vx * vy * vz


@wp.kernel
def k_mean_vel(vel: wp.array2d(dtype=wp.vec3), mv: wp.array(dtype=wp.vec3), n: int):
    b = wp.tid()
    s = wp.vec3(0.0)
    for i in range(n):
        s += vel[i, b]
    mv[b] = s / float(n)


@wp.kernel
def k_reach(vel: wp.array2d(dtype=wp.vec3), mv: wp.array(dtype=wp.vec3), reach: wp.array2d(dtype=float), dt_frame: float, r_max: float):
    i, b = wp.tid()
    r = wp.length(vel[i, b] - mv[b]) * dt_frame
    if r > r_max:                                  # (not wp.min: a NaN must stay visible)
        r = r_max
    reach[i, b] = r


@wp.kernel
def k_tri_count(pos: wp.array2d(dtype=wp.vec3), tris: wp.array2d(dtype=int), reach: wp.array2d(dtype=float), tri_reach: wp.array2d(dtype=float),
                cnt: wp.array2d(dtype=int), overflow: wp.array(dtype=int), pad: float, icx: float, icz: float, mask: int):
    k, b = wp.tid()
    for corner in range(3):
        r = reach[tris[k, corner], b]
        if not wp.isfinite(r) or r < 0.0:
            tri_reach[k, b] = -1.0
            wp.atomic_add(overflow, b, 1)
            return
    tr = wp.max(wp.max(reach[tris[k, 0], b], reach[tris[k, 1], b]), reach[tris[k, 2], b])
    tri_reach[k, b] = tr
    x0, x1, y0, y1, z0, z1, valid = tri_cells(pos, tris, k, b, pad + tr, icx, icz)
    if valid == 0:
        tri_reach[k, b] = -1.0
        wp.atomic_add(overflow, b, 1)
        return
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            for z in range(z0, z1 + 1):
                wp.atomic_add(cnt, cell_hash(x, y, z, mask) + 1, b, 1)


@wp.kernel
def k_scan_chunks(cnt: wp.array2d(dtype=int), chunk_sum: wp.array2d(dtype=int), total: int):
    c, b = wp.tid()
    s = int(0)
    for k in range(c * SCAN_CHUNK, wp.min((c + 1) * SCAN_CHUNK, total)):
        s += cnt[k, b]
        cnt[k, b] = s
    chunk_sum[c, b] = s


@wp.kernel
def k_scan_offsets(chunk_sum: wp.array2d(dtype=int), n_chunks: int):
    b = wp.tid()
    run = int(0)
    for c in range(n_chunks):
        v = chunk_sum[c, b]
        chunk_sum[c, b] = run
        run += v


@wp.kernel
def k_scan_apply(cnt: wp.array2d(dtype=int), chunk_sum: wp.array2d(dtype=int), fill: wp.array2d(dtype=int), total: int, n_buckets: int):
    c, b = wp.tid()
    off = chunk_sum[c, b]
    for k in range(c * SCAN_CHUNK, wp.min((c + 1) * SCAN_CHUNK, total)):
        v = cnt[k, b] + off
        cnt[k, b] = v
        if k < n_buckets:
            fill[k, b] = v


@wp.kernel
def k_hash_scatter(pos: wp.array2d(dtype=wp.vec3), tris: wp.array2d(dtype=int), tri_reach: wp.array2d(dtype=float), fill: wp.array2d(dtype=int),
                   items: wp.array2d(dtype=int), overflow: wp.array(dtype=int), cap: int, pad: float, icx: float, icz: float, mask: int):
    k, b = wp.tid()
    if tri_reach[k, b] < 0.0:
        return
    x0, x1, y0, y1, z0, z1, valid = tri_cells(pos, tris, k, b, pad + tri_reach[k, b], icx, icz)
    if valid == 0:
        overflow[b] = 1
        return
    for x in range(x0, x1 + 1):
        for y in range(y0, y1 + 1):
            for z in range(z0, z1 + 1):
                idx = wp.atomic_add(fill, cell_hash(x, y, z, mask), b, 1)
                if idx < cap:
                    items[idx, b] = k
                else:
                    overflow[b] = 1


@wp.kernel
def k_copy_contacts(c_cnt: wp.array2d(dtype=int), c_tri: wp.array3d(dtype=int), c_side: wp.array3d(dtype=int),
                    o_cnt: wp.array2d(dtype=int), o_tri: wp.array3d(dtype=int), o_side: wp.array3d(dtype=int), S: int):
    i, b = wp.tid()
    o_cnt[i, b] = c_cnt[i, b]
    for s in range(S):
        o_tri[i, s, b] = c_tri[i, s, b]
        o_side[i, s, b] = c_side[i, s, b]


@wp.kernel
def k_contacts(pos: wp.array2d(dtype=wp.vec3), fnrm: wp.array2d(dtype=wp.vec3), tris: wp.array2d(dtype=int), tri_cell: wp.array2d(dtype=int),
               reach: wp.array2d(dtype=float), tri_reach: wp.array2d(dtype=float), cnt: wp.array2d(dtype=int), items: wp.array2d(dtype=int),
               o_cnt: wp.array2d(dtype=int), o_tri: wp.array3d(dtype=int), o_side: wp.array3d(dtype=int),
               c_cnt: wp.array2d(dtype=int), c_tri: wp.array3d(dtype=int), c_side: wp.array3d(dtype=int), c_bary: wp.array3d(dtype=wp.vec3),
               N: int, t: float, margin: float, L: float, icx: float, icz: float, mask: int, S: int, cap: int, overflow: wp.array(dtype=int)):
    """Candidate triangles for vertex p: within reach, not within one cell of p on the grid, inside the triangle's prism up
    to a tolerance; one candidate per layer (side and distance), the most interior one wins; at most S kept."""
    p, b = wp.tid()
    P = pos[p, b]
    r = reach[p, b]
    c_cnt[p, b] = 0
    if not wp.isfinite(r) or r < 0.0:
        wp.atomic_add(overflow, b, 1)
        return
    pi = p % N
    pj = p / N
    qx0, qx1, vx = cell_interval(P[0] - r, P[0] + r, icx, 8)
    qy0, qy1, vy = cell_interval(P[1] - r, P[1] + r, icx, 8)
    qz0, qz1, vz = cell_interval(P[2] - r, P[2] + r, icz, 12)
    if vx * vy * vz == 0:
        wp.atomic_add(overflow, b, 1)
        return
    qK = vec8i()
    qS = vec8i()
    qD = vec8f()
    qM = vec8f()
    qU = vec8f()
    qV = vec8f()
    qW = vec8f()
    nq = int(0)
    for x in range(qx0, qx1 + 1):
        for y in range(qy0, qy1 + 1):
            for z in range(qz0, qz1 + 1):
                h = cell_hash(x, y, z, mask)
                for it in range(cnt[h, b], wp.min(cnt[h + 1, b], cap)):
                    k = items[it, b]
                    ci = tri_cell[k, 0]
                    cj = tri_cell[k, 1]
                    gi = int(0)
                    if pi < ci:
                        gi = ci - pi
                    elif pi > ci + 1:
                        gi = pi - ci - 1
                    gj = int(0)
                    if pj < cj:
                        gj = cj - pj
                    elif pj > cj + 1:
                        gj = pj - cj - 1
                    if wp.max(gi, gj) <= 1:
                        continue
                    A = pos[tris[k, 0], b]
                    v2 = P - A
                    d = wp.dot(v2, fnrm[k, b])
                    lim = t + margin + r + tri_reach[k, b]
                    if d > lim or d < -lim:
                        continue
                    v0 = pos[tris[k, 1], b] - A
                    v1 = pos[tris[k, 2], b] - A
                    d00 = wp.dot(v0, v0)
                    d01 = wp.dot(v0, v1)
                    d11 = wp.dot(v1, v1)
                    d20 = wp.dot(v2, v0)
                    d21 = wp.dot(v2, v1)
                    den = d00 * d11 - d01 * d01
                    if den < 1.0e-20:
                        continue
                    bv = (d11 * d20 - d01 * d21) / den
                    bw = (d00 * d21 - d01 * d20) / den
                    bu = 1.0 - bv - bw
                    tol = -0.12 - wp.min((r + tri_reach[k, b]) / L, 0.5)
                    if bu < tol or bv < tol or bw < tol:
                        continue
                    s = int(1)
                    if d < 0.0:
                        s = -1
                    if d < 1.5 * t and d > -1.5 * t:            # close: keep last frame's side
                        for o in range(o_cnt[p, b]):
                            if o_tri[p, o, b] == k:
                                s = o_side[p, o, b]
                                break
                    mb = wp.min(wp.min(bu, bv), bw)
                    qi = int(0)
                    found = int(0)
                    while qi < nq:
                        if qS[qi] == s and wp.abs(qD[qi] - d) < 0.6 * t:
                            found = 1
                            break
                        qi += 1
                    if found == 1 and qM[qi] >= mb:
                        continue
                    if found == 0:
                        if nq == 8:
                            continue
                        qi = nq
                        nq += 1
                    qK[qi] = k
                    qD[qi] = d
                    qM[qi] = mb
                    qS[qi] = s
                    qU[qi] = bu
                    qV[qi] = bv
                    qW[qi] = bw
    out = int(0)
    for qi in range(nq):
        if out >= S:
            break
        bu = wp.max(qU[qi], 0.0)
        bv = wp.max(qV[qi], 0.0)
        bw = wp.max(qW[qi], 0.0)
        bs = 1.0 / (bu + bv + bw)
        c_tri[p, out, b] = qK[qi]
        c_side[p, out, b] = qS[qi]
        c_bary[p, out, b] = wp.vec3(bu * bs, bv * bs, bw * bs)
        out += 1
    c_cnt[p, b] = out


@wp.kernel
def k_contacts_solve(pos: wp.array2d(dtype=wp.vec3), prev: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), tris: wp.array2d(dtype=int),
                     c_cnt: wp.array2d(dtype=int), c_tri: wp.array3d(dtype=int), c_side: wp.array3d(dtype=int), c_bary: wp.array3d(dtype=wp.vec3),
                     par: wp.array2d(dtype=float), t: float, n: int):
    """Sequential (per sheet) projection of every vertex-triangle contact, then Coulomb friction on the relative motion."""
    b = wp.tid()
    mu = par[P_MU_PAPER, b]
    for p in range(n):
        for o in range(c_cnt[p, b]):
            k = c_tri[p, o, b]
            ia = tris[k, 0]
            ib = tris[k, 1]
            ic = tris[k, 2]
            A = pos[ia, b]
            v0 = pos[ib, b] - A
            v1 = pos[ic, b] - A
            nn = wp.cross(v0, v1)
            nl = wp.length(nn)
            if nl < 1.0e-12:
                continue
            nn = nn * (float(c_side[p, o, b]) / nl)
            bary = c_bary[p, o, b]
            bu = bary[0]
            bv = bary[1]
            bw = bary[2]
            d = wp.dot(pos[p, b] - (A + v0 * bv + v1 * bw), nn)
            if d >= t:
                continue
            wpp = w[p, b]
            wa = w[ia, b] * bu
            wb = w[ib, b] * bv
            wc = w[ic, b] * bw
            W = wpp + wa * bu + wb * bv + wc * bw
            if W == 0.0:
                continue
            # No participant moves farther than the correction itself. With a free vertex this never binds (weights <= 4);
            # it binds when a kinematic vertex touches a triangle whose only free corner has a small barycentric weight b,
            # where the plain PBD update would move that corner by Cc / b.
            W = wp.max(W, wp.max(wp.max(wpp, wa), wp.max(wb, wc)))
            Cc = wp.min(t - d, 0.5 * t)
            lam = Cc / W
            pos[p, b] = pos[p, b] + nn * (lam * wpp)
            pos[ia, b] = pos[ia, b] - nn * (lam * wa)
            pos[ib, b] = pos[ib, b] - nn * (lam * wb)
            pos[ic, b] = pos[ic, b] - nn * (lam * wc)
            rel = (pos[p, b] - prev[p, b]) - ((pos[ia, b] - prev[ia, b]) * bu + (pos[ib, b] - prev[ib, b]) * bv + (pos[ic, b] - prev[ic, b]) * bw)
            rel = rel - nn * wp.dot(rel, nn)
            rl = wp.length(rel)
            if rl > 1.0e-12:
                f = wp.min(mu * Cc / rl, 1.0) / W
                pos[p, b] = pos[p, b] - rel * (f * wpp)
                pos[ia, b] = pos[ia, b] + rel * (f * wa)
                pos[ib, b] = pos[ib, b] + rel * (f * wb)
                pos[ic, b] = pos[ic, b] + rel * (f * wc)
