"""Inextensibility: position-based distance constraints swept along grid lines, the multi-scale coarse lines, the extra
sweeps around a pinch grab, and the buckling half-waves.

All arrays are indexed [item, world]. A thread owns one grid line of one world and sweeps it in order (Gauss-Seidel along
the line, lines of one family in parallel); the direction alternates every substep. `w` is the inverse-mass weight the
sweeps use (0 = kinematic).
"""
import warp as wp

SQRT2 = 1.4142135623730951


@wp.func
def stretch_edge(pos: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), a: int, bb: int, rest: float, b: int):
    wa = w[a, b]
    wb = w[bb, b]
    ws = wa + wb
    if ws == 0.0:
        return
    dl = pos[bb, b] - pos[a, b]
    d = wp.length(dl)
    if d < 1.0e-12:
        return
    s = (d - rest) / (d * ws)
    pos[a, b] = pos[a, b] + dl * (s * wa)
    pos[bb, b] = pos[bb, b] - dl * (s * wb)


@wp.kernel
def k_stretch_rows(pos: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), N: int, L: float, forward: int):
    """Row edges (i, j)-(i+1, j), one thread per row."""
    j, b = wp.tid()
    for c in range(N - 1):
        i = c
        if forward == 0:
            i = N - 2 - c
        stretch_edge(pos, w, j * N + i, j * N + i + 1, L, b)


@wp.kernel
def k_stretch_cols(pos: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), N: int, L: float, forward: int):
    """Column edges (i, j)-(i, j+1), one thread per column."""
    i, b = wp.tid()
    for c in range(N - 1):
        j = c
        if forward == 0:
            j = N - 2 - c
        stretch_edge(pos, w, j * N + i, (j + 1) * N + i, L, b)


@wp.func
def diag_line(l: int, N: int, family: int):
    """Diagonal line l of a family: family 0 = even cells (edge (i,j)-(i+1,j+1), lines i - j = k), family 1 = odd cells
    (edge (i+1,j)-(i,j+1), lines i + j = k). Returns k, first and last cell row (j1 < j0 for an empty line)."""
    k = int(0)
    j0 = int(0)
    j1 = int(-1)
    if family == 0:
        k = 2 * l - ((N - 2) - ((N - 2) & 1))
        if k >= -(N - 2) and k <= N - 2:
            j0 = wp.max(0, -k)
            j1 = wp.min(N - 2, N - 2 - k)
    else:
        k = 2 * l + 1
        if k <= 2 * N - 3:
            j0 = wp.max(0, k - (N - 2))
            j1 = wp.min(N - 2, k)
    return k, j0, j1


@wp.kernel
def k_stretch_diag(pos: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), N: int, L: float, forward: int, family: int):
    """Cell-diagonal edges chained along diagonal lines, one thread per line."""
    l, b = wp.tid()
    k, j0, j1 = diag_line(l, N, family)
    for c in range(j1 - j0 + 1):
        j = j0 + c
        if forward == 0:
            j = j1 - c
        if family == 0:
            stretch_edge(pos, w, j * N + j + k, (j + 1) * N + j + k + 1, L * SQRT2, b)
        else:
            stretch_edge(pos, w, j * N + k - j + 1, (j + 1) * N + k - j, L * SQRT2, b)


def stretch_sweep(pos, w, N, L, sub, B, device, aspect=1.0):
    """One sweep over every stretch edge: rows, columns, both diagonal families (reversed on odd substeps)."""
    if sub % 2 == 0:
        wp.launch(k_stretch_rows, (N, B), inputs=[pos, w, N, L * aspect, 1], device=device)
        wp.launch(k_stretch_cols, (N, B), inputs=[pos, w, N, L, 1], device=device)
        wp.launch(k_stretch_diag, (N, B), inputs=[pos, w, N, L * ((1+aspect*aspect)/2)**.5, 1, 0], device=device)
        wp.launch(k_stretch_diag, (N, B), inputs=[pos, w, N, L * ((1+aspect*aspect)/2)**.5, 1, 1], device=device)
    else:
        wp.launch(k_stretch_diag, (N, B), inputs=[pos, w, N, L * ((1+aspect*aspect)/2)**.5, 0, 1], device=device)
        wp.launch(k_stretch_diag, (N, B), inputs=[pos, w, N, L * ((1+aspect*aspect)/2)**.5, 0, 0], device=device)
        wp.launch(k_stretch_cols, (N, B), inputs=[pos, w, N, L, 0], device=device)
        wp.launch(k_stretch_rows, (N, B), inputs=[pos, w, N, L * aspect, 0], device=device)


@wp.kernel
def k_coarse_lines(pos: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), coarse: wp.array2d(dtype=int), coarse_d: wp.array(dtype=float),
                   line_elems: wp.array(dtype=int), line_start: wp.array(dtype=int), line_count: wp.array(dtype=int), first_line: int):
    """Coarse max-distance elements: two vertices sc cells apart may come closer (the sheet bends) but not farther apart than
    their rest distance. One thread per lattice line of one (scale, direction) group."""
    l, b = wp.tid()
    ln = first_line + l
    s0 = line_start[ln]
    for c in range(line_count[ln]):
        e = line_elems[s0 + c]
        a = coarse[e, 0]
        bb = coarse[e, 1]
        wa = w[a, b]
        wb = w[bb, b]
        dl = pos[bb, b] - pos[a, b]
        d2 = wp.dot(dl, dl)
        r = 2.0 * coarse_d[e]
        if d2 > r * r and wa + wb > 0.0:
            d = wp.sqrt(d2)
            sc = (d - r) / (d * (wa + wb))
            pos[a, b] = pos[a, b] + dl * (sc * wa)
            pos[bb, b] = pos[bb, b] - dl * (sc * wb)


@wp.kernel
def k_local(pos: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), edges: wp.array2d(dtype=int), rest: wp.array(dtype=float),
            local_list: wp.array2d(dtype=int), local_cnt: wp.array(dtype=int), iters: int):
    """Extra sweeps over the edges near active pinch grabs (nearest first), one thread per world."""
    b = wp.tid()
    for it in range(iters):
        for c in range(local_cnt[b]):
            e = local_list[c, b]
            stretch_edge(pos, w, edges[e, 0], edges[e, 1], rest[e], b)


@wp.func
def buckle_line(b: int, pos: wp.array2d(dtype=wp.vec3), ws: wp.array2d(dtype=float), nrm: wp.array2d(dtype=wp.vec3),
                N: int, L: float, eps: float, gain: float, stride: int, base: int):
    """One chain of vertices base + k stride (k = 0..N-1, rest spacing L). Every run of free vertices between kinematic ones
    (or the chain's ends) whose edges are compressed past eps lifts out of plane as a half-wave (1 - cos over the run),
    the summed lift equal to gain times the run's summed compression."""
    k0 = int(0)
    while k0 < N - 1:
        while k0 < N and ws[base + k0 * stride, b] == 0.0:
            k0 += 1
        if k0 >= N:
            break
        k1 = k0
        while k1 + 1 < N and ws[base + (k1 + 1) * stride, b] != 0.0:
            k1 += 1
        m = k1 - k0 + 1
        ea = k0
        if k0 > 0:
            ea = k0 - 1
        eb = k1
        if k1 == N - 1:
            eb = N - 2
        comp = float(0.0)
        for k in range(ea, eb + 1):
            c = L * (1.0 - eps) - wp.length(pos[base + (k + 1) * stride, b] - pos[base + k * stride, b])
            if c > 0.0:
                comp += c
        if comp > 0.0:
            wsum = float(0.0)
            for k in range(m):
                wsum += 1.0 - wp.cos(6.2831853 * float(k + 1) / float(m + 1))
            if wsum > 0.0:
                for k in range(m):
                    i = base + (k0 + k) * stride
                    nn = nrm[i, b]
                    if nn[2] < 0.0:
                        nn = -nn
                    if wp.length(nn) < 0.5:
                        nn = wp.vec3(0.0, 0.0, 1.0)
                    pos[i, b] = pos[i, b] + nn * (gain * comp * (1.0 - wp.cos(6.2831853 * float(k + 1) / float(m + 1))) / wsum)
        k0 = k1 + 1


@wp.kernel
def k_buckle_lines(pos: wp.array2d(dtype=wp.vec3), ws: wp.array2d(dtype=float), nrm: wp.array2d(dtype=wp.vec3), N: int, L: float, aspect: float,
                   eps: float, gain: float, family: int):
    """Paper carries almost no in-plane compression: a strip buckles at a load far below what any tool applies. Distance
    projections alone never resolve compression between two kinematic points (they only move it along the line), so each
    row, column and diagonal chain lifts its compressed free runs as half-waves; the stretch sweeps then settle the edge
    lengths on the longer path and the bending solve shapes the hump. A model of buckling along grid lines only."""
    l, b = wp.tid()
    if family == 0:
        buckle_line(b, pos, ws, nrm, N, L * aspect, eps, gain, 1, l * N)
    elif family == 1:
        buckle_line(b, pos, ws, nrm, N, L, eps, gain, N, l)
    else:
        k, j0, j1 = diag_line(l, N, family - 2)
        if j1 < j0:
            return
        if family == 2:
            buckle_line(b, pos, ws, nrm, j1 - j0 + 2, L * wp.sqrt(1.0+aspect*aspect), eps, gain, N + 1, j0 * N + j0 + k)
        else:
            buckle_line(b, pos, ws, nrm, j1 - j0 + 2, L * wp.sqrt(1.0+aspect*aspect), eps, gain, N - 1, j0 * N + k + 1 - j0)
