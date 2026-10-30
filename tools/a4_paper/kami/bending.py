"""Implicit bending.

Each bending element K = a + b - c1 - c2 is a linear spring toward a target vector (zero for a flat rest state, a vector of
length 2 h sin(rest / 2) along the current fold for a set crease). Its per-unit-mass stiffness is k = D / (rho_A L^4)
(the discrete scaling of a plate of bending stiffness D), doubled for the cell-diagonal hinges (`DIAG_FACTOR`, which makes
the bending of the union-jack grid isotropic: without it a fold about a 45-degree axis is 25 % too soft), halved for the
row and column triples along the free edges (they bend half-width strips), and softened by crease damage.

Per substep the position increment x solves (M + dt^2 K) x = r, with r the elastic residual (force dt^2 / mass), M the
lumped mass and K the constant stiffness matrix. K also holds two extra spring families:

- edge springs (stiffness `STRETCH_REL` x the hinge stiffness) along the stretch edges, so the bending increment respects
  the edge lengths instead of being undone by the inextensibility projections (which leaves a zigzag);
- opening springs on hinges folded past 2 asin(0.4) = 47 deg: a spring on the separation of the two outer vertices. The
  chord measure K has no moment at a closed fold; with this spring the moment tracks the linear hinge law within 10 %
  from 45 to 180 deg, so a closed fold springs open unless it has been set plastically.

Before the solve the residual is (1) capped by table static friction (the net tangential elastic force on the resting part
of the sheet must exceed mu m g before it moves; the bend cap) and (2) projected onto the edge-length constraints (the
tension projection: r' = r - W J^T lambda with J r' = 0, lambda by Gauss-Seidel sweeps along lattice lines), so the
increment is one the stretch sweeps accept and the constrained equilibrium is a fixed point of the substep.

The matrix depends only on masses, crease damage and active opening springs: it is refactored (batched block-tridiagonal
Cholesky in torch, blocks of the band width 2 N) only when one of those changes, and solved every substep by two chains of
Warp kernels.
"""
import torch
import warp as wp

from ._layout import P_D, P_RHO, P_MU_TABLE, P_SOFT

DIAG_FACTOR = 2.0      # stiffness of a cell-diagonal hinge relative to a row/column triple (isotropic bending)
STRETCH_REL = 4.0      # edge springs in the implicit matrix, relative to the hinge stiffness
TENSION_SWEEPS = 2     # Gauss-Seidel sweeps of the tension projection
OPEN_REL = 0.4         # opening springs act on hinges with |K| / 2h above this (fold > 47 deg)
GRAB_STIFF = 50.0      # hinges within one cell of a pinch grab are this much stiffer (the grabbed patch stays flat)


@wp.func
def hinge_k(e: int, b: int, par: wp.array2d(dtype=float), L: float, damage: wp.array2d(dtype=float), diag: wp.array(dtype=int),
            stiff: wp.array2d(dtype=int), bwt: wp.array(dtype=float)):
    """Per-unit-mass stiffness of bending element e (1/s^2)."""
    k0 = par[P_D, b] / (par[P_RHO, b] * L * L * L * L)
    if stiff[e, b] != 0:
        return k0 * GRAB_STIFF * bwt[e]
    if diag[e] != 0:
        k0 = k0 * DIAG_FACTOR
    return k0 * bwt[e] / (1.0 + par[P_SOFT, b] * damage[e, b])


@wp.func
def open_k(e: int, b: int, par: wp.array2d(dtype=float), L: float, diag: wp.array(dtype=int), damage: wp.array2d(dtype=float),
           bwt: wp.array(dtype=float)):
    """Per-unit-mass stiffness of the opening spring on S = a - b; pi k matches the linear hinge law near closed folds."""
    k0 = par[P_D, b] / (par[P_RHO, b] * L * L * L * L)
    if diag[e] != 0:
        k0 = k0 * 0.5                                  # the outer separation of a diagonal hinge spans L sqrt 2
    return k0 * 3.1416 * bwt[e] / (1.0 + par[P_SOFT, b] * damage[e, b])


@wp.func
def bend_target(e: int, b: int, pos: wp.array2d(dtype=wp.vec3), nrm: wp.array2d(dtype=wp.vec3), bend: wp.array2d(dtype=int),
                bend_h: wp.array(dtype=float), rest: wp.array2d(dtype=float), sigma: wp.array2d(dtype=int)):
    """Target of K for a hinge with signed rest measure `rest` = sin(rest fold / 2): length 2 h |rest|, along K itself for a
    clearly bent hinge (blended toward the mean normal of the hinge axis as the hinge flattens), signed by `sigma`, the
    side the hinge folds to."""
    tr = rest[e, b]
    a = bend[e, 0]
    bb = bend[e, 1]
    c1 = bend[e, 2]
    c2 = bend[e, 3]
    K = pos[a, b] + pos[bb, b] - pos[c1, b] - pos[c2, b]
    h2 = 2.0 * bend_h[e]
    sr = int(1)
    if tr < 0.0:
        sr = -1
    mag = wp.length(K)
    rel = mag / h2
    wl = float(0.0)
    if rel >= 0.6:
        wl = 1.0
    elif rel > 0.25:
        wl = (rel - 0.25) / 0.35
    wl = wl * wl * (3.0 - 2.0 * wl)
    d = wp.vec3(0.0)
    if wl > 0.0 and mag > 1.0e-12:
        sg = wl / mag
        if sigma[e, b] != sr:
            sg = -wl / mag
        d = K * sg
    if wl < 1.0:
        nn = nrm[c1, b] + nrm[c2, b]
        nl = wp.length(nn)
        if nl > 1.0e-6:
            d = d + nn * (float(sr) * (1.0 - wl) / nl)
    dl = wp.length(d)
    if dl > 1.0e-9:
        return d * (wp.abs(tr) * h2 / dl)
    return wp.vec3(0.0)


@wp.kernel
def k_open_activate(pos: wp.array2d(dtype=wp.vec3), bend: wp.array2d(dtype=int), bend_h: wp.array(dtype=float), open_on: wp.array2d(dtype=int),
                    dirty: wp.array(dtype=int), rel_on: float):
    e, b = wp.tid()
    K = pos[bend[e, 0], b] + pos[bend[e, 1], b] - pos[bend[e, 2], b] - pos[bend[e, 3], b]
    on = int(0)
    if wp.length(K) / (2.0 * bend_h[e]) > rel_on:
        on = 1
    if on != open_on[e, b]:
        open_on[e, b] = on
        dirty[b] = 1


@wp.kernel
def k_open_res(pos: wp.array2d(dtype=wp.vec3), bend: wp.array2d(dtype=int), bend_h: wp.array(dtype=float), diag: wp.array(dtype=int),
               open_on: wp.array2d(dtype=int), rest: wp.array2d(dtype=float), damage: wp.array2d(dtype=float), par: wp.array2d(dtype=float),
               res: wp.array2d(dtype=wp.vec3), dt: float, L: float, bwt: wp.array(dtype=float)):
    """Opening-spring residual on S = a - b, rest length 2 h cos(rest fold / 2) (2 h for a flat hinge)."""
    e, b = wp.tid()
    res[e, b] = wp.vec3(0.0)
    if open_on[e, b] == 0:
        return
    S = pos[bend[e, 0], b] - pos[bend[e, 1], b]
    ln = wp.length(S)
    if ln < 1.0e-7:
        return
    tr = wp.clamp(rest[e, b], -0.999, 0.999)
    r0 = 2.0 * bend_h[e] * wp.sqrt(1.0 - tr * tr)
    k = open_k(e, b, par, L, diag, damage, bwt) * dt * dt
    res[e, b] = S * ((r0 - ln) / ln * k)


@wp.kernel
def k_bend_elem_res(pos: wp.array2d(dtype=wp.vec3), nrm: wp.array2d(dtype=wp.vec3), bend: wp.array2d(dtype=int), bend_h: wp.array(dtype=float),
                    diag: wp.array(dtype=int), rest: wp.array2d(dtype=float), sigma: wp.array2d(dtype=int), damage: wp.array2d(dtype=float),
                    stiff: wp.array2d(dtype=int), par: wp.array2d(dtype=float), res: wp.array2d(dtype=wp.vec3), dt: float, L: float, bwt: wp.array(dtype=float)):
    e, b = wp.tid()
    K = pos[bend[e, 0], b] + pos[bend[e, 1], b] - pos[bend[e, 2], b] - pos[bend[e, 3], b]
    tgt = wp.vec3(0.0)
    if rest[e, b] != 0.0:
        tgt = bend_target(e, b, pos, nrm, bend, bend_h, rest, sigma)
    res[e, b] = (tgt - K) * (hinge_k(e, b, par, L, damage, diag, stiff, bwt) * dt * dt)


@wp.kernel
def k_bend_gather(w: wp.array2d(dtype=float), vert_elems: wp.array2d(dtype=int), vert_coef: wp.array2d(dtype=float), res: wp.array2d(dtype=wp.vec3),
                  open_res: wp.array2d(dtype=wp.vec3), open_on: wp.array2d(dtype=int), bend: wp.array2d(dtype=int), r: wp.array3d(dtype=float),
                  ne_max: int, pos: wp.array2d(dtype=wp.vec3), zfloor: float, fsum: wp.array(dtype=wp.vec3), fcnt: wp.array(dtype=float), fric_on: int):
    """Vertex residual (force dt^2 / mass) and, with `fric_on`, the net tangential residual over vertices resting on the table."""
    i, b = wp.tid()
    acc = wp.vec3(0.0)
    if w[i, b] != 0.0:
        for s in range(ne_max):
            e = vert_elems[i, s]
            if e < 0:
                break
            acc += res[e, b] * vert_coef[i, s]
            if open_on[e, b] != 0:
                if i == bend[e, 0]:
                    acc += open_res[e, b]
                elif i == bend[e, 1]:
                    acc -= open_res[e, b]
        if fric_on != 0 and pos[i, b][2] < zfloor:
            wp.atomic_add(fsum, b, wp.vec3(acc[0], acc[1], 0.0))
            wp.atomic_add(fcnt, b, 1.0)
    r[b, i, 0] = acc[0]
    r[b, i, 1] = acc[1]
    r[b, i, 2] = acc[2]


@wp.kernel
def k_bend_cap(w: wp.array2d(dtype=float), r: wp.array3d(dtype=float), pos: wp.array2d(dtype=wp.vec3), vert_edges: wp.array2d(dtype=int),
               vert_rest: wp.array2d(dtype=float), par: wp.array2d(dtype=float), dt: float, L: float, zfloor: float, g: float,
               fsum: wp.array(dtype=wp.vec3), fcnt: wp.array(dtype=float), fric_on: int):
    """Bend cap: the resting part of the sheet moves in its plane only if the net tangential elastic force exceeds its static
    table friction (mu m g); only the excess acts. Then the edge-spring residuals are added, uncapped."""
    i, b = wp.tid()
    if w[i, b] == 0.0:
        return
    acc = wp.vec3(r[b, i, 0], r[b, i, 1], r[b, i, 2])
    if fric_on != 0 and pos[i, b][2] < zfloor:
        F = fsum[b]
        fl = wp.sqrt(F[0] * F[0] + F[1] * F[1])
        capacity = fcnt[b] * par[P_MU_TABLE, b] * g * dt * dt
        fs = float(0.0)
        if fl > capacity and fl > 1.0e-12:
            fs = (fl - capacity) / fl
        acc = wp.vec3(acc[0] * fs, acc[1] * fs, acc[2])
    ks = STRETCH_REL * par[P_D, b] / (par[P_RHO, b] * L * L * L * L) * dt * dt
    for s in range(8):
        j = vert_edges[i, s]
        if j < 0:
            break
        d = pos[j, b] - pos[i, b]
        ln = wp.length(d)
        if ln > 1.0e-9:
            acc += d * ((ln - vert_rest[i, s]) / ln * ks)
    r[b, i, 0] = acc[0]
    r[b, i, 1] = acc[1]
    r[b, i, 2] = acc[2]


@wp.kernel
def k_fric_zero(fsum: wp.array(dtype=wp.vec3), fcnt: wp.array(dtype=float)):
    b = wp.tid()
    fsum[b] = wp.vec3(0.0)
    fcnt[b] = 0.0


# --- tension projection
@wp.func
def tension_res(i: int, b: int, w: wp.array2d(dtype=float), pos: wp.array2d(dtype=wp.vec3), r: wp.array3d(dtype=float), lam: wp.array2d(dtype=float),
                vert_edges: wp.array2d(dtype=int), vert_eid: wp.array2d(dtype=int)):
    """r'_i = r_i + w_i sum_e d_hat(i -> j) lambda_e. Kinematic and heavily weighted (grabbed, w < 0.5) vertices are fixed."""
    wi = w[i, b]
    if wi < 0.5:
        return wp.vec3(0.0)
    acc = wp.vec3(r[b, i, 0], r[b, i, 1], r[b, i, 2])
    for s in range(8):
        j = vert_edges[i, s]
        if j < 0:
            break
        d = pos[j, b] - pos[i, b]
        ln = wp.length(d)
        if ln > 1.0e-9:
            acc += d * (wi * lam[vert_eid[i, s], b] / ln)
    return acc


@wp.func
def edge_slot(i: int, j: int, vert_edges: wp.array2d(dtype=int)):
    for s in range(8):
        if vert_edges[i, s] == j:
            return s
    return -1


@wp.func
def tension_edge(a: int, bb: int, ra: wp.vec3, b: int, w: wp.array2d(dtype=float), pos: wp.array2d(dtype=wp.vec3), r: wp.array3d(dtype=float),
                 lam: wp.array2d(dtype=float), vert_edges: wp.array2d(dtype=int), vert_eid: wp.array2d(dtype=int)):
    """Gauss-Seidel update of lambda on edge a-bb given r'_a; returns r'_bb after the update."""
    rb = tension_res(bb, b, w, pos, r, lam, vert_edges, vert_eid)
    s = edge_slot(a, bb, vert_edges)
    if s < 0:
        return rb
    wa = w[a, b]
    wb = w[bb, b]
    if wa < 0.5:
        wa = 0.0
    if wb < 0.5:
        wb = 0.0
    if wa + wb == 0.0:
        return rb
    d = pos[bb, b] - pos[a, b]
    ln = wp.length(d)
    if ln < 1.0e-9:
        return rb
    dl = wp.dot(d, rb - ra) / (ln * (wa + wb))
    e = vert_eid[a, s]
    lam[e, b] = lam[e, b] + dl
    return rb - d * (wb * dl / ln)


@wp.kernel
def k_tension_lines(w: wp.array2d(dtype=float), pos: wp.array2d(dtype=wp.vec3), r: wp.array3d(dtype=float), lam: wp.array2d(dtype=float),
                    vert_edges: wp.array2d(dtype=int), vert_eid: wp.array2d(dtype=int), N: int, skip: wp.array(dtype=int), family: int):
    """One thread per lattice line of one family (rows, columns, the two diagonal directions); launch families in turn."""
    l, b = wp.tid()
    if skip[b] != 0:
        return
    if family == 0:
        ra = tension_res(l * N, b, w, pos, r, lam, vert_edges, vert_eid)
        for c in range(N - 1):
            ra = tension_edge(l * N + c, l * N + c + 1, ra, b, w, pos, r, lam, vert_edges, vert_eid)
    elif family == 1:
        ra = tension_res(l, b, w, pos, r, lam, vert_edges, vert_eid)
        for c in range(N - 1):
            ra = tension_edge(c * N + l, (c + 1) * N + l, ra, b, w, pos, r, lam, vert_edges, vert_eid)
    elif family == 2:
        k = 2 * l - (N - 1)
        j0 = wp.max(0, -k)
        j1 = wp.min(N - 1, N - 1 - k)
        ra = tension_res(j0 * N + (j0 + k), b, w, pos, r, lam, vert_edges, vert_eid)
        for j in range(j0, j1):
            ra = tension_edge(j * N + (j + k), (j + 1) * N + (j + 1 + k), ra, b, w, pos, r, lam, vert_edges, vert_eid)
    else:
        k = 2 * l
        j0 = wp.max(0, k - (N - 1))
        j1 = wp.min(N - 1, k)
        ra = tension_res(j0 * N + (k - j0), b, w, pos, r, lam, vert_edges, vert_eid)
        for j in range(j0, j1):
            ra = tension_edge(j * N + (k - j), (j + 1) * N + (k - j - 1), ra, b, w, pos, r, lam, vert_edges, vert_eid)


@wp.kernel
def k_tension_apply(w: wp.array2d(dtype=float), pos: wp.array2d(dtype=wp.vec3), r: wp.array3d(dtype=float), lam: wp.array2d(dtype=float),
                    vert_edges: wp.array2d(dtype=int), vert_eid: wp.array2d(dtype=int), out: wp.array3d(dtype=float), skip: wp.array(dtype=int)):
    i, b = wp.tid()
    v = wp.vec3(r[b, i, 0], r[b, i, 1], r[b, i, 2])
    if skip[b] == 0:
        v = tension_res(i, b, w, pos, r, lam, vert_edges, vert_eid)
    out[b, i, 0] = v[0]
    out[b, i, 1] = v[1]
    out[b, i, 2] = v[2]


@wp.kernel
def k_bend_apply(w: wp.array2d(dtype=float), pos: wp.array2d(dtype=wp.vec3), delta: wp.array3d(dtype=float)):
    i, b = wp.tid()
    if w[i, b] != 0.0:
        pos[i, b] = pos[i, b] + wp.vec3(delta[b, i, 0], delta[b, i, 1], delta[b, i, 2])


# --- assembly of the block-tridiagonal matrix (blocks of size bs >= the half bandwidth)
@wp.kernel
def k_bend_diag(w: wp.array2d(dtype=float), D: wp.array4d(dtype=float), n: int, bs: int):
    i, b = wp.tid()
    blk = i / bs
    v = float(1.0)
    if i < n:
        if w[i, b] > 0.0:
            v = 1.0 / w[i, b]
    D[b, blk, i - blk * bs, i - blk * bs] = v


@wp.func
def add_entry(D: wp.array4d(dtype=float), E: wp.array4d(dtype=float), b: int, i: int, q: int, bs: int, val: float):
    """Add val at (row i, column q <= i + bs) of the lower block-tridiagonal matrix (diagonal blocks D, sub-diagonal E)."""
    bi = i / bs
    bq = q / bs
    if bq == bi:
        D[b, bi, i - bi * bs, q - bq * bs] = D[b, bi, i - bi * bs, q - bq * bs] + val
    elif bi == bq + 1:
        E[b, bq, i - bi * bs, q - bq * bs] = E[b, bq, i - bi * bs, q - bq * bs] + val


@wp.kernel
def k_bend_blocks(w: wp.array2d(dtype=float), vert_elems: wp.array2d(dtype=int), vert_coef: wp.array2d(dtype=float), bend: wp.array2d(dtype=int),
                  diag: wp.array(dtype=int), damage: wp.array2d(dtype=float), stiff: wp.array2d(dtype=int), par: wp.array2d(dtype=float),
                  D: wp.array4d(dtype=float), E: wp.array4d(dtype=float), open_on: wp.array2d(dtype=int), vert_edges: wp.array2d(dtype=int),
                  bs: int, ne_max: int, dt: float, L: float, bwt: wp.array(dtype=float)):
    """Row i of dt^2 K (mass rows come from k_bend_diag). Kinematic vertices (w = 0) are not unknowns."""
    i, b = wp.tid()
    if w[i, b] == 0.0:
        return
    dt2 = dt * dt
    ks = STRETCH_REL * par[P_D, b] / (par[P_RHO, b] * L * L * L * L) * dt2
    for s in range(8):                                   # edge springs
        j = vert_edges[i, s]
        if j < 0:
            break
        add_entry(D, E, b, i, i, bs, ks)
        if w[j, b] != 0.0:
            add_entry(D, E, b, i, j, bs, -ks)
    for s in range(ne_max):                              # opening springs: pattern (+1 on a, -1 on b)
        e = vert_elems[i, s]
        if e < 0:
            break
        if open_on[e, b] != 0 and (i == bend[e, 0] or i == bend[e, 1]):
            other = bend[e, 1]
            if i == other:
                other = bend[e, 0]
            ko = open_k(e, b, par, L, diag, damage, bwt) * dt2
            add_entry(D, E, b, i, i, bs, ko)
            if w[other, b] != 0.0:
                add_entry(D, E, b, i, other, bs, -ko)
    for s in range(ne_max):                              # hinges
        e = vert_elems[i, s]
        if e < 0:
            break
        k = hinge_k(e, b, par, L, damage, diag, stiff, bwt) * dt2 * vert_coef[i, s]
        c1 = bend[e, 2]
        if w[bend[e, 0], b] != 0.0:
            add_entry(D, E, b, i, bend[e, 0], bs, k)
        if w[bend[e, 1], b] != 0.0:
            add_entry(D, E, b, i, bend[e, 1], bs, k)
        if c1 == bend[e, 3]:
            if w[c1, b] != 0.0:
                add_entry(D, E, b, i, c1, bs, -2.0 * k)
        else:
            if w[c1, b] != 0.0:
                add_entry(D, E, b, i, c1, bs, -k)
            if w[bend[e, 3], b] != 0.0:
                add_entry(D, E, b, i, bend[e, 3], bs, -k)


@wp.kernel
def k_bsolve_fwd(LinvT: wp.array4d(dtype=float), PT: wp.array4d(dtype=float), rhs: wp.array3d(dtype=float), y: wp.array3d(dtype=float), k: int, bs: int):
    """y_k = Linv_k rhs_k - P_k y_{k-1}; one thread per (world, row), matrices read transposed for coalesced access."""
    b, r = wp.tid()
    base = k * bs
    a = wp.vec3(0.0)
    for c in range(bs):
        l = LinvT[b, k, c, r]
        a += wp.vec3(rhs[b, base + c, 0], rhs[b, base + c, 1], rhs[b, base + c, 2]) * l
    if k > 0:
        for c in range(bs):
            pv = PT[b, k, c, r]
            a -= wp.vec3(y[b, base - bs + c, 0], y[b, base - bs + c, 1], y[b, base - bs + c, 2]) * pv
    y[b, base + r, 0] = a[0]
    y[b, base + r, 1] = a[1]
    y[b, base + r, 2] = a[2]


@wp.kernel
def k_bsolve_bwd(Linv: wp.array4d(dtype=float), QT: wp.array4d(dtype=float), y: wp.array3d(dtype=float), x: wp.array3d(dtype=float), k: int, bs: int, K: int):
    """x_k = Linv_k^T y_k - Q_k x_{k+1}."""
    b, r = wp.tid()
    base = k * bs
    a = wp.vec3(0.0)
    for c in range(bs):
        l = Linv[b, k, c, r]
        a += wp.vec3(y[b, base + c, 0], y[b, base + c, 1], y[b, base + c, 2]) * l
    if k < K - 1:
        for c in range(bs):
            qv = QT[b, k, c, r]
            a -= wp.vec3(x[b, base + bs + c, 0], x[b, base + bs + c, 1], x[b, base + bs + c, 2]) * qv
    x[b, base + r, 0] = a[0]
    x[b, base + r, 1] = a[1]
    x[b, base + r, 2] = a[2]


class BendSolver:
    """Batched block-tridiagonal Cholesky in float32 on the position increment (no repeated rounding bias). `factor()` runs
    eagerly in torch; `solve()` is Warp launches only, so it can live inside a captured CUDA graph."""

    def __init__(self, B, n, bs, device):
        self.B, self.n, self.bs, self.device = B, n, bs, device
        self.K = K = -(-n // bs)
        self.n_pad = K * bs
        z = lambda *shape: torch.zeros(*shape, device=device, dtype=torch.float32)
        self.D, self.E = z(B, K, bs, bs), z(B, max(K - 1, 1), bs, bs)
        self.Linv, self.LinvT, self.P, self.Q = z(B, K, bs, bs), z(B, K, bs, bs), z(B, K, bs, bs), z(B, K, bs, bs)
        self.r, self.y, self.delta = z(B, self.n_pad, 3), z(B, self.n_pad, 3), z(B, self.n_pad, 3)
        self.L, self.S, self.Lk1k, self.tmp = z(B, bs, bs), z(B, bs, bs), z(B, bs, bs), z(B, bs, bs)
        self.info = torch.zeros(B, device=device, dtype=torch.int32)
        self.eye = torch.eye(bs, device=device, dtype=torch.float32).expand(B, bs, bs).contiguous()
        f = wp.from_torch
        self.D_w, self.E_w, self.Linv_w, self.LinvT_w = f(self.D), f(self.E), f(self.Linv), f(self.LinvT)
        self.P_w, self.Q_w, self.r_w, self.y_w, self.delta_w = f(self.P), f(self.Q), f(self.r), f(self.y), f(self.delta)

    def factor(self):
        D, E, S, L = self.D, self.E, self.S, self.L
        S.copy_(D[:, 0])
        for k in range(self.K):
            torch.linalg.cholesky_ex(S, out=(L, self.info))
            Linv = self.Linv[:, k]
            torch.linalg.solve_triangular(L, self.eye, upper=False, out=Linv)
            self.LinvT[:, k].copy_(Linv.transpose(-1, -2))
            if k > 0:                                                   # P holds P_k transposed
                torch.bmm(Linv, self.Lk1k, out=self.tmp)
                self.P[:, k].copy_(self.tmp.transpose(-1, -2))
            if k < self.K - 1:                                          # Q holds Q_k transposed = L_{k+1,k} Linv_k
                torch.bmm(E[:, k], Linv.transpose(-1, -2), out=self.Lk1k)
                torch.bmm(self.Lk1k, Linv, out=self.tmp)
                self.Q[:, k].copy_(self.tmp)
                torch.bmm(self.Lk1k, self.Lk1k.transpose(-1, -2), out=self.tmp)
                torch.sub(D[:, k + 1], self.tmp, out=S)

    def solve(self):
        for k in range(self.K):
            wp.launch(k_bsolve_fwd, (self.B, self.bs), inputs=[self.LinvT_w, self.P_w, self.r_w, self.y_w, k, self.bs], device=self.device)
        for k in range(self.K - 1, -1, -1):
            wp.launch(k_bsolve_bwd, (self.B, self.bs), inputs=[self.Linv_w, self.Q_w, self.y_w, self.delta_w, k, self.bs, self.K], device=self.device)
