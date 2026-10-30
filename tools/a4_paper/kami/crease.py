"""Crease laws: plasticity of the hinges, advanced once per frame after the substeps.

Both laws write the hinge rest measure `rest` (sin of half the rest fold angle, signed; the bending solve's target) and the
softening state `damage` (hinge stiffness / (1 + softening x damage)).

- K1c (`k_k1_*`): the crease clock. Inputs are the true turning angle of each hinge (the angle between the two edges of a
  row/column triple, the dihedral angle for a cell diagonal), signed by the side it folds to. One neighbour pass along
  each hinge's grid line gives window sums, so a crease carried by two or three hinges is treated as one crease (one
  springback allowance, one plastic rotation). See `CreaseLaw` for the constants.
- classic (`k_classic`): yield, rate and cap act on the fold angle of each hinge measured from the chord |K| = 2 h sin(fold / 2).
"""
import warp as wp

from ._layout import P_D

PI = 3.14159265


@wp.func
def turning_angle(a: wp.vec3, b: wp.vec3, c: wp.vec3):
    """Turning angle at c of the chain a-c-b: 0 straight, pi folded back."""
    u = a - c
    v = b - c
    lu = wp.length(u)
    lv = wp.length(v)
    if lu < 1.0e-9 or lv < 1.0e-9:
        return 0.0
    return PI - wp.acos(wp.clamp(wp.dot(u / lu, v / lv), -1.0, 1.0))


@wp.func
def dihedral_angle(e1: wp.vec3, e2: wp.vec3, pa: wp.vec3, pb: wp.vec3):
    """Unsigned fold angle between triangles (e1, e2, pa) and (e2, e1, pb) about the edge e1-e2: 0 flat, pi closed."""
    edge = e2 - e1
    le = wp.max(wp.length(edge), 1.0e-9)
    n1 = wp.cross(edge, pa - e1)
    n2 = wp.cross(pb - e1, edge)
    u = n1 / wp.sqrt(wp.max(wp.dot(n1, n1), 1.0e-20))
    v = n2 / wp.sqrt(wp.max(wp.dot(n2, n2), 1.0e-20))
    return wp.abs(wp.atan2(wp.dot(wp.cross(u, v), edge / le), wp.dot(u, v)))


@wp.kernel
def k_measure(pos: wp.array2d(dtype=wp.vec3), nrm: wp.array2d(dtype=wp.vec3), bend: wp.array2d(dtype=int), bend_h: wp.array(dtype=float),
              sigma: wp.array2d(dtype=int), theta: wp.array2d(dtype=float), chord: wp.array2d(dtype=float)):
    """Signed fold angle of every hinge: `theta` the true turning angle, `chord` 2 asin(|K| / 2h). The sign `sigma` (the side
    the hinge folds to, from K against the hinge's normal) is only updated while the fold is below 60 deg, so a closed fold
    keeps its side."""
    e, b = wp.tid()
    a = bend[e, 0]
    bb = bend[e, 1]
    c1 = bend[e, 2]
    c2 = bend[e, 3]
    K = pos[a, b] + pos[bb, b] - pos[c1, b] - pos[c2, b]
    rel = wp.min(wp.length(K) / (2.0 * bend_h[e]), 1.0)
    sg = sigma[e, b]
    if rel < 0.5:
        dn = wp.dot(K, nrm[c1, b] + nrm[c2, b])
        if dn > 1.0e-9:
            sg = 1
        elif dn < -1.0e-9:
            sg = -1
        sigma[e, b] = sg
    th = float(0.0)
    if c1 == c2:
        th = turning_angle(pos[a, b], pos[bb, b], pos[c1, b])
    else:
        th = dihedral_angle(pos[c1, b], pos[c2, b], pos[a, b], pos[bb, b])
    theta[e, b] = float(sg) * th
    chord[e, b] = float(sg) * 2.0 * wp.asin(rel)


@wp.func
def signed_sin_half(x: float):
    s = float(1.0)
    if x < 0.0:
        s = -1.0
    return s * wp.sin(0.5 * wp.abs(x))


@wp.func
def signed_angle(x: float):
    """Fold angle from the signed sin(fold / 2) measure."""
    s = float(1.0)
    if x < 0.0:
        s = -1.0
    return s * 2.0 * wp.asin(wp.min(wp.abs(x), 1.0))


@wp.kernel
def k_classic(chord: wp.array2d(dtype=float), rest: wp.array2d(dtype=float), damage: wp.array2d(dtype=float), crease: wp.array2d(dtype=float),
              noyield: wp.array2d(dtype=int), damage_at: wp.array2d(dtype=float), worst: wp.array(dtype=float),
              yield0: float, yield_floor: float, frac: float, theta_max: float):
    e, b = wp.tid()
    dm = damage[e, b]
    th = chord[e, b]
    tr = signed_angle(rest[e, b])
    yl = 2.0 * wp.asin(wp.min(yield0 * (1.0 - dm) * (1.0 - dm) + yield_floor, 1.0))
    cap = 2.0 * wp.asin(wp.min(theta_max, 1.0))
    diff = th - tr
    ad = wp.abs(diff)
    if ad > yl and noyield[e, b] == 0:
        step = (ad - yl) * frac
        if diff > 0.0:
            tr += step
        else:
            tr -= step
        tr = wp.clamp(tr, -cap, cap)
        r = signed_sin_half(tr)
        rest[e, b] = r
        if wp.abs(r) > dm:
            dm = wp.abs(r)
            damage[e, b] = dm
    crease[e, b] = dm
    wp.atomic_max(worst, b, dm - damage_at[e, b])


@wp.kernel
def k_k1_member(th: wp.array2d(dtype=float), rp: wp.array2d(dtype=float), nbr: wp.array2d(dtype=int), bend_h: wp.array(dtype=float),
                k1: wp.array(dtype=float), m_out: wp.array2d(dtype=float)):
    """Crease membership: the hinge or a direct neighbour is bent, or plastically set, past the elastic bend kappa_y h."""
    e, b = wp.tid()
    amax = wp.abs(th[e, b])
    rmax = wp.abs(rp[e, b])
    for q in range(1, 3):
        j = nbr[e, q]
        if j >= 0:
            amax = wp.max(amax, wp.abs(th[j, b]))
            rmax = wp.max(rmax, wp.abs(rp[j, b]))
    ref = k1[13] * bend_h[e]
    m = wp.clamp((amax / ref - 1.0) / 0.25, 0.0, 1.0)
    m_out[e, b] = wp.max(m, wp.clamp((rmax / ref - 1.0) / 0.25, 0.0, 1.0))


@wp.kernel
def k_k1_window(th: wp.array2d(dtype=float), m: wp.array2d(dtype=float), rp: wp.array2d(dtype=float), c: wp.array2d(dtype=float),
                nbr: wp.array2d(dtype=int), win: wp.array2d(dtype=wp.vec4)):
    """Window sums over crease members: total |angle| over 5 hinges, peak-to-total ratio, set and creep over 3 hinges."""
    e, b = wp.tid()
    am = wp.abs(th[e, b]) * m[e, b]
    T5 = am
    mx = am
    RP3 = rp[e, b] * m[e, b]
    C3 = c[e, b] * m[e, b]
    for q in range(4):
        j = nbr[e, q]
        if j >= 0:
            aj = wp.abs(th[j, b]) * m[j, b]
            T5 += aj
            mx = wp.max(mx, aj)
            if q == 1 or q == 2:
                RP3 += rp[j, b] * m[j, b]
                C3 += c[j, b] * m[j, b]
    win[e, b] = wp.vec4(T5, mx / wp.max(T5, 1.0e-12), RP3, C3)


@wp.func
def is_free(v: int, b: int, w: wp.array2d(dtype=float), pressed: wp.array2d(dtype=int), press: wp.array2d(dtype=float)):
    return w[v, b] != 0.0 and pressed[v, b] == 0 and press[v, b] <= 0.0


@wp.kernel
def k_k1_law(th: wp.array2d(dtype=float), m: wp.array2d(dtype=float), win: wp.array2d(dtype=wp.vec4), w: wp.array2d(dtype=float),
             pressed: wp.array2d(dtype=int), press: wp.array2d(dtype=float), noyield: wp.array2d(dtype=int), bend: wp.array2d(dtype=int),
             bend_h: wp.array(dtype=float), par: wp.array2d(dtype=float), k1: wp.array(dtype=float),
             rp_a: wp.array2d(dtype=float), c_a: wp.array2d(dtype=float), d_a: wp.array2d(dtype=float), w_a: wp.array2d(dtype=float),
             rpm_a: wp.array2d(dtype=float), mf_a: wp.array2d(dtype=float), wk_a: wp.array2d(dtype=float),
             rest: wp.array2d(dtype=float), damage: wp.array2d(dtype=float), crease: wp.array2d(dtype=float),
             damage_at: wp.array2d(dtype=float), worst: wp.array(dtype=float), dt: float, cap: float):
    """K1c update of one hinge over one frame of length dt. State: plastic set rp, creep c, damage d, crease width w, peak
    set rpm, refold memory mf, sharpest width wk. Writes the rest measure sin(rest / 2) and damage = L_d d / h (with
    softening 1 this is the crease zone compliance h / (h + L_d d))."""
    e, b = wp.tid()
    if noyield[e, b] != 0:
        return
    a = bend[e, 0]
    bb = bend[e, 1]
    c1 = bend[e, 2]
    c2 = bend[e, 3]
    hd = float(1.0)                                # held: every vertex kinematic or pressed by a tool
    if is_free(a, b, w, pressed, press) or is_free(bb, b, w, pressed, press) or is_free(c1, b, w, pressed, press) or is_free(c2, b, w, pressed, press):
        hd = 0.0
    m_y = k1[0]
    h_k = k1[1]
    w0 = k1[2]
    w_min = k1[3]
    f_c = k1[4]
    L_d = k1[5]
    rate_s = k1[6]
    Ma = k1[7]
    H_c = k1[8]
    bk = k1[9]
    w_y = k1[10]
    w_reset = k1[11]
    s_d = k1[12]
    ky = k1[13]
    Bst = par[P_D, b]
    L = bend_h[e]
    theta = th[e, b]
    rp = rp_a[e, b]
    c = c_a[e, b]
    d = d_a[e, b]
    wz = w_a[e, b]
    mf = mf_a[e, b]
    f = wp.max(press[c1, b], press[c2, b])         # press line load on the hinge (N/m), mean over the frame
    if f > 0.0:
        wz = wp.min(wz, w_min + (w0 - w_min) / (1.0 + f / f_c))
    mem = m[e, b]
    wv = win[e, b]
    T5 = wv[0]
    P5 = wv[1]
    RP3 = wv[2]
    CC = wv[3]
    sm = wp.clamp((0.4 - P5) / 0.15, 0.0, 1.0)     # 1 for a resolved (uniform) bend, 0 for a localised crease
    share = wp.min(wp.abs(theta) * mem / wp.max(T5, 1.0e-12), 1.0)
    RP = rp
    if mem > 0.0:
        RP = RP3
    rpm = wp.max(rpm_a[e, b], wp.abs(RP))
    wk = wp.min(wk_a[e, b], wz)
    if w_reset > 0.0 and rpm > 1.0e-6 and f <= 0.0:          # a flattened crease: smooth ramp of the undone fraction
        u = 1.0 - wp.abs(RP) / rpm
        r = wp.clamp((u - (1.0 - w_reset - 0.2)) / 0.4, 0.0, 1.0)
        mf = wp.max(mf, r * d * wp.min(wk / w0, 1.0))
        wz = wp.max(wz, wk + r * (w0 - wk))
    lam = L + L_d * d
    lz = wz + L_d * d
    MY = m_y * Bst * ky * (1.0 + h_k * wp.abs(RP)) * (1.0 - s_d * mf)
    e_res = MY * lam / Bst
    e_kink = share * MY * lz / Bst
    e_y = wp.max(hd * ((1.0 - mem) * e_res + mem * (sm * e_res + (1.0 - sm) * e_kink)) + (1.0 - hd) * e_res, 1.0e-9)
    kM = MY / e_y
    shc = mem * share + (1.0 - mem)
    X = H_c * (mem * (CC + bk * RP) + (1.0 - mem) * (c + bk * rp))
    sh = bk * X / kM
    frac = 1.0 - wp.exp(-rate_s * dt)
    diff = (theta - c - sh) - rp
    ad = wp.abs(diff)
    rp_new = rp
    yielding = int(0)
    if ad > e_y:                                    # rate-independent set, relaxed toward the yield surface at rate_s
        yielding = 1
        sgd = float(1.0)
        if diff < 0.0:
            sgd = -1.0
        rp_new = wp.clamp(rp + sgd * (ad - e_y) * frac, -cap, cap)
    dn = wp.sin(0.5 * wp.min(wp.abs(RP + (rp_new - rp)), PI))
    if yielding != 0 and dn > d:
        d = dn
    # creep over the frame, exact: tanh(u / 2) -> exp(-lambda) tanh(u0 / 2), in a form that cannot underflow
    ee = theta - rp_new - c
    RPn2 = RP + mem * (rp_new - rp)
    X = H_c * (mem * (CC + bk * RPn2) + (1.0 - mem) * (c + bk * rp_new))
    u0 = wp.clamp(ee * kM - X, -MY, MY) / Ma
    sc = (shc * kM + H_c) / Ma
    r = MY / Ma
    A = wp.abs(u0)
    lsh = r + wp.log(wp.max(0.5 * (1.0 - wp.exp(-2.0 * r)), 1.0e-30))      # log sinh(r)
    llam = wp.log(sc * dt * w_y) - lsh                                      # log lambda, lambda = sc dt w_y / sinh(r)
    eA = wp.exp(-A)
    om = 2.0 * eA / (1.0 + eA)                                              # 1 - tanh(A / 2)
    lamA = float(0.0)                                                       # (1 - exp(-lambda)) exp(A)
    if llam < -7.0:
        lamA = wp.exp(llam + A)
    else:
        lamA = (1.0 - wp.exp(-wp.exp(llam))) * wp.exp(A)
    q = 2.0 / (1.0 + eA) + lamA * (1.0 - om)
    du = -wp.log(wp.max(2.0 / q - eA, eA))
    su = float(1.0)
    if u0 < 0.0:
        su = -1.0
    c_new = c + shc * su * du / sc
    rp_a[e, b] = rp_new
    c_a[e, b] = c_new
    d_a[e, b] = d
    w_a[e, b] = wz
    rpm_a[e, b] = rpm
    mf_a[e, b] = mf
    wk_a[e, b] = wk
    rest[e, b] = signed_sin_half(wp.clamp(rp_new + c_new, -cap, cap))
    dm = L_d * d / L
    damage[e, b] = dm
    crease[e, b] = d
    wp.atomic_max(worst, b, dm - damage_at[e, b])
