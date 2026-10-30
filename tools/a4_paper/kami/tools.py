"""Generic tools acting on the sheet: capsule tools, pins and pinch grabs.

Capsule tools (`k_tools`, every substep). A tool is a rigid capsule: a segment a-b with radius r (a sphere when a = b),
optionally with a flat core: a strip of half-width hw across the segment in the horizontal plane (a flat bar, e.g. a bone
folder), or for a sphere a horizontal disc of radius hw (a fingertip pad flattened by pressing).
Endpoints are commanded once per frame and interpolated over the substeps. A vertex inside a tool is pushed out along the
surface normal; Coulomb friction (budget mu x penetration) drags it with the tool's surface. A tool given a press force
presses: paper it touches from above on a support (the table, or a stack within `SUPPORT_DEPTH` of it) carries the press
line load (force / contact length, N/m) to the crease law, which treats its hinges as held. Squeezed between such a tool
and the table, paper follows the surface with the larger friction coefficient. A tool is rigid and placed by position:
put a pressing tool on the surface of the stack (sunk a fraction of the layer spacing), not into it. Contact is between
the tool and the sheet's vertices: a round tip smaller than a cell can sit between vertices without touching any, so give
it a flat pad (hw of about a cell) or place it over a vertex. To hold paper in place, pin it.

Pins (`k_pin_*`). Any vertex can be made kinematic (inverse mass 0) and moved to a target, interpolated per substep.

Pinch grabs (`k_control`, `k_grab_drive`, `k_slip`). A grab takes the vertex nearest to a point (within a reach), makes it
heavy (inverse mass 0.02) and its 8 neighbours moderately heavy (0.12), and every substep pulls the vertex toward the
point (0.9 of the way, at most 0.6 m/s) and the ring toward a flat patch around it that is free to turn, like paper held
between two fingertips. Hinges within one cell are stiffened (the patch stays flat) and within four cells do not yield; the
edges within four cells get extra stretch sweeps. A grab lets go when the paper lags the point by more than
`GRAB_SLIP` for `GRAB_SLIP_TIME`.
"""
import warp as wp

from ._layout import P_MU_TABLE, RING, SAVED

SUPPORT_DEPTH = 4.0e-3    # m above the table within which paper counts as supported (a stack of ~10 layers)
GRAB_W, RING_W = 0.02, 0.12
DRIVE_SPEED = 0.6         # m/s: fastest a grab drags paper toward its point
GRAB_SLIP = 6.0e-3        # m
GRAB_SLIP_TIME = 0.1      # s
LOCAL_SWEEPS = 4


# ------------------------------------------------------------------------------------------------ capsule tools
@wp.kernel
def k_tools(pos: wp.array2d(dtype=wp.vec3), prev: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), par: wp.array2d(dtype=float),
            tool_on: wp.array2d(dtype=int), tool_a: wp.array2d(dtype=wp.vec3), tool_b: wp.array2d(dtype=wp.vec3),
            tool_pa: wp.array2d(dtype=wp.vec3), tool_pb: wp.array2d(dtype=wp.vec3), tool_r: wp.array2d(dtype=float),
            tool_hw: wp.array2d(dtype=float), tool_mu: wp.array2d(dtype=float), tool_load: wp.array2d(dtype=float),
            pressed: wp.array2d(dtype=int), touch: wp.array2d(dtype=int), t_n: wp.array2d(dtype=wp.vec3), t_v: wp.array2d(dtype=wp.vec3),
            press: wp.array2d(dtype=float), C: int, fr: float, inv_sub: float, t: float, z_support: float):
    """pressed: 0 untouched, 1 pressed by a tool, 2 pressed onto the table by a tool that grips less than the table (the
    table holds it in place, k_table). touch, t_n, t_v: contact normal and tool surface displacement for k_table's inelastic
    velocity update. press: line load (N/m) accumulated as a mean over the frame's substeps."""
    i, b = wp.tid()
    pressed[i, b] = 0
    touch[i, b] = 0
    if w[i, b] == 0.0:
        return
    P = pos[i, b]
    Q = prev[i, b]
    acc = wp.vec3(0.0)
    cnt = int(0)
    state = int(0)
    load = float(0.0)
    for c in range(C):
        if tool_on[c, b] == 0:
            continue
        a1 = tool_a[c, b]
        b1 = tool_b[c, b]
        a0 = tool_pa[c, b]
        b0 = tool_pb[c, b]
        ca = a0 + (a1 - a0) * fr
        cb = b0 + (b1 - b0) * fr
        R = tool_r[c, b] + 0.5 * t
        seg = cb - ca
        dd = wp.dot(seg, seg)
        s = float(0.0)
        if dd > 1.0e-16:
            s = wp.clamp(wp.dot(P - ca, seg) / dd, 0.0, 1.0)
        e = P - (ca + seg * s)
        hw = tool_hw[c, b]
        hl = wp.sqrt(seg[0] * seg[0] + seg[1] * seg[1])
        if hw > 0.0:
            if hl > 1.0e-9:                              # flat bar: core widened by hw across the axis, horizontally
                px = wp.vec3(-seg[1] / hl, seg[0] / hl, 0.0)
                e = e - px * wp.clamp(wp.dot(e, px), -hw, hw)
            else:                                        # flat pad: a sphere's core widened into a horizontal disc of radius hw
                rh = wp.sqrt(e[0] * e[0] + e[1] * e[1])
                if rh > 0.0:
                    sh = (rh - wp.min(rh, hw)) / rh
                    e = wp.vec3(e[0] * sh, e[1] * sh, e[2])
        el = wp.length(e)
        if el >= R:
            continue
        nn = wp.vec3(0.0, 0.0, 1.0)
        if el > 1.0e-9:
            nn = e / el
        pen = R - el
        corr = nn * pen
        da = (a1 - a0) * inv_sub
        sv = da + ((b1 - b0) * inv_sub - da) * s       # the tool surface's displacement this substep
        mu = tool_mu[c, b]
        on_support = int(0)                              # pressing from above onto supported paper
        if nn[2] < -0.7 and P[2] < z_support:
            on_support = 1
        table_wins = int(0)
        if on_support != 0 and tool_load[c, b] > 0.0:
            load = wp.max(load, tool_load[c, b])
            if mu < par[P_MU_TABLE, b]:
                table_wins = 1
                state = 2
            elif state == 0:
                state = 1
        rel = P + corr - Q - sv
        rel = rel - nn * wp.dot(rel, nn)
        rl = wp.length(rel)
        if table_wins == 0 and rl > 1.0e-12:
            corr = corr - rel * wp.min(mu * pen / rl, 1.0)
        acc += corr
        cnt += 1
        t_n[i, b] = nn
        t_v[i, b] = sv
    if cnt > 0:
        pos[i, b] = P + acc / float(cnt)
        touch[i, b] = 1
    pressed[i, b] = state
    if load > 0.0:
        press[i, b] = press[i, b] + load * inv_sub


# ------------------------------------------------------------------------------------------------ pins
@wp.kernel
def k_pin_begin(pin_on: wp.array2d(dtype=int), pos: wp.array2d(dtype=wp.vec3), pin_from: wp.array2d(dtype=wp.vec3)):
    i, b = wp.tid()
    if pin_on[i, b] != 0:
        pin_from[i, b] = pos[i, b]


@wp.kernel
def k_pin_drive(pin_on: wp.array2d(dtype=int), w: wp.array2d(dtype=float), pos: wp.array2d(dtype=wp.vec3), pin_from: wp.array2d(dtype=wp.vec3),
                pin_to: wp.array2d(dtype=wp.vec3), fr: float):
    i, b = wp.tid()
    if pin_on[i, b] != 0 and w[i, b] == 0.0:
        pos[i, b] = pin_from[i, b] + (pin_to[i, b] - pin_from[i, b]) * fr


# ------------------------------------------------------------------------------------------------ pinch grabs
@wp.func
def grab_flags(b: int, v: int, bend: wp.array2d(dtype=int), stiff: wp.array2d(dtype=int), noyield: wp.array2d(dtype=int), N: int, nb: int):
    ci = v % N
    cj = v / N
    for e in range(nb):
        m = bend[e, 2]
        md = wp.max(wp.abs(m % N - ci), wp.abs(m / N - cj))
        if md <= 1:
            stiff[e, b] = 1
        if md <= 4:
            noyield[e, b] = 1


@wp.func
def local_edges_add(b: int, v: int, local_table: wp.array2d(dtype=int), local_list: wp.array2d(dtype=int), local_cnt: wp.array(dtype=int), LE: int):
    cnt = local_cnt[b]
    for s in range(local_table.shape[1]):
        e = local_table[v, s]
        if e < 0:
            break
        if cnt < LE:
            local_list[cnt, b] = e
            cnt += 1
    local_cnt[b] = cnt


@wp.func
def taken_by_other(b: int, k: int, q: int, K: int, g_active: wp.array2d(dtype=int), g_saved_v: wp.array3d(dtype=int)):
    for k2 in range(K):
        if k2 != k and g_active[k2, b] != 0:
            for s in range(SAVED):
                if g_saved_v[k2, s, b] == q:
                    return 1
    return 0


@wp.func
def grab_make(b: int, k: int, v: int, w: wp.array2d(dtype=float), g_active: wp.array2d(dtype=int), g_vert: wp.array2d(dtype=int),
              g_ring: wp.array3d(dtype=int), g_ring_d: wp.array3d(dtype=wp.vec2), g_saved_v: wp.array3d(dtype=int), g_saved_w: wp.array3d(dtype=float),
              g_ijk: wp.array3d(dtype=int), bend: wp.array2d(dtype=int), stiff: wp.array2d(dtype=int), noyield: wp.array2d(dtype=int),
              local_table: wp.array2d(dtype=int), local_list: wp.array2d(dtype=int), local_cnt: wp.array(dtype=int), dirty: wp.array(dtype=int),
              N: int, nb: int, K: int, LE: int):
    ci = v % N
    cj = v / N
    g_active[k, b] = 1
    g_vert[k, b] = v
    for s in range(SAVED):
        g_saved_v[k, s, b] = -1
    for r in range(RING):
        g_ring[k, r, b] = -1
    g_saved_v[k, 0, b] = v
    g_saved_w[k, 0, b] = w[v, b]
    w[v, b] = GRAB_W
    ns = int(1)
    nr = int(0)
    for dj in range(-1, 2):
        for di in range(-1, 2):
            ni = ci + di
            nj = cj + dj
            if (di == 0 and dj == 0) or ni < 0 or nj < 0 or ni >= N or nj >= N:
                continue
            q = nj * N + ni
            if w[q, b] == 0.0 or taken_by_other(b, k, q, K, g_active, g_saved_v) != 0:
                continue
            g_ring[k, nr, b] = q
            g_ring_d[k, nr, b] = wp.vec2(float(di), float(dj))
            nr += 1
            g_saved_v[k, ns, b] = q
            g_saved_w[k, ns, b] = w[q, b]
            w[q, b] = RING_W
            ns += 1
    g_ijk[k, 0, b] = cj * N + wp.max(0, ci - 1)          # the patch's frame: along the row and along the column
    g_ijk[k, 1, b] = cj * N + wp.min(N - 1, ci + 1)
    g_ijk[k, 2, b] = wp.max(0, cj - 1) * N + ci
    g_ijk[k, 3, b] = wp.min(N - 1, cj + 1) * N + ci
    local_edges_add(b, v, local_table, local_list, local_cnt, LE)
    grab_flags(b, v, bend, stiff, noyield, N, nb)
    dirty[b] = 1


@wp.func
def grab_release(b: int, k: int, why: int, w: wp.array2d(dtype=float), g_active: wp.array2d(dtype=int), g_vert: wp.array2d(dtype=int),
                 g_why: wp.array2d(dtype=int), g_saved_v: wp.array3d(dtype=int), g_saved_w: wp.array3d(dtype=float), bend: wp.array2d(dtype=int),
                 stiff: wp.array2d(dtype=int), noyield: wp.array2d(dtype=int), local_table: wp.array2d(dtype=int), local_list: wp.array2d(dtype=int),
                 local_cnt: wp.array(dtype=int), dirty: wp.array(dtype=int), N: int, nb: int, K: int, LE: int):
    for s in range(SAVED):
        v = g_saved_v[k, s, b]
        if v < 0:
            break
        w[v, b] = g_saved_w[k, s, b]
    g_active[k, b] = 0
    g_why[k, b] = why
    for e in range(nb):
        stiff[e, b] = 0
        noyield[e, b] = 0
    local_cnt[b] = 0
    for k2 in range(K):
        if g_active[k2, b] != 0:
            local_edges_add(b, g_vert[k2, b], local_table, local_list, local_cnt, LE)
            grab_flags(b, g_vert[k2, b], bend, stiff, noyield, N, nb)
    dirty[b] = 1


@wp.kernel
def k_control(cmd_grab: wp.array2d(dtype=int), cmd_at: wp.array2d(dtype=wp.vec3), cmd_point: wp.array2d(dtype=wp.vec3), cmd_reach: wp.array2d(dtype=float),
              g_active: wp.array2d(dtype=int), g_vert: wp.array2d(dtype=int), g_point: wp.array2d(dtype=wp.vec3), g_off: wp.array2d(dtype=wp.vec3),
              g_slipf: wp.array2d(dtype=int), g_why: wp.array2d(dtype=int), g_ring: wp.array3d(dtype=int), g_ring_d: wp.array3d(dtype=wp.vec2),
              g_saved_v: wp.array3d(dtype=int), g_saved_w: wp.array3d(dtype=float), g_ijk: wp.array3d(dtype=int),
              w: wp.array2d(dtype=float), pos: wp.array2d(dtype=wp.vec3), bend: wp.array2d(dtype=int), stiff: wp.array2d(dtype=int),
              noyield: wp.array2d(dtype=int), local_table: wp.array2d(dtype=int), local_list: wp.array2d(dtype=int), local_cnt: wp.array(dtype=int),
              dirty: wp.array(dtype=int), cmd_on: wp.array2d(dtype=int), cmd_a: wp.array2d(dtype=wp.vec3), cmd_b: wp.array2d(dtype=wp.vec3),
              cmd_r: wp.array2d(dtype=float), cmd_hw: wp.array2d(dtype=float), cmd_mu: wp.array2d(dtype=float), cmd_load: wp.array2d(dtype=float),
              tool_on: wp.array2d(dtype=int), tool_a: wp.array2d(dtype=wp.vec3), tool_b: wp.array2d(dtype=wp.vec3), tool_pa: wp.array2d(dtype=wp.vec3),
              tool_pb: wp.array2d(dtype=wp.vec3), tool_r: wp.array2d(dtype=float), tool_hw: wp.array2d(dtype=float), tool_mu: wp.array2d(dtype=float),
              tool_load: wp.array2d(dtype=float), N: int, n: int, nb: int, K: int, C: int, LE: int):
    """Applies the frame's commands. Grab command 1 = take the vertex nearest `cmd_at` within reach and drive it toward
    `cmd_point` (becomes 2 when taken, 0 when nothing is in reach), 2 = keep holding, 0 = let go. Tools take their new
    endpoints; a tool switched on this frame starts there (no sweep from an old position)."""
    b = wp.tid()
    for k in range(K):
        cmd = cmd_grab[k, b]
        if g_active[k, b] == 0:
            if cmd == 1:
                P = cmd_at[k, b]
                best = int(-1)
                bd = cmd_reach[k, b] * cmd_reach[k, b]
                for i in range(n):
                    if w[i, b] < 0.5:                     # pinned or part of another grab
                        continue
                    d = pos[i, b] - P
                    if wp.dot(d, d) < bd:
                        bd = wp.dot(d, d)
                        best = i
                if best >= 0:
                    grab_make(b, k, best, w, g_active, g_vert, g_ring, g_ring_d, g_saved_v, g_saved_w, g_ijk, bend, stiff, noyield,
                              local_table, local_list, local_cnt, dirty, N, nb, K, LE)
                    g_point[k, b] = cmd_point[k, b]
                    g_off[k, b] = pos[best, b] - P          # hold the paper where it was caught
                    g_slipf[k, b] = 0
                    g_why[k, b] = 0
                    cmd_grab[k, b] = 2
                else:
                    cmd_grab[k, b] = 0
        else:
            if cmd == 0:
                grab_release(b, k, 1, w, g_active, g_vert, g_why, g_saved_v, g_saved_w, bend, stiff, noyield, local_table, local_list,
                             local_cnt, dirty, N, nb, K, LE)
            else:
                g_point[k, b] = cmd_point[k, b]
    for c in range(C):
        on = cmd_on[c, b]
        if on != 0:
            if tool_on[c, b] == 0:
                tool_pa[c, b] = cmd_a[c, b]
                tool_pb[c, b] = cmd_b[c, b]
            tool_a[c, b] = cmd_a[c, b]
            tool_b[c, b] = cmd_b[c, b]
            tool_r[c, b] = cmd_r[c, b]
            tool_hw[c, b] = cmd_hw[c, b]
            tool_mu[c, b] = cmd_mu[c, b]
            tool_load[c, b] = cmd_load[c, b]
        tool_on[c, b] = on


@wp.kernel
def k_grab_drive(pos: wp.array2d(dtype=wp.vec3), g_active: wp.array2d(dtype=int), g_vert: wp.array2d(dtype=int), g_point: wp.array2d(dtype=wp.vec3),
                 g_off: wp.array2d(dtype=wp.vec3), g_ring: wp.array3d(dtype=int), g_ring_d: wp.array3d(dtype=wp.vec2), g_ijk: wp.array3d(dtype=int),
                 K: int, L: float, drive_cap: float):
    b = wp.tid()
    for k in range(K):
        if g_active[k, b] == 0:
            continue
        i = g_vert[k, b]
        pi = pos[i, b]
        stp = (g_point[k, b] + g_off[k, b] - pi) * 0.9
        sl = wp.length(stp)
        if sl > drive_cap:
            stp = stp * (drive_cap / sl)
        pi = pi + stp
        pos[i, b] = pi
        t1 = pos[g_ijk[k, 1, b], b] - pos[g_ijk[k, 0, b], b]
        tl1 = wp.length(t1)
        if tl1 < 1.0e-9:
            continue
        t1 = t1 / tl1
        t2 = pos[g_ijk[k, 3, b], b] - pos[g_ijk[k, 2, b], b]
        t2 = t2 - t1 * wp.dot(t1, t2)
        tl2 = wp.length(t2)
        if tl2 < 1.0e-9:
            continue
        t2 = t2 / tl2
        for r in range(RING):
            q = g_ring[k, r, b]
            if q < 0:
                break
            d = g_ring_d[k, r, b]
            stp = (pi + t1 * (d[0] * L) + t2 * (d[1] * L) - pos[q, b]) * 0.8
            sl = wp.length(stp)
            if sl > drive_cap:
                stp = stp * (drive_cap / sl)
            pos[q, b] = pos[q, b] + stp


@wp.kernel
def k_grab_flag(g_active: wp.array2d(dtype=int), K: int, flag: wp.array(dtype=int)):
    b = wp.tid()
    f = int(0)
    for k in range(K):
        if g_active[k, b] != 0:
            f = 1
    flag[b] = f


@wp.kernel
def k_slip(pos: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float), cmd_grab: wp.array2d(dtype=int), g_active: wp.array2d(dtype=int),
           g_vert: wp.array2d(dtype=int), g_point: wp.array2d(dtype=wp.vec3), g_off: wp.array2d(dtype=wp.vec3), g_slip: wp.array2d(dtype=float),
           g_slipf: wp.array2d(dtype=int), g_why: wp.array2d(dtype=int), g_saved_v: wp.array3d(dtype=int), g_saved_w: wp.array3d(dtype=float),
           bend: wp.array2d(dtype=int), stiff: wp.array2d(dtype=int), noyield: wp.array2d(dtype=int), local_table: wp.array2d(dtype=int),
           local_list: wp.array2d(dtype=int), local_cnt: wp.array(dtype=int), dirty: wp.array(dtype=int), slip_frames: int,
           N: int, nb: int, K: int, LE: int):
    """End of frame: a grab whose paper lags its point by more than GRAB_SLIP for more than slip_frames frames lets go."""
    b = wp.tid()
    for k in range(K):
        if g_active[k, b] == 0:
            continue
        d = wp.length(pos[g_vert[k, b], b] - (g_point[k, b] + g_off[k, b]))
        g_slip[k, b] = d
        if d > GRAB_SLIP:
            g_slipf[k, b] = g_slipf[k, b] + 1
            if g_slipf[k, b] > slip_frames:
                grab_release(b, k, 2, w, g_active, g_vert, g_why, g_saved_v, g_saved_w, bend, stiff, noyield, local_table, local_list,
                             local_cnt, dirty, N, nb, K, LE)
                cmd_grab[k, b] = 0
        else:
            g_slipf[k, b] = 0


@wp.kernel
def k_frame_end(tool_on: wp.array2d(dtype=int), tool_a: wp.array2d(dtype=wp.vec3), tool_b: wp.array2d(dtype=wp.vec3),
                tool_pa: wp.array2d(dtype=wp.vec3), tool_pb: wp.array2d(dtype=wp.vec3), frame: wp.array(dtype=int), C: int):
    b = wp.tid()
    for c in range(C):
        if tool_on[c, b] != 0:
            tool_pa[c, b] = tool_a[c, b]
            tool_pb[c, b] = tool_b[c, b]
    frame[b] = frame[b] + 1
