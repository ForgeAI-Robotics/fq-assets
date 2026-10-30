"""Time integration, the table and the velocity update.

`k_integrate` starts a substep: gravity, optional numerical damping, linear and quadratic drag on the velocity normal to
the sheet, then a free-flight step. `k_table` ends it: the floor (a rigid plane), Coulomb friction with the table and the
new velocities from the positions (position-based dynamics).
"""
import warp as wp

from ._layout import P_MU_TABLE, P_AIR


@wp.kernel
def k_integrate(pos: wp.array2d(dtype=wp.vec3), prev: wp.array2d(dtype=wp.vec3), vel: wp.array2d(dtype=wp.vec3), w: wp.array2d(dtype=float),
                nrm: wp.array2d(dtype=wp.vec3), par: wp.array2d(dtype=float), pint: wp.array2d(dtype=wp.vec3),
                dt: float, kt: float, normal_drag: float, g: wp.vec3):
    i, b = wp.tid()
    P = pos[i, b]
    if w[i, b] == 0.0:                                  # kinematic: stays where it was put
        prev[i, b] = P
        pint[i, b] = P
        return
    v = vel[i, b] + g * dt
    nn = nrm[i, b]
    vn = wp.dot(v, nn) * (1.0 - wp.exp(-normal_drag * dt))
    v = (v - nn * vn) * kt
    air = par[P_AIR, b]
    if air > 0.0:                                       # dv/dt = -c v_n |v_n|, implicit over the substep
        vn = wp.dot(v, nn)
        v = v + nn * (vn / (1.0 + air * wp.abs(vn) * dt) - vn)
    prev[i, b] = P
    pos[i, b] = P + v * dt
    pint[i, b] = P + v * dt                             # free-flight position: the floor counts its penetration too


@wp.kernel
def k_table(pos: wp.array2d(dtype=wp.vec3), prev: wp.array2d(dtype=wp.vec3), vel: wp.array2d(dtype=wp.vec3), par: wp.array2d(dtype=float),
            pint: wp.array2d(dtype=wp.vec3), pressed: wp.array2d(dtype=int), ref_pos: wp.array2d(dtype=wp.vec3),
            vert_edges: wp.array2d(dtype=int), vert_rest: wp.array2d(dtype=float), touch: wp.array2d(dtype=int), t_n: wp.array2d(dtype=wp.vec3),
            t_v: wp.array2d(dtype=wp.vec3), z_floor: float, table_on: int, inv_dt: float, vmax: float, slip_strain: float,
            max_speed2: wp.array(dtype=float)):
    """Floor and table friction, then velocity = displacement / dt.

    Friction budget per substep: mu_table x the floor penetration (the displacement-level normal impulse; at rest g dt^2,
    so the budget is the Coulomb limit mu g), against the vertex's tangential motion over the substep. Where the sheet is
    already strained past `slip_strain` the table lets it slide (paper slides before it stretches). Paper pressed onto the
    table by a tool that grips less than the table stays put (`pressed` = 2). Contact with the floor and with a tool is
    inelastic: no normal velocity is left relative to the surface."""
    i, b = wp.tid()
    P = pos[i, b]
    Q = prev[i, b]
    on_floor = int(0)
    if table_on != 0 and P[2] <= z_floor:
        on_floor = 1
        pen = wp.max(z_floor - P[2], z_floor - pint[i, b][2])
        P[2] = z_floor
        tx = P[0] - Q[0]
        ty = P[1] - Q[1]
        tl = wp.sqrt(tx * tx + ty * ty)
        if pressed[i, b] == 2:
            P[0] = Q[0]
            P[1] = Q[1]
            tl = 0.0
        if tl > 1.0e-12:
            strain = float(0.0)
            for s in range(8):
                j = vert_edges[i, s]
                if j < 0:
                    break
                strain = wp.max(strain, wp.length(ref_pos[j, b] - P) / vert_rest[i, s] - 1.0)
            if strain <= slip_strain:
                cap = par[P_MU_TABLE, b] * pen
                fs = float(1.0)
                if cap < tl:
                    fs = cap / tl
                P[0] = P[0] - tx * fs
                P[1] = P[1] - ty * fs
    v = (P - Q) * inv_dt
    if touch[i, b] != 0:
        nn = t_n[i, b]
        v = v - nn * wp.dot(v - t_v[i, b] * inv_dt, nn)
    if on_floor == 1 and v[2] > 0.0:
        v = wp.vec3(v[0], v[1], 0.0)
    v2 = wp.dot(v, v)
    if v2 > vmax * vmax:
        v = v * (vmax / wp.sqrt(v2))
        v2 = vmax * vmax
    wp.atomic_max(max_speed2, b, v2)
    vel[i, b] = v
    pos[i, b] = P
