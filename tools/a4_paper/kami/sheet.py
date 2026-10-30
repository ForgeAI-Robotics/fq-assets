"""`Sheet`: a batch of B independent sheets of paper simulated together (one per "world").

Per frame (`step`):

    pre    commands (pins, grabs, tools), opening-spring activation, self-contact detection
    factor refactor the bending matrix if masses, crease damage or opening springs changed (torch, eager)
    body   `substeps` x [pins, integrate, grab drive, stretch sweep, implicit bending, coarse lines, stretch sweep,
           grab-local sweeps, buckle lines, tools, self contact, stretch sweep, table + friction + velocities]
    post   normals, crease law, grab slip check

Arrays are Warp arrays indexed [item, world]; `self.T` holds torch views of the ones a user reads or writes. On CUDA the
pre and body+post parts replay as captured CUDA graphs. Units: metres, seconds, kilograms; z up; the table is z = 0.
"""
import math
import numpy as np
import torch
import warp as wp

from . import topology as topo_mod
from ._layout import N_PAR, P_D, P_RHO, P_MU_TABLE, P_MU_PAPER, P_SOFT, P_AIR, SLOTS, RING, SAVED, HASH_BUCKETS, SCAN_CHUNK
from .bending import (BendSolver, OPEN_REL, TENSION_SWEEPS, k_bend_apply, k_bend_blocks, k_bend_cap, k_bend_diag, k_bend_elem_res,
                      k_bend_gather, k_fric_zero, k_open_activate, k_open_res, k_tension_apply, k_tension_lines)
from .config import CreaseLaw, Paper, Sim
from .constraints import k_buckle_lines, k_coarse_lines, k_local, stretch_sweep
from .contact import (k_contacts, k_contacts_solve, k_copy_contacts, k_hash_scatter, k_mean_vel, k_reach, k_scan_apply,
                      k_scan_chunks, k_scan_offsets, k_tri_count)
from .crease import k_classic, k_k1_law, k_k1_member, k_k1_window, k_measure
from .table import k_integrate, k_table
from .tools import (DRIVE_SPEED, GRAB_SLIP_TIME, LOCAL_SWEEPS, SUPPORT_DEPTH, k_control, k_frame_end, k_grab_drive, k_grab_flag,
                    k_pin_begin, k_pin_drive, k_slip, k_tools)

BUCKLE_EPS, BUCKLE_GAIN = 0.005, 1.0    # compressive strain past which a line buckles; lift per unit of compression
REFACTOR_DAMAGE = 0.03                  # refactor when any hinge's damage moved this much since the last factorization
SLIP_STRAIN = 0.02                      # the table lets paper slide where it is strained past this


def default_device():
    try:
        if wp.is_cuda_available() and torch.cuda.is_available():
            return 'cuda:0'
    except Exception:
        pass
    return 'cpu'


_inited = False


def _init(device):
    global _inited
    if not _inited:
        wp.init()
        _inited = True
    if device.startswith('cuda'):                      # torch and Warp share one stream, so eager torch ops order with the graphs
        torch.cuda.set_device(device)
        torch.cuda.set_stream(wp.stream_to_torch(wp.get_stream(device)))


class _Graph:
    """A captured CUDA graph of Warp launches (capture records, it does not execute)."""

    def __init__(self, fn, device):
        wp.synchronize_device(device)
        with wp.ScopedCapture(device=device) as cap:
            fn()
        self.graph = cap.graph

    def __call__(self):
        wp.capture_launch(self.graph)


@wp.kernel
def k_reset_pos(mask: wp.array(dtype=int), pose: wp.array2d(dtype=float), axis: wp.array(dtype=int), pos: wp.array2d(dtype=wp.vec3),
                prev: wp.array2d(dtype=wp.vec3), vel: wp.array2d(dtype=wp.vec3), N: int, L: float, S: float, aspect: float):
    """A flat sheet centred at (x, y, z) turned by `angle` about z, optionally rolled into a cylinder of curvature `curl`
    (1/m) along u (axis 0) or v (axis 1): positive lifts the two edges, negative lifts the middle (it rests on its edges)."""
    i, b = wp.tid()
    if mask[b] == 0:
        return
    u = (float(i % N) * L - S * 0.5) * aspect
    v = float(i / N) * L - S * 0.5
    uu = u
    vv = v
    z = float(0.0)
    curl = pose[4, b]
    if curl != 0.0:
        R = 1.0 / curl
        s = u
        if axis[b] != 0:
            s = v
        z = R * (1.0 - wp.cos(s / R))
        if curl < 0.0:
            z = z - R * (1.0 - wp.cos(0.5 * S / R))
        if axis[b] != 0:
            vv = R * wp.sin(s / R)
        else:
            uu = R * wp.sin(s / R)
    ca = wp.cos(pose[3, b])
    sa = wp.sin(pose[3, b])
    p = wp.vec3(pose[0, b] + ca * uu - sa * vv, pose[1, b] + sa * uu + ca * vv, pose[2, b] + z)
    pos[i, b] = p
    prev[i, b] = p
    vel[i, b] = wp.vec3(0.0)


@wp.kernel
def k_normals(pos: wp.array2d(dtype=wp.vec3), tris: wp.array2d(dtype=int), nrm: wp.array2d(dtype=wp.vec3), N: int):
    """Vertex normals: normalised sum of the (area-weighted) normals of the triangles around the vertex."""
    i, b = wp.tid()
    ii = i % N
    jj = i / N
    acc = wp.vec3(0.0)
    for dj in range(-1, 1):
        for di in range(-1, 1):
            ci = ii + di
            cj = jj + dj
            if ci < 0 or cj < 0 or ci >= N - 1 or cj >= N - 1:
                continue
            for tq in range(2):
                k = 2 * (cj * (N - 1) + ci) + tq
                a = tris[k, 0]
                bb = tris[k, 1]
                c = tris[k, 2]
                if a == i or bb == i or c == i:
                    acc += wp.cross(pos[bb, b] - pos[a, b], pos[c, b] - pos[a, b])
    l = wp.length(acc)
    nrm[i, b] = wp.vec3(0.0, 0.0, 1.0)
    if l > 1.0e-12:
        nrm[i, b] = acc / l


@wp.kernel
def k_face_normals(pos: wp.array2d(dtype=wp.vec3), tris: wp.array2d(dtype=int), fnrm: wp.array2d(dtype=wp.vec3)):
    k, b = wp.tid()
    a = pos[tris[k, 0], b]
    nn = wp.cross(pos[tris[k, 1], b] - a, pos[tris[k, 2], b] - a)
    l = wp.length(nn)
    if l > 0.0:
        nn = nn / l
    fnrm[k, b] = nn


class Sheet:
    """B sheets of one grid size and crease law, each with its own state, material (`set_paper`), pins, tools and grabs.

    sheet = Sheet(Paper.printer_80gsm(), Sim(N=33, device='cpu'), CreaseLaw.k1c(), batch=4)
    sheet.step(60)                       # one second at the default 60 Hz
    xyz = sheet.positions                # torch [B, N*N, 3] view
    """

    def __init__(self, paper=None, sim=None, crease=None, batch=1):
        self.paper = paper if paper is not None else Paper()
        self.sim = sim if sim is not None else Sim()
        self.law = crease if crease is not None else CreaseLaw()
        s = self.sim
        self.device = dev = str(s.device) if s.device else default_device()
        if dev == 'cuda':
            self.device = dev = 'cuda:0'
        _init(dev)
        self.B = B = int(batch)
        self.topo = tp = topo_mod.build(s.N, s.size, s.aspect_x)
        self.N, self.n, self.L = tp.N, tp.n, tp.L
        self.t = float(s.layer_spacing)
        self.z_floor = 0.5 * self.t if s.table else -1.0e9
        n, nb, nt, ne = tp.n, tp.n_bend, tp.n_tri, tp.n_edges
        K, C = s.max_grabs, s.max_tools
        self.K, self.C = K, C
        arr = lambda v, dt: wp.array(v, dtype=dt, device=dev)
        z = lambda shape, dt: wp.zeros(shape, dtype=dt, device=dev)
        # shared topology
        self.tris, self.tri_cell = arr(tp.tris, wp.int32), arr(tp.tri_cell, wp.int32)
        self.edges, self.rest_len = arr(tp.edges, wp.int32), arr(tp.rest, wp.float32)
        self.bend, self.bend_h, self.bend_diag = arr(tp.bend, wp.int32), arr(tp.bend_h, wp.float32), arr(tp.bend_diag, wp.int32)
        self.coarse, self.coarse_d = arr(tp.coarse, wp.int32), arr(tp.coarse_d, wp.float32)
        le, ls, lc, self.coarse_groups = topo_mod.coarse_lines(tp)
        self.line_elems, self.line_start, self.line_count = arr(le, wp.int32), arr(ls, wp.int32), arr(lc, wp.int32)
        ve, vc = topo_mod.vertex_elements(tp)
        self.vert_elems, self.vert_coef, self.ne_max = arr(ve, wp.int32), arr(vc, wp.float32), ve.shape[1]
        vn, vr, vi = topo_mod.vertex_edges(tp)
        self.vert_edges, self.vert_rest, self.vert_eid = arr(vn, wp.int32), arr(vr, wp.float32), arr(vi, wp.int32)
        self.k1_nbr = arr(topo_mod.hinge_neighbours(tp), wp.int32)
        table = topo_mod.local_edge_table(tp)
        self.local_table, self.LE = arr(table, wp.int32), K * table.shape[1]
        inv_mass, bwt = topo_mod.boundary_weights(tp)
        self.bend_wt = arr(bwt, wp.float32)
        self.w_lump = torch.as_tensor(inv_mass, device=dev)
        self.k1 = arr(np.array(self.law.k1c_vector(), np.float32), wp.float32)
        # state
        v3, f32, i32 = wp.vec3, wp.float32, wp.int32
        self.pos, self.prev, self.vel, self.nrm, self.pint, self.ref_pos = [z((n, B), v3) for _ in range(6)]
        self.fnrm = z((nt, B), v3)
        self.w, self.w_free, self.press = [z((n, B), f32) for _ in range(3)]
        self.pin_on, self.pressed, self.touch = [z((n, B), i32) for _ in range(3)]
        self.pin_from, self.pin_to, self.t_n, self.t_v = [z((n, B), v3) for _ in range(4)]
        self.rest, self.damage, self.damage_at, self.crease, self.theta, self.chord = [z((nb, B), f32) for _ in range(6)]
        self.sigma, self.stiff, self.noyield, self.open_on = [z((nb, B), i32) for _ in range(4)]
        self.open_res, self.elem_res = z((nb, B), v3), z((nb, B), v3)
        self.k1_m, self.rp, self.cr, self.dk, self.wz, self.rpm, self.mf, self.wk = [z((nb, B), f32) for _ in range(8)]
        self.k1_win = z((nb, B), wp.vec4)
        self.lam, self.r_raw, self.grab_flag = z((ne, B), f32), z((B, n, 3), f32), z(B, i32)
        self.fsum, self.fcnt = z(B, v3), z(B, f32)
        self.dirty, self.worst, self.frame, self.max_speed2 = z(B, i32), z(B, f32), z(B, i32), z(B, f32)
        self.par = z((N_PAR, B), f32)
        # self contact
        self.items_cap = int(s.contact_items_per_triangle) * nt
        self.c_cnt, self.o_cnt = z((n, B), i32), z((n, B), i32)
        self.c_tri, self.c_side, self.o_tri, self.o_side = [z((n, SLOTS, B), i32) for _ in range(4)]
        self.c_bary = z((n, SLOTS, B), v3)
        self.cell_cnt, self.cell_fill, self.items = z((HASH_BUCKETS + 1, B), i32), z((HASH_BUCKETS, B), i32), z((self.items_cap, B), i32)
        self.n_chunks = -(-(HASH_BUCKETS + 1) // SCAN_CHUNK)
        self.chunk_sum = z((self.n_chunks, B), i32)
        self.overflow, self.mv = z(B, i32), z(B, v3)
        self.reach, self.tri_reach = z((n, B), f32), z((nt, B), f32)
        # tools and grabs
        self.tool_on, self.cmd_on = z((C, B), i32), z((C, B), i32)
        self.tool_a, self.tool_b, self.tool_pa, self.tool_pb, self.cmd_a, self.cmd_b = [z((C, B), v3) for _ in range(6)]
        self.tool_r, self.tool_hw, self.tool_mu, self.tool_load = [z((C, B), f32) for _ in range(4)]
        self.cmd_r, self.cmd_hw, self.cmd_mu, self.cmd_load = [z((C, B), f32) for _ in range(4)]
        self.cmd_grab, self.g_active, self.g_vert, self.g_slipf, self.g_why = [z((K, B), i32) for _ in range(5)]
        self.cmd_at, self.cmd_point, self.g_point, self.g_off = [z((K, B), v3) for _ in range(4)]
        self.cmd_reach, self.g_slip = z((K, B), f32), z((K, B), f32)
        self.g_ring, self.g_ring_d, self.g_ijk = z((K, RING, B), i32), z((K, RING, B), wp.vec2), z((K, 4, B), i32)
        self.g_saved_v, self.g_saved_w = z((K, SAVED, B), i32), z((K, SAVED, B), f32)
        self.local_list, self.local_cnt = z((self.LE, B), i32), z(B, i32)
        self.solver = BendSolver(B, n, tp.band, dev)
        names = ['pos', 'prev', 'vel', 'w', 'w_free', 'pin_on', 'pin_to', 'pressed', 'press', 'rest', 'damage', 'crease', 'theta', 'dk', 'rp',
                 'dirty', 'par', 'frame', 'overflow', 'max_speed2', 'cmd_on', 'cmd_a', 'cmd_b', 'cmd_r', 'cmd_hw', 'cmd_mu', 'cmd_load',
                 'cmd_grab', 'cmd_at', 'cmd_point', 'cmd_reach', 'g_active', 'g_vert', 'g_why', 'g_slip']
        self.T = {k: wp.to_torch(getattr(self, k)) for k in names}
        self.frames_done = 0
        self._graphs = None
        self.set_paper(self.paper)
        self.reset()

    # ------------------------------------------------------------------------------------------- set-up
    def _worlds(self, worlds):
        """Bool mask [B] on the device from None (all), a mask, or indices."""
        if worlds is None:
            return torch.ones(self.B, dtype=torch.bool, device=self.device)
        m = torch.as_tensor(worlds, device=self.device)
        if m.dtype == torch.bool:
            return m
        out = torch.zeros(self.B, dtype=torch.bool, device=self.device)
        out[m.long()] = True
        return out

    def set_paper(self, paper, worlds=None):
        """Material of the selected worlds (all by default); refactors the bending matrix."""
        m = self._worlds(worlds)
        par = self.T['par']
        soft = 1.0 if self.law.kind == 'k1c' else self.law.softening
        for row, val in ((P_D, paper.bending_stiffness), (P_RHO, paper.areal_density), (P_MU_TABLE, paper.mu_table),
                         (P_MU_PAPER, paper.mu_paper), (P_SOFT, soft), (P_AIR, paper.air_drag)):
            par[row, m] = float(val)
        self.T['dirty'][m] = 1
        if worlds is None:
            self.paper = paper

    def reset(self, center=(0.0, 0.0), angle=0.0, height=None, curl=0.0, curl_axis=0, worlds=None):
        """Flat (or curled) sheets at rest, centred at `center` (x, y), turned by `angle` (rad) about z, at `height` (m,
        default: resting on the table). Clears crease state, pins, tools and grabs of the selected worlds."""
        m = self._worlds(worlds)
        mb, B, dev = m, self.B, self.device
        z0 = (self.z_floor if self.sim.table else 0.0) if height is None else float(height)
        full = lambda v: torch.as_tensor(v, dtype=torch.float32, device=dev).expand(B)
        pose = torch.stack([full(center[0]), full(center[1]), full(z0), full(angle), full(curl)])
        mask = wp.from_torch(mb.to(torch.int32).contiguous())
        wp.launch(k_reset_pos, (self.n, B), inputs=[mask, wp.from_torch(pose.contiguous()), wp.from_torch(full(curl_axis).to(torch.int32).contiguous()),
                                                    self.pos, self.prev, self.vel, self.N, self.L, self.topo.size, self.sim.aspect_x], device=dev)
        T = self.T
        T['w'][:, mb] = self.w_lump[:, None]
        T['w_free'][:, mb] = self.w_lump[:, None]
        for name in ('rest', 'damage', 'crease', 'theta', 'rp', 'dk'):
            T[name][:, mb] = 0.0
        for a in (self.damage_at, self.cr, self.rpm, self.mf, self.chord, self.k1_win):
            wp.to_torch(a)[:, mb] = 0.0
        for a in (self.stiff, self.noyield, self.open_on, self.c_cnt, self.o_cnt, self.pin_on, self.pressed, self.touch):
            wp.to_torch(a)[:, mb] = 0
        wp.to_torch(self.sigma)[:, mb] = 1
        wp.to_torch(self.wz)[:, mb] = self.law.w0
        wp.to_torch(self.wk)[:, mb] = self.law.w0
        for a in (self.tool_on, self.cmd_on, self.cmd_grab, self.g_active):
            wp.to_torch(a)[:, mb] = 0
        for a in (self.local_cnt, self.frame, self.overflow):
            wp.to_torch(a)[mb] = 0
        T['press'][:, mb] = 0.0
        T['dirty'][mb] = 1
        self._normals()

    def _normals(self):
        wp.launch(k_normals, (self.n, self.B), inputs=[self.pos, self.tris, self.nrm, self.N], device=self.device)
        wp.launch(k_face_normals, (self.topo.n_tri, self.B), inputs=[self.pos, self.tris, self.fnrm], device=self.device)

    # ------------------------------------------------------------------------------------------- pins
    def _vmask(self, vertices, worlds):
        """[n, B] bool mask from vertex ids, a bool mask [n] / [B, n], or None (all) and a world selection."""
        m = self._worlds(worlds)
        if vertices is None:
            vm = torch.ones(self.n, self.B, dtype=torch.bool, device=self.device)
        else:
            v = torch.as_tensor(vertices, device=self.device)
            if v.dtype != torch.bool:
                vm1 = torch.zeros(self.n, dtype=torch.bool, device=self.device)
                vm1[v.long()] = True
                v = vm1
            vm = v[:, None].expand(-1, self.B) if v.ndim == 1 else v.T
        return vm & m[None, :]

    def pin(self, vertices=None, worlds=None, targets=None):
        """Make vertices kinematic where they are (or at `targets`, [B, n, 3] or [n, 3]). Refactors the bending matrix."""
        T, vm = self.T, self._vmask(vertices, worlds)
        new = vm & (T['pin_on'] == 0)
        T['w_free'][new] = T['w'][new]
        T['w'][vm] = 0.0
        T['pin_on'][vm] = 1
        T['pin_to'][vm] = T['pos'][vm]
        if targets is not None:
            self.move_pins(targets, vertices, worlds)
        T['dirty'][vm.any(0)] = 1

    def move_pins(self, targets, vertices=None, worlds=None):
        """Move pinned vertices to `targets` ([B, n, 3] or [n, 3]) over the next frame, interpolated per substep."""
        T, vm = self.T, self._vmask(vertices, worlds) & (self.T['pin_on'] != 0)
        tg = torch.as_tensor(targets, dtype=torch.float32, device=self.device)
        tg = tg[:, None, :].expand(-1, self.B, -1) if tg.ndim == 2 else tg.permute(1, 0, 2)
        T['pin_to'][vm] = tg[vm]

    def unpin(self, vertices=None, worlds=None):
        """Release pinned vertices (all by default) with their mass restored and at rest."""
        T, vm = self.T, self._vmask(vertices, worlds) & (self.T['pin_on'] != 0)
        T['w'][vm] = T['w_free'][vm]
        T['pin_on'][vm] = 0
        T['dirty'][vm.any(0)] = 1

    def set_positions(self, positions, vertices=None, worlds=None):
        """Overwrite positions ([B, n, 3] or [n, 3]) of the selected vertices, at rest. Pinned vertices stay pinned there."""
        T, vm = self.T, self._vmask(vertices, worlds)
        p = torch.as_tensor(positions, dtype=torch.float32, device=self.device)
        p = p[:, None, :].expand(-1, self.B, -1) if p.ndim == 2 else p.permute(1, 0, 2)
        T['pos'][vm] = p[vm]
        T['prev'][vm] = p[vm]
        T['vel'][vm] = 0.0
        T['pin_to'][vm & (T['pin_on'] != 0)] = p[vm & (T['pin_on'] != 0)]
        self._normals()

    # ------------------------------------------------------------------------------------------- tools and grabs
    def set_tool(self, slot, a, b=None, radius=0.008, half_width=0.0, mu=0.5, force=0.0, contact_length=None, worlds=None):
        """Place capsule tool `slot` with axis a-b (a sphere if b is None) for the next frame. half_width > 0 widens its core
        into a flat bar across the axis, or a sphere's into a horizontal disc (a flat fingertip pad). force (N) is the press force it applies when pressing paper onto a support, spread over
        `contact_length` (m; default max(|b - a|, 1.5 radius)) as the crease law's line load. Contact is with the sheet's
        vertices (see kami.tools)."""
        m = self._worlds(worlds)
        a = torch.as_tensor(a, dtype=torch.float32, device=self.device).expand(self.B, 3)
        b = a if b is None else torch.as_tensor(b, dtype=torch.float32, device=self.device).expand(self.B, 3)
        if contact_length is None:
            contact_length = max(float(torch.linalg.norm(b - a, dim=-1).max()), 1.5 * radius)
        T = self.T
        T['cmd_a'][slot, m] = a[m]
        T['cmd_b'][slot, m] = b[m]
        T['cmd_r'][slot, m] = float(radius)
        T['cmd_hw'][slot, m] = float(half_width)
        T['cmd_mu'][slot, m] = float(mu)
        T['cmd_load'][slot, m] = float(force) / max(float(contact_length), 1e-6)
        T['cmd_on'][slot, m] = 1

    def remove_tool(self, slot=None, worlds=None):
        m = self._worlds(worlds)
        if slot is None:
            self.T['cmd_on'][:, m] = 0
        else:
            self.T['cmd_on'][slot, m] = 0

    def grab(self, slot, point, reach=0.01, worlds=None):
        """Pinch the vertex nearest to `point` within `reach` (m) at the next frame; `move_grab` then drags it. Check
        `grabbed` after a step: nothing is taken when no vertex is in reach."""
        m = self._worlds(worlds)
        p = torch.as_tensor(point, dtype=torch.float32, device=self.device)
        self.T['cmd_at'][slot, m] = p
        self.T['cmd_point'][slot, m] = p
        self.T['cmd_reach'][slot, m] = float(reach)
        self.T['cmd_grab'][slot, m] = 1

    def move_grab(self, slot, point, worlds=None):
        self.T['cmd_point'][slot, self._worlds(worlds)] = torch.as_tensor(point, dtype=torch.float32, device=self.device)

    def release(self, slot=None, worlds=None):
        m = self._worlds(worlds)
        if slot is None:
            self.T['cmd_grab'][:, m] = 0
        else:
            self.T['cmd_grab'][slot, m] = 0

    @property
    def grabbed(self):
        """[B, K] vertex held by each grab slot, -1 if none (valid after a step)."""
        return torch.where(self.T['g_active'] != 0, self.T['g_vert'], torch.full_like(self.T['g_vert'], -1)).T

    # ------------------------------------------------------------------------------------------- views
    @property
    def positions(self):
        """[B, n, 3] view of the vertex positions (m)."""
        return self.T['pos'].permute(1, 0, 2)

    @property
    def velocities(self):
        return self.T['vel'].permute(1, 0, 2)

    def rest_uv(self):
        """[n, 2] rest coordinates (m), sheet centre at 0: vertex (i, j) is at ((i, j) L - size / 2)."""
        return torch.as_tensor(self.topo.rest_uv(), dtype=torch.float32, device=self.device)

    def vertex(self, i, j):
        return j * self.N + i

    # ------------------------------------------------------------------------------------------- one frame
    def _pre(self):
        B, n, dev, t, L = self.B, self.n, self.device, self.t, self.L
        wp.launch(k_control, B, inputs=[self.cmd_grab, self.cmd_at, self.cmd_point, self.cmd_reach, self.g_active, self.g_vert, self.g_point, self.g_off,
                                        self.g_slipf, self.g_why, self.g_ring, self.g_ring_d, self.g_saved_v, self.g_saved_w, self.g_ijk,
                                        self.w, self.pos, self.bend, self.stiff, self.noyield, self.local_table, self.local_list, self.local_cnt,
                                        self.dirty, self.cmd_on, self.cmd_a, self.cmd_b, self.cmd_r, self.cmd_hw, self.cmd_mu, self.cmd_load,
                                        self.tool_on, self.tool_a, self.tool_b, self.tool_pa, self.tool_pb, self.tool_r, self.tool_hw, self.tool_mu,
                                        self.tool_load, self.N, n, self.topo.n_bend, self.K, self.C, self.LE], device=dev)
        wp.launch(k_pin_begin, (n, B), inputs=[self.pin_on, self.pos, self.pin_from], device=dev)
        self.press.zero_()
        self.max_speed2.zero_()
        wp.launch(k_open_activate, (self.topo.n_bend, B), inputs=[self.pos, self.bend, self.bend_h, self.open_on, self.dirty, OPEN_REL], device=dev)
        # self contact candidates for this frame
        nt, margin = self.topo.n_tri, 0.7 * t
        icx, icz, mask = 1.0 / (1.3 * L), 1.0 / (1.3 * L), HASH_BUCKETS - 1
        self.cell_cnt.zero_()
        wp.launch(k_copy_contacts, (n, B), inputs=[self.c_cnt, self.c_tri, self.c_side, self.o_cnt, self.o_tri, self.o_side, SLOTS], device=dev)
        wp.launch(k_mean_vel, B, inputs=[self.vel, self.mv, n], device=dev)
        wp.launch(k_reach, (n, B), inputs=[self.vel, self.mv, self.reach, self.sim.frame_dt, 2.0 * L], device=dev)
        wp.launch(k_tri_count, (nt, B), inputs=[self.pos, self.tris, self.reach, self.tri_reach, self.cell_cnt, self.overflow, t + margin, icx, icz, mask], device=dev)
        wp.launch(k_scan_chunks, (self.n_chunks, B), inputs=[self.cell_cnt, self.chunk_sum, HASH_BUCKETS + 1], device=dev)
        wp.launch(k_scan_offsets, B, inputs=[self.chunk_sum, self.n_chunks], device=dev)
        wp.launch(k_scan_apply, (self.n_chunks, B), inputs=[self.cell_cnt, self.chunk_sum, self.cell_fill, HASH_BUCKETS + 1, HASH_BUCKETS], device=dev)
        wp.launch(k_hash_scatter, (nt, B), inputs=[self.pos, self.tris, self.tri_reach, self.cell_fill, self.items, self.overflow, self.items_cap,
                                                    t + margin, icx, icz, mask], device=dev)
        wp.launch(k_contacts, (n, B), inputs=[self.pos, self.fnrm, self.tris, self.tri_cell, self.reach, self.tri_reach, self.cell_cnt, self.items,
                                              self.o_cnt, self.o_tri, self.o_side, self.c_cnt, self.c_tri, self.c_side, self.c_bary, self.N, t, margin,
                                              L, icx, icz, mask, SLOTS, self.items_cap, self.overflow], device=dev)

    def _factor_due(self):
        if self.frames_done == 0:
            return True
        return bool((self.T['dirty'] != 0).any()) or bool((wp.to_torch(self.worst) > REFACTOR_DAMAGE).any())

    def _factor(self):
        bs = self.solver
        dt = self.sim.dt
        self.T['dirty'].zero_()
        wp.to_torch(self.damage_at).copy_(self.T['damage'])
        self.worst.zero_()
        bs.D_w.zero_()
        bs.E_w.zero_()
        wp.launch(k_bend_diag, (bs.n_pad, self.B), inputs=[self.w, bs.D_w, self.n, bs.bs], device=self.device)
        wp.launch(k_bend_blocks, (self.n, self.B), inputs=[self.w, self.vert_elems, self.vert_coef, self.bend, self.bend_diag, self.damage, self.stiff,
                                                           self.par, bs.D_w, bs.E_w, self.open_on, self.vert_edges, bs.bs, self.ne_max, dt, self.L,
                                                           self.bend_wt], device=self.device)
        if not bool(torch.isfinite(bs.D).all()):
            raise FloatingPointError('non-finite bending system: a sheet diverged')
        bs.factor()

    def _body(self):
        s, B, n, nb, dev, t, L, N = self.sim, self.B, self.n, self.topo.n_bend, self.device, self.t, self.L, self.N
        dt = s.dt
        kt = 1.0 - s.damping * dt
        g = wp.vec3(*[float(x) for x in s.gravity])
        g_normal = max(-float(s.gravity[2]), 0.0)
        zfric = self.z_floor + 2.0e-4
        fric_on = int(s.table)
        slip_frames = max(1, int(round(GRAB_SLIP_TIME / s.frame_dt)))
        bsv = self.solver
        for sub in range(s.substeps):
            fr = (sub + 1) / s.substeps
            wp.launch(k_pin_drive, (n, B), inputs=[self.pin_on, self.w, self.pos, self.pin_from, self.pin_to, fr], device=dev)
            wp.launch(k_integrate, (n, B), inputs=[self.pos, self.prev, self.vel, self.w, self.nrm, self.par, self.pint, dt, kt, s.normal_drag, g], device=dev)
            wp.launch(k_grab_drive, B, inputs=[self.pos, self.g_active, self.g_vert, self.g_point, self.g_off, self.g_ring, self.g_ring_d, self.g_ijk,
                                               self.K, L, DRIVE_SPEED * dt], device=dev)
            stretch_sweep(self.pos, self.w, N, L, sub, B, dev, s.aspect_x)
            # implicit bending
            wp.launch(k_bend_elem_res, (nb, B), inputs=[self.pos, self.nrm, self.bend, self.bend_h, self.bend_diag, self.rest, self.sigma, self.damage,
                                                        self.stiff, self.par, self.elem_res, dt, L, self.bend_wt], device=dev)
            wp.launch(k_open_res, (nb, B), inputs=[self.pos, self.bend, self.bend_h, self.bend_diag, self.open_on, self.rest, self.damage, self.par,
                                                   self.open_res, dt, L, self.bend_wt], device=dev)
            wp.launch(k_fric_zero, B, inputs=[self.fsum, self.fcnt], device=dev)
            wp.launch(k_bend_gather, (n, B), inputs=[self.w, self.vert_elems, self.vert_coef, self.elem_res, self.open_res, self.open_on, self.bend,
                                                     self.r_raw, self.ne_max, self.pos, zfric, self.fsum, self.fcnt, fric_on], device=dev)
            wp.launch(k_bend_cap, (n, B), inputs=[self.w, self.r_raw, self.pos, self.vert_edges, self.vert_rest, self.par, dt, L, zfric, g_normal,
                                                  self.fsum, self.fcnt, fric_on], device=dev)
            self.lam.zero_()
            wp.launch(k_grab_flag, B, inputs=[self.g_active, self.K, self.grab_flag], device=dev)
            for _ in range(TENSION_SWEEPS):
                for family in range(4):
                    wp.launch(k_tension_lines, (N, B), inputs=[self.w, self.pos, self.r_raw, self.lam, self.vert_edges, self.vert_eid, N,
                                                               self.grab_flag, family], device=dev)
            wp.launch(k_tension_apply, (n, B), inputs=[self.w, self.pos, self.r_raw, self.lam, self.vert_edges, self.vert_eid, bsv.r_w,
                                                       self.grab_flag], device=dev)
            bsv.solve()
            wp.launch(k_bend_apply, (n, B), inputs=[self.w, self.pos, bsv.delta_w], device=dev)
            for first, nl in self.coarse_groups:
                wp.launch(k_coarse_lines, (nl, B), inputs=[self.pos, self.w, self.coarse, self.coarse_d, self.line_elems, self.line_start,
                                                           self.line_count, first], device=dev)
            stretch_sweep(self.pos, self.w, N, L, sub, B, dev, s.aspect_x)
            wp.launch(k_local, B, inputs=[self.pos, self.w, self.edges, self.rest_len, self.local_list, self.local_cnt, LOCAL_SWEEPS], device=dev)
            for family in range(4):
                wp.launch(k_buckle_lines, (N, B), inputs=[self.pos, self.w, self.nrm, N, L, s.aspect_x, BUCKLE_EPS, BUCKLE_GAIN, family], device=dev)
            # tools, then self contact, then one more stretch sweep
            wp.launch(k_tools, (n, B), inputs=[self.pos, self.prev, self.w, self.par, self.tool_on, self.tool_a, self.tool_b, self.tool_pa, self.tool_pb,
                                               self.tool_r, self.tool_hw, self.tool_mu, self.tool_load, self.pressed, self.touch, self.t_n, self.t_v,
                                               self.press, self.C, fr, 1.0 / s.substeps, t, self.z_floor + SUPPORT_DEPTH], device=dev)
            wp.launch(k_contacts_solve, B, inputs=[self.pos, self.prev, self.w, self.tris, self.c_cnt, self.c_tri, self.c_side, self.c_bary,
                                                   self.par, t, n], device=dev)
            stretch_sweep(self.pos, self.w, N, L, sub, B, dev, s.aspect_x)
            wp.copy(self.ref_pos, self.pos)
            wp.launch(k_table, (n, B), inputs=[self.pos, self.prev, self.vel, self.par, self.pint, self.pressed, self.ref_pos,
                                               self.vert_edges, self.vert_rest, self.touch, self.t_n, self.t_v, self.z_floor, int(s.table),
                                               1.0 / dt, s.max_speed, SLIP_STRAIN, self.max_speed2], device=dev)
        # end of frame
        wp.launch(k_frame_end, B, inputs=[self.tool_on, self.tool_a, self.tool_b, self.tool_pa, self.tool_pb, self.frame, self.C], device=dev)
        self.worst.zero_()
        self._normals()
        wp.launch(k_measure, (nb, B), inputs=[self.pos, self.nrm, self.bend, self.bend_h, self.sigma, self.theta, self.chord], device=dev)
        law = self.law
        cap = 2.0 * math.asin(min(law.theta_max, 1.0))
        if law.kind == 'k1c':
            wp.launch(k_k1_member, (nb, B), inputs=[self.theta, self.rp, self.k1_nbr, self.bend_h, self.k1, self.k1_m], device=dev)
            wp.launch(k_k1_window, (nb, B), inputs=[self.theta, self.k1_m, self.rp, self.cr, self.k1_nbr, self.k1_win], device=dev)
            wp.launch(k_k1_law, (nb, B), inputs=[self.theta, self.k1_m, self.k1_win, self.w, self.pressed, self.press, self.noyield, self.bend, self.bend_h,
                                                 self.par, self.k1, self.rp, self.cr, self.dk, self.wz, self.rpm, self.mf, self.wk, self.rest,
                                                 self.damage, self.crease, self.damage_at, self.worst, s.frame_dt, cap], device=dev)
        else:
            frac = 1.0 - math.exp(-law.rate * s.frame_dt)
            wp.launch(k_classic, (nb, B), inputs=[self.chord, self.rest, self.damage, self.crease, self.noyield, self.damage_at, self.worst,
                                                  law.yield0, law.yield_floor, frac, law.theta_max], device=dev)
        wp.launch(k_slip, B, inputs=[self.pos, self.w, self.cmd_grab, self.g_active, self.g_vert, self.g_point, self.g_off, self.g_slip, self.g_slipf,
                                     self.g_why, self.g_saved_v, self.g_saved_w, self.bend, self.stiff, self.noyield, self.local_table, self.local_list,
                                     self.local_cnt, self.dirty, slip_frames, N, nb, self.K, self.LE], device=dev)

    def step(self, frames=1, graph=True):
        """Advance `frames` frames. On CUDA (graph=True) the Warp work replays from captured graphs."""
        use_graph = graph and self.device.startswith('cuda')
        for _ in range(int(frames)):
            if use_graph:
                if self._graphs is None:
                    self._graphs = (_Graph(self._pre, self.device), _Graph(self._body, self.device))
                self._graphs[0]()
                if self._factor_due():
                    self._factor()
                self._graphs[1]()
            else:
                self._pre()
                if self._factor_due():
                    self._factor()
                self._body()
            self.frames_done += 1

    @property
    def time(self):
        """[B] simulated time since each sheet's last reset (s)."""
        return self.T['frame'].float() * self.sim.frame_dt
