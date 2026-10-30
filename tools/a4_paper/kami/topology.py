"""The sheet's grid: an N x N "union jack" triangulation (cell diagonals alternate direction) and the tables built on it.

Vertex (i, j) has id j * N + i and rest position ((i, j) L - size / 2, 0). Elements:

- stretch edges: row, column and cell-diagonal edges (rest L, L, L sqrt 2);
- bending elements K = a + b - c1 - c2: row and column triples (a, b the outer vertices, c1 = c2 the centre, hinge
  spacing h = L) and one per cell across its diagonal (a, b the off-diagonal corners, c1, c2 the diagonal, h = L / sqrt 2).
  |K| = 2 h sin(fold / 2), so a hinge's fold angle can be read from K;
- coarse elements: pairs of vertices 2, 4, 8 and 16 cells apart along the four lattice directions, kept within their rest
  distance (long-range inextensibility, resolved per substep without iterating to convergence).
"""
from dataclasses import dataclass, replace
import numpy as np


@dataclass(frozen=True)
class Topology:
    N: int
    size: float
    L: float
    tris: np.ndarray        # [n_tri, 3]
    tri_cell: np.ndarray    # [n_tri, 2] grid cell (i, j) of each triangle
    edges: np.ndarray       # [n_edges, 2]
    rest: np.ndarray        # [n_edges] rest lengths
    bend: np.ndarray        # [n_bend, 4] a, b, c1, c2
    bend_h: np.ndarray      # [n_bend] hinge spacing h (L, or L / sqrt 2 for cell diagonals)
    bend_diag: np.ndarray   # [n_bend] 1 for cell-diagonal hinges
    bend_seg: np.ndarray    # [n_bend, 4] hinge axis segment in grid units (x0, y0, x1, y1)
    coarse: np.ndarray      # [n_coarse, 2] vertex pairs
    coarse_d: np.ndarray    # [n_coarse] half of the rest distance

    aspect_x: float = 1.0

    @property
    def n(self): return self.N * self.N
    @property
    def n_tri(self): return len(self.tris)
    @property
    def n_edges(self): return len(self.edges)
    @property
    def n_bend(self): return len(self.bend)
    @property
    def n_coarse(self): return len(self.coarse)
    @property
    def band(self): return 2 * self.N          # half bandwidth of the bending matrix in vertex order

    def id(self, i, j): return j * self.N + i

    def rest_uv(self):
        """[n, 2] rest coordinates (m) with the sheet centre at the origin."""
        g = np.arange(self.n)
        return (np.stack([(g % self.N) * self.L, (g // self.N) * self.L], 1) - 0.5 * self.size) * [self.aspect_x, 1.0]


COARSE_SCALES = (2, 4, 8, 16)
DIRECTIONS = ((1, 0), (0, 1), (1, 1), (1, -1))


def build(N=33, size=0.15, aspect_x=1.0):
    if N < 3:
        raise ValueError('N must be at least 3')
    L = size / (N - 1)
    vid = lambda i, j: j * N + i
    tris, cell = [], []
    for j in range(N - 1):
        for i in range(N - 1):
            p00, p10, p01, p11 = vid(i, j), vid(i + 1, j), vid(i, j + 1), vid(i + 1, j + 1)
            tris += [[p00, p10, p11], [p00, p11, p01]] if (i + j) & 1 == 0 else [[p00, p10, p01], [p10, p11, p01]]
            cell += [[i, j], [i, j]]
    ed, rl = [], []
    for j in range(N):
        for i in range(N - 1):
            ed.append([vid(i, j), vid(i + 1, j)]); rl.append(L)
    for j in range(N - 1):
        for i in range(N):
            ed.append([vid(i, j), vid(i, j + 1)]); rl.append(L)
    for j in range(N - 1):
        for i in range(N - 1):
            ed.append([vid(i, j), vid(i + 1, j + 1)] if (i + j) & 1 == 0 else [vid(i + 1, j), vid(i, j + 1)]); rl.append(L * np.sqrt(2))
    be, bh, bseg, bdiag = [], [], [], []
    for j in range(N):
        for i in range(1, N - 1):
            be.append([vid(i - 1, j), vid(i + 1, j), vid(i, j), vid(i, j)]); bh.append(L); bdiag.append(0)
            bseg.append([i, max(0, j - 0.5), i, min(N - 1, j + 0.5)])
    for j in range(1, N - 1):
        for i in range(N):
            be.append([vid(i, j - 1), vid(i, j + 1), vid(i, j), vid(i, j)]); bh.append(L); bdiag.append(0)
            bseg.append([max(0, i - 0.5), j, min(N - 1, i + 0.5), j])
    for j in range(N - 1):
        for i in range(N - 1):
            if (i + j) & 1 == 0:
                be.append([vid(i + 1, j), vid(i, j + 1), vid(i, j), vid(i + 1, j + 1)]); bseg.append([i, j, i + 1, j + 1])
            else:
                be.append([vid(i, j), vid(i + 1, j + 1), vid(i + 1, j), vid(i, j + 1)]); bseg.append([i + 1, j, i, j + 1])
            bh.append(L / np.sqrt(2)); bdiag.append(1)
    ca, cd = [], []
    for sc in COARSE_SCALES:
        st = max(2, sc // 2)
        for di, dj in DIRECTIONS:
            for jj in range(0, N, st):
                for ii in range(0, N, st):
                    ai, aj, bi, bj = ii - sc * di, jj - sc * dj, ii + sc * di, jj + sc * dj
                    if min(ai, aj, bi, bj) < 0 or max(ai, aj, bi, bj) >= N:
                        continue
                    ca.append([vid(ai, aj), vid(bi, bj)]); cd.append(sc * L * np.hypot(di, dj))
    result = Topology(N, size, L, np.array(tris, np.int32), np.array(cell, np.int32), np.array(ed, np.int32),
                    np.array(rl, np.float32), np.array(be, np.int32), np.array(bh, np.float32), np.array(bdiag, np.int32),
                    np.array(bseg, np.float32), np.array(ca, np.int32).reshape(-1, 2), np.array(cd, np.float32))


    result=replace(result,aspect_x=aspect_x)
    uv=result.rest_uv()
    rest=np.linalg.norm(uv[result.edges[:,1]]-uv[result.edges[:,0]],axis=1).astype(np.float32)
    coarse_d=(np.linalg.norm(uv[result.coarse[:,1]]-uv[result.coarse[:,0]],axis=1)/2).astype(np.float32)
    h=result.bend_h.copy()
    for e,(a,b,c,d) in enumerate(result.bend):
        if c==d: h[e]=np.linalg.norm(uv[a]-uv[c])
        else: h[e]=L*aspect_x/(1+aspect_x**2)**.5
    return replace(result,rest=rest,coarse_d=coarse_d,bend_h=h)


def coarse_lines(topo):
    """Coarse elements grouped into lattice lines per (scale, direction), in build order along each line. Lines of one group
    share no vertex, so a group is solved in parallel, one thread per line. Returns (elements, line starts, line counts,
    [(first line, number of lines)] per group)."""
    N, elems, starts, counts, groups, e = topo.N, [], [], [], [], 0
    for sc in COARSE_SCALES:
        st = max(2, sc // 2)
        for di, dj in DIRECTIONS:
            lines = {}
            for jj in range(0, N, st):
                for ii in range(0, N, st):
                    ai, aj, bi, bj = ii - sc * di, jj - sc * dj, ii + sc * di, jj + sc * dj
                    if min(ai, aj, bi, bj) < 0 or max(ai, aj, bi, bj) >= N:
                        continue
                    key = jj if (di, dj) == (1, 0) else ii if (di, dj) == (0, 1) else (ii - jj) if (di, dj) == (1, 1) else (ii + jj)
                    lines.setdefault(key, []).append(e); e += 1
            first = len(starts)
            for key in sorted(lines):
                starts.append(len(elems)); counts.append(len(lines[key])); elems += lines[key]
            groups.append((first, len(lines)))
    assert e == topo.n_coarse
    return np.array(elems, np.int32), np.array(starts, np.int32), np.array(counts, np.int32), groups


def vertex_elements(topo, cap=12):
    """Per vertex: the bending elements it belongs to and its coefficient in K = a + b - c1 - c2 (a triple's centre: -2)."""
    ve = -np.ones((topo.n, cap), np.int32); vc = np.zeros((topo.n, cap), np.float32); fill = np.zeros(topo.n, np.int32)
    for e, (a, b, c1, c2) in enumerate(topo.bend):
        for v, c in ([(a, 1.0), (b, 1.0), (c1, -2.0)] if c1 == c2 else [(a, 1.0), (b, 1.0), (c1, -1.0), (c2, -1.0)]):
            ve[v, fill[v]] = e; vc[v, fill[v]] = c; fill[v] += 1
    return ve, vc


def vertex_edges(topo, cap=8):
    """Per vertex: neighbour vertex, rest length and edge id of each stretch edge at it (padded with -1 / 0)."""
    nb, rl, eid, cnt = -np.ones((topo.n, cap), np.int32), np.zeros((topo.n, cap), np.float32), -np.ones((topo.n, cap), np.int32), np.zeros(topo.n, np.int32)
    for e, ((a, b), r0) in enumerate(zip(topo.edges, topo.rest)):
        for u, v in ((a, b), (b, a)):
            nb[u, cnt[u]], rl[u, cnt[u]], eid[u, cnt[u]] = v, r0, e; cnt[u] += 1
    return nb, rl, eid


def hinge_neighbours(topo):
    """[n_bend, 4] hinges at offsets -2, -1, +1, +2 along each hinge's grid line (-1: none). Row triples step along the row,
    column triples along the column, cell diagonals along the perpendicular diagonal (same-parity cells carry parallel
    diagonals L / sqrt 2 apart). The K1c law sums over these windows so that a crease spread over neighbouring hinges is
    treated as one crease."""
    N, out, offs = topo.N, -np.ones((topo.n_bend, 4), np.int32), (-2, -1, 1, 2)
    row = lambda i, j: j * (N - 2) + (i - 1) if 1 <= i <= N - 2 and 0 <= j <= N - 1 else -1
    col = lambda i, j: N * (N - 2) + (j - 1) * N + i if 1 <= j <= N - 2 and 0 <= i <= N - 1 else -1
    dia = lambda i, j: 2 * N * (N - 2) + j * (N - 1) + i if 0 <= i <= N - 2 and 0 <= j <= N - 2 else -1
    for j in range(N):
        for i in range(1, N - 1):
            out[row(i, j)] = [row(i + k, j) for k in offs]
    for j in range(1, N - 1):
        for i in range(N):
            out[col(i, j)] = [col(i, j + k) for k in offs]
    for j in range(N - 1):
        for i in range(N - 1):
            s = -1 if (i + j) & 1 == 0 else 1
            out[dia(i, j)] = [dia(i + k, j + s * k) for k in offs]
    return out


def local_edge_table(topo, reach=4):
    """Per vertex: the stretch edges with both ends within `reach` cells (Chebyshev), nearest first. A pinch grab re-solves
    these edges a few extra times per substep so that the paper around a dragged point stays inextensible."""
    N, ed = topo.N, topo.edges
    ei, ej, fi, fj = ed[:, 0] % N, ed[:, 0] // N, ed[:, 1] % N, ed[:, 1] // N
    rows = []
    for v in range(topo.n):
        gi, gj = v % N, v // N
        ok = (np.maximum(np.abs(ei - gi), np.abs(ej - gj)) <= reach) & (np.maximum(np.abs(fi - gi), np.abs(fj - gj)) <= reach)
        idx = np.flatnonzero(ok)
        rows.append(idx[np.argsort(np.hypot(ei[idx] - gi, ej[idx] - gj), kind='stable')])
    table = -np.ones((topo.n, max(len(r) for r in rows)), np.int32)
    for v, r in enumerate(rows):
        table[v, :len(r)] = r
    return table


def boundary_weights(topo):
    """Lumped masses for the free edges. Edge vertices carry half a cell of paper, corners a quarter: returns the inverse-mass
    factor per vertex (1, 2 or 4) and the stiffness factor per bending element (0.5 for a row or column triple whose centre
    lies on the boundary row or column it bends along: it bends a half-width strip; 1 otherwise)."""
    N = topo.N
    g = np.arange(topo.n)
    i, j = g % N, g // N
    inv_mass = np.where((j == 0) | (j == N - 1), 2.0, 1.0) * np.where((i == 0) | (i == N - 1), 2.0, 1.0)
    bw = np.ones(topo.n_bend, np.float32)
    for e, (a, b, c1, c2) in enumerate(topo.bend):
        if c1 == c2:
            ci, cj = c1 % N, c1 // N
            if (a // N == cj and cj in (0, N - 1)) or (a % N == ci and ci in (0, N - 1)):
                bw[e] = 0.5
    return inv_mass.astype(np.float32), bw
