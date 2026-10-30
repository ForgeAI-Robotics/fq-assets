"""Measurements on a `Sheet` (torch, on the sheet's device; every function returns one value per world).

- `max_strain`: largest tension and compression over the stretch edges;
- `fold_angle`: angle between the two sides of a straight line on the sheet (mean normals of the triangles beyond a margin);
- `crease_state`: per-hinge set angle, crease state and current fold angle;
- `line_fold`: set angle and fold angle across a line, summed over the hinges that carry the fold;
- `layer_count`: number of layers of the sheet crossing a vertical line.
"""
import math
import torch


def _rest_tensors(sheet):
    cache = getattr(sheet, '_metric_cache', None)
    if cache is None:
        tp, dev = sheet.topo, sheet.device
        uv = sheet.rest_uv()
        tris = torch.as_tensor(tp.tris, dtype=torch.long, device=dev)
        seg = torch.as_tensor(tp.bend_seg, dtype=torch.float32, device=dev) * tp.L - 0.5 * tp.size
        cache = sheet._metric_cache = {
            'uv': uv, 'tris': tris, 'tri_c': uv[tris].mean(1),
            'edges': torch.as_tensor(tp.edges, dtype=torch.long, device=dev), 'rest': torch.as_tensor(tp.rest, device=dev),
            'hinge_mid': 0.5 * (seg[:, :2] + seg[:, 2:]), 'hinge_axis': torch.nn.functional.normalize(seg[:, 2:] - seg[:, :2], dim=1)}
    return cache


def max_strain(sheet):
    """(tension, compression) [B] each: max over edges of length / rest - 1 and of 1 - length / rest (fractions)."""
    c = _rest_tensors(sheet)
    q = sheet.positions
    s = (q[:, c['edges'][:, 0]] - q[:, c['edges'][:, 1]]).norm(dim=2) / c['rest'] - 1.0
    return s.max(1).values, (-s).max(1).values


def _line_frame(point, direction, device):
    p = torch.as_tensor(point, dtype=torch.float32, device=device)
    d = torch.nn.functional.normalize(torch.as_tensor(direction, dtype=torch.float32, device=device), dim=0)
    return p, d, torch.stack([-d[1], d[0]])


def fold_angle(sheet, point=(0.0, 0.0), direction=(1.0, 0.0), margin=0.015):
    """[B] fold angle (deg) about a line of the rest sheet through `point` along `direction`: the angle between the mean
    normals of the triangles more than `margin` (m) on either side of it. 0 flat, 180 folded flat."""
    c = _rest_tensors(sheet)
    p, d, nv = _line_frame(point, direction, sheet.device)
    side = (c['tri_c'] - p) @ nv
    q = sheet.positions
    t = c['tris']
    n = torch.nn.functional.normalize(torch.cross(q[:, t[:, 1]] - q[:, t[:, 0]], q[:, t[:, 2]] - q[:, t[:, 0]], dim=2), dim=2)
    a = torch.nn.functional.normalize(n[:, side > margin].mean(1), dim=1)
    b = torch.nn.functional.normalize(n[:, side < -margin].mean(1), dim=1)
    return torch.rad2deg(torch.acos((a * b).sum(1).clamp(-1.0, 1.0)))


def crease_state(sheet):
    """Per hinge, [B, n_bend] each: 'set_deg' the signed rest fold angle the crease law has set, 'crease' the crease state
    (K1c: damage d in 0..1; classic: sin(set / 2)), 'fold_deg' the current signed turning angle."""
    T = sheet.T
    rest = T['rest'].T
    return {'set_deg': torch.rad2deg(2.0 * torch.asin(rest.abs().clamp(max=1.0))) * torch.sign(rest),
            'crease': T['crease'].T.clone(), 'fold_deg': torch.rad2deg(T['theta'].T)}


def line_fold(sheet, point=(0.0, 0.0), direction=(1.0, 0.0), width=None):
    """Fold carried by the hinges along a line of the rest sheet. Hinges parallel to the line with their rest midpoint
    within `width` (m, default two cells) of it are grouped by position along the line; each group's set and current fold
    angles are summed (a crease spreads over neighbouring hinges) and the median over the line is returned:
    {'set_deg': [B], 'fold_deg': [B], 'crease': [B] (max crease state on the line)}."""
    c = _rest_tensors(sheet)
    p, d, nv = _line_frame(point, direction, sheet.device)
    width = 2.0 * sheet.L if width is None else width
    rel = c['hinge_mid'] - p
    parallel = (c['hinge_axis'] @ d).abs() > 0.95
    sel = parallel & ((rel @ nv).abs() <= width + 1e-6)
    if not bool(sel.any()):
        raise ValueError('no hinge along this line (the grid has hinges along rows, columns and the 45-degree diagonals)')
    key = torch.round((rel[sel] @ d) / (0.5 * sheet.L)).long()
    _, group = torch.unique(key, return_inverse=True)
    st = crease_state(sheet)
    out = {}
    for name in ('set_deg', 'fold_deg'):
        vals = st[name][:, sel]
        sums = torch.zeros(vals.shape[0], int(group.max()) + 1, device=vals.device).index_add_(1, group, vals)
        out[name] = sums.abs().clamp(max=180.0).median(1).values
    out['crease'] = st['crease'][:, sel].max(1).values
    return out


def layer_count(sheet, x, y):
    """[B] number of layers of the sheet crossing the vertical line through (x, y) (hits closer than a quarter of the layer
    spacing count once)."""
    c = _rest_tensors(sheet)
    q = sheet.positions
    t = c['tris']
    A, Bv, Cv = q[:, t[:, 0]], q[:, t[:, 1]], q[:, t[:, 2]]
    v0, v1 = Bv[..., :2] - A[..., :2], Cv[..., :2] - A[..., :2]
    v2 = torch.tensor([x, y], dtype=q.dtype, device=q.device) - A[..., :2]
    den = v0[..., 0] * v1[..., 1] - v0[..., 1] * v1[..., 0]
    ok = den.abs() > 1e-14
    den = torch.where(ok, den, torch.ones_like(den))
    bv = (v2[..., 0] * v1[..., 1] - v2[..., 1] * v1[..., 0]) / den
    bw = (v0[..., 0] * v2[..., 1] - v0[..., 1] * v2[..., 0]) / den
    inside = ok & (bv >= -1e-6) & (bw >= -1e-6) & (bv + bw <= 1 + 1e-6)
    zh = A[..., 2] + bv * (Bv[..., 2] - A[..., 2]) + bw * (Cv[..., 2] - A[..., 2])
    out = []
    for b in range(q.shape[0]):
        z = torch.sort(zh[b][inside[b]]).values
        out.append(0 if z.numel() == 0 else 1 + int((z[1:] - z[:-1] > 0.25 * sheet.t).sum()))
    return torch.tensor(out, device=q.device)
