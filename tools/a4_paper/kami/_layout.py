"""Array layout constants shared by the kernels: rows of the per-world parameter table and fixed capacities."""

# rows of the per-world material table `par` [N_PAR, B]
P_D, P_RHO, P_MU_TABLE, P_MU_PAPER, P_SOFT, P_AIR = 0, 1, 2, 3, 4, 5
N_PAR = 6

SLOTS = 8          # self-contact candidates kept per vertex
RING = 8           # vertices around a pinch grab's centre
SAVED = 9          # vertices whose weight a grab changes (centre + ring)
HASH_BUCKETS = 8192
SCAN_CHUNK = 128
EDGES_PER_VERTEX = 8
ELEMS_PER_VERTEX = 12
