"""Typed parameters in SI units: the paper (material), the simulation (numerics) and the crease law.

Everything a user would tune is here. Numerical constants that are part of the method (stretch spring ratio, tension
sweeps, opening-spring threshold, ...) live next to the code that uses them and are documented there.
"""
from dataclasses import dataclass, replace
import math

RHO_AIR = 1.2      # kg/m^3
CD_PLATE = 1.2     # drag coefficient of a flat plate moving face-on


def plate_air_drag(areal_density, rho_air=RHO_AIR, cd=CD_PLATE):
    """Quadratic drag constant c (1/m) of a sheet moving along its normal: dv/dt = -c v |v|, c = rho_air Cd / (2 rho_A)."""
    return rho_air * cd / (2.0 * areal_density)


@dataclass(frozen=True)
class Paper:
    """The sheet's material.

    bending_stiffness  D, bending stiffness per unit width (N m). Paper is stiff in bending for its weight: the bending
                       length (D / (rho_A g))^(1/3) is about 7 cm for 80 gsm paper.
    areal_density      rho_A, mass per area (kg/m^2); the grammage in g/m^2 divided by 1000.
    thickness          physical thickness (m). Documentation and contact sizing only; the simulated layer spacing is
                       `Sim.layer_spacing`.
    mu_paper           Coulomb friction between two layers of the sheet.
    mu_table           Coulomb friction between the sheet and the table.
    air_drag           quadratic air drag on the normal velocity, c in dv/dt = -c v_n |v_n| (1/m); 0 turns it off.
    """
    bending_stiffness: float = 2.5e-4
    areal_density: float = 0.080
    thickness: float = 1.0e-4
    mu_paper: float = 0.45
    mu_table: float = 0.5
    air_drag: float = plate_air_drag(0.080)

    @staticmethod
    def printer_80gsm():
        """80 gsm office/printer paper. ESTIMATE: D from E t^3 / 12 is 3-6e-4 N m with caliper thickness (an upper
        bound); 2.5e-4 N m is the simulator's working value until measured. Thickness 0.1 mm (caliper 105-110 um).
        Friction: paper on paper 0.35-0.5 kinetic for office grades; the table value depends on the table."""
        return Paper(2.5e-4, 0.080, 1.0e-4, 0.45, 0.5, plate_air_drag(0.080))

    @staticmethod
    def origami_70gsm():
        """70 gsm origami paper (kami). ESTIMATE: D 0.9-1.9e-4 N m derived from typical thickness; 1.2e-4 used."""
        return Paper(1.2e-4, 0.070, 0.75e-4, 0.45, 0.5, plate_air_drag(0.070))

    def with_(self, **kw):
        """A copy with some fields changed, e.g. paper.with_(mu_table=0.8)."""
        return replace(self, **kw)

    @property
    def bending_length(self):
        """(D / (rho_A g))^(1/3) in metres: the length scale over which the sheet's own weight bends it."""
        return (self.bending_stiffness / (self.areal_density * 9.81)) ** (1.0 / 3.0)


@dataclass(frozen=True)
class Sim:
    """Numerics of the simulation.

    N              vertices per side of the square grid (N x N vertices, cell size L = size / (N - 1)).
    size           side of the square sheet (m).
    substeps       solver substeps per frame.
    frame_dt       frame length (s). Contacts are detected, the bending system refactored when needed and the crease law
                   advanced once per frame.
    device         'cpu', 'cuda', 'cuda:0', ...; None picks the first CUDA device when Warp and torch both see one.
    layer_spacing  distance kept between two layers in self contact and above the table (m). Numerical: larger than real
                   paper (0.1 mm) so that fast motion does not tunnel through a layer at the default substeps.
    damping        global velocity damping (1/s), v <- (1 - damping dt) v every substep. Not physical (it also slows rigid
                   motion); 0 by default, the quadratic air drag of `Paper` provides the dissipation.
    normal_drag    linear drag on the normal velocity (1/s); 0 by default.
    gravity        gravity vector (m/s^2). Tilting it is an easy way to put the sheet on an incline.
    table          an infinite rigid table at z = 0 (the sheet rests at z = layer_spacing / 2).
    max_speed      speed cap per vertex (m/s); a numerical safety net.
    max_tools      capacity for capsule tools per sheet.
    max_grabs      capacity for pinch grabs per sheet.
    contact_items_per_triangle  capacity of the self-contact hash (entries per triangle); overflow is counted in
                   `Sheet.T['overflow']` (contacts are then missed, never invented).
    """
    aspect_x: float = 1.0  # FQ rectangle extension: width / length
    N: int = 33
    size: float = 0.15
    substeps: int = 18
    frame_dt: float = 1.0 / 60.0
    device: str = None
    layer_spacing: float = 3.0e-4
    damping: float = 0.0
    normal_drag: float = 0.0
    gravity: tuple = (0.0, 0.0, -9.81)
    table: bool = True
    max_speed: float = 2.0
    max_tools: int = 8
    max_grabs: int = 4
    contact_items_per_triangle: int = 64

    def with_(self, **kw):
        return replace(self, **kw)

    @property
    def cell(self):
        """Grid cell size L (m)."""
        return self.size / (self.N - 1)

    @property
    def dt(self):
        """Substep length (s)."""
        return self.frame_dt / self.substeps


K1C_KEYS = ('m_y', 'h_k', 'w0', 'w_min', 'f_c', 'L_d', 'rate_s', 'M_a', 'H_c', 'b', 'w_y', 'w_reset', 's_d', 'kappa_y')


@dataclass(frozen=True)
class CreaseLaw:
    """Plasticity of the hinges (how folds set, spring back, creep and are remembered).

    kind = 'k1c' (default): a crease clock in physical units, independent of the grid and frame rate to within a few
    degrees. Per hinge it tracks the plastic set rp (rad), a creep angle c (rad), the crease damage d (0..1), the crease
    width w (m) and a refold memory. A fold yields at the moment M_Y = m_y D kappa_y (1 + h_k |rp|) (1 - s_d memory);
    a held fold gets one elastic springback allowance for the whole crease however many hinges carry it; the crease
    zone compliance L / (L + L_d d) softens the hinge; a press of line load f (N/m) sharpens the width toward w_min;
    creep follows dc/dt = w_rate sinh((M - X) / M_a) with a kinematic back moment X = H_c (c + b rp), integrated exactly
    over the frame. Constants fitted on a 1-D reference model of a paper strip (not yet on measured paper):

        m_y      yield moment factor (-)                 h_k      hardening per radian of set (-)
        w0       default crease width (m)                w_min    narrowest crease width under a press (m)
        f_c      press line load scale (N/m)             L_d      damage length of a crease (m)
        rate_s   plastic flow rate (1/s)                 M_a      creep activation moment per width (N)
        H_c      creep back-moment stiffness (N/rad)     b        weight of the set in the back moment (-)
        w_y      creep rate at the yield moment (rad/s)  w_reset  undone fraction at which a crease counts as flattened
        s_d      yield softening from the refold memory  kappa_y  yield curvature 2 eps_y / h (1/m)

    kind = 'classic': the earlier angle-space plastic law of the simulator, kept for comparison. A hinge whose fold
    angle departs from its rest angle by more than 2 asin(yield0 (1 - damage)^2 + yield_floor) moves its rest angle
    toward the fold at `rate` (1/s, 13.4 = 20 % per 60 Hz frame), up to 2 asin(theta_max); damage = sin(rest / 2)
    softens the hinge by 1 / (1 + softening damage). Its yield is per hinge, so results depend on the grid.
    """
    kind: str = 'k1c'
    m_y: float = 0.5019474478258749
    h_k: float = 0.30251107150602025
    w0: float = 0.009265513589008768
    w_min: float = 0.001418883101147908
    f_c: float = 130.8557535066104
    L_d: float = 0.00015453901857605143
    rate_s: float = 41.35873053502235
    M_a: float = 0.0002548739088238583
    H_c: float = 0.04895772932785271
    b: float = 0.2368342937952876
    w_y: float = 0.011379013030435859
    w_reset: float = 0.3
    s_d: float = 0.35
    kappa_y: float = 80.0
    yield0: float = 0.19
    yield_floor: float = 0.15
    softening: float = 3.0
    rate: float = -60.0 * math.log(0.8)
    theta_max: float = 0.992

    def __post_init__(self):
        if self.kind not in ('k1c', 'classic'):
            raise ValueError(f"CreaseLaw.kind must be 'k1c' or 'classic', not {self.kind!r}")

    @staticmethod
    def k1c(**kw):
        return CreaseLaw(kind='k1c', **kw)

    @staticmethod
    def classic(**kw):
        return CreaseLaw(kind='classic', **kw)

    def k1c_vector(self):
        """The K1c constants in kernel order."""
        return [float(getattr(self, k)) for k in K1C_KEYS]
