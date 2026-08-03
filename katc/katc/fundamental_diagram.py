"""
Fundamental diagram and shockwave speeds for the KATC reproduction.

Reproduces the triangular flow-density relationship (Fig. 2) and the
queue formation / dissipation shockwave speeds (Eqs. 2-3) from:

    Behbahani & Poorjafari (2022), "Proposing a kinematic wave-based
    adaptive transit signal priority control using genetic algorithm",
    IET Intelligent Transport Systems, 17(5), 912-928.
    DOI: 10.1049/itr2.12316

All symbols follow the paper's notation as closely as possible.

Triangular fundamental diagram parameters (per lane group):
    v_f  : free-flow speed                     [m/s]
    k_j  : jam density                         [veh/m]
    q_c  : capacity (max flow rate)            [veh/s]
    k_c  : capacity (critical) density         [veh/m]   = q_c / v_f
    w    : backward wave speed (congested)     [m/s]     = q_c / (k_j - k_c)

For a given ARRIVAL state on the lane group with flow q_g and the
corresponding density k_g (read off the uncongested branch, k_g = q_g / v_f):

    v_qf : queue-FORMATION shockwave speed (Eq. 2)
           speed of the interface between the arriving state (q_g, k_g)
           and the jammed state (0, k_j). It is negative (moving upstream).

                    q_g - 0            q_g
           v_qf = -------------  =  -----------
                    k_g - k_j        k_g - k_j

    v_qd : queue-DISSIPATION shockwave speed (Eq. 3)
           speed of the interface between the capacity/discharge state
           (q_c, k_c) and the jammed state (0, k_j). Also negative.

                    q_c - 0            q_c
           v_qd = -------------  =  -----------
                    k_c - k_j        k_c - k_j

Sign convention: distances are measured positive upstream from the stop
line, so upstream-moving shockwaves have NEGATIVE speed. Throughout the
delay derivation the paper uses the magnitudes; we keep signed speeds
internally and take magnitudes where the geometry requires it, documenting
each such point.
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass(frozen=True)
class FundamentalDiagram:
    """Triangular fundamental diagram for a lane group.

    Parameters
    ----------
    v_f : float
        Free-flow speed [m/s].
    k_j : float
        Jam density [veh/m].
    q_c : float
        Capacity / maximum flow rate [veh/s].
    """
    v_f: float
    k_j: float
    q_c: float

    def __post_init__(self) -> None:
        if self.v_f <= 0:
            raise ValueError("v_f must be positive")
        if self.k_j <= 0:
            raise ValueError("k_j must be positive")
        if self.q_c <= 0:
            raise ValueError("q_c must be positive")
        if self.k_c >= self.k_j:
            raise ValueError(
                "critical density k_c must be < jam density k_j; "
                f"got k_c={self.k_c:.4f}, k_j={self.k_j:.4f}"
            )

    @property
    def k_c(self) -> float:
        """Critical (capacity) density [veh/m] = q_c / v_f."""
        return self.q_c / self.v_f

    @property
    def w(self) -> float:
        """Backward congested-branch wave speed magnitude [m/s]."""
        return self.q_c / (self.k_j - self.k_c)

    def density_from_arrival_flow(self, q_g: float) -> float:
        """Density on the uncongested branch for arrival flow q_g."""
        if q_g < 0:
            raise ValueError("arrival flow q_g must be non-negative")
        if q_g > self.q_c + 1e-12:
            raise ValueError(
                f"arrival flow q_g={q_g:.4f} exceeds capacity q_c={self.q_c:.4f}; "
                "an uncongested-branch density is undefined"
            )
        return q_g / self.v_f

    def v_qf(self, q_g: float) -> float:
        """Queue-formation shockwave speed (Eq. 2), signed (negative = upstream)."""
        k_g = self.density_from_arrival_flow(q_g)
        denom = k_g - self.k_j
        # denom is negative (k_g < k_c < k_j), so v_qf is negative.
        return q_g / denom

    def v_qd(self) -> float:
        """Queue-dissipation shockwave speed (Eq. 3), signed (negative = upstream)."""
        denom = self.k_c - self.k_j
        return self.q_c / denom

    # ------------------------------------------------------------------
    # MAGNITUDE convention.
    #
    # The paper's kinematic-wave equations (Eqs. 4-42) are written using the
    # POSITIVE MAGNITUDES of the shockwave speeds: distances grow upstream
    # from the stop line, and every formula (e.g. Eq. 11's L_q, Eq. 9's t_rq)
    # only produces physically sensible non-negative lengths/times when v_qf
    # and v_qd are inserted as positive quantities. We therefore expose
    # explicit magnitude accessors and use THESE throughout the delay and
    # PT-delay modules, matching the paper's algebra exactly. (Verified: with
    # signed/negative speeds Eq. 11 yields a negative L_q; with magnitudes it
    # yields the correct positive queue length.)
    # ------------------------------------------------------------------
    def v_qf_mag(self, q_g: float) -> float:
        """|v_qf| : queue-formation shockwave speed magnitude [m/s]."""
        return abs(self.v_qf(q_g))

    def v_qd_mag(self) -> float:
        """|v_qd| : queue-dissipation shockwave speed magnitude [m/s]."""
        return abs(self.v_qd())
