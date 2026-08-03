"""
Public-transport (PT) vehicle delay via kinematic wave theory (Section 3.4).

Built directly from the author-verified equations (Eqs. 13-42). A PT vehicle
is detected at initial position x0b (distance upstream of the stop line at the
start of the first cycle c1) and is assumed to clear the intersection within
two cycles (c1, c2), travelling at free-flow speed v_f until it joins a queue.

The delay depends on the saturation state of c1 and c2, giving four cases,
each split into sub-cases by the interval in which x0b falls:

  Case A  c1 undersaturated, c2 undersaturated   (A1-A3)
  Case B  c1 undersaturated, c2 oversaturated    (B1-B3)
  Case C  c1 oversaturated,  c2 undersaturated   (C1-C4)
  Case D  c1 oversaturated,  c2 oversaturated    (D1-D4)

All shockwave speeds are positive magnitudes (paper convention). Cycle-level
queue quantities (L_q, L_rq, g_qd, t_rq) come from general_delay.analyze_cycle.

Equation map:
  A1: interval Eq.13 ; t_b Eq.14 ; x_b Eq.15 ; delay Eq.16
  A2: interval Eq.17 ; delay 0
  A3: interval Eq.18 ; t_b Eq.19 ; x_b Eq.20 ; delay Eq.21
  B1: interval Eq.22 ; delay Eq.16
  B2: interval Eq.23 ; delay Eq.24 (= Eq.16 term + r_c2)
  B3: interval Eq.25 ; t_b1 Eq.26 ; x_b Eq.27 ; t_b2 Eq.28 ; delay Eq.29
  C1: interval Eq.30 ; delay Eq.31 (= r_c1)
  C2: interval Eq.32 ; t_b1 Eq.33 ; x_b Eq.34 ; t_b2 Eq.35 ; delay Eq.36
  C3: interval Eq.37 ; delay 0
  C4: like A3 -> Eqs.19-21
  D1: interval Eq.38 ; delay r_c1
  D2: interval Eq.39 ; delay Eq.40 (= r_c1 + r_c2)
  D3: interval Eq.41 ; delay Eq.42 (= Eqs.33-36 result + r_c2)
  D4: like B3
"""

from __future__ import annotations
from dataclasses import dataclass
from .fundamental_diagram import FundamentalDiagram
from .general_delay import CycleResult


@dataclass
class PTCycle:
    """Per-cycle inputs needed for PT delay (from general-traffic analysis)."""
    r: float          # red time [s]
    g: float          # green time [s]
    L_q: float        # total queue length [m]
    L_rq: float       # residual queue length [m]
    g_qd: float       # min green to clear [s]
    t_rq: float       # residual formation time [s]
    oversaturated: bool

    @classmethod
    def from_result(cls, res: CycleResult) -> "PTCycle":
        return cls(r=res.r, g=res.g, L_q=res.L_q, L_rq=res.L_rq,
                   g_qd=res.g_qd, t_rq=res.t_rq,
                   oversaturated=res.oversaturated)


def _delay_join_undersat_queue(r: float, L_q: float, x_b: float) -> float:
    """Eq. 16 / Eq. 21: delay for a PT vehicle joining an undersaturated queue."""
    if L_q <= 0:
        return 0.0
    return r * (L_q - x_b) / L_q


def pt_vehicle_delay(
    x0b: float,
    c1: PTCycle,
    c2: PTCycle,
    fd: FundamentalDiagram,
    q_g: float,
) -> float:
    """Delay of a single PT vehicle over the (c1, c2) window.

    Parameters
    ----------
    x0b : float
        Initial PT position upstream of the stop line at start of c1 [m].
    c1, c2 : PTCycle
        Per-cycle queue state for the first and second cycles.
    fd : FundamentalDiagram
    q_g : float
        General-traffic arrival flow on the lane group [veh/s].

    Returns
    -------
    float
        PT vehicle delay [s], >= 0.
    """
    vqf1 = fd.v_qf_mag(q_g)
    vqf2 = vqf1  # same arrival state assumed across the 2-cycle window
    vqd = fd.v_qd_mag()
    vf = fd.v_f

    # Dispatch on the saturation states of the two cycles.
    if not c1.oversaturated and not c2.oversaturated:
        return _case_A(x0b, c1, c2, vqf1, vqf2, vqd, vf)
    if not c1.oversaturated and c2.oversaturated:
        return _case_B(x0b, c1, c2, vqf1, vqf2, vqd, vf)
    if c1.oversaturated and not c2.oversaturated:
        return _case_C(x0b, c1, c2, vqf1, vqf2, vqd, vf)
    return _case_D(x0b, c1, c2, vqf1, vqf2, vqd, vf)


# ----------------------------------------------------------------------
# Case A: c1 undersaturated, c2 undersaturated
# ----------------------------------------------------------------------
def _case_A(x0b, c1, c2, vqf1, vqf2, vqd, vf) -> float:
    b_A1 = vf * (c1.r + c1.g_qd)                       # Eq. 13 upper bound
    b_A2 = vf * (c1.r + c1.g)                          # Eq. 17 upper bound
    b_A3 = vf * (c1.r + c1.g + c2.r + c2.g_qd)         # Eq. 18 upper bound

    if x0b < b_A1:                                     # A1 (Eq. 13)
        t_b = x0b / (vqf1 + vf)                        # Eq. 14
        x_b = vqf1 * t_b                               # Eq. 15
        return _delay_join_undersat_queue(c1.r, c1.L_q, x_b)  # Eq. 16
    if x0b < b_A2:                                     # A2 (Eq. 17)
        return 0.0
    if x0b < b_A3:                                     # A3 (Eq. 18)
        return _a3_delay(x0b, c1, c2, vqf2, vf)
    # Beyond the 2-cycle window: vehicle clears freely.
    return 0.0


def _a3_delay(x0b, c1, c2, vqf2, vf) -> float:
    """A3 / C4: PT vehicle joins the undersaturated queue of c2 (Eqs. 19-21)."""
    t_b2 = (x0b + vqf2 * (c1.r + c1.g)) / (vqf2 + vf) - (c1.r + c1.g)  # Eq. 19
    x_b2 = vqf2 * t_b2                                                 # Eq. 20
    return _delay_join_undersat_queue(c2.r, c2.L_q, x_b2)             # Eq. 21


# ----------------------------------------------------------------------
# Case B: c1 undersaturated, c2 oversaturated
# ----------------------------------------------------------------------
def _case_B(x0b, c1, c2, vqf1, vqf2, vqd, vf) -> float:
    # Shared B1/B2 boundary (Eqs. 22 & 23): the "green just serves PT" position.
    b_B12 = (c1.g * vf * vqd * (vf + vqf1)) / (vqf1 * (vqd + vf))
    b_B2 = vf * (c1.r + c1.g_qd)                       # Eq. 23 upper bound
    b_B3 = vf * (c1.r + c1.g + c2.r + c2.g_qd)         # Eq. 25 upper bound

    if x0b < b_B12:                                    # B1 (Eq. 22)
        t_b = x0b / (vqf1 + vf)                        # Eq. 14
        x_b = vqf1 * t_b                               # Eq. 15
        return _delay_join_undersat_queue(c1.r, c1.L_q, x_b)  # Eq. 16
    if x0b < b_B2:                                     # B2 (Eq. 23)
        t_b = x0b / (vqf1 + vf)                        # Eq. 14
        x_b = vqf1 * t_b                               # Eq. 15
        d1 = _delay_join_undersat_queue(c1.r, c1.L_q, x_b)    # Eq. 16 part
        return d1 + c2.r                               # Eq. 24 (+ r_c2)
    if x0b < b_B3:                                     # B3 (Eq. 25)
        return _b3_delay(x0b, c1, c2, vqf2, vqd, vf)
    return 0.0


def _b3_delay(x0b, c1, c2, vqf2, vqd, vf) -> float:
    """B3 / D4: PT joins the normal queue of the oversaturated c2 (Eqs. 26-29)."""
    t_b1 = ((vqf2 * (c1.r + c1.g + c2.t_rq) + x0b - c2.L_rq)
            / (vqf2 + vf)) - (c1.r + c1.g)             # Eq. 26
    x_b = x0b - vf * (c1.r + c1.g + t_b1)              # Eq. 27
    t_b2 = c2.r + x_b / vqd                            # Eq. 28
    return max(t_b2 - t_b1, 0.0)                       # Eq. 29


# ----------------------------------------------------------------------
# Case C: c1 oversaturated, c2 undersaturated
# ----------------------------------------------------------------------
def _case_C(x0b, c1, c2, vqf1, vqf2, vqd, vf) -> float:
    b_C1 = (vf + vqd) * c1.t_rq                        # Eq. 30 upper bound
    b_C2 = vf * (c1.r + c1.g_qd)                       # Eq. 32 upper bound
    b_C3 = vf * (c1.r + c1.g)                          # Eq. 37 upper bound
    b_C4 = vf * (c1.r + c1.g + c2.r + c2.g_qd)         # like A3 (Eq. 18)

    if x0b < b_C1:                                     # C1 (Eq. 30)
        return c1.r                                   # Eq. 31
    if x0b < b_C2:                                     # C2 (Eq. 32)
        return _c2_delay(x0b, c1, vqf1, vqd, vf)
    if x0b < b_C3:                                     # C3 (Eq. 37)
        return 0.0
    if x0b < b_C4:                                     # C4 (like A3)
        return _a3_delay(x0b, c1, c2, vqf2, vf)        # Eqs. 19-21
    return 0.0


def _c2_delay(x0b, c1, vqf1, vqd, vf) -> float:
    """C2: PT joins the normal queue of the oversaturated c1 (Eqs. 33-36)."""
    t_b1 = (vqf1 * c1.t_rq + x0b - c1.L_rq) / (vqf1 + vf)  # Eq. 33
    x_b = x0b - t_b1 * vf                                  # Eq. 34
    t_b2 = c1.r + x_b / vqd                                # Eq. 35
    return max(t_b2 - t_b1, 0.0)                           # Eq. 36


# ----------------------------------------------------------------------
# Case D: c1 oversaturated, c2 oversaturated
# ----------------------------------------------------------------------
def _case_D(x0b, c1, c2, vqf1, vqf2, vqd, vf) -> float:
    b_D1 = vf * c1.g                                   # Eq. 38 upper bound
    b_D2 = (vf + vqd) * c1.t_rq                        # Eq. 39 upper bound
    b_D3 = vf * (c1.r + c1.g_qd)                       # Eq. 41 upper bound

    if x0b < b_D1:                                     # D1 (Eq. 38)
        return c1.r                                   # like C1
    if x0b < b_D2:                                     # D2 (Eq. 39)
        return c1.r + c2.r                            # Eq. 40
    if x0b < b_D3:                                     # D3 (Eq. 41)
        d_c1 = _c2_delay(x0b, c1, vqf1, vqd, vf)      # Eqs. 33-36
        return d_c1 + c2.r                            # Eq. 42 (+ r_c2)
    # D4: joins normal queue of c2, identical to B3.
    return _b3_delay(x0b, c1, c2, vqf2, vqd, vf)
