"""
General-traffic delay via kinematic wave theory (Section 3.3).

Faithful to the author-verified equations (Mathpix transcription of the
published PDF, cross-checked by the user against a LaTeX renderer):

  Eq. 4  queue length, no incoming residual:
             L_q^{l,c} = r * v_qf * v_qd / (v_qd - v_qf)
         (equivalently Eq. 11 with t_rq = 0 -- verified consistent)
  Eq. 5  minimum green to clear the queue:
             g_qd^{l,c} = L_q * (1/v_qd + 1/v_f)
  Eq. 6/8 saturation test: undersaturated iff g_qd < g ; oversaturated iff g_qd >= g
  Eq. 7  undersaturated delay (area of the queue triangle):
             d_g^{l,c} = L_q * r / 2
  Eq. 9  residual formation time (when the PREVIOUS cycle overflowed):
             t_rq^{l,c1} = [ (r_c0+g_c0)*v_qd
                             + (r_c0 + L_q_c0/v_qd)*(v_qf+v_f) ] / (v_f+v_qd)
                           - (r_c0+g_c0)
  Eq. 10 residual queue length:      L_rq^{l,c1} = t_rq * v_qd
  Eq. 11 total queue length w/ residual:
             L_q^{l,c1} = v_qd*( t_rq*(v_qd - v_qf) + r_c1*v_qf ) / (v_qd - v_qf)
  Eq. 12 oversaturated delay:
             d_g^{l,c1} = L_rq*r_c1 + (L_q - L_rq)*r_c1 / 2

IMPORTANT: d_g here is a queue-profile AREA (length x time). The conversion
to passenger-seconds happens in the objective (Eq. 43) via the jam density
k_j and occupancy o_g. We deliberately do NOT multiply by k_j here.

Shockwave speeds are used as POSITIVE MAGNITUDES (paper convention); see
FundamentalDiagram.v_qf_mag / v_qd_mag.

Cycle chaining: each cycle's delay depends on whether the PREVIOUS cycle
overflowed. If prev.oversaturated, this cycle carries a residual computed by
Eq. 9-10 and its delay follows Eq. 12; otherwise a fresh queue forms
(Eq. 4) and the delay follows Eq. 7 (undersaturated) -- and if the fresh
queue itself cannot be cleared, the cycle is flagged oversaturated so the
NEXT cycle computes the residual via Eq. 9. This chaining interpretation is
documented as a modeling note in the report.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
from .fundamental_diagram import FundamentalDiagram


@dataclass
class CycleResult:
    """State + delay for one lane-group / cycle."""
    d_g: float            # general-traffic delay (queue-profile area) [m.s]
    L_q: float            # total queue length reached [m]
    L_rq: float           # residual queue length present this cycle [m]
    g_qd: float           # min green to clear this cycle's queue [s]
    t_rq: float           # residual formation time [s] (0 if none)
    oversaturated: bool   # did this cycle's green fail to clear its queue?
    r: float              # red time used [s]
    g: float              # green time used [s]


def _queue_no_residual(r: float, vqf: float, vqd: float) -> float:
    """Eq. 4: fresh queue length (no incoming residual)."""
    return r * vqf * vqd / (vqd - vqf)


def _min_green(L_q: float, vqd: float, vf: float) -> float:
    """Eq. 5: minimum green to clear a queue of length L_q."""
    return L_q * (1.0 / vqd + 1.0 / vf)


def _t_rq(r_c0: float, g_c0: float, L_q_c0: float,
          vqf: float, vqd: float, vf: float) -> float:
    """Eq. 9: residual formation time at c1 given c0 overflowed."""
    num = ((r_c0 + g_c0) * vqd
           + (r_c0 + L_q_c0 / vqd) * (vqf + vf))
    return num / (vf + vqd) - (r_c0 + g_c0)


def _queue_with_residual(t_rq: float, r_c1: float,
                         vqf: float, vqd: float) -> float:
    """Eq. 11: total queue length including residual."""
    return vqd * (t_rq * (vqd - vqf) + r_c1 * vqf) / (vqd - vqf)


def analyze_cycle(
    r: float,
    g: float,
    fd: FundamentalDiagram,
    q_g: float,
    prev: Optional[CycleResult] = None,
) -> CycleResult:
    """Analyse one lane-group / cycle: queue state + general-traffic delay.

    Parameters
    ----------
    r, g : float
        Red and green time of this lane group this cycle [s].
    fd : FundamentalDiagram
    q_g : float
        General-traffic arrival flow [veh/s].
    prev : CycleResult or None
        Previous cycle's result (for residual chaining). None => first cycle
        with no initial residual (paper's "no residual at start of c0").
    """
    vqf = fd.v_qf_mag(q_g)
    vqd = fd.v_qd_mag()
    vf = fd.v_f

    incoming_residual = prev is not None and prev.oversaturated

    if incoming_residual:
        # ---- Residual enters from the previous (overflowed) cycle ----
        t_rq = _t_rq(prev.r, prev.g, prev.L_q, vqf, vqd, vf)   # Eq. 9
        t_rq = max(t_rq, 0.0)
        L_rq = t_rq * vqd                                       # Eq. 10
        L_q = _queue_with_residual(t_rq, r, vqf, vqd)           # Eq. 11
        g_qd = _min_green(L_q, vqd, vf)                         # Eq. 5
        d_g = L_rq * r + (L_q - L_rq) * r / 2.0                 # Eq. 12
        oversat = g_qd >= g                                     # Eq. 8
        return CycleResult(d_g=d_g, L_q=L_q, L_rq=L_rq, g_qd=g_qd,
                           t_rq=t_rq, oversaturated=oversat, r=r, g=g)

    # ---- Fresh queue, no incoming residual ----
    L_q = _queue_no_residual(r, vqf, vqd)                       # Eq. 4
    g_qd = _min_green(L_q, vqd, vf)                             # Eq. 5
    oversat = g_qd >= g                                         # Eq. 8

    # Undersaturated delay (Eq. 7). If the fresh queue is itself oversaturated,
    # its own-cycle delay is still the triangle area (Eq. 7); the extra delay
    # of the un-cleared portion is captured in the NEXT cycle via Eq. 12's
    # L_rq*r term (documented chaining interpretation).
    d_g = L_q * r / 2.0
    return CycleResult(d_g=d_g, L_q=L_q, L_rq=0.0, g_qd=g_qd,
                       t_rq=0.0, oversaturated=oversat, r=r, g=g)
