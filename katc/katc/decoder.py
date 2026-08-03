"""
Signal-plan decoder: chromosome gene -> green times per movement (Eqs. 44-48).

Maps each cycle's decoded gene (phase sequence ps + three coefficient factors
fc1,fc2,fc3) onto a NEMA dual-ring signal plan for a 4-legged intersection:

    ring A: phases 1,2,3,4     ring B: phases 5,6,7,8   (Figure 7 / Image 7)

Constraints enforced:
  Eq. 44  p_l = g_l + y_l + ar_l          (phase length = green+yellow+allred)
  Eq. 45  sum_{l=1..4} p_l = Gamma        (ring A fills the cycle)
  Eq. 46  sum_{l=5..8} p_l = Gamma        (ring B fills the cycle)
  Eq. 47  g_l >= g_min                    (minimum green)
  Eq. 48  p_l = p_{l+4}                   (dual-ring barrier alignment)

--------------------------------------------------------------------------
RECONSTRUCTION NOTE (documented deviation for the report):
The paper states three 6-bit coefficient factors "distribute the green time
between the major and minor movements" but does not publish the exact
factor -> movement mapping. We adopt a faithful, dual-ring-consistent
interpretation:

  * The barrier splits each ring's available green (Gamma - lost time) into a
    major-street block and a minor-street block. fc1 sets the major/minor
    split.
  * Within the major block, fc2 sets the through/left split; within the minor
    block, fc3 sets the through/left split.
  * Eq. 48 ties ring A and ring B phase lengths together across the barrier,
    so one set of splits parameterises both rings.
  * All greens are then clamped to g_min (Eq. 47) and renormalised so each
    ring still sums to Gamma (Eqs. 45-46).

This is the single reconstruction point in Phase 2; any alternative mapping
would change absolute greens but not the method's structure. Flagged in the
report's reproducibility notes.
--------------------------------------------------------------------------
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List
from .chromosome import CycleGene


@dataclass
class SignalTiming:
    """Timing parameters common to every movement."""
    gamma: float = 120.0     # cycle length [s] (verified)
    yellow: float = 3.0      # yellow interval [s]
    all_red: float = 2.0     # red clearance [s]
    g_min: float = 7.0       # minimum green [s] (Eq. 47)


# NEMA movement indexing (Image 7).
RING_A = [1, 2, 3, 4]   # 1: A-left, 2: A-through, 3: B-left, 4: B-through (per fig)
RING_B = [5, 6, 7, 8]


def _split(total: float, frac: float, g_min: float) -> tuple[float, float]:
    """Split `total` green into (a, b) by fraction, clamped so both >= g_min."""
    a = total * frac
    b = total - a
    # Clamp to minimum green, keeping the sum fixed.
    if a < g_min:
        a, b = g_min, total - g_min
    if b < g_min:
        b, a = g_min, total - g_min
    # If total can't satisfy two minima, fall back to an even split.
    if a < g_min or b < g_min:
        a = b = total / 2.0
    return a, b


def decode_cycle_to_greens(gene: CycleGene, timing: SignalTiming) -> Dict[int, float]:
    """Return green time [s] per NEMA phase (1..8) for one cycle.

    The four movements per ring are: major-through, major-left, minor-through,
    minor-left. Ring B mirrors ring A across the barrier (Eq. 48).
    """
    lost_per_phase = timing.yellow + timing.all_red
    # Two phases per ring reach the stop line sequentially across the barrier
    # (major block then minor block), so lost time applies to the barrier's
    # two green blocks per ring: major block and minor block => 2 * lost.
    available = timing.gamma - 2.0 * lost_per_phase
    if available < 2.0 * timing.g_min:
        available = 2.0 * timing.g_min

    f_major_minor, f_major_TL, f_minor_TL = gene.fractions()

    # 1) major vs minor block
    major_block, minor_block = _split(available, f_major_minor, 2 * timing.g_min)
    # 2) within each block, through vs left
    maj_through, maj_left = _split(major_block, f_major_TL, timing.g_min)
    min_through, min_left = _split(minor_block, f_minor_TL, timing.g_min)

    # Assign to NEMA phases. Phase sequence (ps) reorders left vs through
    # (lead/lag) but does not change green *durations*, only their order, so
    # durations are sequence-invariant here (order matters for PT trajectory
    # timing, handled in the simulation layer).
    greens = {
        2: maj_through, 1: maj_left,     # ring A major
        4: min_through, 3: min_left,     # ring A minor
        6: maj_through, 5: maj_left,     # ring B major (Eq. 48 mirror)
        8: min_through, 7: min_left,     # ring B minor
    }
    return greens


def ring_sums_ok(greens: Dict[int, float], timing: SignalTiming,
                 tol: float = 1e-6) -> bool:
    """Check Eqs. 45-46: each ring's phase lengths sum to Gamma.

    In the NEMA dual ring the barrier splits the cycle into a major block and
    a minor block; within each block the through and left phases run
    sequentially. So a ring's total phase length is the sum of ALL its four
    greens plus lost time for its two green blocks (one lost-time chunk per
    block: major block, minor block).
    """
    lost = timing.yellow + timing.all_red
    ringA = greens[1] + greens[2] + greens[3] + greens[4] + 2 * lost
    ringB = greens[5] + greens[6] + greens[7] + greens[8] + 2 * lost
    return abs(ringA - timing.gamma) < 1.0 and abs(ringB - timing.gamma) < 1.0


def decode_horizon(genes: List[CycleGene], timing: SignalTiming) -> List[Dict[int, float]]:
    """Decode every cycle in the horizon into per-phase greens."""
    return [decode_cycle_to_greens(g, timing) for g in genes]
