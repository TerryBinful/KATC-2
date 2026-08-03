"""
GA chromosome encoding (Section 4, Figure 6) -- author-verified.

Per cycle, the candidate solution is:

    [ ps (2 bits) ][ fc1 (6 bits) ][ fc2 (6 bits) ][ fc3 (6 bits) ]  = 20 bits

  * ps  : phase-sequence decision variable in {1,2,3,4} = {lag-lag,
          lead-lag, lag-lead, lead-lead} -- the order of the left-turn phase
          on each side of the NEMA dual-ring barrier (Eq. 48 structure).
  * fc1,fc2,fc3 : three 6-bit coefficient factors that distribute green time
          between major and minor movements (each integer 0..63).

The decision horizon is 3 cycles (verified: "optimal solution ... over a
3-cycle decision horizon"), so a full chromosome is 3 x 20 = 60 bits.

Encoding after Park et al. [46,47], revised for the protected left-turn case.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import List
import random

PS_BITS = 2
FC_BITS = 6
N_FC = 3
BITS_PER_CYCLE = PS_BITS + N_FC * FC_BITS   # 20
FC_MAX = (1 << FC_BITS) - 1                  # 63


@dataclass
class CycleGene:
    """Decoded gene for one cycle."""
    ps: int            # phase sequence in {1,2,3,4}
    fc: List[int]      # three coefficient factors, each 0..63

    def fractions(self) -> List[float]:
        """Coefficient factors as fractions in [0,1] (fc_i / 63)."""
        return [f / FC_MAX for f in self.fc]


def random_bits(n_cycles: int, rng: random.Random | None = None) -> List[int]:
    """Random bit-string chromosome for n_cycles."""
    r = rng or random
    return [r.randint(0, 1) for _ in range(n_cycles * BITS_PER_CYCLE)]


def _bits_to_int(bits: List[int]) -> int:
    v = 0
    for b in bits:
        v = (v << 1) | (b & 1)
    return v


def decode(bits: List[int]) -> List[CycleGene]:
    """Decode a bit-string chromosome into per-cycle genes.

    Raises ValueError if length is not a multiple of BITS_PER_CYCLE.
    """
    if len(bits) % BITS_PER_CYCLE != 0:
        raise ValueError(
            f"chromosome length {len(bits)} is not a multiple of "
            f"{BITS_PER_CYCLE} (bits per cycle)"
        )
    n_cycles = len(bits) // BITS_PER_CYCLE
    genes: List[CycleGene] = []
    for c in range(n_cycles):
        base = c * BITS_PER_CYCLE
        ps_bits = bits[base:base + PS_BITS]
        ps = _bits_to_int(ps_bits) + 1          # {00,01,10,11} -> {1,2,3,4}
        fc: List[int] = []
        for i in range(N_FC):
            s = base + PS_BITS + i * FC_BITS
            fc.append(_bits_to_int(bits[s:s + FC_BITS]))
        genes.append(CycleGene(ps=ps, fc=fc))
    return genes


def encode(genes: List[CycleGene]) -> List[int]:
    """Encode per-cycle genes back into a bit string (inverse of decode)."""
    bits: List[int] = []
    for g in genes:
        ps_val = g.ps - 1
        bits.extend([(ps_val >> (PS_BITS - 1 - k)) & 1 for k in range(PS_BITS)])
        for i in range(N_FC):
            v = g.fc[i]
            bits.extend([(v >> (FC_BITS - 1 - k)) & 1 for k in range(FC_BITS)])
    return bits
