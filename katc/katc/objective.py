"""
Average passenger delay objective (Section 3.5, Eq. 43) -- the KATC fitness.

Eq. 43:
    d_a = ( sum_c sum_l [ o_g * k_j * d_g^{l,c} + o_b * d_b^{l,c} ] )
          / ( sum_c sum_l [ o_g * q_g^{l,c} * Gamma + o_b * n_b^{l,c} ] )

where
    d_g^{l,c}  general-traffic delay AREA (length x time) from general_delay
    d_b^{l,c}  total PT-vehicle delay [veh.s] on lane group l, cycle c
    o_g, o_b   passenger occupancy of general traffic and PT vehicles
    k_j        jam density [veh/m] -- converts the d_g area to vehicle-seconds
    q_g        general-traffic arrival flow [veh/s]
    Gamma      cycle length [s]
    n_b        number of PT vehicles detected on lane group l in cycle c

The numerator is total passenger-seconds of delay; the denominator is the
total number of passengers served over the decision horizon. Their ratio is
the average passenger delay. The GA minimises d_a (fitness = 1 / d_a, or
-d_a); we expose d_a directly and let the optimiser wrap it.

This module only assembles the objective from per-(lane, cycle) delay terms;
computing those terms (general_delay.analyze_cycle, pt_delay.pt_vehicle_delay)
is the caller's job, which keeps the objective simulator-agnostic.
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable


@dataclass
class LaneCycleTerm:
    """One (lane group, cycle) contribution to the objective."""
    d_g_area: float   # general-traffic delay AREA (length x time) [m.s]
    d_b_total: float  # total PT delay on this lane/cycle [veh.s]
    q_g: float        # general-traffic arrival flow [veh/s]
    n_b: int          # number of PT vehicles this lane/cycle


def average_passenger_delay(
    terms: Iterable[LaneCycleTerm],
    o_g: float,
    o_b: float,
    k_j: float,
    gamma: float,
) -> float:
    """Eq. 43: average passenger delay over the decision horizon.

    Parameters
    ----------
    terms : iterable of LaneCycleTerm
        One entry per (lane group, cycle) in the decision horizon.
    o_g, o_b : float
        Occupancy (passengers/vehicle) of general traffic and PT vehicles.
    k_j : float
        Jam density [veh/m] (converts d_g area to vehicle-seconds).
    gamma : float
        Cycle length [s].

    Returns
    -------
    float
        Average passenger delay [s].
    """
    num = 0.0
    den = 0.0
    for t in terms:
        num += o_g * k_j * t.d_g_area + o_b * t.d_b_total
        den += o_g * t.q_g * gamma + o_b * t.n_b
    if den <= 0:
        return 0.0
    return num / den
