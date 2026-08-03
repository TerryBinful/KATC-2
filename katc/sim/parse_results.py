"""
Parse a SUMO tripinfo.xml into KATC delay metrics (Eq. 43 components).

For each scenario run we extract:
  * average passenger delay     [s/passenger]  -- both modes, occupancy-weighted
  * average PT passenger delay  [s/passenger]  -- buses only
  * average general delay       [s/passenger]  -- cars only

SUMO tripinfo gives per-vehicle timeLoss (delay vs. free-flow). We weight each
vehicle by passenger occupancy: cars by o_g = 1.5, buses by the scenario PT
occupancy. This mirrors the paper's passenger-based objective.
"""
from __future__ import annotations
import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass
class DelayMetrics:
    avg_passenger_delay: float
    avg_pt_passenger_delay: float
    avg_general_delay: float
    n_cars: int
    n_buses: int


def parse_tripinfo(path: str, o_g: float = 1.5, pt_occupancy: float = 40.0) -> DelayMetrics:
    tree = ET.parse(path)
    root = tree.getroot()

    car_delay_pax = 0.0; car_pax = 0.0; n_cars = 0
    bus_delay_pax = 0.0; bus_pax = 0.0; n_buses = 0

    for trip in root.findall("tripinfo"):
        time_loss = float(trip.get("timeLoss", "0"))
        vtype = trip.get("vType", "")
        is_bus = ("bus" in vtype) or (trip.get("id", "").startswith(("f_pt", "pt")))
        if is_bus:
            bus_delay_pax += time_loss * pt_occupancy
            bus_pax += pt_occupancy
            n_buses += 1
        else:
            car_delay_pax += time_loss * o_g
            car_pax += o_g
            n_cars += 1

    total_delay_pax = car_delay_pax + bus_delay_pax
    total_pax = car_pax + bus_pax
    return DelayMetrics(
        avg_passenger_delay=(total_delay_pax / total_pax) if total_pax else 0.0,
        avg_pt_passenger_delay=(bus_delay_pax / bus_pax) if bus_pax else 0.0,
        avg_general_delay=(car_delay_pax / car_pax) if car_pax else 0.0,
        n_cars=n_cars, n_buses=n_buses,
    )


if __name__ == "__main__":
    import sys
    m = parse_tripinfo(sys.argv[1] if len(sys.argv) > 1 else "tripinfo_katc2.xml")
    print(f"avg passenger delay:    {m.avg_passenger_delay:6.2f} s/p")
    print(f"avg PT passenger delay: {m.avg_pt_passenger_delay:6.2f} s/p")
    print(f"avg general delay:      {m.avg_general_delay:6.2f} s/p")
    print(f"cars={m.n_cars} buses={m.n_buses}")
