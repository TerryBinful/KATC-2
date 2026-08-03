"""
Generate SUMO demand (demand.rou.xml) for the KATC intersection.

Reproduces paper Section 5.2:
  * General traffic with verified turning splits (Fig. 7a):
      major approaches: 70% through, 20% left, 10% right
      minor approaches: same proportions
  * General-traffic demand scaled by Degree of Saturation (DOS).
  * Eight mixed-use PT routes with verified headways (Fig. 7b):
      routes 1&5: 310 s   2&6: 210 s   3&7: 580 s   4&8: 460 s
  * PT passenger occupancy carried as a <param> for the control loop to read.

Usage:
  python make_demand.py --dos 0.6 --pt-occupancy 40 --out demand.rou.xml
"""
import argparse

# Saturation flow ~1800 veh/h/lane. Major = 2 lanes, minor = 1 lane.
SAT_FLOW_PER_LANE = 1800.0
MAJOR_LANES = 2
MINOR_LANES = 1

# Turning splits (Fig. 7a).
SPLIT = dict(through=0.70, left=0.20, right=0.10)

# PT route definitions: (route_id, edges, headway_s).
PT_ROUTES = [
    ("pt1", "W_C C_E", 310),   # major through (route 1)
    ("pt5", "W_C C_N", 310),   # major left -> C_N (route 5, corrected)
    ("pt2", "E_C C_W", 210),   # major through (route 2)
    ("pt6", "E_C C_S", 210),   # major left -> C_S (route 6, corrected)
    ("pt3", "S_C C_N", 580),   # minor through (route 3)
    ("pt7", "S_C C_W", 580),   # minor left (route 7)
    ("pt4", "N_C C_S", 460),   # minor through (route 4)
    ("pt8", "N_C C_E", 460),   # minor left (route 8)
]

# General-traffic movement routes: (id, from_edge, to_edge, street, movement).
# depart_lane forces vehicles onto the correct lane so SUMO's "best" heuristic
# cannot place through traffic on the exclusive-left lane (or vice versa).
#   Major approaches: 3 lanes -> lane 2 = exclusive left, lanes 0-1 = through/right.
#   Minor approaches: 2 lanes -> lane 1 = exclusive left, lane 0 = through/right.
GT_ROUTES = [
    # Major West approach (eastbound): left->C_N, right->C_S
    ("gtWE", "W_C", "C_E", "major", "through", "0"),
    ("gtWN", "W_C", "C_N", "major", "left",    "2"),
    ("gtWS", "W_C", "C_S", "major", "right",   "0"),
    # Major East approach (westbound): left->C_S, right->C_N
    ("gtEW", "E_C", "C_W", "major", "through", "0"),
    ("gtES", "E_C", "C_S", "major", "left",    "2"),
    ("gtEN", "E_C", "C_N", "major", "right",   "0"),
    # Minor North approach
    ("gtNS", "N_C", "C_S", "minor", "through", "0"),
    ("gtNE", "N_C", "C_E", "minor", "left",    "1"),
    ("gtNW", "N_C", "C_W", "minor", "right",   "0"),
    # Minor South approach
    ("gtSN", "S_C", "C_N", "minor", "through", "0"),
    ("gtSW", "S_C", "C_W", "minor", "left",    "1"),
    ("gtSE", "S_C", "C_E", "minor", "right",   "0"),
]


def build(dos: float, pt_occupancy: float, horizon: int = 3600) -> str:
    lines = ['<?xml version="1.0" encoding="UTF-8"?>', "<routes>"]
    # Vehicle types.
    lines.append('  <vType id="car" vClass="passenger" length="5.0" '
                 'accel="2.6" decel="4.5" sigma="0.5"/>')
    lines.append('  <vType id="bus" vClass="bus" length="12.0" '
                 'accel="1.2" decel="4.0" sigma="0.5" color="1,0,0"/>')

    # Routes.
    for rid, from_e, to_e, street, mv, dlane in GT_ROUTES:
        lines.append(f'  <route id="{rid}" edges="{from_e} {to_e}"/>')
    for rid, edges, hw in PT_ROUTES:
        e = edges.split()
        lines.append(f'  <route id="{rid}" edges="{e[0]} {e[-1]}"/>')

    # General-traffic flows (veh/h) scaled by DOS and split.
    for rid, from_e, to_e, street, mv, dlane in GT_ROUTES:
        lanes = MAJOR_LANES if street == "major" else MINOR_LANES
        approach_flow = SAT_FLOW_PER_LANE * lanes * dos
        flow = approach_flow * SPLIT[mv]
        lines.append(
            f'  <flow id="f_{rid}" type="car" route="{rid}" '
            f'departLane="{dlane}" begin="0" end="{horizon}" vehsPerHour="{flow:.1f}"/>')

    # PT flows by headway, each vehicle tagged with passenger occupancy.
    for rid, edges, hw in PT_ROUTES:
        lines.append(
            f'  <flow id="f_{rid}" type="bus" route="{rid}" '
            f'begin="0" end="{horizon}" period="{hw}">')
        lines.append(f'    <param key="occupancy" value="{pt_occupancy}"/>')
        lines.append('  </flow>')

    lines.append("</routes>")
    return "\n".join(lines)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--dos", type=float, default=0.6)
    ap.add_argument("--pt-occupancy", type=float, default=40)
    ap.add_argument("--horizon", type=int, default=3600)
    ap.add_argument("--out", default="demand.rou.xml")
    a = ap.parse_args()
    with open(a.out, "w") as f:
        f.write(build(a.dos, a.pt_occupancy, a.horizon))
    print(f"Wrote {a.out}  (DOS={a.dos}, PT occupancy={a.pt_occupancy})")
