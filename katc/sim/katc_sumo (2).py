"""
KATC closed control loop on SUMO via TraCI (Phase 3) -- replaces VISSIM+COM.

Every cycle:
  1. Read state from SUMO: per-lane-group vehicle counts / arrival estimate,
     and the position + occupancy of approaching PT vehicles (via TraCI).
  2. Run the KATC optimiser (Phase 1 delay model + Phase 2 GA) to choose the
     signal plan (green splits + phase sequence) for the next cycle.
  3. Write the optimised NEMA plan to SUMO as a phase-state program (TraCI
     setProgramLogic / setRedYellowGreenState).
  4. Advance one cycle; repeat over the simulation horizon.

Controller modes:
  * KATC-2 : optimise green times AND phase sequence (full model)
  * KATC-1 : optimise green times only (ps fixed)
  * FIXED  : Webster/fixed-time baseline (no optimisation) -- SYNCHRO substitute

Run (on a machine with SUMO):
  python katc_sumo.py --mode katc2 --dos 0.6 --pt-occupancy 40 --gui

This script imports the KATC core from the parent package. Ensure the project
root (containing the `katc/` package) is on PYTHONPATH, e.g. run from the repo
root:  python sim/katc_sumo.py ...
"""
from __future__ import annotations
import argparse, os, sys

# Make the katc package importable when run from repo root or sim/.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from katc.fundamental_diagram import FundamentalDiagram
from katc.general_delay import analyze_cycle
from katc.pt_delay import pt_vehicle_delay, PTCycle
from katc.objective import average_passenger_delay, LaneCycleTerm
from katc.chromosome import decode
from katc.decoder import SignalTiming, decode_cycle_to_greens
from katc.ga import run_ga, GAConfig

from nema_to_sumo import build_state_program

# NEMA movement -> the controlled connection(s). Filled at runtime from TraCI
# link data; the mapping to (fromEdge, fromLane, toEdge) is in MOVEMENTS.md.
# Exclusive-left lanes: major left = lane 2 (of 3), minor left = lane 1 (of 2).
MOVEMENT_DEF = {
    1: ("E_C", 2, "C_N"),  # major EB left  (exclusive left lane)
    2: ("W_C", 0, "C_E"),  # major WB through
    3: ("S_C", 1, "C_W"),  # minor SB left  (exclusive left lane)
    4: ("N_C", 0, "C_S"),  # minor NB through
    5: ("W_C", 2, "C_S"),  # major WB left  (exclusive left lane)
    6: ("E_C", 0, "C_W"),  # major EB through
    7: ("N_C", 1, "C_E"),  # minor NB left  (exclusive left lane)
    8: ("S_C", 0, "C_N"),  # minor SB through
}

TIMING = SignalTiming(gamma=120.0, yellow=3.0, all_red=2.0, g_min=7.0)
FD = FundamentalDiagram(v_f=15.0, k_j=0.15, q_c=0.5)
TLS_ID = "C"
LANE_GROUP_EDGES = {2: "W_C", 1: "E_C", 4: "N_C", 3: "S_C"}  # detector edges


def build_movement_links(traci):
    """Map each NEMA movement to the SUMO linkIndices controlling it.

    Also returns the set of RIGHT-TURN link indices (connections not part of
    the 8 signalised NEMA movements), which run permissively so shared
    through+right lanes are never blocked by a right-turner on a red arrow.
    """
    links = traci.trafficlight.getControlledLinks(TLS_ID)
    mv_links = {m: [] for m in MOVEMENT_DEF}
    claimed = set()
    for idx, link_tuples in enumerate(links):
        for (inLane, outLane, _via) in link_tuples:
            in_edge = inLane.rsplit("_", 1)[0]
            in_lane_no = int(inLane.rsplit("_", 1)[1])
            out_edge = outLane.rsplit("_", 1)[0]
            for m, (fe, fl, te) in MOVEMENT_DEF.items():
                if in_edge == fe and in_lane_no == fl and out_edge == te:
                    mv_links[m].append(idx)
                    claimed.add(idx)
    # Any controlled link not claimed by a NEMA movement is a right turn.
    permissive = [i for i in range(len(links)) if i not in claimed]
    return mv_links, len(links), permissive


def read_arrivals(traci, edge, cycle_len):
    """Estimate arrival flow [veh/s] on an approach, clamped below capacity.

    We use the number of vehicles that ENTERED the edge in the last step as a
    proxy for arrivals, scaled to per-second. A raw count/cycle_len over-counts
    because it includes queued (accumulated) vehicles, so we cap the estimate
    just below the fundamental-diagram capacity: a movement cannot *arrive*
    faster than free-flow discharge -- any true excess manifests as queue
    (oversaturation), which the delay model handles via its residual-queue
    (Case B/D) logic. Capping here keeps the shockwave math defined.
    """
    n = traci.edge.getLastStepVehicleNumber(edge)
    raw = n / cycle_len
    cap = FD.q_c * 0.98   # just below capacity to stay on the uncongested branch
    return min(max(raw, 1e-3), cap)


def read_pt_vehicles(traci, edge):
    """Return list of (distance_to_stopline, occupancy) for PT vehicles on edge."""
    out = []
    for vid in traci.edge.getLastStepVehicleIDs(edge):
        if traci.vehicle.getTypeID(vid) == "bus":
            lane_pos = traci.vehicle.getLanePosition(vid)
            lane_len = traci.lane.getLength(traci.vehicle.getLaneID(vid))
            dist = max(lane_len - lane_pos, 0.0)
            try:
                occ = float(traci.vehicle.getParameter(vid, "occupancy") or 40)
            except Exception:
                occ = 40.0
            out.append((dist, occ))
    return out


def evaluate_plan(bits, arrivals, pt_state, mode):
    """Average passenger delay for a candidate chromosome given current state."""
    genes = decode(bits)
    if mode == "katc1":
        for g in genes:      # freeze phase sequence for KATC-1
            g.ps = 1
    terms = []
    cap = FD.q_c * 0.98
    for phase, edge in LANE_GROUP_EDGES.items():
        q_g = min(arrivals.get(edge, 0.05), cap)   # stay on uncongested branch
        prev = None
        cyc = []
        for gene in genes:
            greens = decode_cycle_to_greens(gene, TIMING)
            g = greens[phase]
            r = TIMING.gamma - g
            res = analyze_cycle(r=r, g=g, fd=FD, q_g=q_g, prev=prev)
            cyc.append(res); prev = res
        for i in range(len(cyc)):
            c1 = PTCycle.from_result(cyc[i])
            c2 = PTCycle.from_result(cyc[i + 1] if i + 1 < len(cyc) else cyc[i])
            d_b = 0.0; n_b = 0
            for (dist, occ) in pt_state.get(edge, []):
                d_b += pt_vehicle_delay(dist, c1, c2, FD, q_g); n_b += 1
            terms.append(LaneCycleTerm(d_g_area=cyc[i].d_g, d_b_total=d_b,
                                       q_g=q_g, n_b=max(n_b, 0)))
    return average_passenger_delay(terms, o_g=1.5, o_b=40, k_j=FD.k_j,
                                   gamma=TIMING.gamma)


def optimise(arrivals, pt_state, mode, seed):
    """Run the GA (or fixed plan) and return the best chromosome."""
    if mode == "fixed":
        # Webster-ish fixed plan: mid-range factors, ps=1.
        from katc.chromosome import CycleGene, encode
        genes = [CycleGene(ps=1, fc=[32, 32, 32]) for _ in range(3)]
        return encode(genes)
    cfg = GAConfig(n_cycles=3, pop_size=60, n_generations=40, seed=seed)
    res = run_ga(lambda b: evaluate_plan(b, arrivals, pt_state, mode), cfg)
    return res.best_bits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["katc2", "katc1", "fixed"], default="katc2")
    ap.add_argument("--dos", type=float, default=0.6)
    ap.add_argument("--pt-occupancy", type=float, default=40)
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--horizon", type=int, default=3600)
    ap.add_argument("--warmup-cycles", type=int, default=2)
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args()

    import traci
    sumo_bin = "sumo-gui" if a.gui else "sumo"
    traci.start([sumo_bin, "-c", "intersection.sumocfg",
                 "--start", "--quit-on-end",
                 "--tripinfo-output", f"tripinfo_{a.mode}.xml"])

    mv_links, n_links, permissive_links = build_movement_links(traci)
    cycle = 0
    total_pt_delay = 0.0; pt_count = 0

    while traci.simulation.getMinExpectedNumber() > 0 and \
            traci.simulation.getTime() < a.horizon:
        # Read state.
        arrivals = {e: read_arrivals(traci, e, TIMING.gamma)
                    for e in LANE_GROUP_EDGES.values()}
        pt_state = {e: read_pt_vehicles(traci, e)
                    for e in LANE_GROUP_EDGES.values()}

        # Optimise (skip during warm-up: use fixed plan).
        if cycle < a.warmup_cycles:
            bits = optimise({}, {}, "fixed", a.seed)
        else:
            bits = optimise(arrivals, pt_state, a.mode, a.seed + cycle)

        # Deploy the FIRST cycle of the horizon plan.
        gene = decode(bits)[0]
        greens = decode_cycle_to_greens(gene, TIMING)
        prog = build_state_program(greens, gene.ps, TIMING.yellow,
                                   TIMING.all_red, mv_links, n_links,
                                   permissive_links)

        # Apply the program by stepping through its stages.
        for state, dur in prog:
            traci.trafficlight.setRedYellowGreenState(TLS_ID, state)
            end = traci.simulation.getTime() + dur
            while traci.simulation.getTime() < end and \
                    traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
        cycle += 1

    traci.close()
    print(f"Finished {cycle} cycles in mode={a.mode}. "
          f"Tripinfo written to tripinfo_{a.mode}.xml")


if __name__ == "__main__":
    main()
