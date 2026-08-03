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

# NEMA movement -> the controlled connection(s). Each movement lists ALL
# (fromEdge, fromLane, toEdge) triples it controls, so a through movement served
# from two lanes on a major approach is controlled as one unit (protected green
# on both lanes). Exclusive-left lanes: major = lane 2 (of 3), minor = lane 1.
MOVEMENT_DEF = {
    1: [("E_C", 2, "C_S")],                      # major EB left  -> C_S (corrected)
    2: [("W_C", 0, "C_E"), ("W_C", 1, "C_E")],   # major WB through (2 lanes)
    3: [("S_C", 1, "C_W")],                      # minor SB left  -> C_W
    4: [("N_C", 0, "C_S")],                      # minor NB through
    5: [("W_C", 2, "C_N")],                      # major WB left  -> C_N (corrected)
    6: [("E_C", 0, "C_W"), ("E_C", 1, "C_W")],   # major EB through (2 lanes)
    7: [("N_C", 1, "C_E")],                      # minor NB left  -> C_E
    8: [("S_C", 0, "C_N")],                      # minor SB through
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
            for m, triples in MOVEMENT_DEF.items():
                for (fe, fl, te) in triples:
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


def optimise(arrivals, pt_state, mode, seed, forced_ps=None):
    """Run the GA (or fixed plan) and return the best chromosome.

    Green-time distribution is optimised analytically by the GA (fast).
    Phase sequence is handled separately:
      * fixed / katc1 : ps forced to 1 (no phase-sequence optimisation)
      * katc2         : ps set to `forced_ps`, chosen by the OUTER loop from
                        simulation-measured bus delay (see PhaseSeqSelector).
    """
    if mode == "fixed":
        from katc.chromosome import CycleGene, encode
        genes = [CycleGene(ps=1, fc=[32, 32, 32]) for _ in range(3)]
        return encode(genes)
    cfg = GAConfig(n_cycles=3, pop_size=60, n_generations=40, seed=seed)
    res = run_ga(lambda b: evaluate_plan(b, arrivals, pt_state, mode), cfg)
    bits = res.best_bits
    # Overwrite the phase-sequence bits with the externally chosen ps.
    if mode == "katc1":
        forced_ps = 1
    if forced_ps is not None:
        from katc.chromosome import decode as _dec, encode as _enc
        genes = _dec(bits)
        for g in genes:
            g.ps = forced_ps
        bits = _enc(genes)
    return bits


class PhaseSeqSelector:
    """Choose phase sequence from SIMULATION-measured bus delay (KATC-2).

    This is what makes KATC-2 differ from KATC-1: instead of trusting the
    analytical model (which is phase-sequence-invariant in our implementation),
    KATC-2 measures the actual bus delay produced by each phase-sequence option
    in live simulation and adopts the best-performing one. KATC-1 never calls
    this (ps stays 1).

    Method: epsilon-greedy over the 4 phase-sequence values {1,2,3,4}. Each
    cycle KATC-2 mostly uses the current best ps, but occasionally trials an
    alternative and records the mean bus delay observed during that cycle,
    updating a running estimate per ps. Over the hour it converges to the ps
    that genuinely minimises measured bus delay for the scenario.
    """
    def __init__(self, rng_seed=0, epsilon=0.25):
        import random
        self.rng = random.Random(rng_seed)
        self.epsilon = epsilon
        self.sum_delay = {p: 0.0 for p in (1, 2, 3, 4)}
        self.count = {p: 0 for p in (1, 2, 3, 4)}
        self.current = 1

    def choose(self):
        """Pick the ps to use this cycle (explore or exploit)."""
        untried = [p for p in (1, 2, 3, 4) if self.count[p] == 0]
        if untried:
            self.current = untried[0]              # try each option once first
        elif self.rng.random() < self.epsilon:
            self.current = self.rng.choice([1, 2, 3, 4])   # explore
        else:
            # exploit: lowest mean measured delay
            self.current = min((1, 2, 3, 4),
                               key=lambda p: self.sum_delay[p] / self.count[p])
        return self.current

    def record(self, measured_delay):
        """Record the bus delay observed for the ps used this cycle."""
        self.sum_delay[self.current] += measured_delay
        self.count[self.current] += 1

    def best(self):
        tried = [p for p in (1, 2, 3, 4) if self.count[p] > 0]
        if not tried:
            return 1
        return min(tried, key=lambda p: self.sum_delay[p] / self.count[p])


def measure_bus_delay_this_step(traci, prev_bus_times):
    """Accumulate bus time-loss seen on the network (proxy for cycle bus delay).

    Returns (delay_increment, updated_prev). We sum each bus's incremental
    accumulated waiting time since last observation.
    """
    total = 0.0
    new_prev = {}
    for edge in LANE_GROUP_EDGES.values():
        for vid in traci.edge.getLastStepVehicleIDs(edge):
            if traci.vehicle.getTypeID(vid) == "bus":
                wt = traci.vehicle.getAccumulatedWaitingTime(vid)
                new_prev[vid] = wt
                inc = wt - prev_bus_times.get(vid, 0.0)
                if inc > 0:
                    total += inc
    return total, new_prev


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
    selector = PhaseSeqSelector(rng_seed=a.seed) if a.mode == "katc2" else None
    prev_bus_times = {}

    while traci.simulation.getMinExpectedNumber() > 0 and \
            traci.simulation.getTime() < a.horizon:
        # Read state.
        arrivals = {e: read_arrivals(traci, e, TIMING.gamma)
                    for e in LANE_GROUP_EDGES.values()}
        pt_state = {e: read_pt_vehicles(traci, e)
                    for e in LANE_GROUP_EDGES.values()}

        # Phase sequence: KATC-2 picks via simulation-measured bus delay.
        chosen_ps = selector.choose() if selector is not None else None

        # Optimise green times (skip during warm-up: use fixed plan).
        if cycle < a.warmup_cycles:
            bits = optimise({}, {}, "fixed", a.seed)
        else:
            bits = optimise(arrivals, pt_state, a.mode, a.seed + cycle,
                            forced_ps=chosen_ps)

        # Deploy the FIRST cycle of the horizon plan.
        gene = decode(bits)[0]
        greens = decode_cycle_to_greens(gene, TIMING)
        prog = build_state_program(greens, gene.ps, TIMING.yellow,
                                   TIMING.all_red, mv_links, n_links,
                                   permissive_links)

        # Apply the program, measuring bus delay accrued during THIS cycle.
        cycle_bus_delay = 0.0
        for state, dur in prog:
            traci.trafficlight.setRedYellowGreenState(TLS_ID, state)
            end = traci.simulation.getTime() + dur
            while traci.simulation.getTime() < end and \
                    traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
                inc, prev_bus_times = measure_bus_delay_this_step(traci, prev_bus_times)
                cycle_bus_delay += inc

        # Feed the measured bus delay back to the phase-sequence selector.
        if selector is not None and cycle >= a.warmup_cycles:
            selector.record(cycle_bus_delay)
        cycle += 1

    if selector is not None:
        print(f"KATC-2 selected phase sequence: {selector.best()} "
              f"(counts {selector.count})")
    traci.close()
    print(f"Finished {cycle} cycles in mode={a.mode}. "
          f"Tripinfo written to tripinfo_{a.mode}.xml")


if __name__ == "__main__":
    main()
