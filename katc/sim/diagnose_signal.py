"""
Diagnostic: run a few cycles and report which NEMA movements ever get green,
and total green time per movement. Reveals whether any movement is starved.
Run:  python diagnose_signal.py
"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import traci
from katc.decoder import SignalTiming, decode_cycle_to_greens
from katc.chromosome import decode
from katc.ga import GAConfig
import katc_sumo as K

TIMING = K.TIMING
traci.start(["sumo", "-c", "intersection.sumocfg", "--start", "--quit-on-end"])
mv_links, n_links, permissive_links = K.build_movement_links(traci)

print("Movement -> linkIndices resolved from net:")
for m in sorted(mv_links):
    print(f"  NEMA {m}: links {mv_links[m]}")
print(f"Total controlled links: {n_links}\n")

green_time = {m: 0.0 for m in mv_links}
seen_green = {m: False for m in mv_links}

cycle = 0
MAX_CYCLES = 4
while traci.simulation.getMinExpectedNumber() > 0 and cycle < MAX_CYCLES:
    arrivals = {e: K.read_arrivals(traci, e, TIMING.gamma) for e in K.LANE_GROUP_EDGES.values()}
    pt_state = {e: K.read_pt_vehicles(traci, e) for e in K.LANE_GROUP_EDGES.values()}
    bits = K.optimise(arrivals, pt_state, "fixed" if cycle < 2 else "katc2", 1 + cycle)
    gene = decode(bits)[0]
    greens = decode_cycle_to_greens(gene, TIMING)
    from nema_to_sumo import build_state_program
    prog = build_state_program(greens, gene.ps, TIMING.yellow, TIMING.all_red, mv_links, n_links, permissive_links)
    print(f"--- cycle {cycle} (ps={gene.ps}) greens per NEMA: "
          + ", ".join(f"{m}:{greens[m]:.0f}" for m in sorted(greens)))
    for state, dur in prog:
        # record which movements are green ('G') in this state
        for m, links in mv_links.items():
            if links and all(state[li] == 'G' for li in links):
                green_time[m] += dur
                seen_green[m] = True
        traci.trafficlight.setRedYellowGreenState("C", state)
        end = traci.simulation.getTime() + dur
        while traci.simulation.getTime() < end and traci.simulation.getMinExpectedNumber() > 0:
            traci.simulationStep()
    cycle += 1

traci.close()
print("\n=== Green exposure over", cycle, "cycles ===")
for m in sorted(mv_links):
    flag = "" if seen_green[m] else "   <-- NEVER GREEN"
    print(f"  NEMA {m}: total green {green_time[m]:6.1f}s{flag}")
