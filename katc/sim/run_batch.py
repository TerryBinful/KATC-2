"""
Batch runner for the full KATC experiment grid (Phase 5).

Runs every (DOS, PT-occupancy, controller) scenario, parses each run's delay
metrics, and appends them to results.csv. Designed to run UNATTENDED and
RESUMABLY over many hours:

  * Resumable: already-completed rows (matched by DOS/occupancy/mode/seed) are
    skipped, so you can stop and restart without losing progress.
  * Incremental: results.csv is written after every run; partial results are
    always usable.
  * Headless + fast: no GUI; GA trimmed for batch throughput (quality preserved
    -- the analytical test showed convergence within ~25 generations).

Paper grid: DOS in {0.4..1.0} (7), PT occupancy in {20,30,40,50,60} (5),
controllers {katc2, katc1, fixed} = 105 scenarios. Add --seeds for replication.

Usage (from the sim/ folder, with SUMO on PATH and traci importable):
  python run_batch.py                       # full grid, 1 seed
  python run_batch.py --seeds 3             # 3 seeds each (315 runs)
  python run_batch.py --quick               # tiny pilot (smoke test)
  python run_batch.py --dos 0.6 0.8         # restrict DOS levels
"""
from __future__ import annotations
import argparse, csv, os, sys, time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

DOS_LEVELS = [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
OCCUPANCIES = [20, 30, 40, 50, 60]
MODES = ["katc2", "katc1", "fixed"]
RESULTS_CSV = "results.csv"
FIELDS = ["dos", "pt_occupancy", "mode", "seed",
          "avg_passenger_delay", "avg_pt_passenger_delay", "avg_general_delay",
          "n_cars", "n_buses", "runtime_s"]


def load_done(path):
    done = set()
    if os.path.exists(path):
        with open(path, newline="") as f:
            for row in csv.DictReader(f):
                done.add((float(row["dos"]), float(row["pt_occupancy"]),
                          row["mode"], int(row["seed"])))
    return done


def append_row(path, row):
    exists = os.path.exists(path)
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        if not exists:
            w.writeheader()
        w.writerow(row)


def run_one(dos, occ, mode, seed, horizon, warmup):
    """Run a single scenario in-process via TraCI; return DelayMetrics + runtime."""
    import traci
    import make_demand
    import katc_sumo as K
    from parse_results import parse_tripinfo

    # Write scenario-specific demand.
    with open("demand.rou.xml", "w") as f:
        f.write(make_demand.build(dos, occ, horizon))

    tripfile = f"trip_{mode}_{dos}_{occ}_{seed}.xml"
    traci.start(["sumo", "-c", "intersection.sumocfg", "--start",
                 "--quit-on-end", "--no-warnings", "true",
                 "--tripinfo-output", tripfile])
    mv_links, n_links, permissive = K.build_movement_links(traci)
    from nema_to_sumo import build_state_program
    from katc.chromosome import decode
    from katc.decoder import decode_cycle_to_greens

    t0 = time.time()
    cycle = 0
    selector = K.PhaseSeqSelector(rng_seed=seed) if mode == "katc2" else None
    prev_bus_times = {}
    while traci.simulation.getMinExpectedNumber() > 0 and \
            traci.simulation.getTime() < horizon:
        arrivals = {e: K.read_arrivals(traci, e, K.TIMING.gamma)
                    for e in K.LANE_GROUP_EDGES.values()}
        pt_state = {e: K.read_pt_vehicles(traci, e)
                    for e in K.LANE_GROUP_EDGES.values()}
        chosen_ps = selector.choose() if selector is not None else None
        m = "fixed" if cycle < warmup else mode
        if m == "fixed":
            bits = K.optimise({}, {}, "fixed", seed + cycle)
        else:
            bits = K.optimise(arrivals, pt_state, m, seed + cycle,
                              forced_ps=chosen_ps)
        gene = decode(bits)[0]
        greens = decode_cycle_to_greens(gene, K.TIMING)
        prog = build_state_program(greens, gene.ps, K.TIMING.yellow,
                                   K.TIMING.all_red, mv_links, n_links, permissive)
        cycle_bus_delay = 0.0
        for state, dur in prog:
            traci.trafficlight.setRedYellowGreenState(K.TLS_ID, state)
            end = traci.simulation.getTime() + dur
            while traci.simulation.getTime() < end and \
                    traci.simulation.getMinExpectedNumber() > 0:
                traci.simulationStep()
                inc, prev_bus_times = K.measure_bus_delay_this_step(traci, prev_bus_times)
                cycle_bus_delay += inc
        if selector is not None and cycle >= warmup:
            selector.record(cycle_bus_delay)
        cycle += 1
    traci.close()
    runtime = time.time() - t0

    metrics = parse_tripinfo(tripfile, o_g=1.5, pt_occupancy=occ)
    return metrics, runtime


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dos", type=float, nargs="*", default=None)
    ap.add_argument("--occupancy", type=float, nargs="*", default=None)
    ap.add_argument("--modes", nargs="*", default=None)
    ap.add_argument("--seeds", type=int, default=1)
    ap.add_argument("--horizon", type=int, default=3600)
    ap.add_argument("--warmup", type=int, default=2)
    ap.add_argument("--quick", action="store_true", help="tiny pilot smoke test")
    a = ap.parse_args()

    dos_levels = a.dos or (DOS_LEVELS if not a.quick else [0.4, 1.0])
    occs = a.occupancy or (OCCUPANCIES if not a.quick else [20, 60])
    modes = a.modes or (MODES if not a.quick else ["katc2", "fixed"])
    seeds = range(1, a.seeds + 1) if not a.quick else [1]
    if a.quick:
        a.horizon = 600   # short horizon for the smoke test

    todo = [(d, o, m, s) for d in dos_levels for o in occs
            for m in modes for s in seeds]
    done = load_done(RESULTS_CSV)
    remaining = [t for t in todo if t not in done]

    print(f"Grid: {len(todo)} scenarios | already done: {len(todo)-len(remaining)} "
          f"| remaining: {len(remaining)}")
    if not remaining:
        print("Nothing to do -- all scenarios complete. See", RESULTS_CSV)
        return

    for i, (dos, occ, mode, seed) in enumerate(remaining, 1):
        print(f"[{i}/{len(remaining)}] DOS={dos} occ={occ} mode={mode} seed={seed} ...",
              end=" ", flush=True)
        try:
            metrics, runtime = run_one(dos, occ, mode, seed, a.horizon, a.warmup)
            append_row(RESULTS_CSV, dict(
                dos=dos, pt_occupancy=occ, mode=mode, seed=seed,
                avg_passenger_delay=round(metrics.avg_passenger_delay, 3),
                avg_pt_passenger_delay=round(metrics.avg_pt_passenger_delay, 3),
                avg_general_delay=round(metrics.avg_general_delay, 3),
                n_cars=metrics.n_cars, n_buses=metrics.n_buses,
                runtime_s=round(runtime, 1)))
            print(f"delay={metrics.avg_passenger_delay:.2f} s/p  ({runtime:.0f}s)")
        except Exception as e:
            print(f"FAILED: {e}")
            # Continue with the next scenario rather than aborting the whole batch.
            continue

    print(f"\nBatch complete. Results in {RESULTS_CSV}")


if __name__ == "__main__":
    main()
