# Phase 5 - Batch runner (built; run locally)

Runs the full paper grid unattended + resumably, parses delay metrics per Eq. 43.

## Files (into sim\)
- parse_results.py : tripinfo.xml -> avg passenger / PT / general delay
                     (occupancy-weighted; math verified)
- run_batch.py     : resumable grid runner, appends to results.csv per run

## The full grid
7 DOS (0.4-1.0) x 5 PT occupancy (20-60) x 3 controllers (katc2/katc1/fixed)
= 105 scenarios (x seeds if --seeds > 1).

## How to run (from sim\, SUMO on PATH, traci importable)

    # FIRST: a quick smoke test (~2 short runs) to confirm the pipeline end-to-end
    python run_batch.py --quick

    # If that produces results.csv rows, launch the full grid:
    python run_batch.py

    # Optional: replication over seeds (multiplies runtime)
    python run_batch.py --seeds 3

## Resumability
- Progress saved to results.csv after EVERY run.
- Stop anytime (Ctrl-C); re-running skips completed scenarios.
- A failed scenario is logged and skipped, not fatal to the batch.

## Time budget
Each run = up to 1 simulated hour with the GA every cycle. Wall-clock depends
on your CPU; expect minutes per run. 105 runs could be a few hours. Start it,
leave it, check results.csv periodically. Consider running overnight.

## Speed levers if too slow
- Lower GA in katc_sumo.optimise (pop/gen) - analytical test converged ~25 gens.
- Shorten --horizon (e.g. 1800) - documented deviation from the 1 h paper horizon.
- Reduce --warmup if needed.

## Output
results.csv columns:
  dos, pt_occupancy, mode, seed, avg_passenger_delay, avg_pt_passenger_delay,
  avg_general_delay, n_cars, n_buses, runtime_s

This CSV is the input to Phase 6 (compare against the paper's Image-4 surfaces
and build the improvement curves).
