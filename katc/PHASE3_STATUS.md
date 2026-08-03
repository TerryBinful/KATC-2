# Phase 3 - SUMO integration (network + control loop built)

Full NEMA dual-ring representation (chosen for fidelity to the paper's
phase-sequence contribution). SUMO not runnable in the build environment, so
these files are verified structurally + by pure-logic tests; you run them
locally where SUMO lives.

## Files (put in a `sim\` folder inside the repo)
| File | Role | Paper link |
|------|------|-----------|
| intersection.nod/edg/con.xml | network geometry (2-lane major, 1-lane minor, exclusive lefts) | Fig. 7 |
| build_network.sh | compiles the .net.xml via netconvert | - |
| intersection.sumocfg | SUMO run config | - |
| make_demand.py | general traffic (70/20/10 splits) + 8 PT routes (verified headways), scaled by DOS | Sec. 5.2, Fig. 7 |
| nema_to_sumo.py | maps 8 NEMA greens + phase sequence -> SUMO state strings (dual-ring, lead/lag) | Sec 3.6, Image 7 |
| katc_sumo.py | TraCI closed loop: read state -> GA optimise -> write plan -> step | Sec 2, Fig 1; replaces VISSIM+COM |
| test_nema.py | 10 pure-logic tests for the NEMA mapping (all pass) | - |
| MOVEMENTS.md | NEMA movement <-> connection reference | Image 7 |

## How to run locally (from repo root, with SUMO installed + on PATH)

    cd sim
    bash build_network.sh                 # -> intersection.net.xml
    sumo-gui -n intersection.net.xml      # eyeball geometry (optional)
    python make_demand.py --dos 0.6 --pt-occupancy 40
    python katc_sumo.py --mode katc2 --dos 0.6 --pt-occupancy 40 --gui
    python test_nema.py                   # 10 pass

`katc_sumo.py --mode` accepts katc2 / katc1 / fixed (fixed = Webster substitute
for SYNCHRO). Tripinfo is written per mode for the Phase 5/6 delay analysis.

## First-run checklist (expect to debug these on your machine)
1. `netconvert` builds without connection warnings (fix any lane mismatches).
2. In sumo-gui, confirm buses appear (red) and follow the 8 routes.
3. Confirm the TL cycles through major-left/through then minor-left/through.
4. If TraCI can't import: `pip install traci` OR set SUMO_HOME and add
   %SUMO_HOME%\tools to PYTHONPATH.
5. Reduce GA (pop=60,gen=40) keeps each cycle's optimise fast enough for a
   live loop; raise later for final runs.

## Deviations logged (for the report)
- VISSIM+COM -> SUMO+TraCI (closed loop on open-source simulator).
- Right turns run permissively with the parallel through (not separately
  signalised) - consistent with the paper's through+left phasing focus.
- Detector-based arrival estimate replaces V2I demand feed.
