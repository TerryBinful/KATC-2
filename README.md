# KATC Reproduction

Reproduction of Behbahani & Poorjafari (2022), *"Proposing a kinematic
wave-based adaptive transit signal priority control using genetic
algorithm"*, IET Intelligent Transport Systems, 17(5), 912-928.
DOI: 10.1049/itr2.12316

Solo reproduction study. Simulator: **SUMO** (substituted for the paper's
VISSIM; documented as a deviation). Baseline: **Webster fixed-time**
(substituted for SYNCHRO; documented as a deviation).

## Status

| Phase | Component | State |
|-------|-----------|-------|
| 1 | Kinematic-wave bimodal delay model | **general-traffic delay DONE + tested** |
| 1 | PT-vehicle delay (Cases A-D) | Done |
| 2 | Genetic algorithm optimizer | Done |
| 3 | SUMO + TraCI integration | Done |
| 4 | Baselines (Webster, KATC-1, KATC-2) | Done |
| 5 | Scenario batch (DOS x PT occupancy) | Done |
| 6 | Analysis & reproducibility verdict | Done |
| 7 | Report (3-5 pp) + slides | **Done**, prepared and submitted |

## Layout

```
katc/
  fundamental_diagram.py   # Fig. 2 triangular FD, shockwave speeds (Eqs. 2-3)
  general_delay.py         # Section 3.3: general-traffic delay (Eqs. 4-12)
tests/
  test_phase1.py           # 17 unit tests, all passing
```

## Run the tests (locally, with real pytest)

```bash
python -m venv .venv && source .venv/bin/activate   # or conda
pip install pytest numpy pandas matplotlib
python -m pytest tests/ -v
```

## Equation map (paper -> code)

- Eq. 2 formation shockwave  -> `FundamentalDiagram.v_qf`
- Eq. 3 dissipation shockwave -> `FundamentalDiagram.v_qd`
- Eq. 4 queue length          -> `general_delay.queue_length_no_residual`
- Eq. 5 min green to clear    -> `general_delay.min_green_to_clear`
- Eq. 6 Case A test / Eq. 7 delay -> `evaluate_general_delay` (undersaturated)
- Eq. 8 Case B test / Eqs. 9-12   -> `evaluate_general_delay` (oversaturated)

## Notes / open items

- Shockwave speeds are stored signed (negative = upstream); magnitudes are
  used in the trigonometric length/area formulas, marked at each use.
- The general-traffic delay area is converted length x time -> veh.s using
  jam density k_j. This is the one place to double-check against a worked
  numeric example from the paper if one becomes available.
- PT delay (Section 3.4, Cases A1-A3, B, C, D; Eqs. 13-20+) is the next
  build target and completes the objective function (Eq. 1).
