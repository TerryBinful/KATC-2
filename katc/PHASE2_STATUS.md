# Phase 2 COMPLETE - Genetic Algorithm (37/37 tests pass)

Reproduces Section 4 of Behbahani & Poorjafari (2022), verified against the
clean HTML/LaTeX source and Figures 5-7.

## New modules (go in katc\katc\)
| File | Paper | Verified specifics |
|------|-------|--------------------|
| chromosome.py | Fig. 6 | 2-bit ps + three 6-bit fc = 20 bits/cycle; 3-cycle horizon = 60 bits |
| decoder.py | Eqs. 44-48 | per-ring sum = Gamma=120s; g_min; dual-ring barrier mirror (Eq.48) |
| ga.py | Sec. 4 | pop=150, gen=150, elitism, mask crossover, adaptive mutation 0.01->0.02, fixed-gen termination, fitness=1/avg passenger delay |

## New tests (go in katc\tests\)
- test_phase2.py : 15 tests (encoding round-trip, constraint satisfaction, GA convergence, reproducibility)

## Milestone script (go in project root)
- analytical_test.py : GA + Phase-1 delay model, NO simulator. Reproduces the
  paper's "preliminary analytical test".

## Result of the analytical test (your first real KATC result)
- GA cuts avg passenger delay 45.7% below mean random signal plan (191 -> 104 s/p)
- Converges in ~25 generations, stable thereafter (matches paper behaviour)
- Runtime 2.8s for pop=150 x gen=150 (paper target ~10s on slower 2.00GHz i7)

## Verified fixed parameters (from HTML/images) for later phases
- Cycle length Gamma = 120 s (all methods)
- Decision horizon = 3 cycles; signal updated every 1 cycle
- General-traffic occupancy o_g = 1.5
- DOS sweep: 0.4,0.5,0.6,0.7,0.8,0.9,1.0 ; PT occupancy: 20,30,40,50,60 (35 scenarios)
- 10 runs/scenario, 1 h horizon, 2-cycle warm-up
- Turning splits (Fig 7a): major 70% through / 20% left / 10% right; minor similar
- Bus headways (Fig 7b): routes 1&5=310s, 2&6=210s, 3&7=580s, 4&8=460s
- Baselines: SYNCHRO (-> substitute Webster), KATC-1 (green only), KATC-2 (green+phase seq)

## Paper's actual result numbers now captured (for Phase 6 comparison)
- SYNCHRO avg passenger delay surface: 38.36-47.43 s/p
- KATC-1: 39.34-47.60 s/p ; KATC-2: 35.54-46.77 s/p
- Improvement curves (Fig 9): KATC-2 vs SYNCHRO and vs KATC-1, per DOS/occupancy

## Reconstruction note (report)
The exact mapping of the 3 coefficient factors -> 8 movements is not published.
decoder.py uses a faithful dual-ring interpretation (fc1: major/minor split;
fc2: major through/left; fc3: minor through/left), clamped to g_min and
renormalised to Gamma. Flagged as the single Phase-2 reconstruction point.
