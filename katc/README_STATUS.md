# KATC Reproduction - Phase 1 COMPLETE

Reproduction of Behbahani & Poorjafari (2022), IET ITS 17(5), 912-928,
DOI 10.1049/itr2.12316. Solo study. Simulator: SUMO (deviation from VISSIM).

## Phase 1 status: the full analytical core is built and tested (22/22 passing)

| Module | Paper section | Equations | State |
|--------|---------------|-----------|-------|
| fundamental_diagram.py | Fig. 2, 3.3 | Eqs. 2-3 | done |
| general_delay.py | 3.3 | Eqs. 4,5,7,8,9,10,11,12 | done |
| pt_delay.py | 3.4 | Eqs. 13-42 (cases A-D) | done |
| objective.py | 3.5 | Eq. 43 | done |

All equations author-verified against the PDF (Mathpix + LaTeX-render check).

## Key modelling notes (for the report)

1. Shockwave speeds used as POSITIVE MAGNITUDES (paper convention); with
   signed speeds Eq. 11 yields a negative queue length. Documented in code.
2. d_g is a queue-profile AREA (length x time); the k_j conversion to
   vehicle-seconds lives in the objective (Eq. 43), not inside d_g. This
   matches Eq. 43's "o_g * k_j * d_g" term.
3. Eq. 11 with t_rq=0 reduces exactly to Eq. 4 (verified in tests) - a strong
   internal-consistency check on the transcription.
4. Cycle-to-cycle residual chaining: a fresh oversaturated cycle uses Eq. 7
   for its own delay and flags oversaturation so the NEXT cycle computes the
   residual via Eq. 9. Documented interpretation.
5. Absolute delay magnitudes depend on the fundamental-diagram parameters
   (v_f, k_j, q_c). Placeholder values used for testing; the paper's actual
   values will be substituted when the SUMO scenario is built, for
   comparability.

## Next: Phase 2 - genetic algorithm

The objective above is the GA fitness. Phase 2 reproduces the paper's GA
(binary encoding of green splits via 6-bit coefficient factors + 2-bit phase
sequence, pop=150, gen=150, mask crossover, adaptive mutation 0.01->0.02,
elitism) and runs the analytical-only optimisation test before SUMO.
