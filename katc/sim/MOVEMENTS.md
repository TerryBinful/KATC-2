# NEMA movement map for the KATC SUMO intersection (paper Fig. 7 geometry)

Major street = East-West (3 lanes: 2 through/right + 1 exclusive left).
Minor street = North-South (2 lanes: 1 through/right + 1 exclusive left).
SUMO lane 0 = rightmost. Exclusive LEFT lane = highest index (major lane 2,
minor lane 1). Left and through queues develop independently, as the paper
requires ("The exclusive left-turn lanes ... so left-turn and through queues
develop independently").

| NEMA phase | Movement        | From edge | Lane | To edge |
|-----------|-----------------|-----------|------|---------|
| 1  | Major EB left   | E_C       | 2    | C_N     |
| 2  | Major WB through| W_C       | 0    | C_E     |
| 3  | Minor SB left   | S_C       | 1    | C_W     |
| 4  | Minor NB through| N_C       | 0    | C_S     |
| 5  | Major WB left   | W_C       | 2    | C_S     |
| 6  | Major EB through| E_C       | 0    | C_W     |
| 7  | Minor NB left   | N_C       | 1    | C_E     |
| 8  | Minor SB through| S_C       | 0    | C_N     |

Dual-ring barrier structure:
  Ring A: phase 1 (major left) + phase 2 (major through) || phase 3 (minor left) + phase 4 (minor through)
  Ring B: phase 5 (major left) + phase 6 (major through) || phase 7 (minor left) + phase 8 (minor through)
  Barrier separates the major-street block (1,2,5,6) from the minor block (3,4,7,8).

Eq. 48 (p_l = p_{l+4}) ties phase l with l+4 in duration.

Right turns run permissively with the parallel through (not separately
signalised), consistent with the paper's through+left phasing focus.

Phase sequence ps in {1,2,3,4} = {lag-lag, lead-lag, lag-lead, lead-lead}
controls whether the left phase LEADS or LAGS the through on each barrier side.
