"""
Map KATC's 8 NEMA movement greens + phase sequence onto SUMO phase-state
strings, honouring the dual-ring barrier and lead/lag phase sequence.

SUMO signals are a linkIndex-ordered string of per-connection states
('G' protected green, 'g' permissive green, 'y' yellow, 'r' red). The link
order is fixed by netconvert; we read it once at runtime (via
traci.trafficlight.getControlledLinks) and build a movement->linkIndex map.
This module contains the pure logic (no TraCI dependency) so it is unit
testable: given a movement->linkIndex map and a per-NEMA-phase green plan, it
emits the ordered list of (state_string, duration) phases for one cycle.

NEMA dual-ring (MOVEMENTS.md):
  Ring A: 1(maj-left) 2(maj-thru) | 3(min-left) 4(min-thru)
  Ring B: 5(maj-left) 6(maj-thru) | 7(min-left) 8(min-thru)
  Barrier splits major block {1,2,5,6} from minor block {3,4,7,8}.
  Eq. 48 ties phase l with l+4 in duration.

Phase sequence ps in {1,2,3,4} = {lag-lag, lead-lag, lag-lead, lead-lead}:
  first token = major side left LEADS (lead) or LAGS (lag) its through;
  second token = minor side left leads/lags. "lead" => left phase before
  through; "lag" => left phase after through.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, List, Tuple

# Which NEMA movements are "left" and their paired "through".
MAJOR_LEFTS = [1, 5]
MAJOR_THRUS = [2, 6]
MINOR_LEFTS = [3, 7]
MINOR_THRUS = [4, 8]

PS_DECODE = {
    1: ("lag", "lag"),
    2: ("lead", "lag"),
    3: ("lag", "lead"),
    4: ("lead", "lead"),
}


@dataclass
class NemaPhase:
    """One signal stage: the set of NEMA movements that are green, + duration."""
    green_movements: List[int]
    duration: float
    kind: str = "green"   # "green" or "yellow" or "allred"


def build_cycle_phases(
    greens: Dict[int, float],   # NEMA phase -> green time [s]
    ps: int,
    yellow: float,
    all_red: float,
) -> List[NemaPhase]:
    """Build the ordered NEMA stages for one cycle under phase sequence ps.

    Structure: major block (left+through, order by lead/lag) -> barrier
    (yellow+allred) -> minor block (left+through, order by lead/lag) -> barrier.
    Concurrent movements: the two majors (dirs) run together; within a side the
    left pair {1,5} and through pair {2,6} are separate stages ordered by ps.
    """
    major_order, minor_order = PS_DECODE[ps]
    stages: List[NemaPhase] = []

    def add_block(lefts, thrus, order, left_g, thru_g):
        left_stage = NemaPhase(green_movements=list(lefts), duration=left_g)
        thru_stage = NemaPhase(green_movements=list(thrus), duration=thru_g)
        if order == "lead":
            seq = [left_stage, thru_stage]
        else:  # lag: through first, then left
            seq = [thru_stage, left_stage]
        for i, st in enumerate(seq):
            stages.append(st)
            # inter-stage yellow+allred
            stages.append(NemaPhase(green_movements=st.green_movements,
                                    duration=yellow, kind="yellow"))
            stages.append(NemaPhase(green_movements=[], duration=all_red,
                                    kind="allred"))

    # Major block: left green = green of movement 1 (== 5 by Eq.48), etc.
    add_block(MAJOR_LEFTS, MAJOR_THRUS, major_order,
              greens[1], greens[2])
    # Minor block.
    add_block(MINOR_LEFTS, MINOR_THRUS, minor_order,
              greens[3], greens[4])
    return stages


def stage_to_state(green_movements: List[int], kind: str,
                   movement_links: Dict[int, List[int]],
                   n_links: int,
                   permissive_links: List[int] | None = None) -> str:
    """Render one stage as a SUMO state string of length n_links.

    movement_links maps a NEMA phase -> list of linkIndices it controls.
    permissive_links are right-turn (or other non-signalised) links that run
    permissively: green ('g') whenever the stage is not all-red, so the
    rightmost shared through+right lanes are never blocked by a right-turner
    sitting on a red arrow. Right-turners still yield to conflicting traffic
    because 'g' (lower-case) is permissive in SUMO.
    """
    state = ["r"] * n_links
    permissive_links = permissive_links or []
    if kind == "allred":
        return "".join(state)   # everything red between stages
    # Permissive right turns get lower-case 'g' in every non-all-red stage.
    for li in permissive_links:
        if 0 <= li < n_links:
            state[li] = "g"
    ch = "G" if kind == "green" else "y"
    for mv in green_movements:
        for li in movement_links.get(mv, []):
            state[li] = ch
    return "".join(state)


def build_state_program(
    greens: Dict[int, float],
    ps: int,
    yellow: float,
    all_red: float,
    movement_links: Dict[int, List[int]],
    n_links: int,
    permissive_links: List[int] | None = None,
) -> List[Tuple[str, float]]:
    """Full one-cycle SUMO program: list of (state_string, duration)."""
    phases = build_cycle_phases(greens, ps, yellow, all_red)
    prog = []
    for ph in phases:
        state = stage_to_state(ph.green_movements, ph.kind, movement_links,
                               n_links, permissive_links)
        prog.append((state, ph.duration))
    return prog
