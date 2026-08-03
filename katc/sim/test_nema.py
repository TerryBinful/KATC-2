"""Tests for the NEMA->SUMO mapping logic (no TraCI needed)."""
import sys; sys.path.insert(0, '.')
from nema_to_sumo import (build_cycle_phases, build_state_program, PS_DECODE)

def run():
    greens = {1:15, 2:40, 3:12, 4:25, 5:15, 6:40, 7:12, 8:25}
    passed = failed = 0
    def check(name, cond):
        nonlocal passed, failed
        if cond: passed += 1
        else: failed += 1; print("  FAIL:", name)

    # 1. Four phase-sequence values decode to lead/lag pairs.
    check("ps decode complete", set(PS_DECODE.keys()) == {1,2,3,4})

    # 2. Lead vs lag changes stage ORDER but not total green.
    lead = build_cycle_phases(greens, ps=4, yellow=3, all_red=2)  # lead-lead
    lag  = build_cycle_phases(greens, ps=1, yellow=3, all_red=2)  # lag-lag
    green_lead = [p for p in lead if p.kind=="green"]
    green_lag  = [p for p in lag  if p.kind=="green"]
    check("same number of green stages", len(green_lead)==len(green_lag)==4)
    check("total green invariant to ps",
          sum(p.duration for p in green_lead)==sum(p.duration for p in green_lag))

    # 3. Lead puts the LEFT stage before the through on the major side.
    #    First green stage under lead-lead should be the major lefts {1,5}.
    first_green = next(p for p in lead if p.kind=="green")
    check("lead => left first (major)", set(first_green.green_movements)=={1,5})
    #    Under lag-lag the first major green stage should be the throughs {2,6}.
    first_green_lag = next(p for p in lag if p.kind=="green")
    check("lag => through first (major)", set(first_green_lag.green_movements)=={2,6})

    # 4. Cycle duration sums to a sensible total (4 greens + yellows + allreds).
    total = sum(p.duration for p in lead)
    # 4 green stages + 4 yellow + 4 allred
    expected = (15+40+12+25) + 4*3 + 4*2
    check("cycle total duration", abs(total-expected) < 1e-6)

    # 5. State program renders strings of correct length with a fake link map.
    mv_links = {m:[m-1] for m in range(1,9)}  # movement m -> link index m-1
    n_links = 8
    prog = build_state_program(greens, ps=2, yellow=3, all_red=2,
                               movement_links=mv_links, n_links=n_links)
    check("all states length n_links", all(len(s)==n_links for s,_ in prog))
    check("green stages have G", any("G" in s for s,_ in prog))
    check("yellow stages have y", any("y" in s for s,_ in prog))
    # all-red stage exists (all 'r')
    check("allred stage present", any(set(s)=={"r"} for s,_ in prog))

    print(f"NEMA mapping: PASSED {passed}  FAILED {failed}")
    return failed==0

if __name__ == "__main__":
    ok = run()
    sys.exit(0 if ok else 1)
