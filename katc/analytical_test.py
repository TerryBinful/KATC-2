"""
Preliminary analytical optimisation test (reproduces the paper's Section 4
"preliminary analytical test") -- GA + kinematic-wave delay model, NO simulator.

Wires the Phase 2 GA to the Phase 1 delay model: each candidate signal plan is
decoded to per-phase greens, the general-traffic and PT delays are computed per
lane group over the 3-cycle horizon, assembled into average passenger delay
(Eq. 43), and inverted into GA fitness. The GA then searches for the signal
plan minimising average passenger delay.

This is the first end-to-end KATC result and needs no VISSIM/SUMO.
"""
import sys, time
sys.path.insert(0, '.')

from katc.fundamental_diagram import FundamentalDiagram
from katc.general_delay import analyze_cycle
from katc.pt_delay import pt_vehicle_delay, PTCycle
from katc.objective import average_passenger_delay, LaneCycleTerm
from katc.chromosome import decode
from katc.decoder import SignalTiming, decode_cycle_to_greens
from katc.ga import run_ga, GAConfig

# --- Scenario (paper-derived where available; illustrative otherwise) ---
TIMING = SignalTiming(gamma=120.0, yellow=3.0, all_red=2.0, g_min=7.0)
FD = FundamentalDiagram(v_f=15.0, k_j=0.15, q_c=0.5)   # placeholder FD params
O_G = 1.5          # general-traffic occupancy (verified)
N_CYCLES = 3       # 3-cycle decision horizon (verified)

# Four lane groups (major-through, major-left, minor-through, minor-left) with
# representative arrival flows and one PT vehicle each per cycle at x0b=30 m.
# Phase index -> (arrival flow q_g, PT occupancy, PT initial position).
LANE_GROUPS = {
    2: dict(q_g=0.40, o_b=40, x0b=30.0, n_b=1),   # major through (busy)
    1: dict(q_g=0.12, o_b=40, x0b=30.0, n_b=1),   # major left
    4: dict(q_g=0.20, o_b=40, x0b=30.0, n_b=1),   # minor through
    3: dict(q_g=0.08, o_b=40, x0b=30.0, n_b=1),   # minor left
}


def average_delay_for_chromosome(bits):
    genes = decode(bits)
    terms = []
    for phase, lg in LANE_GROUPS.items():
        q_g = lg["q_g"]
        prev = None
        cycles = []
        for gene in genes:
            greens = decode_cycle_to_greens(gene, TIMING)
            g = greens[phase]
            r = TIMING.gamma - g            # red seen by this movement
            res = analyze_cycle(r=r, g=g, fd=FD, q_g=q_g, prev=prev)
            cycles.append(res)
            prev = res
        # PT delay over consecutive cycle pairs.
        for i in range(len(cycles)):
            c1 = PTCycle.from_result(cycles[i])
            c2 = PTCycle.from_result(cycles[i + 1] if i + 1 < len(cycles) else cycles[i])
            d_b = pt_vehicle_delay(lg["x0b"], c1, c2, FD, q_g) * lg["n_b"]
            terms.append(LaneCycleTerm(
                d_g_area=cycles[i].d_g, d_b_total=d_b, q_g=q_g, n_b=lg["n_b"]))
    return average_passenger_delay(terms, o_g=O_G, o_b=40, k_j=FD.k_j, gamma=TIMING.gamma)


if __name__ == "__main__":
    import random
    # Random baseline (no optimisation): average over random signal plans.
    from katc.chromosome import random_bits
    rng = random.Random(0)
    rand_delays = [average_delay_for_chromosome(random_bits(N_CYCLES, rng)) for _ in range(150)]
    baseline = sum(rand_delays) / len(rand_delays)
    best_random = min(rand_delays)

    # GA optimisation (paper settings: pop=150, gen=150).
    cfg = GAConfig(n_cycles=N_CYCLES, pop_size=150, n_generations=150, seed=1)
    t0 = time.time()
    res = run_ga(average_delay_for_chromosome, cfg)
    elapsed = time.time() - t0

    print("=== KATC preliminary analytical test ===")
    print(f"Random signal plans:  mean delay = {baseline:6.2f} s/p   best = {best_random:6.2f} s/p")
    print(f"GA-optimised plan:     best delay = {res.best_delay:6.2f} s/p")
    print(f"Improvement over mean random: {100*(baseline-res.best_delay)/baseline:5.1f}%")
    print(f"GA runtime: {elapsed:.1f} s  (pop={cfg.pop_size}, gen={cfg.n_generations})")
    print(f"Convergence (best delay every 25 gens): "
          + ", ".join(f"{res.history[i]:.1f}" for i in range(0, len(res.history), 25)))
