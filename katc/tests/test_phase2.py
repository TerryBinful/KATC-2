"""
Phase 2 unit tests: chromosome encoding, decoder (Eqs. 44-48), GA operators,
and the analytical optimisation milestone (GA reduces passenger delay).
"""

import random
import pytest

from katc.chromosome import (
    decode, encode, random_bits, CycleGene,
    BITS_PER_CYCLE, FC_MAX,
)
from katc.decoder import SignalTiming, decode_cycle_to_greens, ring_sums_ok
from katc.ga import run_ga, GAConfig


# ------------------------------------------------------------- chromosome
class TestChromosome:
    def test_bits_per_cycle(self):
        assert BITS_PER_CYCLE == 20

    def test_length_three_cycles(self):
        assert len(random_bits(3, random.Random(0))) == 60

    def test_decode_encode_roundtrip(self):
        rng = random.Random(1)
        bits = random_bits(3, rng)
        assert encode(decode(bits)) == bits

    def test_ps_in_range(self):
        for _ in range(50):
            genes = decode(random_bits(3, random.Random()))
            for g in genes:
                assert g.ps in (1, 2, 3, 4)

    def test_fc_in_range(self):
        for _ in range(50):
            genes = decode(random_bits(3, random.Random()))
            for g in genes:
                for f in g.fc:
                    assert 0 <= f <= FC_MAX

    def test_bad_length_rejected(self):
        with pytest.raises(ValueError):
            decode([0, 1, 0])   # not a multiple of 20

    def test_fractions_bounds(self):
        g = CycleGene(ps=1, fc=[0, 63, 32])
        fr = g.fractions()
        assert fr[0] == pytest.approx(0.0)
        assert fr[1] == pytest.approx(1.0)
        assert 0 < fr[2] < 1


# ---------------------------------------------------------------- decoder
class TestDecoder:
    def test_all_phases_present(self):
        t = SignalTiming()
        greens = decode_cycle_to_greens(CycleGene(ps=1, fc=[32, 32, 32]), t)
        assert set(greens.keys()) == set(range(1, 9))

    def test_min_green_respected(self):
        t = SignalTiming(g_min=7.0)
        # Extreme factors that would starve a movement:
        for fc in ([0, 0, 0], [63, 63, 63], [63, 0, 63], [0, 63, 0]):
            greens = decode_cycle_to_greens(CycleGene(ps=1, fc=fc), t)
            for phase, g in greens.items():
                assert g >= t.g_min - 1e-6, f"phase {phase} green {g} < g_min"

    def test_ring_sums_to_cycle_length(self):
        t = SignalTiming(gamma=120.0)
        for fc in ([32, 32, 32], [10, 50, 40], [55, 20, 30]):
            greens = decode_cycle_to_greens(CycleGene(ps=2, fc=fc), t)
            assert ring_sums_ok(greens, t)

    def test_barrier_mirror_eq48(self):
        """Eq. 48: ring B phase lengths mirror ring A."""
        t = SignalTiming()
        greens = decode_cycle_to_greens(CycleGene(ps=1, fc=[40, 30, 20]), t)
        assert greens[2] == pytest.approx(greens[6])   # major through
        assert greens[1] == pytest.approx(greens[5])   # major left
        assert greens[4] == pytest.approx(greens[8])   # minor through
        assert greens[3] == pytest.approx(greens[7])   # minor left


# --------------------------------------------------------------------- GA
class TestGA:
    def test_ga_reduces_a_simple_objective(self):
        """GA should minimise a delay function of the chromosome bits.

        Toy objective: delay is smallest when all coefficient factors are
        mid-range (fc ~ 32) -- a smooth target the GA should approach.
        """
        target = 32

        def delay_of(bits):
            genes = decode(bits)
            err = 0.0
            for g in genes:
                for f in g.fc:
                    err += (f - target) ** 2
            return 1.0 + err   # >0, minimised as fc -> 32

        cfg = GAConfig(n_cycles=3, pop_size=40, n_generations=40, seed=7)
        res = run_ga(delay_of, cfg)
        # Final best delay should be much smaller than a random baseline.
        rng = random.Random(123)
        baseline = min(delay_of(random_bits(3, rng)) for _ in range(40))
        assert res.best_delay < baseline

    def test_history_is_monotone_nonincreasing(self):
        """Best-so-far delay never gets worse (elitism guarantee)."""
        def delay_of(bits):
            return 1.0 + sum(bits)   # minimised by all-zero bits
        cfg = GAConfig(n_cycles=3, pop_size=30, n_generations=30, seed=1)
        res = run_ga(delay_of, cfg)
        for a, b in zip(res.history, res.history[1:]):
            assert b <= a + 1e-9

    def test_adaptive_mutation_reaches_endpoints(self):
        """Sanity: with 1 generation, mutation prob is the start value."""
        calls = {"n": 0}
        def delay_of(bits):
            calls["n"] += 1
            return 1.0 + sum(bits)
        cfg = GAConfig(n_cycles=1, pop_size=10, n_generations=1, seed=0)
        res = run_ga(delay_of, cfg)
        assert res.best_delay >= 1.0
        assert calls["n"] > 0

    def test_reproducible_with_seed(self):
        def delay_of(bits):
            return 1.0 + sum((i * b) for i, b in enumerate(bits))
        cfg = GAConfig(n_cycles=3, pop_size=20, n_generations=20, seed=42)
        r1 = run_ga(delay_of, cfg)
        r2 = run_ga(delay_of, cfg)
        assert r1.best_delay == r2.best_delay
        assert r1.best_bits == r2.best_bits
