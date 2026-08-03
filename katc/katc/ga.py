"""
Genetic algorithm (Section 4) -- author-verified settings.

Reproduces the KATC optimiser:
  * binary chromosome (chromosome.py, Fig. 6): 20 bits/cycle x 3 cycles = 60
  * fitness = INVERSE of average passenger delay (GA maximises; delay is
    minimised) -- Eq. 43 supplied via a caller-provided delay function
  * ELITISM: best individual of each generation carried unchanged
  * MASK-BASED CROSSOVER (uniform crossover with a random bit mask)
  * ADAPTIVE MUTATION: probability rises linearly 0.01 -> 0.02 across
    generations (diversity boost in late generations)
  * termination: FIXED number of generations (no convergence test)
  * tuned settings: population = 150, generations = 150

The delay function is injected so the GA stays independent of whether delay
comes from the analytical model (Phase 2 test) or the SUMO loop (Phase 3).
"""

from __future__ import annotations
from dataclasses import dataclass, field
from typing import Callable, List, Optional
import random

from .chromosome import BITS_PER_CYCLE, random_bits

# Verified defaults.
POP_SIZE = 150
N_GENERATIONS = 150
MUT_START = 0.01
MUT_END = 0.02


@dataclass
class GAConfig:
    n_cycles: int = 3
    pop_size: int = POP_SIZE
    n_generations: int = N_GENERATIONS
    mut_start: float = MUT_START
    mut_end: float = MUT_END
    elitism: int = 1
    seed: Optional[int] = None


@dataclass
class GAResult:
    best_bits: List[int]
    best_fitness: float
    best_delay: float
    history: List[float] = field(default_factory=list)   # best delay per gen


def _mask_crossover(a: List[int], b: List[int], rng: random.Random):
    """Uniform (mask-based) crossover: each bit taken from a or b by a mask."""
    child1, child2 = [], []
    for x, y in zip(a, b):
        if rng.random() < 0.5:
            child1.append(x); child2.append(y)
        else:
            child1.append(y); child2.append(x)
    return child1, child2


def _mutate(bits: List[int], p: float, rng: random.Random) -> List[int]:
    """Bit-flip mutation with per-bit probability p."""
    return [(1 - b) if rng.random() < p else b for b in bits]


def _tournament(pop, fits, rng, k=3):
    """Tournament selection: pick the best of k random individuals."""
    best_i = rng.randrange(len(pop))
    for _ in range(k - 1):
        j = rng.randrange(len(pop))
        if fits[j] > fits[best_i]:
            best_i = j
    return pop[best_i]


def run_ga(
    delay_of: Callable[[List[int]], float],
    config: GAConfig | None = None,
) -> GAResult:
    """Run the GA.

    Parameters
    ----------
    delay_of : callable(bits) -> float
        Returns the average passenger delay for a chromosome (Eq. 43).
        Lower is better; the GA maximises 1/delay.
    config : GAConfig
    """
    cfg = config or GAConfig()
    rng = random.Random(cfg.seed)

    def fitness(bits: List[int]) -> tuple[float, float]:
        d = delay_of(bits)
        f = 1.0 / d if d > 0 else 0.0
        return f, d

    # Initial population.
    pop = [random_bits(cfg.n_cycles, rng) for _ in range(cfg.pop_size)]
    scored = [fitness(ind) for ind in pop]
    fits = [s[0] for s in scored]
    delays = [s[1] for s in scored]

    best_i = max(range(len(pop)), key=lambda i: fits[i])
    best = GAResult(best_bits=pop[best_i][:], best_fitness=fits[best_i],
                    best_delay=delays[best_i], history=[delays[best_i]])

    for gen in range(cfg.n_generations):
        # Adaptive mutation probability (linear 0.01 -> 0.02).
        if cfg.n_generations > 1:
            p_mut = cfg.mut_start + (cfg.mut_end - cfg.mut_start) * gen / (cfg.n_generations - 1)
        else:
            p_mut = cfg.mut_start

        # Elitism: keep the top individuals.
        order = sorted(range(len(pop)), key=lambda i: fits[i], reverse=True)
        new_pop = [pop[order[e]][:] for e in range(cfg.elitism)]

        # Fill the rest by selection + crossover + mutation.
        while len(new_pop) < cfg.pop_size:
            p1 = _tournament(pop, fits, rng)
            p2 = _tournament(pop, fits, rng)
            c1, c2 = _mask_crossover(p1, p2, rng)
            c1 = _mutate(c1, p_mut, rng)
            c2 = _mutate(c2, p_mut, rng)
            new_pop.append(c1)
            if len(new_pop) < cfg.pop_size:
                new_pop.append(c2)

        pop = new_pop
        scored = [fitness(ind) for ind in pop]
        fits = [s[0] for s in scored]
        delays = [s[1] for s in scored]

        gi = max(range(len(pop)), key=lambda i: fits[i])
        if fits[gi] > best.best_fitness:
            best.best_bits = pop[gi][:]
            best.best_fitness = fits[gi]
            best.best_delay = delays[gi]
        best.history.append(best.best_delay)

    return best
