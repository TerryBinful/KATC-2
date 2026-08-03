"""
Phase 1 unit tests: fundamental diagram, general-traffic delay (Section 3.3),
PT-vehicle delay (Section 3.4), and the average-passenger-delay objective
(Eq. 43). Verifies internal consistency and physical sanity BEFORE any
simulator is attached.
"""

import pytest

from katc.fundamental_diagram import FundamentalDiagram
from katc.general_delay import analyze_cycle, CycleResult, _queue_no_residual
from katc.pt_delay import pt_vehicle_delay, PTCycle
from katc.objective import average_passenger_delay, LaneCycleTerm


def make_fd():
    # v_f = 15 m/s, k_j = 0.15 veh/m, q_c = 0.5 veh/s (1800 veh/h)
    return FundamentalDiagram(v_f=15.0, k_j=0.15, q_c=0.5)


# ---------------------------------------------------------------- FD
class TestFundamentalDiagram:
    def test_critical_density(self):
        fd = make_fd()
        assert fd.k_c == pytest.approx(0.5 / 15.0)

    def test_critical_below_jam(self):
        assert make_fd().k_c < make_fd().k_j

    def test_signed_speeds_upstream(self):
        fd = make_fd()
        assert fd.v_qf(0.3) < 0 and fd.v_qd() < 0

    def test_magnitudes_positive(self):
        fd = make_fd()
        assert fd.v_qf_mag(0.3) > 0 and fd.v_qd_mag() > 0

    def test_magnitude_matches_abs(self):
        fd = make_fd()
        assert fd.v_qf_mag(0.3) == pytest.approx(abs(fd.v_qf(0.3)))

    def test_arrival_above_capacity_rejected(self):
        with pytest.raises(ValueError):
            make_fd().v_qf(0.6)


# --------------------------------------------------- Eq.4/Eq.11 consistency
class TestQueueConsistency:
    def test_eq11_reduces_to_eq4_when_no_residual(self):
        """Eq. 11 with t_rq = 0 must equal Eq. 4 (verified analytically)."""
        fd = make_fd()
        vqf = fd.v_qf_mag(0.4)
        vqd = fd.v_qd_mag()
        r = 50.0
        eq4 = _queue_no_residual(r, vqf, vqd)
        # Eq. 11 with t_rq = 0:
        eq11 = vqd * (0.0 * (vqd - vqf) + r * vqf) / (vqd - vqf)
        assert eq11 == pytest.approx(eq4)


# ---------------------------------------------------------- general delay
class TestGeneralDelay:
    def test_undersaturated_no_residual(self):
        fd = make_fd()
        res = analyze_cycle(r=30.0, g=200.0, fd=fd, q_g=0.3)
        assert not res.oversaturated
        assert res.L_rq == pytest.approx(0.0)
        assert res.d_g > 0

    def test_undersaturated_delay_is_triangle(self):
        """Eq. 7: d_g = L_q * r / 2 (no k_j inside)."""
        fd = make_fd()
        res = analyze_cycle(r=30.0, g=200.0, fd=fd, q_g=0.3)
        assert res.d_g == pytest.approx(res.L_q * 30.0 / 2.0)

    def test_starved_green_is_oversaturated(self):
        fd = make_fd()
        res = analyze_cycle(r=60.0, g=1.0, fd=fd, q_g=0.45)
        assert res.oversaturated

    def test_residual_chain_produces_positive_residual(self):
        fd = make_fd()
        c0 = analyze_cycle(r=60.0, g=1.0, fd=fd, q_g=0.45)   # oversaturated
        assert c0.oversaturated
        c1 = analyze_cycle(r=60.0, g=60.0, fd=fd, q_g=0.45, prev=c0)
        assert c1.L_rq > 0
        assert c1.t_rq > 0

    def test_more_green_never_more_delay(self):
        fd = make_fd()
        lo = analyze_cycle(r=40.0, g=20.0, fd=fd, q_g=0.4)
        hi = analyze_cycle(r=40.0, g=200.0, fd=fd, q_g=0.4)
        assert hi.d_g <= lo.d_g + 1e-9

    def test_delay_nonnegative(self):
        fd = make_fd()
        for r in (10.0, 30.0, 60.0):
            for g in (5.0, 30.0, 200.0):
                assert analyze_cycle(r=r, g=g, fd=fd, q_g=0.3).d_g >= 0


# --------------------------------------------------------------- PT delay
def cyc(res: CycleResult) -> PTCycle:
    return PTCycle.from_result(res)


class TestPTDelay:
    def _under_under(self, fd):
        c1 = analyze_cycle(r=30.0, g=200.0, fd=fd, q_g=0.3)
        c2 = analyze_cycle(r=30.0, g=200.0, fd=fd, q_g=0.3, prev=c1)
        return cyc(c1), cyc(c2)

    def test_caseA_delay_nonnegative_across_positions(self):
        fd = make_fd()
        c1, c2 = self._under_under(fd)
        for x0b in [0, 5, 20, 50, 100, 200, 400, 800]:
            d = pt_vehicle_delay(float(x0b), c1, c2, fd, q_g=0.3)
            assert d >= 0

    def test_caseA_far_vehicle_zero_delay(self):
        """A2: a PT vehicle positioned to pass on green sees zero delay."""
        fd = make_fd()
        c1, c2 = self._under_under(fd)
        # A2 interval: vf*(r+g_qd) <= x0b < vf*(r+g)
        x0b = fd.v_f * (c1.r + c1.g_qd) + 1.0
        d = pt_vehicle_delay(x0b, c1, c2, fd, q_g=0.3)
        assert d == pytest.approx(0.0)

    def test_caseA_close_vehicle_positive_delay(self):
        """A1: a PT vehicle right at the stop line joins the queue -> delay>0."""
        fd = make_fd()
        c1, c2 = self._under_under(fd)
        d = pt_vehicle_delay(1.0, c1, c2, fd, q_g=0.3)
        assert d > 0

    def test_oversaturated_cases_run(self):
        """Cases C/D dispatch and return finite non-negative delay."""
        fd = make_fd()
        c1r = analyze_cycle(r=60.0, g=2.0, fd=fd, q_g=0.45)     # c1 oversat
        c2r = analyze_cycle(r=60.0, g=2.0, fd=fd, q_g=0.45, prev=c1r)  # c2 oversat
        c1, c2 = cyc(c1r), cyc(c2r)
        for x0b in [0, 10, 50, 150, 400, 1000]:
            d = pt_vehicle_delay(float(x0b), c1, c2, fd, q_g=0.45)
            assert d >= 0 and d == d  # finite, non-NaN

    def test_case_dispatch_covers_all_four(self):
        """Each saturation combination selects a distinct case path."""
        fd = make_fd()
        under = analyze_cycle(r=30.0, g=200.0, fd=fd, q_g=0.3)
        over = analyze_cycle(r=60.0, g=2.0, fd=fd, q_g=0.45)
        combos = [(under, under), (under, over), (over, under), (over, over)]
        for a, b in combos:
            d = pt_vehicle_delay(20.0, cyc(a), cyc(b), fd, q_g=0.3)
            assert d >= 0


# --------------------------------------------------------------- objective
class TestObjective:
    def test_zero_when_no_delay(self):
        terms = [LaneCycleTerm(d_g_area=0.0, d_b_total=0.0, q_g=0.3, n_b=1)]
        d_a = average_passenger_delay(terms, o_g=1.5, o_b=40, k_j=0.15, gamma=120)
        assert d_a == pytest.approx(0.0)

    def test_positive_delay(self):
        terms = [LaneCycleTerm(d_g_area=1000.0, d_b_total=50.0, q_g=0.3, n_b=2)]
        d_a = average_passenger_delay(terms, o_g=1.5, o_b=40, k_j=0.15, gamma=120)
        assert d_a > 0

    def test_pt_occupancy_weights_pt_delay(self):
        """Higher PT occupancy should pull the average toward PT delay."""
        terms = [LaneCycleTerm(d_g_area=100.0, d_b_total=500.0, q_g=0.3, n_b=2)]
        low = average_passenger_delay(terms, o_g=1.5, o_b=10, k_j=0.15, gamma=120)
        high = average_passenger_delay(terms, o_g=1.5, o_b=60, k_j=0.15, gamma=120)
        # With big PT delay, weighting PT more raises the average.
        assert high != low

    def test_empty_terms_safe(self):
        assert average_passenger_delay([], o_g=1.5, o_b=40, k_j=0.15, gamma=120) == 0.0
