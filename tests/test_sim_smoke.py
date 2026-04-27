"""Smoke test: a 5 s rule_tree trial in an empty scenario must complete
without crashing and reach the single waypoint."""

from experiments.sim.runner import TrialConfig, run_trial


def test_empty_scenario_rule_tree_reaches_waypoint():
    m = run_trial(TrialConfig(
        scenario="empty", advisor="rule_tree", seed=0,
        duration_s=10.0, control_hz=100.0, advisor_hz=2.0,
    ))
    assert m.collisions == 0
    assert m.waypoints_reached >= 1
    assert m.completed
    assert m.advisor_ticks > 5


def test_corridor_does_not_crash():
    """White-wall corridor: with the rule-tree advisor + acoustic backup,
    we expect zero collisions even though SLAM textures vanish at x~15.
    The advisor should slow down and the acoustic backup should keep
    clearance > 0."""
    m = run_trial(TrialConfig(
        scenario="corridor_white_wall", advisor="rule_tree", seed=0,
        duration_s=20.0, control_hz=100.0, advisor_hz=2.0,
    ))
    assert m.collisions == 0
    assert m.min_clearance_m > 0.0


def test_slm_offline_fallback_runs():
    """The GeminiNanoClient with backend=None returns a conservative
    hover action; the loop must still close and not crash."""
    m = run_trial(TrialConfig(
        scenario="empty", advisor="slm", seed=1,
        duration_s=5.0, control_hz=100.0, advisor_hz=2.0,
    ))
    assert m.collisions == 0
    assert m.advisor_ticks > 5
