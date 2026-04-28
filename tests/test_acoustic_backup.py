"""Acoustic-backup ablation: with the same advisor and seed, the
trial with acoustic enabled must avoid collision in the wall-ahead
scenario; without it, the drone collides. This is the headline
empirical claim of the paper-acoustic-backup branch and we lock it
in here.
"""

from experiments.sim.runner import TrialConfig, run_trial


SCENARIOS = (
    "corridor_white_wall",
    "texture_mask_burst",
    "illumination_drop",
    "dual_failure_slam_and_range",
)


def _trial(scenario: str, use_acoustic: bool, seed: int = 0):
    return run_trial(TrialConfig(
        scenario=scenario, advisor="rule_tree", seed=seed,
        duration_s=30.0, control_hz=100.0, advisor_hz=1.0,
        use_acoustic=use_acoustic,
    ))


def test_acoustic_on_avoids_collision_across_scenarios():
    for s in SCENARIOS:
        m = _trial(s, use_acoustic=True)
        assert m.collisions == 0, f"collision with acoustic on in {s}"
        assert m.min_clearance_m >= 0.5, (
            f"min_clearance {m.min_clearance_m:.2f} < 0.5 in {s}")


def test_acoustic_off_collides_in_corridor_white_wall():
    m = _trial("corridor_white_wall", use_acoustic=False)
    assert m.collisions == 1, "expected collision without acoustic backup"
    assert m.min_clearance_m < 0.5


def test_pairwise_acoustic_strictly_helps():
    """Across all four scenarios, acoustic on must be at least as
    safe (no more collisions, no smaller clearance) as acoustic off
    on the same seed."""
    for s in SCENARIOS:
        on  = _trial(s, use_acoustic=True,  seed=42)
        off = _trial(s, use_acoustic=False, seed=42)
        assert on.collisions <= off.collisions, (
            f"{s}: acoustic on collided more ({on.collisions} > {off.collisions})")
        assert on.min_clearance_m >= off.min_clearance_m - 1e-9, (
            f"{s}: acoustic on cleared less than off")
