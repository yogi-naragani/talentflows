"""Nonlinear MPC for a quadrotor under GPS-denied perception.

Decision variables: state x_k = [p, v, q, omega] over horizon N, controls
u_k = [thrust, body-rate setpoints] (or four motor speeds, depending on
the platform). Cost penalizes tracking error, control effort, and a
"perception health" term that softly prefers trajectories that keep
features in view and the 1D beam on a usable surface.

We use CasADi to assemble the NLP and IPOPT (or HPIPM) to solve. The
solver wrapper here is a stub so the paper repo can compile without the
full toolchain available.
"""

from dataclasses import dataclass
import numpy as np


@dataclass
class MPCConfig:
    N: int = 20
    dt: float = 0.05
    mass: float = 0.95
    g: float = 9.81
    Q_pos: float = 10.0
    Q_vel: float = 1.0
    Q_att: float = 1.0
    R_thrust: float = 0.01
    R_rates: float = 0.01
    w_perception: float = 0.5     # see PerceptionAwareCost
    thrust_min: float = 2.0
    thrust_max: float = 18.0
    # Hard inequality min clearance enforced when acoustic_confidence
    # exceeds acoustic_gate.
    min_clearance_m: float = 0.6
    acoustic_gate: float = 0.5


@dataclass
class MPCSolution:
    u0: np.ndarray
    x_traj: np.ndarray
    u_traj: np.ndarray
    feasible: bool
    solve_ms: float


class PerceptionAwareCost:
    """Trajectory penalty that grows when expected feature visibility,
    beam-on-ground geometry, or acoustic-derived obstacle proximity
    degrades. Exposed as a callable so the paper can ablate each term
    cleanly."""

    def __call__(
        self,
        x_pred: np.ndarray,
        map_points_w: np.ndarray,
        acoustic_proximity: dict | None = None,
    ) -> float:
        raise NotImplementedError


@dataclass
class AcousticConstraint:
    """Per-axis minimum clearance derived from the acoustic backup. If
    ``confidence < cfg.acoustic_gate`` the controller ignores it; above
    the gate the MPC adds a hard lower bound on body-axis distance to
    obstacle, evaluated over the prediction horizon."""
    clearance_m: dict | None = None
    confidence: float = 0.0


class QuadrotorMPC:
    def __init__(self, cfg: MPCConfig, perception_cost: PerceptionAwareCost | None = None):
        self.cfg = cfg
        self.perception_cost = perception_cost
        self._acoustic = AcousticConstraint()
        self._build_nlp()

    def set_acoustic(self, constraint: AcousticConstraint) -> None:
        self._acoustic = constraint

    def _build_nlp(self) -> None:
        # CasADi NLP construction goes here (Opti stack). Acoustic
        # constraints are added as hard inequalities when active and
        # softened to penalty terms otherwise.
        pass

    def solve(self, x0: np.ndarray, x_ref: np.ndarray) -> MPCSolution:
        raise NotImplementedError
