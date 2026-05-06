from __future__ import annotations

import math

import numpy as np

from ..core.robot import Robot
from ..core.world import World


class APFPlanner:
    """Artificial Potential Field planner for differential-drive robots.

    Uses a conic attractive field (constant magnitude when far, quadratic when
    near the goal) and a nearest-obstacle repulsive field to avoid blowup when
    summing over many grid cells.
    """

    def __init__(
        self,
        k_att: float = 1.5,
        k_rep: float = 1.2,
        d_rep: float = 1.5,
        d_min: float = 0.15,
        v_max: float = 1.5,
        k_turn: float = 3.0,
        goal_threshold: float = 0.4,
    ):
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_rep = d_rep
        self.d_min = d_min
        self.v_max = v_max
        self.k_turn = k_turn
        self.goal_threshold = goal_threshold

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def compute(
        self, robot: Robot, goal: tuple[float, float], world: World
    ) -> tuple[float, float]:
        """Return (v, omega) velocity command for one simulation step."""
        px, py = robot.x, robot.y
        gx, gy = goal

        dx_g = gx - px
        dy_g = gy - py
        d_goal = math.hypot(dx_g, dy_g)

        if d_goal < self.goal_threshold:
            return 0.0, 0.0

        # Attractive: conic far, quadratic near (smooth transition)
        if d_goal > 1.0:
            fx_att = self.k_att * dx_g / d_goal
            fy_att = self.k_att * dy_g / d_goal
        else:
            fx_att = self.k_att * dx_g
            fy_att = self.k_att * dy_g

        fx_rep, fy_rep = self._repulsive_gradient(px, py, world)

        fx = fx_att + fx_rep
        fy = fy_att + fy_rep

        desired_theta = math.atan2(fy, fx)
        angle_err = math.atan2(
            math.sin(desired_theta - robot.theta),
            math.cos(desired_theta - robot.theta),
        )

        f_mag = math.hypot(fx, fy)
        v = self.v_max * max(0.0, math.cos(angle_err)) * min(1.0, f_mag)
        v = min(v, robot.max_v)

        omega = self.k_turn * angle_err
        omega = max(-robot.max_omega, min(robot.max_omega, omega))

        return v, omega

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _repulsive_gradient(
        self, px: float, py: float, world: World
    ) -> tuple[float, float]:
        """Force from the single nearest obstacle cell within d_rep."""
        row0, col0 = world.world_to_grid(px, py)
        pad = int(self.d_rep / world.cell_size) + 1
        r0 = max(0, row0 - pad)
        r1 = min(world.rows, row0 + pad + 1)
        c0 = max(0, col0 - pad)
        c1 = min(world.cols, col0 + pad + 1)

        subgrid = world.grid[r0:r1, c0:c1]
        R, C = np.mgrid[r0:r1, c0:c1]
        ox = (C + 0.5) * world.cell_size
        oy = (R + 0.5) * world.cell_size

        ddx = px - ox
        ddy = py - oy
        d = np.sqrt(ddx ** 2 + ddy ** 2)

        # Restrict to obstacle cells within influence radius
        inf_fill = np.where((subgrid == 1) & (d > 0) & (d < self.d_rep), d, np.inf)
        if np.all(np.isinf(inf_fill)):
            return 0.0, 0.0

        idx = np.unravel_index(np.argmin(inf_fill), inf_fill.shape)
        d_near = float(d[idx])
        d_clamped = max(d_near, self.d_min)

        dx_r = float(ddx[idx])
        dy_r = float(ddy[idx])
        d_r = max(d_near, 1e-6)

        mag = self.k_rep * (1.0 / d_clamped - 1.0 / self.d_rep) / d_clamped
        return mag * dx_r / d_r, mag * dy_r / d_r
