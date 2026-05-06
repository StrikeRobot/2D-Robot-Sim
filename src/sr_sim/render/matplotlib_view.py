"""Headless GIF recorder using matplotlib — no display required.

Usage
-----
    recorder = MatplotlibRecorder(world)
    recorder.record(robot, goal, planner, world, step_fn,
                    output="assets/gifs/apf_demo.gif")
"""
from __future__ import annotations

import math
import os
from typing import Callable


class MatplotlibRecorder:
    """Renders a simulation run into an animated GIF without a live window."""

    def __init__(self, world, figsize: tuple[float, float] = (7, 7)):
        import matplotlib
        matplotlib.use("Agg")  # force non-interactive backend
        import matplotlib.patches as mpatches
        import matplotlib.pyplot as plt

        self._fig, self._ax = plt.subplots(figsize=figsize, facecolor="#1c1c1c")
        ax = self._ax
        ax.set_facecolor("#e8e8de")
        ax.set_xlim(0, world.width)
        ax.set_ylim(0, world.height)
        ax.set_aspect("equal")
        ax.tick_params(colors="#999", labelsize=8)
        for spine in ax.spines.values():
            spine.set_edgecolor("#555")

        ax.grid(True, color="#d4d4c8", linewidth=0.5, zorder=0)
        ax.set_xlabel("x (m)", color="#aaa", fontsize=8)
        ax.set_ylabel("y (m)", color="#aaa", fontsize=8)
        ax.set_title(
            "sr-sim  |  Artificial Potential Fields",
            fontsize=10, color="#ddd", pad=6,
            fontfamily="monospace",
        )

        # Border walls
        bt = world.border_thickness
        if bt > 0:
            border = mpatches.Rectangle(
                (0, 0), world.width, world.height,
                linewidth=bt * 80, edgecolor="#383834", facecolor="none", zorder=1,
            )
            ax.add_patch(border)

        # Obstacles — drawn once on the static background
        for ox, oy, r in world.obstacle_shapes:
            ax.add_patch(mpatches.Circle((ox, oy), r, color="#4a4a46", zorder=2))
            ax.add_patch(
                mpatches.Circle(
                    (ox - r * 0.28, oy + r * 0.28), r * 0.22,
                    color="#6e6e68", zorder=3, alpha=0.65,
                )
            )

        self._fig.tight_layout(pad=0.6)

    # ------------------------------------------------------------------

    def record(
        self,
        robot,
        goal: tuple[float, float],
        planner,
        world,
        step_fn: Callable,
        n_frames: int = 350,
        dt: float = 0.05,
        fps: int = 20,
        output: str = "assets/gifs/apf_demo.gif",
    ) -> None:
        """Run the simulation for *n_frames* steps and save an animated GIF."""
        from matplotlib.animation import FuncAnimation, PillowWriter

        ax = self._ax

        # Mutable trail
        trail_x: list[float] = []
        trail_y: list[float] = []

        # Dynamic artists
        (trail_line,) = ax.plot([], [], color="#6496eb", linewidth=1.5, zorder=4, alpha=0.75)
        goal_patch = ax.add_patch(
            __import__("matplotlib.patches", fromlist=["Circle"]).Circle(
                goal, 0.35, color="#37c350", zorder=5, alpha=0.9
            )
        )
        ax.plot(*goal, "w+", markersize=9, markeredgewidth=2, zorder=6)

        robot_body = ax.add_patch(
            __import__("matplotlib.patches", fromlist=["Circle"]).Circle(
                (robot.x, robot.y), robot.radius, color="#1e64dc", zorder=7
            )
        )
        (heading_line,) = ax.plot([], [], "w-", linewidth=2, zorder=8)

        def _update_heading():
            hx = robot.x + robot.radius * 1.35 * math.cos(robot.theta)
            hy = robot.y + robot.radius * 1.35 * math.sin(robot.theta)
            heading_line.set_data([robot.x, hx], [robot.y, hy])

        def update(_frame: int):
            v, omega = planner.compute(robot, goal, world)
            step_fn(robot, v, omega, dt, world)

            trail_x.append(robot.x)
            trail_y.append(robot.y)
            if len(trail_x) > 500:
                trail_x.pop(0)
                trail_y.pop(0)

            trail_line.set_data(trail_x, trail_y)
            robot_body.center = (robot.x, robot.y)
            _update_heading()
            return trail_line, robot_body, heading_line

        anim = FuncAnimation(
            self._fig, update, frames=n_frames,
            interval=1000 // fps, blit=True,
        )

        os.makedirs(os.path.dirname(output) or ".", exist_ok=True)
        print(f"Recording {n_frames} frames → {output}  (this takes ~10–30 s)")
        anim.save(output, writer=PillowWriter(fps=fps), dpi=110)
        print(f"Saved: {output}")

        import matplotlib.pyplot as plt
        plt.close(self._fig)
