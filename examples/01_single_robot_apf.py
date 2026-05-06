"""Example 1 — Single robot navigating with Artificial Potential Fields.

The robot starts at the bottom-left and navigates to the top-right through
a field of circular obstacles, replanning in real time as you add more.

Controls
--------
Left-click   place a new circular obstacle
Right-click  move the goal
R            reset robot to start position
ESC / Q      quit
"""

import math

from sr_sim.core.physics import step
from sr_sim.core.robot import Robot
from sr_sim.core.world import World
from sr_sim.planners.potential_field import APFPlanner
from sr_sim.render.pygame_view import PygameView

# ── tunables ──────────────────────────────────────────────────────────
START_XY = (1.5, 1.5)
START_THETA = math.pi / 4
GOAL_XY = [18.5, 18.5]
DT = 0.05
GOAL_REACH_RADIUS = 0.4
OBSTACLE_RADIUS = 0.3
MAX_TRAIL = 800
# ──────────────────────────────────────────────────────────────────────


def build_world() -> World:
    world = World(width=20.0, height=20.0, cell_size=0.1)
    world.add_border(thickness=0.3)
    for ox, oy, r in [
        (6.0, 5.0, 0.55),
        (10.0, 10.0, 0.80),
        (14.0, 15.0, 0.50),
        (4.0, 14.0, 0.45),
        (16.0, 7.0, 0.60),
        (8.0, 12.0, 0.35),
        (12.0, 3.5, 0.40),
        (5.0, 8.0, 0.30),
        (15.0, 12.5, 0.50),
        (9.0, 17.0, 0.40),
        (3.0, 11.0, 0.35),
        (17.5, 16.0, 0.45),
    ]:
        world.mark_obstacle(ox, oy, r)
    return world


def main() -> None:
    world = build_world()
    robot = Robot(x=START_XY[0], y=START_XY[1], theta=START_THETA)
    goal = list(GOAL_XY)
    planner = APFPlanner()
    view = PygameView(world, window_size=800)

    reached = False
    trail: list[tuple[float, float]] = []
    print("sr-sim | APF demo")
    print("  left-click  add obstacle    right-click  move goal")
    print("  R           reset robot     ESC/Q        quit")

    running = True
    while running:
        for ev in view.poll():
            if ev["type"] == "quit":
                running = False
            elif ev["type"] == "obstacle":
                world.mark_obstacle(*ev["pos"], radius=OBSTACLE_RADIUS)
                view.mark_grid_dirty()
            elif ev["type"] == "goal":
                goal[0], goal[1] = ev["pos"]
                reached = False
            elif ev["type"] == "reset":
                robot.x, robot.y, robot.theta = START_XY[0], START_XY[1], START_THETA
                trail.clear()
                reached = False

        if not reached:
            v, omega = planner.compute(robot, tuple(goal), world)
            step(robot, v, omega, DT, world)
            trail.append((robot.x, robot.y))
            if len(trail) > MAX_TRAIL:
                trail.pop(0)
            if math.hypot(robot.x - goal[0], robot.y - goal[1]) < GOAL_REACH_RADIUS:
                reached = True
                print(
                    f"Goal reached! "
                    f"x={robot.x:.2f}  y={robot.y:.2f}  "
                    f"theta={math.degrees(robot.theta):.1f}°"
                )

        view.draw(robot, tuple(goal), trail=trail)

    view.quit()


if __name__ == "__main__":
    main()
