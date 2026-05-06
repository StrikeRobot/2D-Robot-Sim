"""Generate assets/gifs/apf_demo.gif — no display required.

    python examples/record_demo.py

Or from Docker (writes the GIF to your host via a bind-mount):

    docker run --rm -v $(pwd)/assets:/app/assets sr-sim \
        python examples/record_demo.py
"""

import math

from sr_sim.core.physics import step
from sr_sim.core.robot import Robot
from sr_sim.core.world import World
from sr_sim.planners.potential_field import APFPlanner
from sr_sim.render.matplotlib_view import MatplotlibRecorder


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


if __name__ == "__main__":
    world = build_world()
    robot = Robot(x=1.5, y=1.5, theta=math.pi / 4)
    goal = (18.5, 18.5)
    planner = APFPlanner()

    recorder = MatplotlibRecorder(world)
    recorder.record(
        robot, goal, planner, world, step,
        n_frames=350,
        dt=0.05,
        fps=20,
        output="assets/gifs/apf_demo.gif",
    )
