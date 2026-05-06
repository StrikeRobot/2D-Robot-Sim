from __future__ import annotations

import math

from .robot import Robot
from .world import World

_PERIMETER_ANGLES = [k * math.pi / 4 for k in range(8)]


def step(robot: Robot, v: float, omega: float, dt: float, world: World) -> None:
    """Advance robot one timestep with sliding wall collision.

    Tries the full (dx, dy) move first; on collision falls back to axis-aligned
    slides so the robot hugs walls rather than stopping dead.
    """
    dx = v * math.cos(robot.theta) * dt
    dy = v * math.sin(robot.theta) * dt
    r = robot.radius

    nx, ny = robot.x + dx, robot.y + dy
    if not _collides(nx, ny, r, world):
        robot.x, robot.y = nx, ny
    elif not _collides(robot.x + dx, robot.y, r, world):
        robot.x += dx
    elif not _collides(robot.x, robot.y + dy, r, world):
        robot.y += dy
    # else fully blocked — position unchanged

    robot.theta += omega * dt


def _collides(x: float, y: float, radius: float, world: World) -> bool:
    if world.is_occupied(x, y):
        return True
    for angle in _PERIMETER_ANGLES:
        if world.is_occupied(x + radius * math.cos(angle), y + radius * math.sin(angle)):
            return True
    return False
