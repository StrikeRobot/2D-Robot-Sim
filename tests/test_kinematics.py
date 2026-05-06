import math

import pytest

from sr_sim.core.physics import step
from sr_sim.core.robot import Robot
from sr_sim.core.world import World


def empty_world() -> World:
    return World(20.0, 20.0, 0.1)


# ------------------------------------------------------------------
# Kinematics
# ------------------------------------------------------------------

def test_straight_motion_along_x():
    w = empty_world()
    r = Robot(x=5.0, y=5.0, theta=0.0)
    step(r, v=1.0, omega=0.0, dt=1.0, world=w)
    assert abs(r.x - 6.0) < 0.01
    assert abs(r.y - 5.0) < 0.01


def test_straight_motion_along_y():
    w = empty_world()
    r = Robot(x=5.0, y=5.0, theta=math.pi / 2)
    step(r, v=1.0, omega=0.0, dt=1.0, world=w)
    assert abs(r.x - 5.0) < 0.01
    assert abs(r.y - 6.0) < 0.01


def test_rotation_only():
    w = empty_world()
    r = Robot(x=5.0, y=5.0, theta=0.0)
    step(r, v=0.0, omega=math.pi / 2, dt=1.0, world=w)
    assert abs(r.x - 5.0) < 0.01
    assert abs(r.y - 5.0) < 0.01
    assert abs(r.theta - math.pi / 2) < 0.01


def test_small_timestep_accumulates():
    w = empty_world()
    r = Robot(x=5.0, y=5.0, theta=0.0)
    for _ in range(20):
        step(r, v=1.0, omega=0.0, dt=0.05, world=w)
    assert abs(r.x - 6.0) < 0.02
    assert abs(r.y - 5.0) < 0.02


# ------------------------------------------------------------------
# Collision
# ------------------------------------------------------------------

def test_obstacle_blocks_forward_motion():
    w = empty_world()
    w.mark_obstacle(6.5, 5.0, radius=0.5)
    r = Robot(x=5.0, y=5.0, theta=0.0, radius=0.2)
    for _ in range(200):
        step(r, v=1.0, omega=0.0, dt=0.05, world=w)
    assert r.x < 6.1


def test_border_blocks_robot():
    w = empty_world()
    w.add_border(thickness=0.3)
    r = Robot(x=1.0, y=1.0, theta=math.radians(225), radius=0.2)
    for _ in range(500):
        step(r, v=2.0, omega=0.0, dt=0.05, world=w)
    assert r.x >= 0.3
    assert r.y >= 0.3


# ------------------------------------------------------------------
# World helpers
# ------------------------------------------------------------------

def test_world_to_grid_round_trip():
    w = World(20.0, 20.0, 0.1)
    for x, y in [(0.55, 0.55), (10.0, 10.0), (18.45, 18.45)]:
        row, col = w.world_to_grid(x, y)
        wx, wy = w.grid_to_world(row, col)
        assert abs(wx - x) < w.cell_size
        assert abs(wy - y) < w.cell_size


def test_mark_obstacle_centre_is_occupied():
    w = empty_world()
    w.mark_obstacle(10.0, 10.0, radius=0.5)
    assert w.is_occupied(10.0, 10.0)


def test_mark_obstacle_outside_radius_is_free():
    w = empty_world()
    w.mark_obstacle(10.0, 10.0, radius=0.3)
    assert not w.is_occupied(10.0 + 0.5, 10.0)


def test_fill_rect():
    w = empty_world()
    w.fill_rect(5.0, 5.0, 2.0, 2.0)
    assert w.is_occupied(5.5, 5.5)
    assert w.is_occupied(6.5, 6.5)
    assert not w.is_occupied(8.0, 8.0)


def test_out_of_bounds_is_occupied():
    w = empty_world()
    assert w.is_occupied(-1.0, 5.0)
    assert w.is_occupied(5.0, 25.0)


# ------------------------------------------------------------------
# APF planner (smoke tests — no pygame needed)
# ------------------------------------------------------------------

def test_apf_moves_toward_goal():
    from sr_sim.planners.potential_field import APFPlanner

    w = empty_world()
    r = Robot(x=2.0, y=2.0, theta=math.pi / 4)
    planner = APFPlanner()
    goal = (18.0, 18.0)

    start_dist = math.hypot(r.x - goal[0], r.y - goal[1])
    for _ in range(100):
        v, omega = planner.compute(r, goal, w)
        step(r, v, omega, 0.05, w)
    end_dist = math.hypot(r.x - goal[0], r.y - goal[1])

    assert end_dist < start_dist


def test_apf_stops_at_goal():
    from sr_sim.planners.potential_field import APFPlanner

    w = empty_world()
    r = Robot(x=9.9, y=9.9, theta=math.pi / 4)
    planner = APFPlanner(goal_threshold=0.4)
    goal = (10.0, 10.0)

    v, omega = planner.compute(r, goal, w)
    assert v == pytest.approx(0.0)
    assert omega == pytest.approx(0.0)
