from __future__ import annotations

import math
from typing import Any

import numpy as np


class PygameView:
    """Interactive pygame window with smooth vector rendering.

    Background + obstacles are cached as a Surface and only rebuilt when the
    world changes. Robot, trail, goal, and HUD are drawn on top every frame.

    Controls:
        Left-click   → add circular obstacle at cursor
        Right-click  → move goal to cursor
        R            → emit 'reset' event
        ESC / Q      → emit 'quit' event
    """

    # ── palette ──────────────────────────────────────────────────────────
    _BG           = (232, 232, 222)
    _GRID_LINE    = (218, 218, 208)
    _BORDER_COLOR = (52, 52, 48)
    _OBS_FILL     = (72, 72, 68)
    _OBS_RIM      = (42, 42, 40)
    _OBS_SHINE    = (110, 108, 104)
    _TRAIL        = (110, 160, 235)
    _GOAL_FILL    = (55, 195, 80)
    _GOAL_RIM     = (35, 155, 60)
    _GOAL_OUTER   = (80, 210, 105, 140)   # RGBA — semi-transparent pulse ring
    _ROBOT_RIM    = (180, 210, 255)
    _HUD_BG       = (18, 18, 18, 185)     # RGBA
    _HUD_TITLE    = (150, 205, 255)
    _HUD_TEXT     = (215, 215, 215)
    # ─────────────────────────────────────────────────────────────────────

    def __init__(self, world, window_size: int = 800):
        import pygame

        pygame.init()
        pygame.font.init()

        self._world = world
        self._ppm = min(window_size / world.width, window_size / world.height)
        self._sw = int(world.width * self._ppm)
        self._sh = int(world.height * self._ppm)
        self._screen = pygame.display.set_mode((self._sw, self._sh))
        pygame.display.set_caption("sr-sim — Artificial Potential Fields")
        self._clock = pygame.time.Clock()
        self._grid_surf: Any = None
        self._grid_dirty = True

        # Fonts — fall back to pygame built-in if DejaVu is unavailable
        _mono = "DejaVu Sans Mono"
        self._font_sm    = pygame.font.SysFont(_mono, 12) or pygame.font.Font(None, 15)
        self._font_title = pygame.font.SysFont(_mono, 13, bold=True) or pygame.font.Font(None, 16)

    # ------------------------------------------------------------------
    # Coord helpers
    # ------------------------------------------------------------------

    def _to_screen(self, x: float, y: float) -> tuple[int, int]:
        """World (x, y) → screen pixel. Flips the y axis."""
        return int(x * self._ppm), self._sh - int(y * self._ppm)

    def _to_world(self, sx: int, sy: int) -> tuple[float, float]:
        return sx / self._ppm, (self._sh - sy) / self._ppm

    # ------------------------------------------------------------------
    # Background surface (cached)
    # ------------------------------------------------------------------

    def mark_grid_dirty(self) -> None:
        self._grid_dirty = True

    def _rebuild_grid(self) -> None:
        import pygame

        surf = pygame.Surface((self._sw, self._sh))
        surf.fill(self._BG)

        # Subtle 1-metre grid
        for gx in range(0, int(self._world.width) + 1):
            sx = int(gx * self._ppm)
            pygame.draw.line(surf, self._GRID_LINE, (sx, 0), (sx, self._sh), 1)
        for gy in range(0, int(self._world.height) + 1):
            sy = self._sh - int(gy * self._ppm)
            pygame.draw.line(surf, self._GRID_LINE, (0, sy), (self._sw, sy), 1)

        # Border walls as solid filled strips
        bt = self._world.border_thickness
        if bt > 0:
            bt_px = max(1, int(bt * self._ppm))
            for rect in [
                (0, 0, self._sw, bt_px),                          # top
                (0, self._sh - bt_px, self._sw, bt_px),           # bottom
                (0, 0, bt_px, self._sh),                          # left
                (self._sw - bt_px, 0, bt_px, self._sh),           # right
            ]:
                pygame.draw.rect(surf, self._BORDER_COLOR, rect)

        # Obstacles as smooth circles with a small highlight spot
        for ox, oy, r in self._world.obstacle_shapes:
            sx, sy = self._to_screen(ox, oy)
            r_px = max(4, int(r * self._ppm))
            pygame.draw.circle(surf, self._OBS_FILL, (sx, sy), r_px)
            pygame.draw.circle(surf, self._OBS_RIM,  (sx, sy), r_px, 2)
            # Glint (top-left quadrant)
            shine_r = max(2, r_px // 4)
            shine_x = sx - r_px // 3
            shine_y = sy - r_px // 3
            pygame.draw.circle(surf, self._OBS_SHINE, (shine_x, shine_y), shine_r)

        self._grid_surf = surf
        self._grid_dirty = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def poll(self) -> list[dict[str, Any]]:
        """Drain the pygame event queue and return sim-level event dicts."""
        import pygame

        events: list[dict[str, Any]] = []
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                events.append({"type": "quit"})
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    events.append({"type": "quit"})
                elif ev.key == pygame.K_r:
                    events.append({"type": "reset"})
            elif ev.type == pygame.MOUSEBUTTONDOWN:
                wx, wy = self._to_world(*ev.pos)
                if ev.button == 1:
                    events.append({"type": "obstacle", "pos": (wx, wy)})
                elif ev.button == 3:
                    events.append({"type": "goal", "pos": (wx, wy)})
        return events

    def draw(
        self,
        robots: Any,
        goal: tuple[float, float],
        trail: list[tuple[float, float]] | None = None,
        fps: int = 60,
    ) -> None:
        """Render one frame: background → trail → goal → robots → HUD."""
        import pygame

        if self._grid_dirty or self._grid_surf is None:
            self._rebuild_grid()

        self._screen.blit(self._grid_surf, (0, 0))

        if trail:
            self._draw_trail(trail)

        self._draw_goal(goal)

        robot_list = robots if isinstance(robots, list) else [robots]
        for robot in robot_list:
            self._draw_robot(robot)

        self._draw_hud(robot_list[0], goal)

        pygame.display.flip()
        self._clock.tick(fps)

    def quit(self) -> None:
        import pygame
        pygame.quit()

    # ------------------------------------------------------------------
    # Draw helpers
    # ------------------------------------------------------------------

    def _draw_trail(self, trail: list[tuple[float, float]]) -> None:
        import pygame

        if len(trail) < 2:
            return
        points = [self._to_screen(x, y) for x, y in trail]
        pygame.draw.lines(self._screen, self._TRAIL, False, points, 2)
        # Bright dot at the freshest point
        pygame.draw.circle(self._screen, self._ROBOT_RIM, points[-1], 3)

    def _draw_goal(self, goal: tuple[float, float]) -> None:
        import pygame

        sx, sy = self._to_screen(*goal)
        t_ms = pygame.time.get_ticks()
        pulse = int(6 * abs(math.sin(t_ms * 0.002)))

        # Pulsing outer ring
        outer_surf = pygame.Surface((self._sw, self._sh), pygame.SRCALPHA)
        pygame.draw.circle(outer_surf, self._GOAL_OUTER, (sx, sy), 18 + pulse, 2)
        self._screen.blit(outer_surf, (0, 0))

        # Static rings
        pygame.draw.circle(self._screen, self._GOAL_FILL, (sx, sy), 12)
        pygame.draw.circle(self._screen, self._GOAL_RIM,  (sx, sy), 12, 2)
        pygame.draw.circle(self._screen, self._GOAL_RIM,  (sx, sy), 6,  2)
        # Centre dot
        pygame.draw.circle(self._screen, (255, 255, 255), (sx, sy), 3)

    def _draw_robot(self, robot: Any) -> None:
        import pygame

        sx, sy = self._to_screen(robot.x, robot.y)
        r_px = max(5, int(robot.radius * self._ppm))
        theta = robot.theta

        # Body
        pygame.draw.circle(self._screen, robot.color, (sx, sy), r_px)
        pygame.draw.circle(self._screen, self._ROBOT_RIM, (sx, sy), r_px, 2)

        # Direction wedge (white filled triangle pointing forward)
        front = (
            int(sx + r_px * 1.25 * math.cos(theta)),
            int(sy - r_px * 1.25 * math.sin(theta)),
        )
        left = (
            int(sx + r_px * 0.55 * math.cos(theta + math.radians(120))),
            int(sy - r_px * 0.55 * math.sin(theta + math.radians(120))),
        )
        right = (
            int(sx + r_px * 0.55 * math.cos(theta - math.radians(120))),
            int(sy - r_px * 0.55 * math.sin(theta - math.radians(120))),
        )
        pygame.draw.polygon(self._screen, (255, 255, 255), [front, left, right])

    def _draw_hud(self, robot: Any, goal: tuple[float, float]) -> None:
        import pygame

        dist  = math.hypot(robot.x - goal[0], robot.y - goal[1])
        fps_v = int(self._clock.get_fps())

        lines = [
            ("sr-sim  |  APF Planner", self._HUD_TITLE),
            (f"pos   x={robot.x:5.2f}  y={robot.y:5.2f} m", self._HUD_TEXT),
            (f"hdg   {math.degrees(robot.theta):+6.1f}°",     self._HUD_TEXT),
            (f"dist  {dist:5.2f} m to goal",                  self._HUD_TEXT),
            (f"fps   {fps_v:3d}",                              self._HUD_TEXT),
        ]

        pad    = 7
        line_h = 15
        panel_w = 210
        panel_h = len(lines) * line_h + pad * 2

        panel = pygame.Surface((panel_w, panel_h), pygame.SRCALPHA)
        panel.fill(self._HUD_BG)
        # Top accent bar
        pygame.draw.rect(panel, (*self._HUD_TITLE, 200), (0, 0, panel_w, 2))
        self._screen.blit(panel, (10, 10))

        font_map = {self._HUD_TITLE: self._font_title, self._HUD_TEXT: self._font_sm}
        for i, (text, color) in enumerate(lines):
            rendered = font_map[color].render(text, True, color)
            self._screen.blit(rendered, (10 + pad, 10 + pad + i * line_h))
