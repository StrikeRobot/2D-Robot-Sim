from __future__ import annotations

import numpy as np


class World:
    """2D occupancy-grid world.

    Coordinate system: x increases right, y increases up (standard math axes).
    Grid: grid[row, col] where row = floor(y / cell_size), col = floor(x / cell_size).
    """

    def __init__(self, width: float = 20.0, height: float = 20.0, cell_size: float = 0.1):
        self.width = width
        self.height = height
        self.cell_size = cell_size
        self.cols = int(width / cell_size)
        self.rows = int(height / cell_size)
        self._grid = np.zeros((self.rows, self.cols), dtype=np.uint8)
        # Rendering metadata — stores shapes for smooth vector drawing
        self._obstacle_shapes: list[tuple[float, float, float]] = []
        self._border_thickness: float = 0.0

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Return (row, col) for world point. No bounds clamping."""
        return int(y / self.cell_size), int(x / self.cell_size)

    def grid_to_world(self, row: int, col: int) -> tuple[float, float]:
        """Return cell centre in world coords for (row, col)."""
        return (col + 0.5) * self.cell_size, (row + 0.5) * self.cell_size

    # ------------------------------------------------------------------
    # Obstacle helpers
    # ------------------------------------------------------------------

    def mark_obstacle(self, x: float, y: float, radius: float = 0.2) -> None:
        """Fill a circle of obstacle cells centred at (x, y)."""
        row0, col0 = self.world_to_grid(x, y)
        pad = int(radius / self.cell_size) + 1
        r0 = max(0, row0 - pad)
        r1 = min(self.rows, row0 + pad + 1)
        c0 = max(0, col0 - pad)
        c1 = min(self.cols, col0 + pad + 1)

        R, C = np.mgrid[r0:r1, c0:c1]
        wx = (C + 0.5) * self.cell_size
        wy = (R + 0.5) * self.cell_size
        mask = (wx - x) ** 2 + (wy - y) ** 2 <= radius ** 2
        self._grid[r0:r1, c0:c1][mask] = 1
        self._obstacle_shapes.append((x, y, radius))

    def fill_rect(self, x: float, y: float, w: float, h: float) -> None:
        """Mark an axis-aligned rectangle of obstacles. (x, y) is the bottom-left corner."""
        r0, c0 = self.world_to_grid(x, y)
        r1, c1 = self.world_to_grid(x + w, y + h)
        r0 = max(0, r0)
        c0 = max(0, c0)
        r1 = min(self.rows, r1 + 1)
        c1 = min(self.cols, c1 + 1)
        self._grid[r0:r1, c0:c1] = 1

    def add_border(self, thickness: float = 0.2) -> None:
        """Mark a solid border of the given thickness along all four edges."""
        t = max(1, int(thickness / self.cell_size))
        self._grid[:t, :] = 1
        self._grid[-t:, :] = 1
        self._grid[:, :t] = 1
        self._grid[:, -t:] = 1
        self._border_thickness = thickness

    # ------------------------------------------------------------------
    # Query
    # ------------------------------------------------------------------

    def is_occupied(self, x: float, y: float) -> bool:
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True
        row, col = self.world_to_grid(x, y)
        row = int(np.clip(row, 0, self.rows - 1))
        col = int(np.clip(col, 0, self.cols - 1))
        return bool(self._grid[row, col])

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    @property
    def obstacle_shapes(self) -> list[tuple[float, float, float]]:
        """List of (x, y, radius) for every mark_obstacle call — used by the renderer."""
        return self._obstacle_shapes

    @property
    def border_thickness(self) -> float:
        return self._border_thickness
