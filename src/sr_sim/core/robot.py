from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Robot:
    """Differential-drive point robot.

    State: (x, y) position in metres, theta heading in radians (0 = +x axis).
    """

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    radius: float = 0.2
    max_v: float = 2.0
    max_omega: float = 3.0
    color: tuple[int, int, int] = field(default_factory=lambda: (30, 100, 220))

    @property
    def pose(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.theta)

    @property
    def heading(self) -> tuple[float, float]:
        return (math.cos(self.theta), math.sin(self.theta))
