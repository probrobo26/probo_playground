from dataclasses import dataclass
import random

# --- Measurements ---
NEAR_ZERO = 1e-6
SEED = random.seed(107)


def floating_mod_zero(n1: float, n2: float):
    factor = n1 / n2
    return abs(round(factor, 3) - float(round(factor))) < NEAR_ZERO


@dataclass(unsafe_hash=True)
class Position:
    """
    Represents an xy coordinate.
    """

    x: float = 0.0
    y: float = 0.0

    def to_dict(self):
        return {
            "x": self.x,
            "y": self.y,
        }

    def to_string(self):
        return f"X{self.x}Y{self.y}"


@dataclass(unsafe_hash=True)
class Pose:
    """
    Represents robot position and heading.
    """

    pos: Position = Position()
    theta: float = 0.0

    def to_dict(self):
        return {
            "pos": self.pos.to_dict(),
            "theta": self.theta,
        }

    def to_string(self):
        return self.pos.to_string() + f"T{self.theta}"


@dataclass(frozen=True)
class BearingRange:
    """
    Represents the relationship between the robot and a landmark.
    """

    bearing: float
    range: float

    def to_dict(self):
        return {
            "bearing": self.bearing,
            "range": self.range,
        }

    def to_string(self):
        return f"B{self.bearing}R{self.range}"


# --- Environment Features ---


@dataclass(frozen=True)
class Landmark:
    """
    Represents an identifiable floating-point landmark.
    """

    pos: Position
    id: int

    def to_dict(self):
        return {
            "id": self.id,
            "pos": self.pos.to_dict(),
        }

    def to_string(self):
        return f"L{self.id}" + self.pos.to_string()


@dataclass(frozen=True)
class Bounds:
    """
    Represents any bounded area, including obstacles such as walls or the environment itself. The edge of a Bounds instance is considered to be contained by that instance.
    """

    x_min: float
    x_max: float
    y_min: float
    y_max: float

    def within_x(self, x: float) -> bool:
        return self.x_max >= x and self.x_min <= x

    def within_y(self, y: float) -> bool:
        return self.y_max >= y and self.y_min <= y

    def within_bounds(self, pos: Position) -> bool:
        return self.within_x(pos.x) and self.within_y(pos.y)

    def to_dict(self):
        return {
            "x_min": self.x_min,
            "x_max": self.x_max,
            "y_min": self.y_min,
            "y_max": self.y_max,
        }
