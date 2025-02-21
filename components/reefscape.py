from enum import StrEnum
from typing import TypedDict

import wpimath.units


class PositionKeys(StrEnum):
    X = "x"
    Y = "y"
    ROT = "rot"


class Position(TypedDict):
    x: float
    y: float
    rot: float


class PickupPositionsKeys(StrEnum):
    P1 = "p1"
    P2 = "p2"
    P3 = "p3"


class PickupPositions(TypedDict):
    p1: Position
    p2: Position
    p3: Position


class CoralStationKeys(StrEnum):
    LEFT = "left"
    RIGHT = "right"


class CoralStation(TypedDict):
    left: PickupPositions
    right: PickupPositions


class ReefPositionsKeys(StrEnum):
    A = "a"
    B = "b"
    C = "c"
    D = "d"
    E = "e"
    F = "f"
    G = "g"
    H = "h"
    I = "i"
    J = "j"
    K = "k"
    L = "l"


class ReefPositions(TypedDict):
    a: Position
    b: Position
    c: Position
    d: Position
    e: Position
    f: Position
    g: Position
    h: Position
    i: Position
    j: Position
    k: Position
    l: Position


class CagePositionKeys(StrEnum):
    LEFT = "left"
    MIDDLE = "middle"
    RIGHT = "right"


# XXX: Do the cages..


class CoralReef(TypedDict):
    coral: CoralStation
    reef: ReefPositions


class Reefscape(TypedDict):
    blue: CoralReef
    red: CoralReef


class CoralLevel(StrEnum):
    L1 = "l1"
    L2 = "l2"
    L3 = "l3"
    L4 = "l4"


FIELD_LENGTH = 17.548
FIELD_WIDTH = 8.052

# Heights for each level
CORAL_LEVEL = {
    "l1": wpimath.units.feetToMeters(2.500),
    "l2": wpimath.units.feetToMeters(5.000),
    "l3": wpimath.units.feetToMeters(7.500),
    "l4": wpimath.units.feetToMeters(1.000),
}


REEFSCAPE: Reefscape = {
    "blue": {
        "coral": {
            "left": {
                "p1": {"x": 0.351154, "y": 6.89648, "rot": 135.0},
                "p2": {"x": 0.851154, "y": 7.39648, "rot": 135.0},
                "p3": {"x": 1.351154, "y": 7.89648, "rot": 135.0},
            },
            "right": {
                "p1": {"x": 0.451154, "y": 1.15532, "rot": -135.0},
                "p2": {"x": 0.851154, "y": 0.65532, "rot": -135.0},
                "p3": {"x": 1.351154, "y": 0.15532, "rot": -135.0},
            },
        },
        "reef": {
            "a": {"x": 3.71, "y": 4.19, "rot": 0.0},
            "b": {"x": 3.71, "y": 3.862, "rot": 0.0},
            "c": {"x": 3.939, "y": 3.434, "rot": 60.0},
            "d": {"x": 4.243, "y": 3.269, "rot": 60.0},
            "e": {"x": 4.736, "y": 3.272, "rot": 120.0},
            "f": {"x": 5.02, "y": 3.435, "rot": 120.0},
            "g": {"x": 5.267, "y": 3.862, "rot": 180.0},
            "h": {"x": 5.267, "y": 4.19, "rot": 180.0},
            "i": {"x": 5.02, "y": 4.615, "rot": -120.0},
            "j": {"x": 4.737, "y": 4.783, "rot": -120.0},
            "k": {"x": 4.782, "y": 4.243, "rot": -60.0},
            "l": {"x": 3.959, "y": 4.616, "rot": -60.0},
        },
    },
    "red": {
        "coral": {
            "left": {
                "p1": {"x": 16.997198, "y": 1.26532, "rot": -45.0},
                "p2": {"x": 16.497198, "y": 0.85532, "rot": -45.0},
                "p3": {"x": 15.897198, "y": 0.45532, "rot": -45.0},
            },
            "right": {
                "p1": {"x": 16.997198, "y": 6.69648, "rot": 45.0},
                "p2": {"x": 16.497198, "y": 7.19648, "rot": 45.0},
                "p3": {"x": 15.897198, "y": 7.59648, "rot": 45.0},
            },
        },
        "reef": {
            "a": {"x": 13.838, "y": 4.19, "rot": 180.0},
            "b": {"x": 13.838, "y": 3.862, "rot": 180.0},
            "c": {"x": 13.609, "y": 3.434, "rot": 120.0},
            "d": {"x": 13.305, "y": 3.269, "rot": 120.0},
            "e": {"x": 12.812, "y": 3.272, "rot": 60.0},
            "f": {"x": 12.528, "y": 3.435, "rot": 60.0},
            "g": {"x": 12.281, "y": 3.862, "rot": 0.0},
            "h": {"x": 12.281, "y": 4.19, "rot": 0.0},
            "i": {"x": 12.528, "y": 4.615, "rot": -60.0},
            "j": {"x": 12.811, "y": 4.783, "rot": -60.0},
            "k": {"x": 12.766, "y": 4.243, "rot": -120.0},
            "l": {"x": 13.589, "y": 4.616, "rot": -120.0},
        },
    },
}
