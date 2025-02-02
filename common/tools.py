import math

import wpilib


def is_autonomous():
    return wpilib.DriverStation.isAutonomous()


def map_value(
    value: float, old_min: float, old_max: float, new_min: float, new_max: float
) -> float:
    # Figure out how 'wide' each range is
    old_span = old_max - old_min
    new_span = new_max - new_min

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - old_min) / float(old_span)

    # Convert the 0-1 range into a value in the right range.
    return new_min + (valueScaled * new_span)


def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def is_blue() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue


def rotate_vector(vector: tuple[float, float], angle: float) -> tuple[float, float]:
    rad = math.radians(angle)
    x: float
    y: float
    x = vector[0] * math.cos(rad) - vector[1] * math.sin(rad)
    y = vector[0] * math.sin(rad) + vector[1] * math.cos(rad)
    return (x, y)


def compute_angle(x: float, y: float) -> float:
    """Returns the absolute angle (in degrees) based on a vector"""
    angle_radians = math.atan2(y, x)
    angle_degrees = math.degrees(angle_radians)
    return angle_degrees


DISABLE_WRAPPER = True


def print_exec_time(name):
    def decorator(function):
        def wrapper(*args, **kwargs):
            if DISABLE_WRAPPER:
                return function(*args, **kwargs)

            start = wpilib.RobotController.getFPGATime()
            res = function(*args, **kwargs)
            delta = wpilib.RobotController.getFPGATime() - start
            delta = round(delta / 1e3, 1)
            if delta >= 0.5:
                log = f"{name} took {delta} ms"
                print(log)
            return res

        return wrapper

    return decorator
