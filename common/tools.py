import math

import wpilib


def apply_deadzone(value: float, threshold: float) -> float:
    """Apply a deadzone to a value in [-1,1].

    This ensures that the rest of the input space maps to [-1,1].
    """
    assert 0 <= threshold < 1
    if abs(value) < threshold:
        return 0
    return (value - math.copysign(threshold, value)) / (1 - threshold)


def map_exponential(value: float, base: float) -> float:
    """Takes a value in [-1,1] and maps it to an exponential curve."""
    assert base > 1
    return math.copysign((base ** abs(value) - 1) / (base - 1), value)


def rescale_js(value: float, deadzone: float, exponential: float = 1.5) -> float:
    """Rescale a joystick input, applying a deadzone and exponential.

    Args:
        value: the joystick value, in the interval [-1, 1].
        deadzone: the deadzone to apply.
        exponential: the strength of the exponential to apply
                     (i.e. how non-linear should the response be)
    """
    return map_exponential(apply_deadzone(value, deadzone), exponential + 1)


def fit_to_boundaries(
    value: float,
    minimum_value: None | float = None,
    maximum_value: None | float = None,
) -> float:
    """Fits a value to boundaries. None to dismiss a boundary."""
    if minimum_value is not None:
        value = max(minimum_value, value)
    if maximum_value is not None:
        value = min(maximum_value, value)
    return value


def square_input(input: float):
    """Retourne la valeur au carré en conservant le signe"""
    return math.copysign(input * input, input)


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
