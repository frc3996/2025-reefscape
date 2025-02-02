import math

BUTTON_A = 1
BUTTON_B = 2
BUTTON_X = 3
BUTTON_Y = 4
BUTTON_LEFT_BUMPER = 5
BUTTON_RIGHT_BUMPER = 6
BUTTON_SELECT = 7
BUTTON_START = 8
BUTTON_LEFT_JOYSTICK = 9
BUTTON_RIGHT_JOYSTICK = 10

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_LEFT_TRIGGER = 2
AXIS_RIGHT_TRIGGER = 3
AXIS_RIGHT_X = 4
AXIS_RIGHT_Y = 5

POV_NOT_PRESSED = -1
POV_UP = 0
POV_UP_RIGHT = 45
POV_RIGHT = 90
POV_DOWN_RIGHT = 135
POV_DOWN = 180
POV_DOWN_LEFT = 225
POV_LEFT = 270
POV_UP_LEFT = 315


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
