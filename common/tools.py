import math

import wpilib
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

def is_autonomous():
    return wpilib.DriverStation.isAutonomous()

def float_equal(a: float, b: float, epsilon: float = 1e-4) -> bool:
    return abs(a - b) < epsilon

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

def constrain_angle(angle: float) -> float:
    """Wrap an angle to the interval [-pi,pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))

def clamp(val: float, low: float, high: float) -> float:
    return max(min(val, high), low)

def rate_limit_2d(
    cur: tuple[float, float], target: tuple[float, float], rate_limit: float, dt: float
) -> tuple[float, float]:
    """Limits the change in a vector to rate_limit * dt"""
    err = (target[0] - cur[0], target[1] - cur[1])
    mag = math.hypot(*err)
    if mag == 0:
        return target
    if mag < rate_limit * dt:
        change = err
    else:
        err_norm = (err[0] / mag, err[1] / mag)
        change = (err_norm[0] * rate_limit * dt, err_norm[1] * rate_limit * dt)
    return cur[0] + change[0], cur[1] + change[1]

def rate_limit_module(
    cur: SwerveModuleState,
    target: SwerveModuleState,
    rate_limit: float,
    dt: float = 0.02,
) -> SwerveModuleState:
    """
    Limit the change in a module state so that the acceleration dosent exceed rate_limit
    """
    cur_vx = cur.angle.cos() * cur.speed
    cur_vy = cur.angle.sin() * cur.speed
    target_vx = target.angle.cos() * target.speed
    target_vy = target.angle.sin() * target.speed

    new_vx, new_vy = rate_limit_2d(
        (cur_vx, cur_vy), (target_vx, target_vy), rate_limit, dt
    )
    new_speed = math.hypot(new_vx, new_vy)
    rot = cur.angle if new_speed == 0 else Rotation2d(new_vx, new_vy)
    return SwerveModuleState(new_speed, rot)


def clamp_2d(val: tuple[float, float], radius: float) -> tuple[float, float]:
    """
    Constrains a vector to be within the unit circle
    """
    mag = math.hypot(*val)
    if mag == 0:
        return (0, 0)
    new_mag = min(mag, radius)
    return new_mag * val[0] / mag, new_mag * val[1] / mag
