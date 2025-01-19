import math

import numpy
import wpilib
import wpimath.units
from wpimath import geometry, kinematics


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


def fit_to_boundaries(value, minimum_value=None, maximum_value=None):
    """Fits a value to boundaries. None to dismiss a boundary."""
    if minimum_value is not None:
        value = max(minimum_value, value)
    if maximum_value is not None:
        value = min(maximum_value, value)
    return value


def square_input(input):
    """Retourne la valeur au carré en conservant le signe"""
    return math.copysign(input * input, input)


def is_autonomous():
    return wpilib.DriverStation.isAutonomous()


def map_value(value, old_min, old_max, new_min, new_max):
    # Figure out how 'wide' each range is
    old_span = old_max - old_min
    new_span = new_max - new_min

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - old_min) / float(old_span)

    # Convert the 0-1 range into a value in the right range.
    return new_min + (valueScaled * new_span)


def is_red() -> bool:
    return wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed


def rotate_vector(vector, angle):
    rad = math.radians(angle)
    x = vector[0] * math.cos(rad) - vector[1] * math.sin(rad)
    y = vector[0] * math.sin(rad) + vector[1] * math.cos(rad)
    return (x, y)


def get_projectile_launch_angle_and_rotation(
    transform: geometry.Transform3d,
    projectile_velocity,
    chassis_speed: kinematics.ChassisSpeeds,
    current_angle: geometry.Rotation2d
):

    # Convert chassis speed to field relative
    vx, vy = rotate_vector([chassis_speed.vx, chassis_speed.vy], current_angle.degrees())
    chassis_speed.vx = vx
    chassis_speed.vy = vy

    # Calculate the direct distance to the target and the effective initial
    # velocity in the direction of the target
    distance_to_target = numpy.sqrt((transform.x) ** 2 + (transform.y) ** 2)
    # Pythagorean theorem for the velocity in the xz-plane
    effective_velocity = numpy.sqrt(projectile_velocity**2 + chassis_speed.vx**2)

    # Calculate launch angle using the formula for projectile motion, ignoring
    # lateral displacement for now
    g = 9.81  # Acceleration due to gravity, m/s^2
    a = g * distance_to_target**2 / (2 * effective_velocity**2)
    b = distance_to_target
    c = -(a + transform.z)

    # Quadratic formula components for calculating the launch angle
    discriminant = b**2 - 4 * a * c
    if discriminant < 0:
        # No real solution, indicating that the target cannot be reached with
        # the given parameters
        return (
            None,
            math.atan2(transform.y, transform.x),
        )

    # Two possible angles, select the smaller one (more direct shot)
    theta_1 = numpy.arctan((-b + numpy.sqrt(discriminant)) / (2 * a))
    theta_2 = numpy.arctan((-b - numpy.sqrt(discriminant)) / (2 * a))
    if (theta_1 < 0 and theta_2 < 0):
        return (
            None,
            compute_angle(transform.x, transform.y),
        )
    elif (theta_1 < 0):
        launch_angle = theta_2
    elif (theta_2 < 0):
        launch_angle = theta_1
    elif (theta_1 > 0 and theta_2 > 0):
        launch_angle = numpy.radians(min(theta_1, theta_2))
    else:
        return (
            None,
            compute_angle(transform.x, transform.y),
        )

    # Calculate rotation angle to compensate for lateral displacement
    # Assuming time of flight based on direct distance and effective velocity
    time_of_flight = distance_to_target / (effective_velocity * numpy.cos(launch_angle))
    lateral_displacement = (
        chassis_speed.vy * time_of_flight
    )  # Simple displacement = speed * time
    rotation_angle = compute_angle(transform.x, transform.y + lateral_displacement)
    return launch_angle, rotation_angle


# def old_calculate_optimal_launch_angle(
#     distance, height_difference, initial_velocity
# ) -> wpimath.units.degrees | None:
#     """
#     Calculates the optimal launch angle to hit a target at a given distance and height difference.
#
#     :param distance: The horizontal distance to the target (meters).
#     :param height_difference: The height difference to the target (meters).
#     :param initial_velocity: The initial velocity of the disk (meters/second).
#     :return: The optimal launch angle (degrees) to hit the target.
#     """
#     # angle = math.degrees(math.atan(height_difference/distance))
#     # return angle
#
#     g = 9.81
#     min_error = float("inf")
#     best_angle = 0
#
#     for angle in range(1, 180):
#         theta = math.radians(angle / 2)
#         t = distance / (initial_velocity * math.cos(theta))
#         y = (initial_velocity * math.sin(theta) * t) - (0.5 * 9.8 * t**2)
#         error = abs(y - height_difference)
#         if error < min_error:
#             min_error = error
#             best_angle = angle / 2
#     return best_angle
#
#     # TODO Make it better?
#     g = 9.81  # Acceleration due to gravity (m/s^2)
#     v = initial_velocity
#
#     # The quadratic formula components (a, b, and c) are used in solving for the angle.
#     a = g * distance**2
#     b = 2 * height_difference * v**2 - 2 * g * distance**2
#     c = v**4 - g * (g * distance**2 + 2 * height_difference * v**2)
#
#     # Check if the discriminant is positive to ensure a real solution exists.
#     discriminant = b**2 - 4 * a * c
#     if discriminant < 0:
#         return None
#
#     # Calculate the two possible angles and select the smaller one for the optimal trajectory.
#     theta1 = math.atan2(v**2 + math.sqrt(discriminant), g * distance)
#     theta2 = math.atan2(v**2 - math.sqrt(discriminant), g * distance)
#
#     # Convert the optimal angle to degrees.
#     angle_deg = math.degrees(min(theta1, theta2))
#
#     return angle_deg


def compute_angle(x, y):
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
