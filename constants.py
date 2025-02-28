import math
from enum import IntEnum

# Swerve
MAX_ANGULAR_VEL = math.degrees(math.pi * 2) * 2
MAX_ANGULAR_ACCEL = math.degrees(math.pi * 5) * 2
LOWER_INPUT_THRESH = 0.1  # Minimum speed before considered 0
MAX_WHEEL_SPEED = 5  # meter per second
MAX_MODULE_SPEED = 5.625
SWERVE_DIRECTION_GEAR_RATIO = (14 / 50) * (10 / 60)


class AnalogIO(IntEnum):
    """Définition des senseurs Analogiques"""

    BEAM_SENSOR = 0
    PIXY_OFFSET = 1

    INTAKE_BEAM_SENSOR = 3


class DigitalIO(IntEnum):
    """Définition des senseurs digitales"""

    PIXY_VALID = 0
    CLIMB_CAGE_IN_LIMITSWITCH_1_AND_2 = 1
    CLIMB_PISTON_OUT_LIMITSWITCH_1 = 2
    CLIMB_PISTON_OUT_LIMITSWITCH_2 = 3

    LIFT_ZERO_LIMITSWITCH_1_AND_2 = 4
    LIFT_SAFETY_LIMITSWITCH_1_AND_2 = 5

    INTAKE_FRONT_LIMIT_SWITCH = 6
    INTAKE_BACK_LIMIT_SWITCH = 7

    LIFT_STRING_ENCODER_A = 8
    LIFT_STRING_ENCODER_B = 9


class CANIds(IntEnum):
    """Définition des ID CAN"""

    SWERVE_ROTATE_FL = 23
    SWERVE_DRIVE_FL = 24
    SWERVE_CANCODER_FL = 12

    SWERVE_ROTATE_FR = 25
    SWERVE_DRIVE_FR = 26
    SWERVE_CANCODER_FR = 11

    SWERVE_ROTATE_RL = 27
    SWERVE_DRIVE_RL = 28
    SWERVE_CANCODER_RL = 14

    SWERVE_ROTATE_RR = 21
    SWERVE_DRIVE_RR = 22
    SWERVE_CANCODER_RR = 13

    LIFT_MOTOR_MAIN = 31
    LIFT_MOTOR_FOLLOW = 32

    CLIMB_GUIDE_MOTOR = 40
    CLIMB_RAISE_MOTOR_MAIN = 41
    CLIMB_RAISE_MOTOR_FOLLOWER = 42

    INTAKE_INTAKE_MOTOR = 33
    INTAKE_OUTPUT_MOTOR = 34
    INTAKE_CHARIOT_MOTOR = 35


class SolenoidChannel(IntEnum):
    """Définition des senseurs Analogiques"""

    CLIMB_PISTON = 0
