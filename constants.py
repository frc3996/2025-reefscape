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
    """ Définition des senseurs Analogiques"""
    BEAM_SENSOR = 0
    PIXY_OFFSET = 1


class DigitalIO(IntEnum):
    """ Définition des senseurs digitales"""
    PIXY_VALID = 0
    CLIMB_CAGE_IN_LIMITSWITCH_1 = 1
    CLIMB_CAGE_IN_LIMITSWITCH_2 = 2
    CLIMB_PISTON_OUT_LIMITSWITCH_1 = 3
    CLIMB_PISTON_OUT_LIMITSWITCH_2 = 4


class CANIds(IntEnum):
    """ Définition des ID CAN"""
    SWERVE_ROTATE_FL = 25
    SWERVE_DRIVE_FL = 26
    SWERVE_CANCODER_FL = 11

    SWERVE_ROTATE_FR = 27
    SWERVE_DRIVE_FR = 28
    SWERVE_CANCODER_FR = 14

    SWERVE_ROTATE_RL = 23
    SWERVE_DRIVE_RL = 24
    SWERVE_CANCODER_RL = 12

    SWERVE_ROTATE_RR = 22
    SWERVE_DRIVE_RR = 21
    SWERVE_CANCODER_RR = 13


    CLIMB_GUIDE_MOTOR = 50
    CLIMB_RAISE_MOTOR_MAIN = 51
    CLIMB_RAISE_MOTOR_FOLLOWER = 52


class SolenoidChannel(IntEnum):
    """ Définition des senseurs Analogiques"""
    CLIMB_PISTON = 0