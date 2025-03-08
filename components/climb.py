"""
Climber Code
By Rikitik 3996
Paul-Hubert
Rimouski, QC, Canada
2025

For
FIRST FRC Competition 2025
Reefscape

Presented by
HAAS
"""

## UNTESTED CODE ##
## DO NOT USE ##

import math
from typing import override

import rev
import wpilib
import wpimath.controller
import wpimath.trajectory
import wpimath.units
from magicbot import StateMachine, tunable, feedback
from magicbot.state_machine import state

from components.lift import Lift
from components.chariot import Chariot

import constants
import common.tools as tools 

class Climb:
    kClimbSpeed = tunable(0.68)
    kDeployTargetDistance = tunable(5.0)
    kPullTargetDistance = tunable(0.0)
    kSpeed = 0
    move : bool = False

    def __init__(self):
        # V = RPM * 60 * (C / GR)
        self.kRpmToVelocity: float = 60 * (
            (wpimath.units.inchesToMeters(3.25) * 2 * math.pi) / (125)
        )
        self.targetClimbDistance: float = 0

    def setup(self) -> None:
        # Climb hardware
        self.climbMain: rev.SparkMax = rev.SparkMax(
            constants.CANIds.CLIMB_RAISE_MOTOR, rev.SparkMax.MotorType.kBrushless
        )

        # Set position conversion factor, revolution to distance
        climbMainConfig = rev.SparkBaseConfig()
        # _ = climbMainConfig.encoder.positionConversionFactor(self.kRpmToVelocity)
        # _ = climbMainConfig.encoder.velocityConversionFactor(self.kRpmToVelocity)
        _ = climbMainConfig.setIdleMode(climbMainConfig.IdleMode.kBrake)
        # _ = climbMainConfig.inverted(True)
        _ = self.climbMain.configure(
            climbMainConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )
        self.climbEncoder: rev.SparkRelativeEncoder = self.climbMain.getEncoder()

        # Limit switches
        self.climb_limitswitch: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.CLIMB_MAIN_LIMITSWITCH
        )

    @feedback
    def getEncoder(self):
        return self.climbEncoder.getPosition()
        
    @feedback
    def getDistanceTarget(self):
        return self.targetClimbDistance

    def on_enable(self):
        self.climbEncoder.setPosition(0)

    def doDeploy(self):
        self.targetClimbDistance = self.kDeployTargetDistance

    def doPull(self):
        self.targetClimbDistance = self.kPullTargetDistance

    def atLimit(self) -> bool:
        return False

    def goUp(self):
        self.kSpeed = 8.5

    def goDown(self):
        self.kSpeed = -8.5

    def stop(self):
        self.kSpeed = 0

    def execute(self):
        self.climbMain.set(self.kSpeed)

    def atGoal(self) -> bool:
        return tools.float_equal(self.climbEncoder.getPosition(), self.targetClimbDistance, 0.05)