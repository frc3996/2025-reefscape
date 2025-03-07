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

from enum import IntEnum
import math
from typing import override

import rev
import wpilib
import wpimath.controller
import wpimath.trajectory
import wpimath.units
from magicbot import StateMachine, tunable, will_reset_to
from magicbot.state_machine import state, timed_state

import constants
import common.tools as tools 

class Climb:
    kClimbMaxSpeed = tunable(0.1)
    kClimbMaxAccel = tunable(0.1)
    kClimbDistance = tunable(0.1)
    kPID_P = tunable(3.0)
    kPID_I = tunable(0.0)
    kPID_D = tunable(0.0)

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
        _ = climbMainConfig.encoder.positionConversionFactor(self.kRpmToVelocity)
        _ = climbMainConfig.encoder.velocityConversionFactor(self.kRpmToVelocity)
        _ = self.climbMain.configure(
            climbMainConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )
        self.climbEncoder: rev.SparkRelativeEncoder = self.climbMain.getEncoder()
        self.climbEncoder.setPosition(0)

        self.climbPID: wpimath.controller.ProfiledPIDController = (
            wpimath.controller.ProfiledPIDController(
                self.kPID_P,
                self.kPID_I,
                self.kPID_D,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    self.kClimbMaxSpeed, self.kClimbMaxAccel
                ),
            )
        )
        self.climbPID.reset(self.climbEncoder.getPosition())

        # Limit switches
        self.climb_limitswitch: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.CLIMB_MAIN_LIMITSWITCH
        )

    def climbUp(self):
        self.targetClimbDistance = self.kClimbDistance

    def climbDown(self):
        self.targetClimbDistance = 0.0

    def climbStop(self):
        self.targetClimbDistance = self.climbEncoder.getPosition()

    def atLimit(self) -> bool:
        return self.climb_limitswitch.get()

    def atGoal(self) -> bool:
        return self.atLimit() or tools.float_equal(self.climbEncoder.getPosition(), self.targetClimbDistance, 0.05)

    def execute(self):
        if self.atGoal():
            self.climbMain.set(0)
        else:
            climbSpeed = self.climbPID.calculate(
                self.climbEncoder.getPosition(), self.targetClimbDistance
            )
            climbSpeed = tools.clamp(climbSpeed, -self.kClimbMaxSpeed, self.kClimbMaxSpeed)
            climbSpeed = tools.clamp(climbSpeed, -1.0, 1.0)
            self.climbMain.set(climbSpeed)
        # print(f"climbOutput {self.climbMain.get()}")

class ClimbTarget(IntEnum):
    DOWN = -1
    STOP = 0
    UP = 1

class ActionClimb(StateMachine):
    climb: Climb

    def __init__(self):
        self.current_target = ClimbTarget.STOP

    def doClimb(self, target: ClimbTarget, initial_state=None) -> None:
        self.current_target = target
        return super().engage(initial_state)

    @state(first=True)
    def move_climb(self, initial_call: bool):
        if self.current_target == ClimbTarget.UP:
            self.climb.climbUp()
        elif self.current_target == ClimbTarget.DOWN:
            self.climb.climbDown()
        else:
            self.done()

    @override
    def done(self):
        self.current_target = ClimbTarget.STOP
        self.climb.climbStop()
        super().done()
