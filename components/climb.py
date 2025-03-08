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
from magicbot import StateMachine, tunable
from magicbot.state_machine import state

from components.lift import Lift
from components.chariot import Chariot

import constants
import common.tools as tools 

class Climb:
    kClimbMaxSpeed = tunable(1.0)
    kClimbMaxAccel = tunable(1.0)
    kPID_P = tunable(1.0)
    kPID_I = tunable(0.0)
    kPID_D = tunable(0.0)
    kDeployTargetDistance = tunable(0.0)
    kPullTargetDistance = tunable(5.0)

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
        _ = climbMainConfig.disableFollowerMode()
        _ = climbMainConfig.setIdleMode(climbMainConfig.IdleMode.kBrake)
        _ = self.climbMain.configure(
            climbMainConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )
        print("Climb motor: ", self.climbMain.isFollower(), self.climbMain.getDeviceId())
        self.climbEncoder: rev.SparkRelativeEncoder = self.climbMain.getEncoder()
        # self.climbMain.set(0)
        # self.climbEncoder.setPosition(0)

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

    def doDeploy(self):
        self.targetClimbDistance = self.kDeployTargetDistance

    def doPull(self):
        self.targetClimbDistance = self.kPullTargetDistance

    def atLimit(self) -> bool:
        return self.climb_limitswitch.get()

    def atGoal(self) -> bool:
        return tools.float_equal(self.climbEncoder.getPosition(), self.targetClimbDistance, 0.05)

    def execute(self):
        if self.atGoal():
            self.climbMain.set(0)
        else:
            climbSpeed = self.climbPID.calculate(
                self.climbEncoder.getPosition(), self.targetClimbDistance
            )
            climbSpeed = tools.clamp(climbSpeed, -self.kClimbMaxSpeed, self.kClimbMaxSpeed)
            # climbSpeed = tools.clamp(climbSpeed, -1.0, 1.0)
            self.climbMain.set(climbSpeed)

class ActionClimbDeploy(StateMachine):
    climb: Climb
    lift : Lift
    chariot : Chariot

    @state(first=True)
    def move_lift_and_chariot(self, initial_call: bool):
        if initial_call:
            self.lift.go_zero()
            self.chariot.move_front()
            return

        if self.lift.atGoal() and self.chariot.target_reached:
            self.lift.lock() # safety
            self.next_state("deploy")

    @state
    def deploy(self, initial_call: bool):
        self.climb.doDeploy()
        if self.climb.atGoal():
            self.done()

class ActionClimbPull(StateMachine):
    climb: Climb

    @state(first=True)
    def pull_climb(self, initial_call: bool):
        self.climb.doPull()
        if self.climb.atGoal() or self.climb.atLimit():
            self.done()
