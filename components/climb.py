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

from threading import ExceptHookArgs
from typing import override

import rev
import wpilib
import wpimath.controller
import wpimath.trajectory
from magicbot import StateMachine, tunable, will_reset_to
from magicbot.state_machine import state, timed_state

import constants

# - sequence avec limit switch

# Climb intake
# Limit switch (2x), pur bonne position
# Activer Piston, limitswitch no 2 pour 2
# Activer 2 moteurs pour grimper
# ratio 125 pour 1, 3.25in pour la roue,


class Climb:
    pneumaticHub: wpilib.PneumaticHub

    def __init__(self):
        self.kGuideMaxSpeed: float = 3750
        self.kGuideMaxAccel: float = 2000
        self.kGuideSpeed: float = 3500

        self.kClimbMaxSpeed: float = 3750
        self.kClimbMaxAccel: float = 2000
        self.kClimbDistance: float = 10

        self._climbDistance: float = 0
        self._guideSpeed: float = 0

    def setup(self) -> None:
        # Declare a Pneumatic Hub or PCM
        # self.pneumaticHub = wpilib.PneumaticHub()

        self.guideMotor: rev.SparkMax = rev.SparkMax(
            constants.CANIds.CLIMB_GUIDE_MOTOR, rev.SparkMax.MotorType.kBrushless
        )
        self.guideEncoder: rev.SparkRelativeEncoder = self.guideMotor.getEncoder()
        self.guidePID: wpimath.controller.ProfiledPIDController = (
            wpimath.controller.ProfiledPIDController(
                1,
                0,
                0,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    # self.kGuideMaxSpeed, self.kGuideMaxAccel
                    1,
                    0.5,
                ),
            )
        )

        self.climbMaster: rev.SparkMax = rev.SparkMax(
            constants.CANIds.CLIMB_RAISE_MOTOR_MAIN, rev.SparkMax.MotorType.kBrushless
        )
        self.climbEncoder: rev.SparkRelativeEncoder = self.climbMaster.getEncoder()

        # Climb motors
        self.climbSlave: rev.SparkMax = rev.SparkMax(
            constants.CANIds.CLIMB_RAISE_MOTOR_FOLLOWER,
            rev.SparkMax.MotorType.kBrushless,
        )

        self.climbPID: wpimath.controller.ProfiledPIDController = (
            wpimath.controller.ProfiledPIDController(
                1,
                0,
                0,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    # self.kClimbMaxSpeed, self.kClimbMaxAccel
                    1,
                    0.5,
                ),
            )
        )

        # Set position conversion factor, revolution to distance
        # climbMasterConfig = rev.SparkBaseConfig()
        # _ = climbMasterConfig.encoder.positionConversionFactor(1)
        # _ = self.climbMaster.configure(
        #     climbMasterConfig,
        #     rev.SparkMax.ResetMode.kResetSafeParameters,
        #     rev.SparkMax.PersistMode.kPersistParameters,
        # )

        # Make it into a follower
        climbSlaveConfig = rev.SparkBaseConfig()
        _ = climbSlaveConfig.follow(constants.CANIds.CLIMB_RAISE_MOTOR_MAIN, False)
        _ = self.climbSlave.configure(
            climbSlaveConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        # Limit switches
        self.cage_in_limitswitch_1: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.CLIMB_CAGE_IN_LIMITSWITCH_1
        )
        self.cage_in_limitswitch_2: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.CLIMB_CAGE_IN_LIMITSWITCH_2
        )
        self.piston_out_limitswitch_1: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.CLIMB_PISTON_OUT_LIMITSWITCH_1
        )
        self.piston_out_limitswitch_2: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.CLIMB_PISTON_OUT_LIMITSWITCH_2
        )

        # Declare solenoid
        # XXX: Don't access this directly unless you know what you're doing
        self._solenoid: wpilib.Solenoid = wpilib.Solenoid(
            1, wpilib.PneumaticsModuleType.REVPH, 1
        )

    def startGuide(self):
        self._guideSpeed = self.kGuideSpeed

    def stopGuide(self):
        self._guideSpeed = 0

    def climbUp(self):
        # Start at zero
        self.climbEncoder.setPosition(0)
        self._climbDistance = self.kClimbDistance
        self.climbPID.reset(0)

    def climbDown(self):
        self._climbDistance = 0

    def isCageIn(self):
        return self.cage_in_limitswitch_1.get() or self.cage_in_limitswitch_2.get()

    def isCageSqueezed(self):
        return (
            self.piston_out_limitswitch_1.get() and self.piston_out_limitswitch_2.get()
        )

    def squeezeCage(self):
        # Let's make damm sure we never toggle that without the limit switch
        if self.isCageIn():
            self._solenoid.set(True)

    def releaseCage(self):
        # Let's make damm sure we never toggle that without the limit switch
        self._solenoid.set(False)

    def execute(self):
        # Position PID for climber
        climbOutput = self.climbPID.calculate(
            self.climbEncoder.getPosition(), self._climbDistance
        )

        self.climbMaster.set(min(max(climbOutput, -1), 1))
        print(f"climbOutput {self.climbMaster.get()}")

        # Velocity PID for guide
        if self._guideSpeed == 0:
            # Just deactivate
            self.guideMotor.set(0)
            self.climbPID.reset(0)
        else:
            guideOutput = self.guidePID.calculate(
                self.guideEncoder.getVelocity(), self._guideSpeed
            )
            self.guideMotor.set(min(max(guideOutput, -1), 1))
        print(f"guideOutput {self.guideMotor.get()}")


class ActionClimb(StateMachine):
    climb: Climb

    @state(first=True)
    def startClimb(self):
        # Move to cage..?
        self.next_state("fetchCage")

    @state
    def fetchCage(self):
        self.climb.startGuide()
        # Move toward cage??
        if self.climb.isCageIn():
            print("CAGE IS IN")
            self.next_state("squeezeCage")

    @state
    def squeezeCage(self):
        self.climb.stopGuide()
        self.climb.squeezeCage()
        if self.climb.isCageSqueezed():
            print("CAGE IS SQUEEZED")
            self.next_state("climbCage")

    @state()
    def climbCage(self, initial_call: bool):
        # Moteur à 100%, à l'infini
        if initial_call:
            self.climb.climbUp()

    @override
    def done(self):
        self.climb.climbDown()
        self.climb.releaseCage()
        self.climb.stopGuide()
        super().done()
