import wpimath.units as units

from typing import override

from magicbot import state, tunable
from magicbot.state_machine import AutonomousStateMachine

from components.swervedrive import SwerveDrive, kMaxSpeed
from components.robot_actions import ActionShoot
from components.lift import LiftTarget

kRunAutoSpeedFactor : float = 0.5

class RunAuto(AutonomousStateMachine):
    DEFAULT: bool = True
    MODE_NAME: str = "RunAuto"
    PATH_NAME: str = "RunAuto"

    drivetrain: SwerveDrive
    actionShoot : ActionShoot

    kDistanceToTravel = tunable(units.inchesToMeters(60)) # From CAD file
    kLiftTarget = LiftTarget.L3

    @override
    def on_enable(self) -> None:
        print("Run auto mode: on_enable recording initial pose")
        self.initialPose = self.drivetrain.getPose()
        super().on_enable()

    @state(first=True)
    def move(self):
        print("Run auto mode: moving state")
        distanceTraveled = self.drivetrain.getPose().translation().distance(self.initialPose.translation())
        if distanceTraveled >= self.kDistanceToTravel:
            self.next_state("shoot")
        else:
            self.drivetrain.drive(kMaxSpeed * kRunAutoSpeedFactor, 0, 0, True)

    @state
    def shoot(self):
        print("Run auto mode: shooting state" )
        if not self.actionShoot.is_executing:
            print("Run auto mode: starting lift")
            self.actionShoot.start(self.kLiftTarget, "move_lift_auto")
        else:
            print("Run auto mode: keep shoot going")
            self.actionShoot.engage()
            if not self.actionShoot.is_executing:
                self.done()

    @override
    def done(self) -> None:
        print("Run auto mode: we're done")
        super().done()
