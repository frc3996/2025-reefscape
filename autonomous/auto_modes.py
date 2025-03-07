import wpimath.units as units

from typing import override

from magicbot import state, tunable
from magicbot.state_machine import AutonomousStateMachine

from components.swervedrive import SwerveDrive
from components.robot_actions import ActionShoot
from components.lift import LiftTarget

class RunAuto(AutonomousStateMachine):
    DEFAULT: bool = True
    MODE_NAME: str = "RunAuto"
    PATH_NAME: str = "RunAuto"

    drivetrain: SwerveDrive
    actionShoot : ActionShoot

    kDistanceToTravel = tunable(units.inchesToMeters(60)) # From CAD file
    kTravelSpeed = tunable(0.6)
    kLiftTarget = LiftTarget.L3

    @override
    def on_enable(self) -> None:
        self.initialPose = self.drivetrain.getPose()
        self.lockedDone = False
        super().on_enable()

    @state(first=True)
    def run(self):
        if self.lockedDone:
            return
        distanceTraveled = self.drivetrain.getPose().translation().distance(self.initialPose.translation())
        if distanceTraveled >= self.kDistanceToTravel:
            self.drivetrain.drive(0, 0, 0, True)
            self.actionShoot.start(self.kLiftTarget, "move_lift_auto")
            if not self.actionShoot.is_executing:
                self.done()
        else:
            self.drivetrain.drive(self.kTravelSpeed, 0, 0, True)

    @override
    def done(self) -> None:
        self.lockedDone = True
        super().done()
