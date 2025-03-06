from typing import override

from magicbot import state
from magicbot.state_machine import AutonomousStateMachine

from components.robot_actions import ActionCycleAutonomous
from components.swervedrive import SwerveDrive
import common.tools

class RunAuto(AutonomousStateMachine):
    DEFAULT: bool = True
    MODE_NAME: str = "RunAuto"
    PATH_NAME: str = "RunAuto"
    actionCycleAutonomous: ActionCycleAutonomous
    drivetrain: SwerveDrive

    @state(first=True)
    def run(self):
        """
        Just call the autonmous statemachine, then call drivetrain.drive like
        in teleopPeriodic

        Make sure the drive is executed at the end!
        """
        # self.actionCycleAutonomous.engage()
        self.drivetrain.drive(0.2, 0, 0, True)
