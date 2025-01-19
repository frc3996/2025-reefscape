import json
import os

from magicbot import tunable
from magicbot.state_machine import AutonomousStateMachine, StateMachine, state
from pathplannerlib.geometry_util import flipFieldPose
from wpimath import geometry

from common import tools
from common.path_helper import PathHelper
from components.robot_actions import (ActionIntake, ActionStow)
from components.swervedrive import SwerveDrive


def get_next_auto_command(auto_name):
    if not auto_name.endswith(".auto"):
        auto_name += ".auto"
    file = os.path.join(
        os.path.dirname(__file__),
        "..",
        "deploy",
        "pathplanner",
        "autos",
        auto_name,
    )

    with open(file, "r") as f:
        auto_json = json.load(f)

    return auto_json


class RunAuto(AutonomousStateMachine):
    MODE_NAME = "DO_NOT_USE"
    PATH_NAME = ""
    look_only = False

    # Tunables
    path_kp = tunable(5)
    path_ki = tunable(0)
    path_kd = tunable(0)
    path_profile = tunable(5)

    # Injection
    drivetrain: SwerveDrive
    actionStow: ActionStow

    current_command = {}

    # def flipFieldPos(pos: Translation2d) -> Translation2d:
    #     """
    #     Flip a field position to the other side of the field, maintaining a blue alliance origin

    #     :param pos: The position to flip
    #     :return: The flipped position
    #     """
    #     return Translation2d(FIELD_LENGTH - pos.X(), pos.Y())

    @state(first=True)
    def get_auto_mode(self):
        json_commands = get_next_auto_command(self.PATH_NAME)
        self.auto_commands = json_commands["command"]["data"]["commands"]  # type: list
        reset_pose = json_commands["startingPose"]
        new_pose = geometry.Pose2d(
            reset_pose["position"]["x"],
            reset_pose["position"]["y"],
            geometry.Rotation2d.fromDegrees(reset_pose["rotation"]),
        )
        if tools.is_red():
            new_pose = flipFieldPose(new_pose)
        self.drivetrain.resetPose(new_pose)
        self.next_state("execute_next_command")

    @state()
    def execute_next_command(self):
        if self.auto_commands:
            self.current_command = self.auto_commands.pop(0)
        else:
            self.done()
        if (
            self.current_command["type"] == "named"
            and self.current_command["data"]["name"] == "enable_look_only"
        ):
            self.look_only = True
            return
        elif (
            self.current_command["type"] == "named"
            and self.current_command["data"]["name"] == "disable_look_only"
        ):
            self.look_only = False
            return
        self.next_state(self.current_command["type"])

    @state
    def named(self, initial_call):
        command_name = self.current_command["data"]["name"]
        command = getattr(self, command_name)  # type: StateMachine
        if not command.is_executing and not initial_call:
            self.next_state("execute_next_command")
        else:
            command.engage()

    @state
    def path(self, initial_call):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        if initial_call:
            path_name = self.current_command["data"]["pathName"]
            self.auto_path = PathHelper(
                self.drivetrain,
                path_name,
                kp=self.path_kp,
                ki=self.path_ki,
                kd=self.path_kd,
                profile_kp=self.path_profile,
            )
            self.auto_path.init_path()

        if self.look_only:
            self.auto_path.target_end_angle()
            self.drivetrain.permanent_snap = True
            if self.auto_path.robot_reached_end_angle(acceptable_angle_error=20):
                self.next_state("execute_next_command")
            return

        self.auto_path.auto_move()
        self.drivetrain.permanent_snap = True
        if self.auto_path.robot_reached_end_position():
            self.next_state("execute_next_command")

class taxi(RunAuto):
    DEFAULT=True
    MODE_NAME = "taxi"
    PATH_NAME = "taxi"