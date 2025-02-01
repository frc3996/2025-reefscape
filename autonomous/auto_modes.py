import json
import os
from pathlib import Path
from typing import cast, final

from magicbot import tunable
from magicbot.state_machine import AutonomousStateMachine, StateMachine, state
from pathplannerlib.util import FlippingUtil
from wpimath import geometry

from common import tools
from common.path_helper import PathHelper
from common.pathplanner import Command, CommandType, PathPlannerAuto
from components.swervedrive import SwerveDrive


def get_next_auto_command(auto_name: str) -> PathPlannerAuto:
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
        auto_json: PathPlannerAuto = json.load(f)

    return auto_json


class RunAuto(AutonomousStateMachine):
    MODE_NAME: str = "DO_NOT_USE"
    PATH_NAME: str = ""
    look_only: bool = False

    # Tunables
    path_kp: tunable[int] = tunable(5)
    path_ki: tunable[int] = tunable(0)
    path_kd: tunable[int] = tunable(0)
    path_profile: tunable[int] = tunable(5)

    # Injection
    drivetrain: SwerveDrive

    # def flipFieldPos(pos: Translation2d) -> Translation2d:
    #     """
    #     Flip a field position to the other side of the field, maintaining a blue alliance origin

    #     :param pos: The position to flip
    #     :return: The flipped position
    #     """
    #     return Translation2d(FIELD_LENGTH - pos.X(), pos.Y())

    @state(first=True)
    def get_auto_mode(self):
        json_commands: PathPlannerAuto = get_next_auto_command(self.PATH_NAME)
        self.auto_commands: list[CommandType] = json_commands["command"]["data"][
            "commands"
        ]
        # reset_pose = json_commands["startingPose"]
        # new_pose = geometry.Pose2d(
        #     reset_pose["position"]["x"],
        #     reset_pose["position"]["y"],
        #     geometry.Rotation2d.fromDegrees(reset_pose["rotation"]),
        # )
        # if tools.is_red():
        #     new_pose = FlippingUtil.flipFieldPose(new_pose)
        # self.drivetrain.resetPose(new_pose)
        self.next_state("execute_next_command")

    @state()
    def execute_next_command(self):
        if self.auto_commands:
            self.current_command: CommandType = self.auto_commands.pop(0)
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
    def named(self, initial_call: bool):
        command_name = self.current_command["data"]["name"]
        command: StateMachine = getattr(self, command_name)
        if not command.is_executing and not initial_call:
            self.next_state("execute_next_command")
        else:
            command.engage()

    @state
    def path(self, initial_call: bool):
        """First state -- waits until shooter is ready before going to the
        next action in the sequence"""
        if initial_call:
            path_name = self.current_command["data"]["pathName"]
            self.auto_path: PathHelper = PathHelper(
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


# class taxi(RunAuto):
#     DEFAULT: bool = True
#     MODE_NAME: str = "Taxi"
#     PATH_NAME: str = "Taxi"
