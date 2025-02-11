import math
from typing import Optional, override

import wpilib
from commands2 import Subsystem
from hal import initialize
from magicbot.state_machine import StateMachine, StateRef, state
from pathplannerlib.auto import AutoBuilder, FollowPathCommand
from pathplannerlib.commands import PathfindingCommand
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.path import (GoalEndState, IdealStartingState,
                                 PathConstraints, PathPlannerPath,
                                 PathPlannerTrajectory)
from pathplannerlib.pathfinding import Pathfinding
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import degreesToRadians

from components import swervedrive


class ActionPathPlannerV3(StateMachine):
    drivetrain: swervedrive.SwerveDrive

    class SwerveSubsystem(Subsystem):
        pass

    def __init__(self) -> None:
        super().__init__()
        self.command: FollowPathCommand | None = None

    def setup(self):
        # Do all subsystem initialization here ...

        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        self.robotConfig = RobotConfig.fromGUISettings()

        # Create the constraints to use while pathfinding
        self.constraints = PathConstraints(
            3.0, 4.0, degreesToRadians(540), degreesToRadians(720)
        )

        self.controller: PPHolonomicDriveController = PPHolonomicDriveController(
            PIDConstants(5.0, 0.0, 0.1),  # Translation PID constants
            PIDConstants(5.0, 0.0, 0.1),  # Rotation PID constants
        )

        self.subsystem: Subsystem = self.SwerveSubsystem()

        # Start the pathfinding thread
        Pathfinding.ensureInitialized()

    def getPose(self):
        # Prevent the return of a negative value. It breaks pathplanner
        pos = self.drivetrain.getPose()
        return Pose2d(
            max(min(pos.X(), 17), 0),
            max(min(pos.Y(), 7.6), 0),
            pos.rotation(),
        )

    @state(first=True)
    def initial(self):
        self.command = None
        self.next_state_now("generate")

    @state()
    def generate(self):
        # Since we are using a holonomic drivetrain, the rotation component of
        # this pose represents the goal holonomic rotation
        start: Pose2d = self.getPose()
        end = Pose2d(2, 2, Rotation2d(math.pi / 2))
        self.goalEndState: GoalEndState = GoalEndState(0.0, end.rotation())
        Pathfinding.setStartPosition(start.translation())
        Pathfinding.setGoalPosition(end.translation())

        self.next_state("wait_for_path")

    @state()
    def wait_for_path(self):
        if Pathfinding.isNewPathAvailable():
            self.currentPath: PathPlannerPath = Pathfinding.getCurrentPath(
                self.constraints, self.goalEndState
            )

            if self.currentPath is not None:
                self.next_state("follow")

    @state()
    def follow(self):
        self.command: FollowPathCommand = FollowPathCommand(
            self.currentPath,
            self.getPose,
            self.drivetrain.getChassisSpeeds,
            lambda speeds, feedforwards: self.drivetrain.drive_auto(speeds),
            self.controller,
            self.robotConfig,
            lambda: False,
            self.subsystem,
        )

        self.command.initialize()
        self.next_state("exec")

    @state()
    def exec(self):
        print(f"ERROR: {self.controller.getPositionalError()}")
        if self.controller.getPositionalError() > 1:
            self.next_state("generate")
        elif self.command.isFinished():
            print("FINISH")
            self.next_state("finish")

    @state()
    def finish(self, initial_call: bool):
        if initial_call:
            self.command.end(False)

    @override
    def execute(self) -> None:
        if self.command:
            self.command.execute()
        return super().execute()

    @override
    def done(self) -> None:
        if self.is_executing and self.command:
            print("CANCELING")
            self.command.end(True)
            self.command = None
        return super().done()


#
# # Load the path we want to pathfind to and follow
#             path = PathPlannerPath.fromPathFile('Example Human Player Pickup');
#
# # Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
#             constraints = PathConstraints(
#                 3.0, 4.0,
#                 degreesToRadians(540), degreesToRadians(720)
#             )
#
# # Since AutoBuilder is configured, we can use it to build pathfinding commands
#             pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
#                 path,
#                 constraints,
#                 rotation_delay_distance=3.0 # Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
#             )
# @state(first=True)
# def initial(self):
#     # XXX: Probably not necessary
#     self.subsystem.periodic()
#
#     # Create a list of waypoints from poses. Each pose represents one
#     # waypoint.
#     # The rotation component of the pose should be the direction of travel.
#     # Do not use holonomic rotation.
#     waypoints = PathPlannerPath.waypointsFromPoses(
#         [
#             Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
#             Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
#             Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)),
#         ]
#     )
#     # The constraints for this path.
#     constraints = PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi)
#
#     # You can also use unlimited constraints, only limited by motor torque
#     # and nominal battery voltage
#     # constraints = PathConstraints.unlimitedConstraints(12.0)
#
#     # Create the path using the waypoints created above
#     path = PathPlannerPath(
#         waypoints,
#         constraints,
#         # The ideal starting state, this is only relevant for pre-planned
#         # paths, so can be None for on-the-fly paths.
#         None,
#         # Goal end state. You can set a holonomic rotation here. If using a
#         # differential drivetrain, the rotation will have no effect.
#         GoalEndState(0.0, Rotation2d.fromDegrees(-90)),
#     )
#
#     # Prevent the path from being flipped if the coordinates are already correct
#     path.preventFlipping = True
