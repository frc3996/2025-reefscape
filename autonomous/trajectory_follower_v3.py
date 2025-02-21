import math
from typing import Optional, override

import wpilib
import wpimath.units
from commands2 import Subsystem
from hal import initialize
from magicbot import timed_state
from magicbot.state_machine import StateMachine, StateRef, state
from pathplannerlib.auto import AutoBuilder, FollowPathCommand
from pathplannerlib.commands import PathfindingCommand
from pathplannerlib.config import (DCMotor, ModuleConfig, PIDConstants,
                                   RobotConfig, Translation2d)
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.path import (GoalEndState, IdealStartingState,
                                 PathConstraints, PathPlannerPath,
                                 PathPlannerTrajectory, PathPoint,
                                 RotationTarget)
from pathplannerlib.pathfinding import Pathfinding
from wpilib import DriverStation
from wpimath._controls._controls import controller
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import degreesToRadians

import robot
from components import swervedrive


class ActionPathPlannerV3(StateMachine):
    drivetrain: swervedrive.SwerveDrive

    class SwerveSubsystem(Subsystem):
        pass

    def __init__(self) -> None:
        super().__init__()
        self.command: FollowPathCommand | None = None

        self.start: Pose2d = Pose2d(0, 0, 0)
        self.end = Pose2d(0, 0, 0)

        # Always finish a path at the right rotation and speed of 0
        self.goalEndState: GoalEndState = GoalEndState(0.0, self.end.rotation())

    def setup(self):
        # Do all subsystem initialization here ...

        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        # self.robotConfig: RobotConfig = RobotConfig.fromGUISettings()
        self.robotConfig: RobotConfig = RobotConfig(
            70,
            6.883,
            ModuleConfig(
                wpimath.units.inchesToMeters(2.0),
                swervedrive.kMaxSpeed,
                1.0,
                DCMotor.falcon500().withReduction(6.12),
                17,
                1,
            ),
            moduleOffsets=[
                Translation2d(0.381, 0.381),
                Translation2d(0.381, -0.381),
                Translation2d(-0.381, 0.381),
                Translation2d(-0.381, -0.381),
            ],
        )

        # Create the constraints to use while pathfinding
        # self.constraints = PathConstraints(
        #     3.0, 4.0, degreesToRadians(540), degreesToRadians(720)
        # )
        self.constraints: PathConstraints = PathConstraints.unlimitedConstraints(
            wpilib.RobotController.getBatteryVoltage()
        )

        self.controller: PPHolonomicDriveController = PPHolonomicDriveController(
            PIDConstants(7.0, 0.0, 0.0),  # Translation PID constants
            PIDConstants(7.0, 0.0, 0.0),  # Rotation PID constants
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
        if self.command:
            self.command.end(True)
            self.command = None
        self.next_state_now("generate")

    @state()
    def generate(self):
        print("RECALCULATING")
        # Since we are using a holonomic drivetrain, the rotation component of
        # this pose represents the goal holonomic rotation
        self.start = self.getPose()
        self.end = Pose2d(2, 2, Rotation2d(math.pi / 2))

        # If X and Y are exact, lets just perform rotation
        if self.start.X() == self.end.X() and self.start.Y() == self.end.Y():
            print("Already at position")
            self.next_state("direct")

        Pathfinding.setStartPosition(self.start.translation())
        Pathfinding.setGoalPosition(self.end.translation())

        self.next_state("wait_for_path")

    # @state
    @timed_state(duration=1, next_state="direct")
    def wait_for_path(self):
        print("WAITING")
        if Pathfinding.isNewPathAvailable():
            self.currentPath: PathPlannerPath = Pathfinding.getCurrentPath(
                self.constraints, self.goalEndState
            )

            if self.currentPath is not None:
                self.next_state("follow")

    @state()
    def direct(self):
        print("DIRECT PATH")
        self.currentPath = PathPlannerPath.fromPathPoints(
            [
                PathPoint(
                    self.end.translation(),
                    RotationTarget(0, self.end.rotation()),
                    self.constraints,
                    0.01,
                )
            ],
            self.constraints,
            self.goalEndState,
        )
        self.next_state("follow")

    @state()
    def follow(self):
        print("START")
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

        try:
            self.command.initialize()
        except:
            self.command = None
        self.next_state("exec")

    @state()
    def exec(self):
        print("FOLLOWING")
        # print(f"ERROR: {self.controller.getPositionalError()}")
        if (
            self.controller.getPositionalError() < 0.02
            and self.drivetrain.getVelocity() < 0.5
            and self.command.isFinished()
        ):
            # We're done
            print("FINISH")
            self.next_state("finish")
        elif self.controller.getPositionalError() > 0.5:
            # Strayed too far, recalculate the path
            self.next_state("generate")

    @state()
    def finish(self, initial_call: bool):
        if initial_call:
            self.command.end(False)
            self.command = None

    @override
    def execute(self) -> None:
        if self.command and not self.command.isFinished():
            self.command.execute()
        return super().execute()

    @override
    def done(self) -> None:
        if self.is_executing and self.command:
            print("CANCELING")
            self.command.end(True)
            self.command = None
        return super().done()


# class Drive(Subsystem):
#     def __init__(self):
#         super().__init__()
#
#         # Other subsystem initialization code
#         # ...
#
#         self.x_controller = PIDController(10.0, 0.0, 0.0)
#         self.y_controller = PIDController(10.0, 0.0, 0.0)
#         self.heading_controller = PIDController(7.5, 0.0, 0.0)
#
#         self.heading_controller.enableContinuousInput(-math.pi, math.pi)
#
#     def follow_trajectory(self, sample):
#         # Get the current pose of the robot
#         pose = self.get_pose()
#
#         # Generate the next speeds for the robot
#         speeds = ChassisSpeeds(
#             sample.vx + self.x_controller.calculate(pose.X(), sample.x),
#             sample.vy + self.y_controller.calculate(pose.Y(), sample.y),
#             sample.omega + self.heading_controller.calculate(pose.rotation().radians(), sample.heading)
#         )
#
#         # Apply the generated speeds
#         self.drive_field_relative(speeds)

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
