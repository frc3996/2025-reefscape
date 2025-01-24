import math
import os

import wpilib
from pathplannerlib.config import ModuleConfig, RobotConfig
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.trajectory import PathPlannerTrajectory
from wpimath import controller, geometry, kinematics, trajectory

import constants
from common import tools
from components.swervedrive import SwerveDrive


class PathHelper:
    kp: int = 1
    ki: int = 0
    kd: int = 0
    profile_kp: int = 1

    def __init__(
        self,
        drivetrain,
        path_name,
        kp=1,
        ki=0,
        kd=0,
        profile_kp=1,
    ):
        self.drivetrain: SwerveDrive = drivetrain
        self.timer: wpilib.Timer = wpilib.Timer()
        self.timer.start()

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.profile_kp = profile_kp

        self.path: PathPlannerPath = PathPlannerPath.fromPathFile(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "deploy",
                "pathplanner",
                "paths",
                path_name,
            )
        )
        if tools.is_red():
            self.path = self.path.flipPath()

    def init_path(self, force_robot_starting_position: bool = False):
        """
        force_robot_starting_position: useful when the starting position
                                       doesn't allow us to get position from
                                       the April Tags
        """
        robot_config = RobotConfig.fromGUISettings()
        self.trajectory: PathPlannerTrajectory = self.path.generateTrajectory(
            kinematics.ChassisSpeeds(0, 0, 0), geometry.Rotation2d(), robot_config
        )
        self.controller: controller.HolonomicDriveController = (
            controller.HolonomicDriveController(
                controller.PIDController(self.kp, self.ki, self.kd),
                controller.PIDController(self.profile_kp, 0, 0),
                controller.ProfiledPIDControllerRadians(
                    1,
                    0,
                    0,
                    trajectory.TrapezoidProfileRadians.Constraints(
                        constants.MAX_ANGULAR_VEL, constants.MAX_ANGULAR_ACCEL
                    ),
                ),
            )
        )
        self.controller.setEnabled(True)
        self.timer.reset()

        if force_robot_starting_position is False:
            return
        reset_pose = self.trajectory.sample(0).pose
        new_pose = geometry.Pose2d(
            reset_pose.X(),
            reset_pose.Y(),
            self.drivetrain.get_odometry_angle(),
        )
        self.drivetrain.resetPose(new_pose)

    def move_to_end(self):
        # If odometry is bad, compute now + 0.02 as goal and now as current position
        # fake_pose = self.trajectory.sample(self.timer.get())
        # goal = self.trajectory.sample(self.timer.get() + 0.02)
        # adjustedSpeeds = self.controller.calculate(fake_pose.getTargetHolonomicPose(), goal.getTargetHolonomicPose(), goal.velocityMps, goal.heading)
        goal = self.trajectory.getEndState()

        target_rotation = self.path.getGoalEndState().rotation

        goal.targetHolonomicRotation = geometry.Rotation2d(0)
        current = self.drivetrain.get_odometry_pose()
        current = geometry.Pose2d(current.X(), current.Y(), geometry.Rotation2d(0))
        adjustedSpeeds = self.controller.calculate(
            current, goal.pose, 0, geometry.Rotation2d(0)
        )
        # speed = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(adjustedSpeeds.vx, adjustedSpeeds.vy, adjustedSpeeds.omega, goal.heading)
        self.drivetrain.set_field_relative_automove_value(
            -adjustedSpeeds.vx, -adjustedSpeeds.vy
        )
        print(adjustedSpeeds.vx, adjustedSpeeds.vy)
        self.drivetrain.snap_angle(target_rotation)

    def target_end_angle(self):
        target_rotation = self.path.getGoalEndState().rotation
        self.drivetrain.snap_angle(target_rotation)

    def auto_move(self):
        # If odometry is bad, compute now + 0.02 as goal and now as current position
        # fake_pose = self.trajectory.sample(self.timer.get())
        # goal = self.trajectory.sample(self.timer.get() + 0.02)
        # adjustedSpeeds = self.controller.calculate(fake_pose.getTargetHolonomicPose(), goal.getTargetHolonomicPose(), goal.velocityMps, goal.heading)
        goal = self.trajectory.sample(self.timer.get())

        target_rotation = self.path.getGoalEndState().rotation

        goal.targetHolonomicRotation = geometry.Rotation2d(0)
        current = self.drivetrain.get_odometry_pose()
        current = geometry.Pose2d(current.X(), current.Y(), geometry.Rotation2d(0))
        adjustedSpeeds = self.controller.calculate(
            current, goal.pose, 0, geometry.Rotation2d(0)
        )
        # speed = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(adjustedSpeeds.vx, adjustedSpeeds.vy, adjustedSpeeds.omega, goal.heading)
        self.drivetrain.set_field_relative_automove_value(
            -adjustedSpeeds.vx, -adjustedSpeeds.vy
        )
        self.drivetrain.snap_angle(target_rotation)

    def path_reached_end(self):
        return self.timer.get() >= self.trajectory.getTotalTimeSeconds()

    def distance_to_end(self):
        goal = self.trajectory.getEndState()
        goal.targetHolonomicRotation = geometry.Rotation2d(0)
        goal = goal.pose

        current = self.drivetrain.get_odometry_pose()
        current = geometry.Pose2d(current.X(), current.Y(), geometry.Rotation2d(0))
        distance = math.sqrt(pow(current.x - goal.x, 2) + pow(current.y - goal.y, 2))

        return distance

    def robot_reached_end_position(
        self, acceptable_distance_error: float = 0.1, acceptable_angle_error: float = 2
    ):
        goal = self.trajectory.getEndState()
        target_rotation = self.path.getGoalEndState().rotation
        goal.targetHolonomicRotation = geometry.Rotation2d(0)
        goal = goal.pose

        current = self.drivetrain.get_odometry_pose()
        current = geometry.Pose2d(current.X(), current.Y(), geometry.Rotation2d(0))
        distance = math.sqrt(pow(current.x - goal.x, 2) + pow(current.y - goal.y, 2))

        angle_error = target_rotation - self.drivetrain.get_odometry_angle()
        angle_error = abs(angle_error.degrees())

        if distance > acceptable_distance_error:
            return False
        if angle_error > acceptable_angle_error:
            return False

        return True

    def robot_reached_end_angle(self, acceptable_angle_error: float = 2):
        target_rotation = self.path.getGoalEndState().rotation
        angle_error = target_rotation - self.drivetrain.get_odometry_angle()
        angle_error = abs(angle_error.degrees())
        print("ANGLE_ERROR", angle_error, acceptable_angle_error)
        if angle_error > acceptable_angle_error:
            return False

        return True
