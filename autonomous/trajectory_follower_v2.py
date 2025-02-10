import math
from typing import final, override

import ntcore
import wpilib
from magicbot import AutonomousStateMachine, StateMachine
from magicbot.state_machine import StateRef, state
from pathplannerlib.config import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.path import (GoalEndState, PathConstraints,
                                 PathPlannerTrajectory)
# from pathplannerlib.path import GoalEndState, PathConstraints, PathPlannerTrajectory
from pathplannerlib.pathfinding import Pathfinding
from wpimath._controls._controls import trajectory
from wpimath.controller import (HolonomicDriveController,
                                LTVUnicycleController, PIDController,
                                ProfiledPIDController,
                                ProfiledPIDControllerRadians)
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.trajectory import (Trajectory, TrajectoryConfig,
                                TrajectoryGenerator, TrapezoidProfile,
                                TrapezoidProfileRadians)
from wpimath.units import degreesToRadians

from components import swervedrive
from components.swervedrive import SwerveDrive


@final
class TrajectoryFollowerV2(StateMachine):
    """
    An autonomous state machine that generates and follows a trajectory.
    """

    MODE_NAME = "TrajectoryFollowerV2"
    DEFAULT = False  # Only run when explicitly selected.

    # Inject the SwerveDrive subsystem.
    drivetrain: SwerveDrive

    # Inject the control loop wait time
    dt: float

    def setup(self) -> None:
        super().__init__()

        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        self.robotConfig = RobotConfig.fromGUISettings()

        # Create the constraints to use while pathfinding
        self.constraints = PathConstraints(
            3.0, 4.0, degreesToRadians(540), degreesToRadians(720)
        )

        Pathfinding.ensureInitialized()
        self.controller: PPHolonomicDriveController = PPHolonomicDriveController(
            PIDConstants(5.0, 0.0, 0.0),  # Translation PID constants
            PIDConstants(5.0, 0.0, 0.0),  # Rotation PID constants
        )

        self.startTime = 0

        # LOG
        # Publish the trajectory for visualization.
        nt = ntcore.NetworkTableInstance.getDefault()
        self.trajectoryPub = nt.getStructArrayTopic(
            f"/AdvantageScope/Trajectory", Pose2d
        ).publish()

    def generate_trajectory(
        self, start: Pose2d, end: Pose2d
    ) -> PathPlannerTrajectory | None:
        Pathfinding.setStartPosition(start.translation())
        Pathfinding.setGoalPosition(end.translation())

        currentPose = self.drivetrain.getPose()
        # XXX: Bug in pathplanner, can't be negative
        currentPose = Pose2d(
            currentPose.X() if currentPose.X() > 0 else 0,
            currentPose.Y() if currentPose.Y() > 0 else 0,
            currentPose.rotation(),
        )

        currentSpeeds = self.drivetrain.getChassisSpeeds()

        if Pathfinding.isNewPathAvailable():
            currentPath = Pathfinding.getCurrentPath(
                self.constraints, GoalEndState(0.0, end.rotation())
            )
        else:
            return None

        assert currentPath
        trajectory = PathPlannerTrajectory(
            currentPath, currentSpeeds, currentPose.rotation(), self.robotConfig
        )

        # Publish the trajectory for visualization.
        sample_count = 10
        delta_t = trajectory.getTotalTimeSeconds() / sample_count
        trajectory_poses = [
            trajectory.sample(delta_t * i).pose for i in range(sample_count)
        ]
        self.trajectoryPub.set(trajectory_poses)

        return trajectory

    @state(first=True)
    def follow_trajectory(self, initial_call):
        """
        This state is called repeatedly during autonomous.
        It computes the desired chassis speeds to follow the trajectory.
        """

        # Get the real robot position
        currentTime = wpilib.Timer.getFPGATimestamp()
        start: Pose2d = self.drivetrain.getPose()
        end = Pose2d(2, 2, Rotation2d(math.pi / 2))

        # Calculate the trajectory
        if (
            initial_call
            or self.trajectory.getTotalTimeSeconds() < currentTime - self.startTime
        ):
            trajectory = self.generate_trajectory(start, end)
            if self.trajectory is None:
                return
            self.trajectory = trajectory
            self.startTime = currentTime

        # Sample the desired state from your trajectory.
        desiredState = self.trajectory.sample(currentTime - self.startTime)

        stray_distance = math.hypot(
            desiredState.pose.X() - currentPos.X(),
            desiredState.pose.Y() - currentPos.Y(),
        )
        if stray_distance > 1:
            # Let's force that initial_call again
            self.next_state("follow_trajectory")

        chassisSpeeds = self.controller.calculateRobotRelativeSpeeds(
            start, desiredState
        )

        if self.controller.getPositionalError() < 0.01:
            self.next_state("finish")

        # Command your drivetrain using these speeds.
        self.drivetrain.drive_auto(chassisSpeeds)

    @state()
    def finish(self):
        print("DESTINATION!")

    @override
    def done(self) -> None:
        """
        Called when the trajectory is complete.
        Stop the robot.
        """
        return super().done()
