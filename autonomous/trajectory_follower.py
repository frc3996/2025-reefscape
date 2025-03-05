import math
from typing import final, override

import ntcore
import wpilib
from magicbot import AutonomousStateMachine, StateMachine
from magicbot.state_machine import StateRef, state
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

from components import swervedrive
from components.swervedrive import SwerveDrive


@final
class TrajectoryFollower(StateMachine):
    """
    An autonomous state machine that generates and follows a trajectory.
    """

    MODE_NAME = "TrajectoryFollower"
    DEFAULT = False  # Only run when explicitly selected.

    # Inject the SwerveDrive subsystem.
    drivetrain: SwerveDrive

    # Inject the control loop wait time
    dt: float

    def setup(self) -> None:
        super().__init__()

        # Instantiate the HolonomicDriveController.
        # You will need to tune these PID values for your robot.
        x_controller = PIDController(1.0, 0.0, 0.0)
        y_controller = PIDController(1.0, 0.0, 0.0)
        # For the rotational (theta) control, we use a ProfiledPIDController.
        # The constraints here (max velocity and acceleration) should be set according to your robot's capabilities.
        theta_constraints = TrapezoidProfileRadians.Constraints(
            maxVelocity=math.radians(180), maxAcceleration=math.radians(180)
        )
        theta_controller = ProfiledPIDControllerRadians(
            1.0, 0.0, 0.0, theta_constraints
        )
        # Optionally, enable continuous input for theta (if your rotation is wrap–around)
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        self.holonomicController = HolonomicDriveController(
            x_controller, y_controller, theta_controller
        )

        self.holonomicController.setTolerance(Pose2d(0.01, 0.01, math.radians(5)))

        self.startTime = 0

        # LOG
        # Publish the trajectory for visualization.
        nt = ntcore.NetworkTableInstance.getDefault()
        self.trajectoryPub = nt.getStructArrayTopic(
            f"/AdvantageScope/Trajectory", Pose2d
        ).publish()

    def generate_trajectory(
        self, start: Pose2d, interior: list[Translation2d], end: Pose2d
    ) -> Trajectory:
        """
        Generate a simple trajectory.
        Adjust waypoints, velocity, and acceleration constraints as needed.
        """
        config = TrajectoryConfig(
            maxVelocity=swervedrive.kMaxSpeed, maxAcceleration=swervedrive.kMaxAccel
        )
        config.setStartVelocity(self.drivetrain.getVelocity())
        config.setKinematics(self.drivetrain.kinematics)
        try:
            trajectory = TrajectoryGenerator.generateTrajectory(
                start, interior, end, config
            )
        except Exception as e:
            raise RuntimeError("Failed to generate trajectory normally.") from e

        # Publish the trajectory for visualization.
        sample_count = 10
        delta_t = trajectory.totalTime() / sample_count
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
        end = Pose2d(2, 2, Rotation2d(math.pi / 2))
        interior: list[Translation2d] = []  # Add interior waypoints if desired.

        # Get the real robot position
        currentPos: Pose2d = self.drivetrain.getPose()
        currentTime = wpilib.Timer.getFPGATimestamp()

        # Find the heading angle, this is a holonomic drive!
        startTranslation = Translation2d(currentPos.X(), currentPos.Y())
        endTranslation = Translation2d(end.X(), end.Y())
        translation = endTranslation - startTranslation

        # Start pose for path generation
        start = Pose2d(currentPos.X(), currentPos.Y(), translation.angle())

        # Check if the motion is effectively pure rotation.
        translation_distance = math.hypot(
            end.X() - currentPos.X(), end.Y() - currentPos.Y()
        )

        try:
            if translation_distance > 0.01:  # Threshold for negligible translation.
                # Calculate the trajectory
                if (
                    initial_call
                    or self.trajectory.totalTime() < currentTime - self.startTime
                ):
                    self.trajectory = self.generate_trajectory(start, interior, end)
                    self.startTime = currentTime

                # Sample the desired state from your trajectory.
                desiredState: Trajectory.State = self.trajectory.sample(
                    currentTime - self.startTime
                )

                stray_distance = math.hypot(
                    desiredState.pose.X() - currentPos.X(),
                    desiredState.pose.Y() - currentPos.Y(),
                )
                if stray_distance > 1:
                    self.next_state("follow_trajectory")

                chassisSpeeds: ChassisSpeeds = self.holonomicController.calculate(
                    currentPos, desiredState, end.rotation()
                )
            else:
                # Pure rotation
                chassisSpeeds: ChassisSpeeds = self.holonomicController.calculate(
                    currentPos, end, 0, end.rotation()
                )
                if self.holonomicController.atReference():
                    self.next_state("finish")
        except Exception:
            # Likely pure rotation?
            chassisSpeeds: ChassisSpeeds = self.holonomicController.calculate(
                currentPos, end, 0, end.rotation()
            )
            if self.holonomicController.atReference():
                self.next_state("finish")

        # Command your drivetrain using these speeds.
        self.drivetrain.drive_auto(chassisSpeeds)

    @state()
    def finish(self, initial_call: bool):
        if initial_call:
            self.drivetrain.drive_auto(ChassisSpeeds())
            print("DESTINATION!")

    @override
    def done(self) -> None:
        """
        Called when the trajectory is complete.
        Stop the robot.
        """
        return super().done()
