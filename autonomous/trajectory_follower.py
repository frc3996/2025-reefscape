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
from components.field import FieldLayout
from common.graph import Graph, AStar, Node

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

    # Inject the field
    field_layout: FieldLayout

    def setup(self) -> None:
        super().__init__()
        self.destinationNodeName : str = ""
    
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

        self.holonomicController.setTolerance(Pose2d(0.02, 0.02, math.radians(5)))

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

    def move(self, destinationNodeName : str) -> None:
        if self.destinationNodeName != destinationNodeName:
            self.destinationNodeName = destinationNodeName
            return self.engage("follow_trajectory", True)
        else:
            return super().engage()

    @state(first=True)
    def follow_trajectory(self, initial_call):
        """
        This state is called repeatedly during autonomous.
        It computes the desired chassis speeds to follow the trajectory.
        """
        assert self.destinationNodeName != ""
        end = self.field_layout.getPose(self.destinationNodeName)

        # Get the real robot position
        currentPos: Pose2d = self.drivetrain.getPose()

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
                currentTime = wpilib.Timer.getFPGATimestamp()
                # Calculate the trajectory
                if (
                    initial_call
                    or self.trajectory.totalTime() < currentTime - self.startTime
                ):
                    graph = self.field_layout.getGraph()
                    startNode = graph.findClosest(startTranslation)
                    endNode = graph.getNode(self.destinationNodeName)
                    if startNode is not None and endNode is not None:
                        print("********** Generating path **********")
                        genStartTime = wpilib.Timer.getFPGATimestamp()
                        interiorPathNodes: list[Node] = AStar.generatePath(graph, startNode, endNode)
                        interiorPathNodes = interiorPathNodes[:-1]  # Remove final interior node because it's the end pose
                        if startNode.position.distance(interiorPathNodes[0].position) < 0.01: # Remove first node if it's the same as the first interior node
                            interiorPathNodes = interiorPathNodes[1:]
                        if interiorPathNodes is None or len(interiorPathNodes) == 0:
                            print("********** ERROR: Cannot find path **********")
                            self.next_state("finish")
                            return
                        interiorPath : list[Translation2d] = [node.position for node in interiorPathNodes]
                        print(f"Start: {start}")
                        print(f"Path: {interiorPath}")
                        print(f"End: {end}")

                        self.trajectory = self.generate_trajectory(start, interiorPath, end)
                        self.startTime = currentTime
                        genEndTime = wpilib.Timer.getFPGATimestamp()
                        print(f"Generated path with {len(interiorPath)} nodes in {(genEndTime - genStartTime) * 1000} ms")
                    else:
                        print("********** ERROR: Cannot find path **********")
                        self.next_state("finish")
                        return

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
            print("DESTINATION REACHED!")
        self.done()

    @override
    def done(self) -> None:
        """
        Called when the trajectory is complete.
        Stop the robot.
        """
        return super().done()
