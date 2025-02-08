import math
import typing
from hashlib import new

import pyfrc
import rev
import wpilib
import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from wpimath import units
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        self.robot = robot
        self.navx = wpilib.simulation.SimDeviceSim("navX-Sensor[2]")
        self.navx_yaw = self.navx.getDouble("Yaw")
        self.navx_rate = self.navx.getDouble("Rate")

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # SimBattery estimates loaded battery voltage
        wpilib.simulation.RoboRioSim.setVInVoltage(
            wpilib.simulation.BatterySim.calculate([0])
        )

        # Call the components periodic
        self.robot.drivetrain.simulationPeriodic()

        # NavX (SPI interface)
        self.navx_rate.set(-1.0 * self.robot.drivetrain.getChassisSpeeds().omega_dps)
        self.navx_yaw.set(self.navx_yaw.get() + self.navx_rate.get() * 0.02)
