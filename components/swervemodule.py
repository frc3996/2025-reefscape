###################################################################################
# MIT License
#
# Copyright (c) PhotonVision
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
###################################################################################
import math
from typing import final

import phoenix6
import phoenix6.sim
import wpilib
import wpilib.simulation
import wpimath.controller
import wpimath.filter
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
import wpimath.units
from phoenix6.configs import MagnetSensorConfigs

kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class DriveEncoder:
    wheel_circumference_meter = math.pi * wpimath.units.inchesToMeters(4.0)
    wheel_gear_ratio: float = 6.12  # L1=8.14; L2=6.75; L3=6.12
    kVelocityToRpsConversionFactor: float = wheel_gear_ratio / wheel_circumference_meter

    def __init__(self, motor: phoenix6.hardware.TalonFX):
        self.motor: phoenix6.hardware.TalonFX = motor

    def getRate(self) -> wpimath.units.meters_per_second:
        """
        Returns the drive velocity in meters per second.
        In simulation the phoenix6 simulation state (see simulationPeriodic) is assumed to be updating
        the TalonFX sensor readings.
        """
        drive_velocity = (
            self.motor.get_velocity().value / self.kVelocityToRpsConversionFactor
        )
        return drive_velocity

    def getDistance(self) -> wpimath.units.meters:
        """
        Returns the distance travelled in meters.
        In simulation the phoenix6 simulation state (see simulationPeriodic) is assumed to be updating
        the TalonFX sensor readings.
        """
        drive_distance = (
            self.motor.get_position().value / self.kVelocityToRpsConversionFactor
        )
        return drive_distance

    def setDistancePerPulse(self, dpp: float):
        # In simulation and on real hardware you might store this conversion factor if needed.
        self.distancePerPulse = dpp


class TurningEncoder:
    def __init__(self, encoder: phoenix6.hardware.CANcoder):
        self.encoder: phoenix6.hardware.CANcoder = encoder

    def getRate(self) -> wpimath.units.radians_per_second:
        rotation_velocity = self.encoder.get_velocity().value
        return rotation_velocity

    def getDistance(self) -> wpimath.units.radians:
        # Assume the CANcoder returns rotations; convert to radians.
        rotation_distance = self.encoder.get_absolute_position().value * math.tau
        return rotation_distance

    def setDistancePerPulse(self, dpp: float):
        self.distancePerPulse = dpp


@final
class SwerveModule:
    def __init__(
        self,
        driveMotorChannel: int,
        turningMotorChannel: int,
        turningEncoderChannel: int,
        moduleNumber: int,
        rotation_zero: int,
    ) -> None:
        """
        Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorChannel:      PWM output for the drive motor.
        :param turningMotorChannel:    PWM output for the turning motor.
        :param turningEncoderChannel: DIO input for the turning encoder channel A
        """
        self.moduleNumber = moduleNumber
        self.desiredState = wpimath.kinematics.SwerveModuleState()
        self.driveMotor = phoenix6.hardware.TalonFX(driveMotorChannel)
        self.turningMotor = phoenix6.hardware.TalonFX(turningMotorChannel)
        self.rotation_zero = rotation_zero
        self.talon_set_config(self.turningMotor)

        self.driveEncoder = DriveEncoder(self.driveMotor)
        self.turningEncoder = TurningEncoder(
            phoenix6.hardware.CANcoder(turningEncoderChannel)
        )

        # Gains are for example purposes only – must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(10, 0, 0)
        self.turningPIDController = wpimath.controller.PIDController(30, 0, 0)
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 3)

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.
        self.driveEncoder.setDistancePerPulse(
            math.tau * kWheelRadius / kEncoderResolution
        )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.
        self.turningEncoder.setDistancePerPulse(math.tau / kEncoderResolution)

        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        # --- Simulation Support ---
        self.simDrivingMotorFilter = wpimath.filter.LinearFilter.singlePoleIIR(
            0.1, 0.02
        )
        self.simTurningMotorFilter = wpimath.filter.LinearFilter.singlePoleIIR(
            0.0001, 0.02
        )

        # These simulation state variables keep track of the “true” positions.
        self.simDrivingEncoderPos = 0.0
        self.simTurningEncoderPos = 0.0

    def talon_set_config(self, talonfx: phoenix6.hardware.TalonFX):
        # cfg = phoenix6.configs.TalonFXConfiguration()
        #
        # # Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor
        # cfg.slot0.k_s = 0.1  # To account for friction, add 0.1 V of static feedforward
        # cfg.slot0.k_v = 0.12  # Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        # cfg.slot0.k_p = 0.11  # An error of 1 rotation per second results in 2V output
        # cfg.slot0.k_i = 0  # No output for integrated error
        # cfg.slot0.k_d = 0  # No output for error derivative
        # # Peak output of 8 volts
        # cfg.voltage.peak_forward_voltage = 8
        # cfg.voltage.peak_reverse_voltage = -8
        #
        # # Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself
        # cfg.slot1.k_s = 2.5  # To account for friction, add 2.5 A of static feedforward
        # cfg.slot1.k_p = 5  # An error of 1 rotation per second results in 5 A output
        # cfg.slot1.k_i = 0  # No output for integrated error
        # cfg.slot1.k_d = 0  # No output for error derivative
        # # Peak output of 40 A
        # cfg.torque_current.peak_forward_torque_current = 40
        # cfg.torque_current.peak_reverse_torque_current = -40

        cfg = MagnetSensorConfigs()
        cfg.magnet_offset = wpimath.units.degreesToRotations(self.rotation_zero)

        # Retry config apply up to 5 times, report if failure
        status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
        for _ in range(0, 5):
            status = talonfx.configurator.apply(cfg)
            if status.is_ok():
                break
        if not status.is_ok():
            print(f"Could not apply configs, error code: {status.name}")

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module."""
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getRate(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module."""
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getDistance(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module."""
        self.desiredState = desiredState

        currentRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

        # Optimize the reference state to avoid spinning further than 90 degrees.
        self.desiredState.optimize(currentRotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        self.desiredState.speed *= (self.desiredState.angle - currentRotation).cos()

        # Calculate the drive outputs from the PID controllers.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getRate(), self.desiredState.speed
        )
        driveFeedforward = self.driveFeedforward.calculate(self.desiredState.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), self.desiredState.angle.radians()
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput)

    def getAbsoluteHeading(self) -> wpimath.geometry.Rotation2d:
        return wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

    def log(self) -> None:
        # Logging can be implemented as needed.
        pass
        # state = self.getState()
        #
        # table = "Module " + str(self.moduleNumber) + "/"
        # wpilib.SmartDashboard.putNumber(
        #     table + "Steer Degrees",
        #     math.degrees(wpimath.angleModulus(state.angle.radians())),
        # )
        # wpilib.SmartDashboard.putNumber(
        #     table + "Steer Target Degrees",
        #     math.degrees(self.turningPIDController.getSetpoint()),
        # )
        # wpilib.SmartDashboard.putNumber(table + "Drive Velocity Feet", state.speed_fps)
        # wpilib.SmartDashboard.putNumber(
        #     table + "Drive Velocity Target Feet", self.desiredState.speed_fps
        # )
        # wpilib.SmartDashboard.putNumber(
        #     table + "Drive Voltage", self.driveMotor.get() * 12.0
        # wpilib.SmartDashboard.putNumber(
        #     table + "Steer Voltage", self.turningMotor.get() * 12.0
        # )

    def simulationPeriodic(self) -> None:
        """
        In simulationPeriodic we update our simulated sensor positions and rates based on the motor outputs.
        Then we push these values into the vendor simulation state objects so that the encoder classes read them.
        """
        # Set the supply voltages
        _ = self.turningEncoder.encoder.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        _ = self.driveMotor.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )
        _ = self.turningMotor.sim_state.set_supply_voltage(
            wpilib.RobotController.getBatteryVoltage()
        )

        driveSpdRaw = (
            self.driveMotor.get_motor_voltage().value
            / 12.0
            * self.driveFeedforward.maxAchievableVelocity(12.0, 0)
        )
        turnSpdRaw = self.turningMotor.get_motor_voltage().value / 0.7

        driveSpd = self.simDrivingMotorFilter.calculate(driveSpdRaw)
        turnSpd = self.simTurningMotorFilter.calculate(turnSpdRaw)

        # Update our simulated encoder positions (assume a 20 ms loop time).
        self.simDrivingEncoderPos += 0.02 * driveSpd
        self.simTurningEncoderPos += 0.02 * turnSpd

        # For the drive motor (TalonFX), update the simulated sensor position and velocity.
        # (Assuming the TalonFXSim interface provides these methods.)
        _ = self.driveMotor.sim_state.set_raw_rotor_position(
            self.simDrivingEncoderPos * self.driveEncoder.kVelocityToRpsConversionFactor
        )
        _ = self.driveMotor.sim_state.set_rotor_velocity(
            driveSpd * self.driveEncoder.kVelocityToRpsConversionFactor
        )

        # For the turning motor (TalonFX), update its simulation state.
        _ = self.turningMotor.sim_state.set_raw_rotor_position(
            wpimath.units.radiansToRotations(self.simTurningEncoderPos)
        )
        _ = self.turningMotor.sim_state.set_rotor_velocity(
            wpimath.units.radiansToRotations(turnSpd)
        )

        # For the turning encoder (CANcoder), update its simulation state.
        _ = self.turningEncoder.encoder.sim_state.set_raw_position(
            wpimath.units.radiansToRotations(self.simTurningEncoderPos)
        )
        _ = self.turningEncoder.encoder.sim_state.set_velocity(
            wpimath.units.radiansToRotations(turnSpd)
        )
