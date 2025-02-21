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

import ntcore
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
from magicbot import tunable
from phoenix6.configs import (CANcoderConfiguration, MagnetSensorConfigs,
                              MotorOutputConfigs, TalonFXConfiguration)
from phoenix6.configs.config_groups import InvertedValue
from phoenix6.sim.talon_fx_sim_state import rotation

import common.gamepad_helper as tools


class MockSwerveModule:
    def __init__(self):
        self.state = wpimath.kinematics.SwerveModuleState()

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        return wpimath.kinematics.SwerveModulePosition()

    def setDesiredState(self, state: wpimath.kinematics.SwerveModuleState):
        self.state = state

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        return self.state

    def log(self):
        return

    def simulationPeriodic(self):
        return


def drive_talonfx_set_config(talonfx: phoenix6.hardware.TalonFX):
    _ = talonfx.get_position().set_update_frequency(10)
    _ = talonfx.optimize_bus_utilization()

    cfg = TalonFXConfiguration()
    cfg.open_loop_ramps = (
        phoenix6.configs.OpenLoopRampsConfigs().with_duty_cycle_open_loop_ramp_period(
            0.01
        )
    )

    motor_output = MotorOutputConfigs()
    cfg.motor_output = motor_output

    # Retry config apply up to 5 times, report if failure
    status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
    for _ in range(0, 5):
        status = talonfx.configurator.apply(cfg)
        if status.is_ok():
            break
    if not status.is_ok():
        print(f"Could not apply configs, error code: {status.name}")


def turning_talonfx_set_config(talonfx: phoenix6.hardware.TalonFX, inverted: bool):
    _ = talonfx.get_position().set_update_frequency(10)
    _ = talonfx.optimize_bus_utilization()

    cfg = TalonFXConfiguration()
    cfg.open_loop_ramps = (
        phoenix6.configs.OpenLoopRampsConfigs().with_duty_cycle_open_loop_ramp_period(
            0.01
        )
    )

    motor_output = MotorOutputConfigs()
    if inverted:
        motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    else:
        motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE

    cfg.motor_output = motor_output

    # Retry config apply up to 5 times, report if failure
    status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
    for _ in range(0, 5):
        status = talonfx.configurator.apply(cfg)
        if status.is_ok():
            break
    if not status.is_ok():
        print(f"Could not apply configs, error code: {status.name}")


def encoder_set_config(encoder: phoenix6.hardware.CANcoder, rotation_zero: float):
    cfg = CANcoderConfiguration()
    magnet_sensor = MagnetSensorConfigs()
    magnet_sensor.magnet_offset = wpimath.units.degreesToRotations(rotation_zero)
    cfg.magnet_sensor = magnet_sensor

    # Retry config apply up to 5 times, report if failure
    status: phoenix6.StatusCode = phoenix6.StatusCode.STATUS_CODE_NOT_INITIALIZED
    for _ in range(0, 5):
        status = encoder.configurator.apply(cfg)
        if status.is_ok():
            break
    if not status.is_ok():
        print(f"Could not apply configs, error code: {status.name}")

    _ = encoder.get_absolute_position().set_update_frequency(10)
    _ = encoder.optimize_bus_utilization()


class DriveEncoder:
    # Diameter is 4in, C = PI * D
    kWheelCircumference = math.pi * wpimath.units.inchesToMeters(4.0)
    # From spec
    kGearRatio: float = 6.12  # L1=8.14; L2=6.75; L3=6.12

    # RPS = V * (GR / C)
    kVelocityToRps: float = kGearRatio / kWheelCircumference

    # V = RPS * (C / GR)
    kRpsToVelocity: float = kWheelCircumference / kGearRatio

    def __init__(self, motor: phoenix6.hardware.TalonFX):
        self.motor: phoenix6.hardware.TalonFX = motor

    def getRate(self) -> wpimath.units.meters_per_second:
        """
        Returns the drive velocity in meters per second.
        In simulation the phoenix6 simulation state (see simulationPeriodic) is assumed to be updating
        the TalonFX sensor readings.
        """
        drive_velocity = self.motor.get_velocity().value * self.kRpsToVelocity
        return drive_velocity

    def getDistance(self) -> wpimath.units.meters:
        """
        Returns the distance travelled in meters.
        In simulation the phoenix6 simulation state (see simulationPeriodic) is assumed to be updating
        the TalonFX sensor readings.
        """
        drive_distance = self.motor.get_position().value * self.kRpsToVelocity
        return drive_distance

    def setDistancePerPulse(self, dpp: float):
        # In simulation and on real hardware you might store this conversion factor if needed.
        self.distancePerPulse = dpp


class TurningEncoder:
    def __init__(self, moduleNumber: int, encoder: phoenix6.hardware.CANcoder):
        self.encoder: phoenix6.hardware.CANcoder = encoder
        self.moduleNumber = moduleNumber

    def getRate(self) -> wpimath.units.radians_per_second:
        rotation_velocity = self.encoder.get_velocity().value * math.tau
        return rotation_velocity

    def getDistance(self) -> wpimath.units.radians:
        # Assume the CANcoder returns rotations; convert to radians.
        abs_pos = self.encoder.get_absolute_position().value
        rotation_distance = abs_pos * math.tau
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
        inverted: bool,
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
        self.inverted = inverted

        self.driveEncoder = DriveEncoder(self.driveMotor)
        self.turningEncoder = TurningEncoder(
            moduleNumber, phoenix6.hardware.CANcoder(turningEncoderChannel)
        )

        # TalonFX Bull
        drive_talonfx_set_config(self.driveMotor)
        turning_talonfx_set_config(self.turningMotor, self.inverted)
        encoder_set_config(self.turningEncoder.encoder, self.rotation_zero)

        # Gains are for example purposes only – must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(1, 0, 0)
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(1, 2)
        self.turningPIDController = wpimath.controller.PIDController(1, 0, 0)

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
            currentRotation.radians(), self.desiredState.angle.radians()
        )

        # TODO: Add a turn feedforward
        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput)

    def getAbsoluteHeading(self) -> wpimath.geometry.Rotation2d:
        return wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

    def log(self) -> None:
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
        # )
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
        _ = self.driveMotor.sim_state.set_raw_rotor_position(
            self.simDrivingEncoderPos * self.driveEncoder.kVelocityToRps
        )
        _ = self.driveMotor.sim_state.set_rotor_velocity(
            driveSpd * self.driveEncoder.kVelocityToRps
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
