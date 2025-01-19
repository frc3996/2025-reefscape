import math
from dataclasses import dataclass
from typing import cast
from common import tools
import constants

import phoenix6
from magicbot import feedback, tunable, will_reset_to
from wpimath import controller, geometry, kinematics, units
from wpimath.geometry import Rotation2d
import wpimath.trajectory


# Classe de configuration des swerve
@dataclass
class SwerveModuleConfig:
    nt_name: str
    inverted: bool
    allow_reverse: bool
    rotation_zero: int


class SwerveModule:
    # Vas chercher moteur, encodeur et configuration par injection
    is_sim: bool
    driveMotor: phoenix6.hardware.TalonFX
    rotateMotor: phoenix6.hardware.TalonFX
    encoder: phoenix6.hardware.CANcoder
    cfg: SwerveModuleConfig

    kP = tunable(0.005)
    kI = tunable(0.0)
    kD = tunable(0.0)

    debug = tunable(False)
    calibration_mode = tunable(False)
    encoder_zero = tunable(0.0)

    def setup(self):
        """
        Appelé après l'injection
        """
        self.encoder_zero = self.cfg.rotation_zero
        # General
        # meter_per_second = (rps / gear_ratio) * wheel_circumference_meter
        # rps = meter_per_second * gear_ratio / wheel_circumference_meter
        fudge_factor = 1
        wheel_circumference_meter = math.pi * units.inchesToMeters(4.0)
        wheel_gear_ratio = 6.12  # L1=8.14; L2=6.75; L3=6.12
        self.velocity_to_rps_conversion_factor = (
            wheel_gear_ratio / wheel_circumference_meter * fudge_factor
        )
        self.sim_currentPosition = kinematics.SwerveModulePosition()
        self.targetState = kinematics.SwerveModuleState()

        # Drive Motor
        config = phoenix6.configs.TalonFXConfiguration()
        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs().with_duty_cycle_open_loop_ramp_period(
            0.1
        )
        config.slot0.k_p = 0.08
        config.slot0.k_i = 0.0
        config.slot0.k_d = 0.0001
        config.slot0.k_v = 0.12
        config.voltage.peak_forward_voltage = 8
        config.voltage.peak_forward_voltage = -8
        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        config.motor_output = motor_config
        self.driveMotor.configurator.apply(config)  # type: ignore
        self.driveMotor_control = phoenix6.controls.VelocityVoltage(
            0, 0, True, 0, 0, False, False, False
        )

        # Rotation Motor
        config = phoenix6.configs.TalonFXConfiguration()
        config.open_loop_ramps = phoenix6.configs.OpenLoopRampsConfigs().with_duty_cycle_open_loop_ramp_period(
            0.01
        )
        motor_config = phoenix6.configs.MotorOutputConfigs()
        motor_config.inverted = phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        config.motor_output = motor_config
        self.rotateMotor.configurator.apply(config)  # type: ignore
        self.rotateMotor_control = phoenix6.controls.DutyCycleOut(0)
        self.rotation_pid = controller.PIDController(
            self.kP, self.kI, self.kD
        )  # PID configuré via le ShuffleBoard
        self.rotation_pid.enableContinuousInput(
            0, 360
        )  # 0 et 360 sont considérés comme la même valeur
        cancoder_config = phoenix6.configs.CANcoderConfiguration()
        cancoder_config.magnet_sensor.absolute_sensor_range = phoenix6.signals.AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF
        self.encoder.configurator.apply(cancoder_config)  # type: ignore

        self.driveMotor.get_position().set_update_frequency(10)
        self.rotateMotor.get_position().set_update_frequency(10)
        self.encoder.get_absolute_position().set_update_frequency(10)
        self.driveMotor.optimize_bus_utilization()
        self.rotateMotor.optimize_bus_utilization()
        self.encoder.optimize_bus_utilization()

    def on_enable(self):
        self.rotation_pid.setP(self.kP)
        self.rotation_pid.setI(self.kI)
        self.rotation_pid.setD(self.kD)

    def flush(self):
        """
        Remets à Zéro l'angle et la vitesse demandé
        ainsi que le PID
        """
        self._requested_degree = self.get_encoder_rotation().degrees()
        self._requested_speed = 0
        self.rotation_pid.reset()

    def get_encoder_rotation(self) -> Rotation2d:
        """Retourne la position actuelle de l'encodeur"""
        abs_pos = Rotation2d(self.encoder.get_absolute_position().value * math.tau)
        rotation = abs_pos + Rotation2d.fromDegrees(self.encoder_zero)
        return rotation

    def resetPose(self):
        if self.is_sim:
            self.sim_currentPosition = kinematics.SwerveModulePosition(
                0, self.targetState.angle
            )
        else:
            self.driveMotor.set_position(0)

    def setTargetState(self, targetState):
        self.targetState = kinematics.SwerveModuleState.optimize(
            targetState, self.targetState.angle
        )

    def getPosition(self):
        """
        Return Swerve module position
        """
        if self.is_sim:
            return self.sim_currentPosition

        drive_position = (
            self.driveMotor.get_position().value
            / self.velocity_to_rps_conversion_factor
        )
        result = kinematics.SwerveModulePosition(-drive_position, self.get_encoder_rotation())
        return result

    def process(self):
        """
        Utilise le PID pour se rendre à la position demandée.
        Modifie la puissance des moteurs pour celle demandée.

        Appelé à chaque itération/boucle
        """
        # Reversing the wheel direction if angle is greater than 90 degree
        rotation_offset = (self.targetState.angle - self.get_encoder_rotation())
        if self.cfg.allow_reverse:
            if rotation_offset.degrees() < -90 or rotation_offset.degrees() > 90:
                self.targetState.speed *= -1
                self.targetState.angle += Rotation2d(math.pi)

        # Scaling speed if wheel is too offset
        self.targetState.speed *= rotation_offset.cos()

        # Calibration mode
        if self.calibration_mode:
            self.targetState.angle = Rotation2d(0)
            self.targetState.speed = 0.05

        # Computing the angle PID
        error = self.rotation_pid.calculate(
            self.get_encoder_rotation().degrees(), self.targetState.angle.degrees()
        )
        self.rotateMotor_control.output = tools.fit_to_boundaries(error, -1, 1)
        self.rotateMotor.set_control(self.rotateMotor_control)

        # Setting drive motor speed
        rps = self.targetState.speed * self.velocity_to_rps_conversion_factor
        self.driveMotor.set_control(self.driveMotor_control.with_velocity(rps))

        # Setting sim values
        if self.is_sim:
            self.sim_currentPosition = kinematics.SwerveModulePosition(
                self.sim_currentPosition.distance + (self.targetState.speed * 0.02),
                self.targetState.angle,
            )

    def execute(self):
        pass
