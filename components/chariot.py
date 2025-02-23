import math

import rev
import wpilib
import wpimath.units
from magicbot import feedback, tunable, will_reset_to

import constants
from components.lift import Lift

TARGET_FRONT = 0
TARGET_BACK = 1
CURRENT_LIMIT_AMP = 5


class Chariot:
    chariot_speed: float = tunable(3)
    safe_lift_height: float = tunable(1.0)
    __chariot_cmd = will_reset_to(0)
    __current_target = will_reset_to(TARGET_FRONT)
    __last_target = TARGET_FRONT
    lift: Lift

    def setup(self):
        """
        Appelé après l'injection
        """
        self.chariot_motor: rev.SparkMax = rev.SparkMax(
            constants.CANIds.INTAKE_CHARIOT_MOTOR, rev.SparkMax.MotorType.kBrushless
        )
        self.chariot_motor_controller: rev.SparkClosedLoopController = (
            self.chariot_motor.getClosedLoopController()
        )
        self.chariot_motor_config: rev.SparkBaseConfig = rev.SparkBaseConfig()
        _ = self.chariot_motor_config.smartCurrentLimit(CURRENT_LIMIT_AMP)
        _ = self.chariot_motor_config.setIdleMode(
            self.chariot_motor_config.IdleMode.kBrake
        )

        # Setting up the encoder
        ## V = RPM * 60 * (C / GR)
        self.kRpmConversionFactor: float = 60 * (
            (wpimath.units.inchesToMeters(1.128) * 2 * math.pi) / (30)
        )
        print(f"Conversion {self.kRpmConversionFactor} ")
        _ = self.chariot_motor_config.encoder.positionConversionFactor(
            self.kRpmConversionFactor
        )
        _ = self.chariot_motor_config.encoder.velocityConversionFactor(
            self.kRpmConversionFactor
        )

        _ = self.chariot_motor.configure(
            self.chariot_motor_config,
            self.chariot_motor.ResetMode.kResetSafeParameters,
            self.chariot_motor.PersistMode.kPersistParameters,
        )

        self.chariot_front_limit_switch: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.INTAKE_FRONT_LIMIT_SWITCH
        )
        self.chariot_back_limit_switch: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.INTAKE_BACK_LIMIT_SWITCH
        )

    def move_intake_back_for_intake(self):
        if (
            self.lift.get_lift_height() < self.safe_lift_height
            or self.lift.get_hauteur_cible() < self.safe_lift_height
        ):
            print("Cancelling to avoid head crash")
            return

        self.__current_target = TARGET_BACK

    @feedback
    def get_chariot_front_limit_switch(self) -> bool:
        return self.chariot_front_limit_switch.get()

    @feedback
    def get_chariot_back_limit_switch(self) -> bool:
        return self.chariot_back_limit_switch.get()

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        if self.__last_target != self.__current_target:
            self.__last_target = self.__current_target

        if self.__current_target == TARGET_FRONT:
            if self.get_chariot_front_limit_switch():
                pass
            else:
                self.__chariot_cmd = self.chariot_speed

        elif self.__current_target == TARGET_BACK:
            if self.get_chariot_back_limit_switch():
                pass
            else:
                self.__chariot_cmd = -self.chariot_speed

        _ = self.chariot_motor_controller.setReference(
            self.__chariot_cmd, self.chariot_motor.ControlType.kVelocity
        )
