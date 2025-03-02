from enum import Enum
import math
from common.tools import float_equal
import rev
import wpilib
import wpimath.units
from magicbot import feedback, tunable, will_reset_to

import constants
from components.lift import Lift

class ChariotTarget(Enum):
    TARGET_STOP = 0
    TARGET_FRONT = 1
    TARGET_BACK = 2

CURRENT_LIMIT_AMP = 100


class Chariot:
    __current_target: ChariotTarget = ChariotTarget.TARGET_STOP
    lift: Lift
    kChariotSpeed = tunable(0.7)

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
        _ = self.chariot_motor_config.inverted(True)
        _ = self.chariot_motor_config.smartCurrentLimit(CURRENT_LIMIT_AMP)
        _ = self.chariot_motor_config.openLoopRampRate(1)
        _ = self.chariot_motor_config.setIdleMode(
            self.chariot_motor_config.IdleMode.kCoast
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

        self.__current_target: ChariotTarget = ChariotTarget.TARGET_FRONT
        self.__last_target = ChariotTarget.TARGET_FRONT
        self.__target_reached = False

    def move_back(self):
        # Only allow moving back when going toward intake height and that we're near it
        if not float_equal(self.lift.get_hauteur_cible(), self.lift.hauteurIntake):
            # We're not moving where we need
            return

        if abs(self.lift.get_lift_height() - self.lift.hauteurIntake) > 0.20:
            # Allow moving back if we're near the intake height
            return

        self.__current_target: ChariotTarget = ChariotTarget.TARGET_BACK

    def move_front(self):
        if float_equal(self.lift.get_lift_height(), self.lift.hauteurLevel4):
            # We're not moving where we need
            return
        self.__current_target: ChariotTarget = ChariotTarget.TARGET_FRONT

    @feedback
    def get_chariot_target_str(self) -> str:
        return self.__current_target.name

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

        # Automatically move the head
        if not float_equal(self.lift.get_hauteur_cible(), self.lift.hauteurIntake):
            # We move back to the front
            self.move_front()
        else:
            self.move_back()

        if self.__last_target != self.__current_target:
            self.__target_reached = False
            self.__last_target = self.__current_target


        # Safety, don't move if we're at the bottom or at the top
        if self.lift.atZero() or self.lift.get_lift_height() <= 0.05:
            # Don't you dare move back or front
            self.chariot_motor.set(0)
            return

        if self.lift.get_hauteur_cible() >= self.lift.hauteurLevel4 - 0.10:
            # Don't you dare move back or front
            self.chariot_motor.set(0)
            return

        # Actually move if we're not on the right state
        if self.__current_target == ChariotTarget.TARGET_FRONT:
            if not self.__target_reached and not self.get_chariot_front_limit_switch():
                self.chariot_motor.set(self.kChariotSpeed)
            else:
                self.__target_reached = True
                self.chariot_motor.set(0)
        elif self.__current_target == ChariotTarget.TARGET_BACK:
            if not self.__target_reached and not self.get_chariot_back_limit_switch():
                self.chariot_motor.set(-self.kChariotSpeed)
            else:
                self.__target_reached = True
                self.chariot_motor.set(0)
