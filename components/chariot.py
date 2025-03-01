import math
from typing import override

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
    kChariotSpeedMax = tunable(3)
    kChariotCurrentLimit = tunable(20.0)

    lift: Lift

    def setup(self):
        """
        Appelé après l'injection
        """
        self.chariot_motor: rev.SparkMax = rev.SparkMax(
            constants.CANIds.INTAKE_CHARIOT_MOTOR, rev.SparkMax.MotorType.kBrushless
        )

        # Setting up current limit and break
        self.chariot_motor_config: rev.SparkBaseConfig = rev.SparkBaseConfig()
        _ = self.chariot_motor_config.smartCurrentLimit(CURRENT_LIMIT_AMP)
        _ = self.chariot_motor_config.setIdleMode(
            self.chariot_motor_config.IdleMode.kBrake
        )
        _ = self.chariot_motor.configure(
            self.chariot_motor_config,
            self.chariot_motor.ResetMode.kResetSafeParameters,
            self.chariot_motor.PersistMode.kPersistParameters,
        )

        self._chariot_speed: int = 0

    def move_back(self):
        if self.lift.get_hauteur_cible() != self.lift.hauteurIntake:
            # We're not moving where we need
            print("Cancelling to avoid head crash")
            return

        if abs(self.lift.get_lift_height() - self.lift.hauteurIntake) > 0.20:
            # We're wayyyy to far
            print("Cancelling to avoid head crash")
            return

        # Should be good..
        self._chariot_speed = -self.kChariotSpeedMax

    def move_front(self):
        self._chariot_speed = self.kChariotSpeedMax

    def stop(self):
        self._chariot_speed = 0

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        current = self.chariot_motor.getOutputCurrent()

        if self.lift.atZero():
            # Don't you date move
            self.chariot_motor.set(0)
            return

        if not self.lift.atGoal():
            # We move back to the front
            self.move_front()

        if current > self.kChariotCurrentLimit:
            # Stop if current exceeds limit
            self.chariot_motor.set(0)
        else:
            self.chariot_motor.set(self._chariot_speed)
