
import wpilib
import constants
from magicbot import tunable, will_reset_to


class Intake:
    max_speed = tunable(1)
    __target_speed = will_reset_to(0)

    def setup(self):
        """
        Appelé après l'injection
        """
        # self.lift_motor = wpilib.PWMMotorController("DemoMotor", constants.PWM_DEMOMOTOR)

    def set_speed(self, speed):
        """
        Fait tourner le moteur à la vitesse spécifiée
        """
        # S'assure que la vitesse maximale ne peut pas être dépassée
        self.__target_speed = speed

    def has_object(self):
        """Retourne si l'intake est déjà en possession d'un objet"""
        return False

    def enable_intake(self):
        """Active le moteur de l'intake"""
        # TODO

    def disable_intake(self):
        """Désactive le moteur de l'intake"""
        # TODO

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """

        # self.lift_motor.set(max(min(self.__target_speed, self.max_speed), -self.max_speed))
