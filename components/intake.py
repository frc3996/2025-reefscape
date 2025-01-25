
import wpilib
import constants
import rev
from magicbot import tunable, will_reset_to, StateMachine
from magicbot.state_machine import state, timed_state


class Intake:
    max_speed = tunable(1)
    analog_threshold = tunable(2048)
    __target_intake_speed = will_reset_to(0)
    __target_output_speed = will_reset_to(0)
    intake_motor: rev.SparkMax
    output_motor: rev.SparkMax


    def setup(self):
        """
        Appelé après l'injection
        """
        self.beam_sensor = wpilib.AnalogInput(constants.AnalogIO.INTAKE_BEAM_SENSOR)

    def set_intake_speed(self, speed):
        """
        Fait tourner le moteur à la vitesse spécifiée 
        """
        # S'assure que la vitesse maximale ne peut pas être dépassée
        self.__target_intake_speed = speed

    def set_output_speed(self, speed):
        """
        Fait tourner le moteur à la vitesse spécifiée 
        """
        # S'assure que la vitesse maximale ne peut pas être dépassée
        self.__target_output_speed = speed

    def has_object(self):
        """Retourne si l'intake est déjà en possession d'un objet"""
        # TODO demander au beam sensor?
        return False

    def enable_intake_motor(self):
        """Active le moteur de l'intake"""
        self.set_intake_speed(0.5)

    def disable_intake_motor(self):
        """Désactive le moteur de l'intake"""
        self.set_intake_speed(0)

    def enable_output_motor(self):
        """Active le moteur de l'intake"""
        self.set_output_speed(0.5)

    def disable_output_motor(self):
        """Désactive le moteur de l'intake"""
        self.set_output_speed(0)

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        self.intake_motor.set(max(min(self.__target_intake_speed, self.max_speed), -self.max_speed))
        self.output_motor.set(max(min(self.__target_output_speed, self.max_speed), -self.max_speed))

class intakeAction(StateMachine):
    pass