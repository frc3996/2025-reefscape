
import wpilib
import constants
import rev
from magicbot import tunable, will_reset_to, StateMachine
from magicbot.state_machine import state, timed_state


class Intake:
    max_speed = tunable(1)
    __target_intake_speed = will_reset_to(0)

    def setup(self):
        """
        Appelé après l'injection
        """

        self.output_motor = rev.SparkMax(constants.CANIds.INTAKE_OUTPUT_MOTOR, rev.SparkMax.MotorType.kBrushless)
        self.intake_motor = rev.SparkMax(constants.CANIds.INTAKE_INTAKE_MOTOR, rev.SparkMax.MotorType.kBrushless)

        self.beam_sensor = wpilib.AnalogInput(constants.AnalogIO.INTAKE_BEAM_SENSOR)


    def set_intake_speed(self, speed):
        """
        Fait tourner le moteur à la vitesse spécifiée 
        """
        # S'assure que la vitesse maximale ne peut pas être dépassée
        self.__target_intake_speed = speed

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

        self.intake_motor.set(max(min(self.__target_intake_speed, self.max_speed), -self.max_speed))

class intakeAction(StateMachine):
    pass