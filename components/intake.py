
import wpilib
import constants
import rev
from magicbot import tunable, will_reset_to, StateMachine, feedback
from magicbot.state_machine import state, timed_state


class Intake:
    max_speed = tunable(1)
    beam_analog_threshold = tunable(204)
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

    @feedback
    def read_beam_sensor(self) -> float:
        return self.beam_sensor.getValue()

    @feedback
    def piece_chargee(self) -> bool:
        if self.read_beam_sensor() < self.beam_analog_threshold:
            return True
        else:
            return False

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        self.intake_motor.set(max(min(self.__target_intake_speed, self.max_speed), -self.max_speed))
        self.output_motor.set(max(min(self.__target_output_speed, self.max_speed), -self.max_speed))


class ActionIntakeEntree(StateMachine):
    intake: Intake
    VITESSE_MOTEUR : float = 0.25

    @state(first=True)
    def intakeEntree(self):
        print("Demarrage de l'entree - intake")
        if self.intake.piece_chargee():
            self.intake.set_intake_speed(0)
            self.intake.set_output_speed(0)
            self.next_state("finish")
        else:
            self.intake.set_intake_speed(self.VITESSE_MOTEUR)
            self.intake.set_output_speed(-(self.VITESSE_MOTEUR/4))

    @state
    def finish(self):
        # TODO Flasher lumiere arduino
        pass


class ActionIntakeSortie(StateMachine):
    intake: Intake
    VITESSE_MOTEUR : float = 0.25
    active: bool = True

    @state(first=True)
    def intakeSortie(self):
        print("Demarrage de la sortie intake")
        active = True
        self.intake.set_intake_speed(self.VITESSE_MOTEUR)
        self.intake.set_output_speed(self.VITESSE_MOTEUR)
        if not self.intake.piece_chargee():
            self.next_state("intakeFinirSortie")


    @timed_state(duration=0.5, next_state="intakeFinish")
    def intakeFinirSortie(self):
        self.intake.set_intake_speed(self.VITESSE_MOTEUR)
        self.intake.set_output_speed(self.VITESSE_MOTEUR)
        print("Demarrage de la finition de la sortie intake")

    @state
    def intakeFinish(self):
        # TODO Flasher lumiere arduino
        active = False
        pass
