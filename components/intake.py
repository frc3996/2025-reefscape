
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
    def piece_chargee(self) -> bool:
        if self.beam_sensor.getValue() < self.beam_analog_threshold:
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

class IntakeEntreeSortieAction(StateMachine):
    intake: Intake
    VITESSE_MOTEUR : float = 0.25
    actif : bool = False

    @state(first=True)
    def intakeEnAttente(self):
        print("Demarrage de l'attente - intake")
        if self.actif:
            if self.intake.piece_chargee():
                self.next_state("intakeSortie")
            else:
                self.next_state("intakeEntree")

    @state
    def intakeEntree(self):
        print("Demarrage de l'entree - intake")
        if self.actif:
            if self.intake.piece_chargee():
                self.intake.set_intake_speed(0)
                self.intake.set_output_speed(0)
                self.next_state("intakeEnAttente")
            else:
                self.intake.set_intake_speed(self.VITESSE_MOTEUR)
                self.intake.set_output_speed(-(self.VITESSE_MOTEUR))
        else:
            self.next_state("intakeEnAttente")

    @state
    def intakeSortie(self):
        print("Demarrage de la sortie intake")
        if self.actif:
            if self.intake.piece_chargee():
                self.intake.set_intake_speed(self.VITESSE_MOTEUR)
                self.intake.set_output_speed(self.VITESSE_MOTEUR)
                self.next_state("intakeEnAttente")
            else:
                self.next_state("intakeFinirSortie")
        else:
            self.next_state("intakeEnAttente")

    @timed_state(duration=2, next_state="intakeEnAttente")
    def intakeFinirSortie(self):
        print("Demarrage de la finition de la sortie intake")

    def estEnAttente(self) -> bool:
        return self.current_state == "intakeEnAttente"

    def activer(self, actif : bool):
        self.actif = actif