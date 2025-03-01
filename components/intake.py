from operator import ne

import rev
import wpilib
from magicbot import StateMachine, feedback, tunable, will_reset_to
from magicbot.state_machine import state, timed_state

import constants
from components.chariot import Chariot


class Intake:
    max_speed = tunable(1)
    beam_analog_threshold = tunable(245)  # TODO rajouter pull-up 5V
    __target_intake_speed = will_reset_to(0)
    __target_output_speed = will_reset_to(0)

    def setup(self):
        """
        Appelé après l'injection
        """
        # Intake
        self.intake_motor = rev.SparkMax(
            constants.CANIds.INTAKE_INTAKE_MOTOR, rev.SparkMax.MotorType.kBrushless
        )

        self.output_motor = rev.SparkMax(
            constants.CANIds.INTAKE_OUTPUT_MOTOR, rev.SparkMax.MotorType.kBrushless
        )
        self.output_motor.setInverted(True)

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
        self.intake_motor.set(
            max(min(self.__target_intake_speed, self.max_speed), -self.max_speed)
        )
        self.output_motor.set(
            max(min(self.__target_output_speed, self.max_speed), -self.max_speed)
        )


class ActionIntakeEntree(StateMachine):
    intake: Intake
    VITESSE_MOTEUR: float = 0.25
    chariot: Chariot

    @state(first=True)
    def intakeEntree(self):
        print("Demarrage de l'entree - intake")
        if self.intake.piece_chargee():
            self.intake.set_intake_speed(0)
            self.intake.set_output_speed(0)
            self.next_state("finish")
        else:
            print("WWEEE gooOOTT Thiss")
            # self.chariot.move_intake_back_for_intake()
            self.intake.set_intake_speed(self.VITESSE_MOTEUR)
            self.intake.set_output_speed(-(self.VITESSE_MOTEUR / 4))

    @state
    def finish(self):
        pass


class ActionIntakeSortie(StateMachine):
    intake: Intake
    VITESSE_MOTEUR: float = 1.0

    @timed_state(first=True, duration=1, must_finish=True, next_state="finish")
    def intakeFinirSortie(self):
        self.intake.set_intake_speed(self.VITESSE_MOTEUR)
        self.intake.set_output_speed(self.VITESSE_MOTEUR)
        print("Demarrage de la finition de la sortie intake")

    @state
    def finish(self):
        pass
