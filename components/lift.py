
import wpilib
import constants
import rev
from magicbot import tunable, will_reset_to, StateMachine, feedback
from magicbot.state_machine import state, timed_state


INTAKE_POSITION = 10


class Lift:
    max_speed = tunable(1)
    __target_speed = will_reset_to(0)

    def setup(self):
        """
        Appelé après l'injection
        """

        self.lift_motor_main = rev.SparkMax(constants.CANIds.LIFT_MOTOR_MAIN, rev.SparkMax.MotorType.kBrushless)
        self.lift_motor_follow = rev.SparkMax(constants.CANIds.LIFT_MOTOR_FOLLOW, rev.SparkMax.MotorType.kBrushless)

        self.zero_limitswitch_1_and_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_1_AND_2)
        # self.zero_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_2)
        self.safety_limitswitch_1_and_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_SAFETY_LIMITSWITCH_1_AND_2)
        # self.safety_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_SAFETY_LIMITSWITCH_2)

        self.string_encoder = wpilib.Encoder(constants.DigitalIO.LIFT_STRING_ENCODER_1, constants.DigitalIO.LIFT_STRING_ENCODER_2)
        self.string_encoder.setDistancePerPulse(1)
        self.string_encoder.setReverseDirection(True)

        # self.lift_motor = wpilib.PWMMotorController("DemoMotor", constants.PWM_DEMOMOTOR)

    def set_speed(self, speed):
        """
        Fait tourner le moteur à la vitesse spécifiée
        """
        # S'assure que la vitesse maximale ne peut pas être dépassée
        self.__target_speed = speed

    def moveLift(self,pos):
        """Set la hauteur du lift selon la position"""
        # TODO

    def __move_to_position(self, position):
        """Move the lift to the proper position"""
        # TODO

    def go_intake(self):
        self.__move_to_position(INTAKE_POSITION)

    def manualOffset():
        """Modifie l'offset de très peu si probleme"""
        # TODO

    @feedback
    def obtenirStringDistance(self) -> float:
        return self.string_encoder.getDistance()

    @feedback
    def obtenirStringDirection(self) -> bool:
        return self.string_encoder.getDirection()

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        stringDistance : float = self.obtenirStringDistance()
        stringDirection : bool = self.obtenirStringDirection()
        print(f"distance:{stringDistance} direction:{stringDirection}")

        # self.lift_motor.set(max(min(self.__target_speed, self.max_speed), -self.max_speed))

        target = 0  # Cette valeur sera tunée par un PID




# class LiftActions(StateMachine):

#     lift : Lift

#     def todo(self):
#         pass

#     @state
#     def liftMovement(self, pos):
#         self.lift.moveLift(self, pos)