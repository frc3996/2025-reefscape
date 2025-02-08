
import wpilib
import constants
import rev
from magicbot import tunable, will_reset_to, StateMachine, feedback
from magicbot.state_machine import state, timed_state

class Lift:
    max_speed = tunable(1)
    __target_speed = will_reset_to(0)

    # les hauteurs sont en metre
    hauteurDeplacement = tunable(0)
    hauteurLeve11 = tunable(1) 
    hauteurLeve12 = tunable(2)
    hauteurLeve13 = tunable(3)
    hauteurIntake = tunable(1)

    def setup(self):
        """
        Appelé après l'injection
        """

        self.lift_motor_main = rev.SparkMax(constants.CANIds.LIFT_MOTOR_MAIN, rev.SparkMax.MotorType.kBrushless)
        self.lift_motor_follow = rev.SparkMax(constants.CANIds.LIFT_MOTOR_FOLLOW, rev.SparkMax.MotorType.kBrushless)

        sparkConfig = rev.SparkBaseConfig()
        sparkConfig.follow(constants.CANIds.LIFT_MOTOR_MAIN, False)
        self.lift_motor_follow.configure(sparkConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

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

    # TODO effacer??
    # def moveLift(self,pos):
    #     """Set la hauteur du lift selon la position"""
    #     # TODO
    # def manualOffset():
    #     """Modifie l'offset de très peu si probleme"""
    #     # TODO

    def go_intake(self):
        self.__move_to_position(self.hauteurIntake)

    def go_level1(self):
        self.__move_to_position(self.hauteurLeve11)

    def go_level2(self):
        self.__move_to_position(self.hauteurLeve12)

    def go_level3(self):
        self.__move_to_position(self.hauteurLeve13)

    def go_deplacement(self):
        self.__move_to_position(self.hauteurDeplacement)

    def __move_to_position(self, position):
        """Move the lift to the proper position"""
        # TODO

    @feedback
    def get_distance(self) -> float:
        return self.string_encoder.getDistance()

    @feedback
    def get_direction(self) -> bool:
        return self.string_encoder.getDirection()

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        distance : float = self.get_distance()
        direction : bool = self.get_direction()
        print(f"distance:{distance} direction:{direction}")

        self.lift_motor_main.set(max(min(self.__target_speed, self.max_speed), -self.max_speed))
        self.lift_motor_follow.set(max(min(self.__target_speed, self.max_speed), -self.max_speed))

        target = 0  # Cette valeur sera tunée par un PID
