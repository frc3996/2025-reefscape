
import math
import wpilib
import constants
import rev
from magicbot import tunable, will_reset_to, StateMachine, feedback
from magicbot.state_machine import state, timed_state

class Lift:
    max_speed = tunable(1)
    target_speed = tunable(0.1) # TODO ajustement

    # TODO ajuster les valeurs
    # les hauteurs sont en metres
    hauteurDeplacement = tunable(0)
    hauteurLeve11 = tunable(2500) 
    hauteurLeve12 = tunable(5000)
    hauteurLeve13 = tunable(7500)
    hauteurIntake = tunable(1000)
    hauteurMargeErreur = tunable(100)

    # hauteur cible
    hauteurCible = 0 # TODO quelque chose d'intelligent ici

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

    # TODO effacer??
    # def moveLift(self,pos):
    #     """Set la hauteur du lift selon la position"""
    #     # TODO
    # def manualOffset():
    #     """Modifie l'offset de très peu si probleme"""
    #     # TODO

    def go_intake(self):
        self.__aller_a_hauteur(self.hauteurIntake)

    def go_level1(self):
        self.__aller_a_hauteur(self.hauteurLeve11)

    def go_level2(self):
        self.__aller_a_hauteur(self.hauteurLeve12)

    def go_level3(self):
        self.__aller_a_hauteur(self.hauteurLeve13)

    def go_deplacement(self):
        self.__aller_a_hauteur(self.hauteurDeplacement)

    def __aller_a_hauteur(self, hauteur : float):
        self.hauteurCible = hauteur

    @feedback
    def get_distance(self) -> float:
        return self.string_encoder.getDistance()

    @feedback
    def get_direction(self) -> bool:
        return self.string_encoder.getDirection()

    @feedback
    def get_hauteur_cible(self) -> float:
        return self.hauteurCible

    @feedback
    def get_direction_mult_factor(self) -> int:
        delta = self.hauteurCible - self.get_distance() 
        if math.fabs(delta) < self.hauteurMargeErreur:
            return 0
        elif delta > 0:
            return 1
        elif delta < 0:
            return -1
        assert(False)
        return 0

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        distance : float = self.get_distance()
        direction : bool = self.get_direction()
        # print(f"distance:{distance} direction:{direction}")
        dirFacteur = self.get_direction_mult_factor()
        self.lift_motor_main.set(max(min(dirFacteur * self.target_speed, self.max_speed), -self.max_speed))
        self.lift_motor_follow.set(max(min(dirFacteur * self.target_speed, self.max_speed), -self.max_speed))