
import math
import wpilib
import wpimath.trajectory
import wpimath.units
import constants
import rev
from magicbot import tunable, will_reset_to, StateMachine, feedback
from magicbot.state_machine import state, timed_state
import wpimath.controller


class Lift:
    max_speed = tunable(0.2)
    target_speed = tunable(0.1) # TODO ajustement
    gamepad_pilote: wpilib.XboxController

    # TODO ajuster les valeurs
    # les hauteurs sont en metres
    hauteurDeplacement = tunable(0)
    hauteurLeve11 = tunable(wpimath.units.feetToMeters(2.500)) 
    hauteurLeve12 = tunable(wpimath.units.feetToMeters(5.000))
    hauteurLeve13 = tunable(wpimath.units.feetToMeters(7.500))
    hauteurIntake = tunable(wpimath.units.feetToMeters(1.000))
    hauteurMargeErreur = tunable(0.01)

    # hauteur cible
    hauteurCible = 0 # TODO quelque chose d'intelligent ici

    def setup(self):
        """
        Appelé après l'injection
        """

        self.lift_motor_main = rev.SparkMax(constants.CANIds.LIFT_MOTOR_MAIN, rev.SparkMax.MotorType.kBrushless)
        self.lift_motor_follow = rev.SparkMax(constants.CANIds.LIFT_MOTOR_FOLLOW, rev.SparkMax.MotorType.kBrushless)

        # self.liftPIDController = wpimath.controller.PIDController(1, 0, 0)
        constraints = wpimath.trajectory.TrapezoidProfile.Constraints(0.2, 0.1)
        self.liftPIDController = wpimath.controller.ProfiledPIDController(1, 0, 0, constraints)
        

        sparkConfig = rev.SparkBaseConfig()
        sparkConfig.follow(constants.CANIds.LIFT_MOTOR_MAIN, False)
        self.lift_motor_follow.configure(sparkConfig, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)

        self.zero_limitswitch_1_and_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_1_AND_2)
        # self.zero_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_2)
        self.safety_limitswitch_1_and_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_SAFETY_LIMITSWITCH_1_AND_2)
        # self.safety_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_SAFETY_LIMITSWITCH_2)

        self.string_encoder = wpilib.Encoder(constants.DigitalIO.LIFT_STRING_ENCODER_1, constants.DigitalIO.LIFT_STRING_ENCODER_2)
        self.string_encoder.setDistancePerPulse(1/6340)
        self.string_encoder.setReverseDirection(True)
        self.string_encoder.reset()

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

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        currentHeight = self.string_encoder.getDistance()
        targetHeight = self.hauteurCible

        liftOutput = self.liftPIDController.calculate(
            targetHeight, currentHeight 
        )

        if (not self.safety_limitswitch_1_and_2.get() or self.gamepad_pilote.getLeftBumperButton()) and liftOutput < 0:
            print(f"SAFETY {self.safety_limitswitch_1_and_2.get()}:{self.gamepad_pilote.getLeftBumperButton()}")
            targetHeight = currentHeight
            self.liftPIDController.reset(currentHeight)
            liftOutput = self.liftPIDController.calculate(
                targetHeight, currentHeight 
            )

        if not self.zero_limitswitch_1_and_2.get() or self.gamepad_pilote.getRightBumperButton():
            print(f"ZERO {self.zero_limitswitch_1_and_2.get()}:{self.gamepad_pilote.getRightBumperButton()}")
            self.string_encoder.reset()
            currentHeight = 0
            targetHeight = 0
            self.liftPIDController.reset(currentHeight)
            liftOutput = self.liftPIDController.calculate(
                targetHeight, currentHeight 
            )


        self.lift_motor_main.set(liftOutput)
        self.lift_motor_follow.set(liftOutput)



