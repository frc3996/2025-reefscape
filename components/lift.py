import math

import rev
import wpilib
import wpilib.simulation
import wpimath.controller
import wpimath.trajectory
import wpimath.units
from magicbot import StateMachine, feedback, tunable, will_reset_to
from magicbot.state_machine import state, timed_state
from wpilib._wpilib import Mechanism2d

import constants

HEIGHT_TOLERANCE = 0.05


class Lift:
    kMaxSpeed = tunable(15)
    kMaxAccel = tunable(15)

    gamepad_pilote: wpilib.XboxController

    # TODO ajuster les valeurs
    # les hauteurs sont en metres
    hauteurDepart = wpimath.units.feetToMeters(2.500)
    hauteurDeplacement = tunable(wpimath.units.feetToMeters(2.500) - hauteurDepart)
    hauteurIntake = tunable(wpimath.units.feetToMeters(2.500) - hauteurDepart)
    hauteurLevel1 = tunable(wpimath.units.feetToMeters(2.500) - hauteurDepart)
    hauteurLevel2 = tunable(wpimath.units.feetToMeters(5.000) - hauteurDepart)
    hauteurLevel3 = tunable(wpimath.units.feetToMeters(7.500) - hauteurDepart)
    hauteurLevel4 = tunable(wpimath.units.feetToMeters(10.000) - hauteurDepart)
    hauteurMargeErreur = tunable(0.01)

    # hauteur cible
    hauteurCible = 0  # TODO quelque chose d'intelligent ici

    def setup(self):
        """
        Appelé après l'injection
        """

        self.liftMaster: rev.SparkMax = rev.SparkMax(
            constants.CANIds.LIFT_MOTOR_MAIN, rev.SparkMax.MotorType.kBrushless
        )
        self.liftSlave: rev.SparkMax = rev.SparkMax(
            constants.CANIds.LIFT_MOTOR_FOLLOW, rev.SparkMax.MotorType.kBrushless
        )

        self.liftPIDController: wpimath.controller.ProfiledPIDController = (
            wpimath.controller.ProfiledPIDController(
                10,
                0,
                0,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    self.kMaxSpeed, self.kMaxAccel
                ),
            )
        )
        self.liftPIDController.setTolerance(HEIGHT_TOLERANCE)

        slaveConfig = rev.SparkBaseConfig()
        _ = slaveConfig.follow(constants.CANIds.LIFT_MOTOR_MAIN, False)
        _ = self.liftSlave.configure(
            slaveConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        self.limitswitchZero: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_1_AND_2
        )
        # self.zero_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_2)
        self.limitswitchSafety: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.LIFT_SAFETY_LIMITSWITCH_1_AND_2
        )
        # self.safety_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.LIFT_SAFETY_LIMITSWITCH_2)

        self.stringEncoder: wpilib.Encoder = wpilib.Encoder(
            constants.DigitalIO.LIFT_STRING_ENCODER_A,
            constants.DigitalIO.LIFT_STRING_ENCODER_B,
        )
        self.stringEncoder.setDistancePerPulse(1 / 6340)
        self.stringEncoder.setReverseDirection(True)
        self.stringEncoder.reset()

    def go_intake(self):
        self.__aller_a_hauteur(self.hauteurIntake)

    def go_level1(self):
        self.__aller_a_hauteur(self.hauteurLevel1)

    def go_level2(self):
        self.__aller_a_hauteur(self.hauteurLevel2)

    def go_level3(self):
        self.__aller_a_hauteur(self.hauteurLevel3)

    def go_level4(self):
        self.__aller_a_hauteur(self.hauteurLevel4)

    def go_deplacement(self):
        self.__aller_a_hauteur(self.hauteurDeplacement)

    def __aller_a_hauteur(self, hauteur: float):
        self.hauteurCible = hauteur

    @feedback
    def get_lift_height(self) -> float:
        return self.stringEncoder.getDistance()

    @feedback
    def get_direction(self) -> bool:
        return self.stringEncoder.getDirection()

    @feedback
    def get_hauteur_cible(self) -> float:
        return self.hauteurCible

    def atGoal(self) -> bool:
        return self.liftPIDController.atGoal()

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        currentHeight = self.get_lift_height()
        targetHeight = self.get_hauteur_cible()

        if self.limitswitchSafety.get() and targetHeight > currentHeight:
            print(f"safety {self.limitswitchSafety.get()}")
            # Stop movement
            targetHeight = currentHeight
            self.liftPIDController.reset(currentHeight)

        if self.limitswitchZero.get():
            print(f"zero {self.limitswitchZero.get()}")
            # Reset encoder at zero
            self.stringEncoder.reset()
            currentHeight = 0
            # Stop movement
            targetHeight = 0
            self.liftPIDController.reset(currentHeight)

        liftOutput = self.liftPIDController.calculate(currentHeight, targetHeight)

        self.liftMaster.set(liftOutput / self.kMaxSpeed)
