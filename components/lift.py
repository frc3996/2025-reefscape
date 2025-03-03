import math
from doctest import master
from enum import IntEnum

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


class LiftTarget(IntEnum):
    DEPLACEMENT = 0
    L1 = 1
    L2 = 2
    L3 = 3
    L4 = 4
    INTAKE = 6


class Lift:
    kMaxSpeed = tunable(1.0)
    kMaxAccel = tunable(1.0)
    __limit_height = will_reset_to(False)

    gamepad_pilote: wpilib.XboxController

    # TODO ajuster les valeurs
    # les hauteurs sont en metres
    hauteurDeplacement = tunable(0.1)
    hauteurIntake = tunable(0.38)
    hauteurLevel1 = tunable(0.35)
    hauteurLevel2 = tunable(0.55)
    hauteurLevel3 = tunable(0.85)
    hauteurLevel4 = tunable(1.26)

    lift_p = tunable(11.0)
    lift_i = tunable(0.0)
    lift_d = tunable(0.0)

    ignoringInput = False

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
                self.lift_p,
                self.lift_i,
                self.lift_d,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    self.kMaxSpeed, self.kMaxAccel
                ),
            )
        )
        self.liftPIDController.setTolerance(HEIGHT_TOLERANCE)

        masterConfig = rev.SparkBaseConfig()
        _ = masterConfig.setIdleMode(masterConfig.IdleMode.kBrake)
        _ = masterConfig.inverted(True)
        _ = self.liftMaster.configure(
            masterConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        slaveConfig = rev.SparkBaseConfig()
        _ = slaveConfig.follow(constants.CANIds.LIFT_MOTOR_MAIN, True)
        _ = self.liftSlave.configure(
            slaveConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        self.limitswitchZero_1: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_1
        )
        self.limitswitchZero_2: wpilib.DigitalInput = wpilib.DigitalInput(
            constants.DigitalIO.LIFT_ZERO_LIMITSWITCH_2
        )

        self.stringEncoder: wpilib.Encoder = wpilib.Encoder(
            constants.DigitalIO.LIFT_STRING_ENCODER_A,
            constants.DigitalIO.LIFT_STRING_ENCODER_B,
        )
        self.stringEncoder.setDistancePerPulse(1 / 6340)
        self.stringEncoder.setReverseDirection(False)
        self.stringEncoder.reset()

    def on_enable(self):
        self.liftPIDController.setP(self.lift_p)
        self.liftPIDController.setI(self.lift_i)
        self.liftPIDController.setD(self.lift_d)
        self.liftPIDController.setConstraints(
            wpimath.trajectory.TrapezoidProfile.Constraints(
                self.kMaxSpeed, self.kMaxAccel
            ),
        )

    def limit_height(self):
        self.__limit_height = True

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
        if not self.ignoringInput:
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

    @feedback
    def atGoal(self) -> bool:
        return self.liftPIDController.atGoal()

    def atZero(self) -> bool:
        return not self.limitswitchZero_1.get() or not self.limitswitchZero_2.get()

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        currentHeight = self.get_lift_height()
        targetHeight = self.get_hauteur_cible()

        if currentHeight <= 0 and targetHeight <= 0 and not self.atZero():
            self.liftMaster.set(-0.1)
            self.ignoringInput = True
        elif self.atZero() and targetHeight <= 0:
            self.liftMaster.set(0)
            self.stringEncoder.reset()
            self.liftPIDController.reset(0)
            self.ignoringInput = False
        else:
            if self.__limit_height and targetHeight >= self.hauteurLevel4 - 0.3:
                targetHeight = targetHeight - 0.3
            liftOutput = self.liftPIDController.calculate(currentHeight, targetHeight)
            self.liftMaster.set(liftOutput / self.kMaxSpeed)
