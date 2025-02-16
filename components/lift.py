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


class Lift:
    max_speed = tunable(8)
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
                1,
                0,
                0,
                wpimath.trajectory.TrapezoidProfile.Constraints(self.max_speed, 15),
            )
        )

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
            constants.DigitalIO.LIFT_STRING_ENCODER_1,
            constants.DigitalIO.LIFT_STRING_ENCODER_2,
        )
        self.stringEncoder.setDistancePerPulse(1 / 6340)
        self.stringEncoder.setReverseDirection(True)
        self.stringEncoder.reset()

        # Simulation
        self.stringEncoderSim: wpilib.simulation.EncoderSim = (
            wpilib.simulation.EncoderSim(self.stringEncoder)
        )

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

    def __aller_a_hauteur(self, hauteur: float):
        self.hauteurCible = hauteur

    @feedback
    def get_distance(self) -> float:
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
        currentHeight = self.stringEncoder.getDistance()
        targetHeight = self.hauteurCible

        if (
            not self.limitswitchSafety.get()
            or self.gamepad_pilote.getLeftBumperButton()
        ) and targetHeight > currentHeight:
            # Stop movement
            targetHeight = currentHeight
            self.liftPIDController.reset(currentHeight)

        if not self.limitswitchZero.get() or self.gamepad_pilote.getRightBumperButton():
            # Reset encoder at zero
            self.stringEncoder.reset()
            currentHeight = 0
            # Stop movement
            targetHeight = 0
            self.liftPIDController.reset(currentHeight)

        liftOutput = self.liftPIDController.calculate(currentHeight, targetHeight)

        # Should be configured as follower
        self.liftMaster.set(liftOutput)
        # XXX: Not necessary as follower
        # self.liftSlave.set(liftOutput)

    def simulationPeriodic(self):
        currentHeight = self.stringEncoder.getDistance()

        rate = self.liftMaster.get()
        self.stringEncoderSim.setRate(rate)
        self.stringEncoderSim.setDistance(currentHeight + rate * 0.02)
