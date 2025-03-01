import math

import rev
import wpilib
import wpimath.controller
import wpimath.trajectory
from magicbot import StateMachine, feedback, tunable
from magicbot.state_machine import state, timed_state

import constants


class Intake:

    intake_kP = tunable(12.0)
    intake_kI = tunable(0.0)
    intake_kD = tunable(0.0)
    shoot_kP = tunable(12.0)
    shoot_kI = tunable(0.0)
    shoot_kD = tunable(0.0)

    kMaxSpeed = tunable(1)
    kMaxAccel = tunable(1)
    beam_analog_threshold = tunable(245)

    def setup(self):
        """
        Appelé après l'injection
        """
        # Intake
        self.intakeMotor: rev.SparkMax = rev.SparkMax(
            constants.CANIds.INTAKE_INTAKE_MOTOR, rev.SparkMax.MotorType.kBrushless
        )
        self.intakeEncoder: rev.SparkRelativeEncoder = self.intakeMotor.getEncoder()

        self.shootMotor: rev.SparkMax = rev.SparkMax(
            constants.CANIds.INTAKE_OUTPUT_MOTOR, rev.SparkMax.MotorType.kBrushless
        )

        shootConfig = rev.SparkBaseConfig()
        _ = shootConfig.inverted(True)
        _ = self.shootMotor.configure(
            shootConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        self.shootEncoder: rev.SparkRelativeEncoder = self.shootMotor.getEncoder()

        self.beam_sensor: wpilib.AnalogInput = wpilib.AnalogInput(
            constants.AnalogIO.INTAKE_BEAM_SENSOR
        )

        self.intakePIDController: wpimath.controller.ProfiledPIDController = (
            wpimath.controller.ProfiledPIDController(
                self.intake_kP,
                self.intake_kI,
                self.intake_kD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    self.kMaxSpeed, self.kMaxAccel
                ),
            )
        )
        self.shootPIDController: wpimath.controller.ProfiledPIDController = (
            wpimath.controller.ProfiledPIDController(
                self.intake_kP,
                self.intake_kI,
                self.intake_kD,
                wpimath.trajectory.TrapezoidProfile.Constraints(
                    self.kMaxSpeed, self.kMaxAccel
                ),
            )
        )

        self._intakeDistance: float = 0
        self._shootDistance: float = 0

    @feedback
    def read_beam_sensor(self) -> float:
        return self.beam_sensor.getValue()

    @feedback
    def piece_chargee(self) -> bool:
        if self.read_beam_sensor() < self.beam_analog_threshold:
            return True
        else:
            return False

    def intake(self, distanceInRotations: float):
        self._intakeDistance = distanceInRotations + self.intakeEncoder.getPosition()

    def shoot(self, distanceInRotations: float):
        self._shootDistance = distanceInRotations + self.shootEncoder.getPosition()

    def intakeStop(self):
        self.intakeMotor.set(0)
        self._intakeDistance = self.intakeEncoder.getPosition()
        self.intakePIDController.reset(self._intakeDistance)

    def shootStop(self):
        self.shootMotor.set(0)
        self._shootDistance = self.intakeEncoder.getPosition()
        self.shootPIDController.reset(self._intakeDistance)

    def shootAtGoal(self) -> bool:
        return self.shootPIDController.atGoal()

    def intakeAtGoal(self) -> bool:
        return self.intakePIDController.atGoal()

    def stop(self):
        self.shootStop()
        self.intakeStop()

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        # Vérifier si on est proche de la distance cible
        if self.intakeEncoder.getPosition() > self._intakeDistance:
            intakeOutput = 0  # Arrêter le moteur
        else:
            intakeOutput: float = self.intakePIDController.calculate(
                self.intakeEncoder.getPosition(), self._intakeDistance
            )

        if self.shootEncoder.getPosition() > self._shootDistance:
            shootOutput = 0  # Arrêter le moteur
        else:
            shootOutput: float = self.shootPIDController.calculate(
                self.shootEncoder.getVelocity(), self._shootDistance
            )

        self.intakeMotor.set(intakeOutput)
        self.shootMotor.set(shootOutput)


class ActionIntakeEntree(StateMachine):
    intake: Intake

    @timed_state(first=True, duration=20, next_state="finish", must_finish=True)
    def intakeEntree(self):
        print("Demarrage de l'entree - intake")
        if self.intake.piece_chargee():
            self.next_state("finish")
        else:
            self.intake.intake(math.inf)
            self.intake.shoot(0)

    @state
    def finish(self):
        self.intake.stop()


class ActionIntakeSortie(StateMachine):
    intake: Intake

    @timed_state(first=True, duration=5, must_finish=True, next_state="finish")
    def intakeFinirSortie(self, initial_call: bool):
        if initial_call:
            self.intake.intake(100)
            self.intake.shoot(100)
        elif self.intake.shootAtGoal():
            self.next_state("finish")
        print("Demarrage de la finition de la sortie intake")

    @state
    def finish(self):
        self.intake.stop()
