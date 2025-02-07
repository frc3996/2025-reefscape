#!/usr/bin/env python3

"""
Patron pour une base pilotable Swerve Drive

Un NAVX est nécessaire, car le positionnement est absolue (field centric).
Le zéro est fait au démarrage du robot, donc il est important de bien le positionner.
Presser le bouton A va également refaire son zéro.

SWERVES (FALCONS x4)
LIME LIGHT


"""

import math
from typing import override

import ntcore
import rev
import wpilib
import wpimath.geometry
from magicbot import MagicRobot
from navx import AHRS
from wpimath.filter import SlewRateLimiter

import components.swervedrive as swervedrive
import constants
from autonomous.auto_modes import RunAuto
from autonomous.trajectory_follower import TrajectoryFollower
from common import gamepad_helper
from components.field import FieldLayout
from components.gyro import Gyro
from components.intake import Intake  # , IntakeEntreeSortieAction
from components.limelight import LimeLightVision
from components.pixy import Pixy
from components.robot_actions import ActionIntake, ActionPathTester, ActionStow

kRobotToCam = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
    wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),
)


class MyRobot(MagicRobot):
    """
    Après avoir créer les 'components' de bas niveau, tel que 'drivetrain' ou 'intake', utiliser leur nom suivi d'un trait souligné (_)
    pour injecter des objets au composant.

    ex.:
    Après avoir créer dans 'components' un fichier intake_driver.py, utiliser une variable nommée "intake_beltMotor: phoenix6.hardware.TalonFX" déclare le type de la variable.
    Quand 'beltMotor' sera appelée depuis 'intake', ce sera un objet de type 'WPI_TalonFX'.

    Utiliser un signe égale (=) pendant la déclaration des variables tel que "intake_beltMotor = phoenix6.hardware.TalonFX(11)" crée l'objet.
    Quand 'beltMotor' est appelé depuis le composant 'intake' et ce sera un WPI_TalonFX avec un ID CAN de 11.

    Utilisez le signe = dans la fonction 'createObjects' pour vous assurer que les données sont biens transmises à leur composantes.

    Pour plus d'information: https://robotpy.readthedocs.io/en/stable/frameworks/magicbot.html
    """

    ##### Auto mode #####
    runAuto: RunAuto

    ##### HIGH Level components first (components that use components) #####
    actionStow: ActionStow
    actionIntake: ActionIntake
    actionPathTester: ActionPathTester
    # actionIntakeEntree: IntakeEntreeSortieAction
    actionTrajectoryFollower: TrajectoryFollower

    ##### LOW Level components #####

    # NAVX
    gyro: Gyro

    # Pixy
    pixy: Pixy

    # SwerveDrive
    drivetrain: swervedrive.SwerveDrive

    # FieldLayout
    field_layout: FieldLayout

    # Vision
    limelight_vision: LimeLightVision

    # Pneumatic Hub
    # pneumatic_hub: wpilib.PneumaticHub

    # Lift
    # lift: Lift

    # climb
    # climb: Climb

    # pneumatic_hub: wpilib.PneumaticHub

    # intake
    intake: Intake

    # Networktables pour de la configuration et retour d'information
    nt: ntcore.NetworkTable

    def __init__(self) -> None:
        super().__init__()
        self.leftXFilter: SlewRateLimiter = SlewRateLimiter(3)
        self.leftYFilter: SlewRateLimiter = SlewRateLimiter(3)
        self.rightXFilter: SlewRateLimiter = SlewRateLimiter(3)

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.is_sim: bool = self.isSimulation()
        self.is_real: bool = self.isReal()
        self.dt: float = self.control_loop_wait_time

        # self.arduino_light = I2CArduinoLight(wpilib.I2C.Port.kOnboard, 0x42)

        # NOTE: Remove comment to increase power over 9000 /s
        # Créé plein de problème sur le modbus, gardé en sourvenir
        # self.status_light = wpilib.Solenoid(10, wpilib.PneumaticsModuleType.CTREPCM, 1)

        # NAVX
        # self.navx: AHRS = AHRS.create_spi()
        self.navx: AHRS = AHRS(AHRS.NavXComType.kUSB1)

        # PhotonVision
        # self.cam = PhotonCamera("YOUR CAMERA NAME")
        # self.camPoseEst = PhotonPoseEstimator(
        #     AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
        #     PoseStrategy.LOWEST_AMBIGUITY,
        #     self.cam,
        #     kRobotToCam,
        # )

        # Pneumatic Hub
        # self.pneumatic_hub = wpilib.PneumaticHub()

        # Pneumatic Hub
        # pneumatic_hub = wpilib.PneumaticHub()

        # Intake
        self.intake_intake_motor = rev.SparkMax(
            constants.CANIds.INTAKE_INTAKE_MOTOR, rev.SparkMax.MotorType.kBrushless
        )
        self.intake_output_motor = rev.SparkMax(
            constants.CANIds.INTAKE_OUTPUT_MOTOR, rev.SparkMax.MotorType.kBrushless
        )

        # General
        self.gamepad_pilote = wpilib.XboxController(0)
        self.gamepad_copilote = wpilib.XboxController(1)
        # self.gamepad = wpilib.PS5Controller(0)
        self.pdp = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)
        self.pdp.clearStickyFaults()

        # TODO Effacer si inutile
        # Update position dans Smart Dashboard
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

    def disabledInit(self) -> None:
        pass
        # self.arduino_light.set_leds(LedMode.Solid, 0, 255, 0)

    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        # self.limelight_vision.execute(()
        pass

    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        self.actionStow.done()
        # self.limelight_vision.light_off()
        # self.arduino_light.set_leds(LedMode.Solid, 0, 255, 0)
        pass

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot en
        tre en mode téléopéré."""
        self.pdp.clearStickyFaults()
        # self.limelight_vision.light_off()
        # self.arduino_light.set_leds(LedMode.Solid, 0, 0, 255)
        self.actionStow.engage()

    @override
    def robotPeriodic(self) -> None:
        # camEstPose = self.camPoseEst.update()
        # if camEstPose:
        #     self.drivetrain.addVisionPoseEstimate(
        #         camEstPose.estimatedPose, camEstPose.timestampSeconds
        #     )
        self.drivetrain.updateOdometry()
        self.drivetrain.log()

    @override
    def teleopPeriodic(self) -> None:

        leftY = gamepad_helper.apply_deadzone(
            self.leftYFilter.calculate(self.gamepad_pilote.getLeftY()), 0.2
        )
        leftX = gamepad_helper.apply_deadzone(
            self.leftXFilter.calculate(self.gamepad_pilote.getLeftX()), 0.2
        )
        rightX = gamepad_helper.apply_deadzone(
            self.rightXFilter.calculate(self.gamepad_pilote.getRawAxis(2)),
            0.2,
            # self.rightXFilter.calculate(self.gamepad_pilote.getRightX(2)), 0.2
        )

        xSpeed = -1.0 * leftY * swervedrive.kMaxSpeed
        ySpeed = -1.0 * leftX * swervedrive.kMaxSpeed
        rot = -1.0 * rightX * swervedrive.kMaxAngularSpeed

        if self.gamepad_pilote.getAButton():
            self.actionTrajectoryFollower.engage()
        elif self.gamepad_pilote.getAButtonReleased():
            # Only call it once..
            self.actionTrajectoryFollower.done()

        self.drivetrain.drive(xSpeed, ySpeed, rot, True)
