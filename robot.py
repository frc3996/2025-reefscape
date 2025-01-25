#!/usr/bin/env python3

"""
Patron pour une base pilotable Swerve Drive

Un NAVX est nécessaire, car le positionnement est absolue (field centric).
Le zéro est fait au démarrage du robot, donc il est important de bien le positionner.
Presser le bouton A va également refaire son zéro.

SWERVES (FALCONS x4)
LIME LIGHT


"""

import ntcore
import phoenix6
import wpilib
from magicbot import MagicRobot
from navx import AHRS
import rev
import constants
from autonomous.auto_modes import RunAuto
# from common.arduino_light import I2CArduinoLight
from components.field import FieldLayout
# from components.climb import Climb
from components.gyro import Gyro
from components.limelight import LimeLightVision
from components.pixy import Pixy
# from components.lift import Lift
from components.intake import Intake
from components.robot_actions import ActionIntake, ActionPathTester, ActionStow
from components.swervedrive import SwerveDrive, SwerveDriveConfig
from components.swervemodule import SwerveModule, SwerveModuleConfig


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

    ##### LOW Level components #####

    # NAVX
    gyro: Gyro

    # Pixy
    pixy: Pixy

    # SwerveDrivecomponents/robot_actions.py
    frontLeftModule: SwerveModule
    frontRightModule: SwerveModule
    rearLeftModule: SwerveModule
    rearRightModule: SwerveModule
    drivetrain: SwerveDrive

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
    is_sim: bool

    def createObjects(self):
        """
        C'est ici que les composants sont vraiment créé avec le signe =.
        Les composants avec un préfix connu tel que "intake_" vont être injectés.
        """
        # NetworkTable
        self.nt = ntcore.NetworkTableInstance.getDefault().getTable("robotpy")
        self.is_sim = self.isSimulation()

        # self.arduino_light = I2CArduinoLight(wpilib.I2C.Port.kOnboard, 0x42)

        # NOTE: Remove comment to increase power over 9000 /s
        # Créé plein de problème sur le modbus, gardé en sourvenir
        # self.status_light = wpilib.Solenoid(10, wpilib.PneumaticsModuleType.CTREPCM, 1)

        # NAVX
        # self.navx = AHRS(AHRS.NavXComType.kUSB1)
        # self.gyro

        # Pneumatic Hub
        # self.pneumatic_hub = wpilib.PneumaticHub()

        # Pneumatic Hub
        # pneumatic_hub = wpilib.PneumaticHub()

        # Configuration de la base swerve
        self.initSwerve()

        #Intake
        self.intake_intake_motor = rev.SparkMax(constants.CANIds.INTAKE_INTAKE_MOTOR, rev.SparkMax.MotorType.kBrushless)
        self.intake_output_motor = rev.SparkMax(constants.CANIds.INTAKE_OUTPUT_MOTOR, rev.SparkMax.MotorType.kBrushless)

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

    def initSwerve(self):
        """
        Configuration de la base Swerve Drive
        """
        # On assigne nos moteurs à nos swerve
        # Il est important d'utiliser le logiciel de la compagnie pour trouver (ou configurer) les CAN id
        # On utilise également les encodeurs absolues CAN pour orienter la roue
        self.drivetrain_cfg = SwerveDriveConfig(
            base_width=20.75,
            base_length=22.75,
        )

        self.frontLeftModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_FL
        )
        self.frontLeftModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_FL
        )
        self.frontLeftModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_FL
        )
        self.frontLeftModule_cfg = SwerveModuleConfig(
            nt_name="frontLeftModule",
            inverted=False,
            allow_reverse=True,
            rotation_zero=193,
        )

        self.frontRightModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_FR
        )
        self.frontRightModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_FR
        )
        self.frontRightModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_FR
        )
        self.frontRightModule_cfg = SwerveModuleConfig(
            nt_name="frontRightModule",
            inverted=True,
            allow_reverse=True,
            rotation_zero=76,
        )

        self.rearLeftModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_RL
        )
        self.rearLeftModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_RL
        )
        self.rearLeftModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_RL
        )
        self.rearLeftModule_cfg = SwerveModuleConfig(
            nt_name="rearLeftModule",
            inverted=True,
            allow_reverse=True,
            rotation_zero=216,
        )

        self.rearRightModule_driveMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_DRIVE_RR
        )
        self.rearRightModule_rotateMotor = phoenix6.hardware.TalonFX(
            constants.CANIds.SWERVE_ROTATE_RR
        )
        self.rearRightModule_encoder = phoenix6.hardware.CANcoder(
            constants.CANIds.SWERVE_CANCODER_RR
        )
        self.rearRightModule_cfg = SwerveModuleConfig(
            nt_name="rearRightModule",
            inverted=False,
            allow_reverse=True,
            rotation_zero=318,
        )

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
        self.limelight_vision.light_off()
        # self.arduino_light.set_leds(LedMode.Solid, 0, 255, 0)
        pass

    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot en
        tre en mode téléopéré."""
        self.pdp.clearStickyFaults()
        self.limelight_vision.light_off()
        # self.arduino_light.set_leds(LedMode.Solid, 0, 0, 255)
        self.actionStow.engage()
        self.drivetrain.permanent_snap = False

    def teleopPeriodic(self):
        """Cette fonction est appelée de façon périodique lors du mode téléopéré."""

        if self.gamepad_pilote.getRawButton(1):
            print("Pressed!")
            self.gamepad_pilote.setOutputs(0)
            self.gamepad_pilote.setRumble(self.gamepad_pilote.RumbleType.kBothRumble, 1)
        else:
            print("Released!")
            self.gamepad_pilote.setOutputs(1)
            self.gamepad_pilote.setRumble(self.gamepad_pilote.RumbleType.kBothRumble, 0)

        self.drivetrain.set_controller_values(
            self.gamepad_pilote.getLeftY(),
            self.gamepad_pilote.getLeftX(),
            self.gamepad_pilote.getRightX(),
            self.gamepad_pilote.getRightY(),
        )

        if self.actionStow.is_executing:
            return
        elif self.gamepad_pilote.getAButton():
            self.intake.set_intake_speed(0.25)
