#!/usr/bin/env python3

"""
Patron pour une base pilotable Swerve Drive

Un NAVX est nécessaire, car le positionnement est absolue (field centric).
Le zéro est fait au démarrage du robot, donc il est important de bien le positionner.
Presser le bouton A va également refaire son zéro.

SWERVES (FALCONS x4) LIME LIGHT


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
from autonomous.pathplanner import ActionPathPlanner
from autonomous.sysid import AngularMaxVelocity, MaxAccel, MaxVelocity
from autonomous.trajectory_follower import TrajectoryFollower
from autonomous.trajectory_follower_v2 import TrajectoryFollowerV2
from autonomous.trajectory_follower_v3 import ActionPathPlannerV3
from common import gamepad_helper
from components.chariot import Chariot
from components.climb import ActionClimb, Climb
from components.field import FieldLayout
from components.gyro import Gyro
from components.intake import ActionIntakeEntree, ActionIntakeSortie, Intake
from components.lift import Lift
from components.limelight import LimeLightVision
from components.pixy import Pixy
from components.rikistick import RikiStick
from components.reefscape import Reefscape
from components.robot_actions import (ActionCycle, ActionIntake, ActionShoot,
                                      ActionStow)

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
    actionCycle: ActionCycle

    ##### HIGH Level components first (components that use components) #####
    actionClimb: ActionClimb
    actionIntakeEntree: ActionIntakeEntree
    actionIntakeSortie: ActionIntakeSortie
    actionTrajectoryFollower: TrajectoryFollower
    actionPathPlannerV3: ActionPathPlannerV3

    ## SysId
    actionAngularMaxVelocity: AngularMaxVelocity
    actionMaxVelocity: MaxVelocity
    actionMaxAccel: MaxAccel

    ## Manual Mode
    actionShoot: ActionShoot
    actionIntake: ActionIntake
    actionStow: ActionStow

    ##### LOW Level components #####

    # NAVX
    gyro: Gyro

    # Pixy
    pixy: Pixy

    # SwerveDrive
    drivetrain: swervedrive.SwerveDrive

    # FieldLayout
    field_layout: FieldLayout
    reefscape : Reefscape

    # Vision
    limelight_vision: LimeLightVision

    # Pneumatic Hub
    pneumaticHub: wpilib.PneumaticHub

    # Rikistick
    rikiStick: RikiStick

    # Lift
    lift: Lift

    # climb
    climb: Climb

    # Chariot
    chariot: Chariot

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
        self.pneumaticHub = wpilib.PneumaticHub()

        # Climb
        # self.climb = Climb()

        # General
        self.gamepad_pilote = wpilib.XboxController(0)

        self.pdp = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)
        self.pdp.clearStickyFaults()

    @override
    def disabledInit(self) -> None:
        pass
        # self.arduino_light.set_leds(LedMode.Solid, 0, 255, 0)

    @override
    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        # self.limelight_vision.execute(()
        pass

    @override
    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        # self.limelight_vision.light_off()
        # self.arduino_light.set_leds(LedMode.Solid, 0, 255, 0)
        pass

    @override
    def autonomous(self):
        """Pour les modes auto de MagicBot, voir le dossier ./autonomous"""
        super().autonomous()

    @override
    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot en
        tre en mode téléopéré."""
        self.pdp.clearStickyFaults()
        # self.limelight_vision.light_off()
        # self.arduino_light.set_leds(LedMode.Solid, 0, 0, 255)

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
        # if not self.gamepad_copilote.getRawButton(1):
        self.teleopManualMode()
        # return

        # self.teleopDrive()
        # self.teleopLift()
        # self.teleopClimb()
        # self.teleopIntake()
        # self.teleopCycle()

    def teleopCycle(self):
        leftY = gamepad_helper.apply_deadzone(
            self.leftYFilter.calculate(self.gamepad_pilote.getLeftY()), 0.1
        )
        leftX = gamepad_helper.apply_deadzone(
            self.leftXFilter.calculate(self.gamepad_pilote.getLeftX()), 0.1
        )
        rightX = gamepad_helper.apply_deadzone(
            self.rightXFilter.calculate(self.gamepad_pilote.getRawAxis(3)),
            0.1,
        )

        # Autonomous cycling
        if self.gamepad_pilote.getAButton():
            self.actionCycle.engage()
        elif self.gamepad_pilote.getAButtonReleased():
            self.actionCycle.done()

        # Intake coral
        if self.gamepad_pilote.getRawAxis(2) > 0.5:
            self.actionIntakeEntree.engage()
        elif self.actionIntakeEntree.is_executing:
            self.actionIntakeEntree.done()

        # Deposit coral
        if self.gamepad_pilote.getRawAxis(5) > 0.5:
            self.actionIntakeSortie.engage()
        elif self.actionIntakeSortie.is_executing:
            self.actionIntakeSortie.done()

        # Coral level
        if abs(self.gamepad_pilote.getPOV() - 0) < 5:
            # LEVEL 4
            pass
        elif abs(self.gamepad_pilote.getPOV() - 270) < 5:
            # LEVEL 3
            pass
        elif abs(self.gamepad_pilote.getPOV() - 90) < 5:
            # LEVEL 2
            pass
        elif abs(self.gamepad_pilote.getPOV() - 180) < 5:
            # LEVEL 2
            pass

        # Climb
        if self.gamepad_pilote.getLeftBumper():
            # Climb deploy
            pass
        if self.gamepad_pilote.getRightBumper():
            # Climb climb
            pass

        xSpeed = -1.0 * leftY * swervedrive.kMaxSpeed
        ySpeed = -1.0 * leftX * swervedrive.kMaxSpeed
        rot = -1.0 * rightX * swervedrive.kMaxAngularSpeed

        self.drivetrain.drive(xSpeed, ySpeed, rot, True)

    def teleopIntake(self):
        if self.gamepad_pilote.getAButton():
            self.actionIntakeEntree.engage()
        if self.gamepad_pilote.getBButton():
            self.actionIntakeSortie.engage()
        else:
            self.actionClimb.done()

    def teleopClimb(self):
        if self.gamepad_pilote.getAButton():
            self.actionClimb.engage()
        elif self.gamepad_pilote.getAButtonReleased():
            self.actionClimb.done()

    def teleopLift(self):
        if self.gamepad_pilote.getAButton():
            self.lift.go_intake()
        elif self.gamepad_pilote.getXButton():
            self.lift.go_level1()
        elif self.gamepad_pilote.getYButton():
            self.lift.go_level2()
        elif self.gamepad_pilote.getBButton():
            self.lift.go_level3()
        else:
            self.lift.go_deplacement()

    def teleopDrive(self):
        # TODO
        # Moduler la vitesse du robot en téléop selon la hauteur du lift

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

        if self.gamepad_pilote.getAButton():

            # self.actionTrajectoryFollower.engage()
            # self.actionPathPlanner.engage()
            self.actionPathPlannerV3.engage()
            # self.actionTrajectoryFollowerV2.engage()
        elif self.gamepad_pilote.getAButtonReleased():
            # Only call it once..
            # self.actionTrajectoryFollower.done()
            # self.actionPathPlanner.done()
            self.actionPathPlannerV3.done()
            # self.actionTrajectoryFollowerV2.done()

        if self.gamepad_pilote.getBButton():
            # self.actionAngularMaxVelocity.engage()
            # self.actionMaxVelocity.engage()
            self.actionMaxAccel.engage()
            return

        xSpeed = -1.0 * leftY * swervedrive.kMaxSpeed
        ySpeed = -1.0 * leftX * swervedrive.kMaxSpeed
        rot = -1.0 * rightX * swervedrive.kMaxAngularSpeed

        self.drivetrain.drive(xSpeed, ySpeed, rot, True)

    def teleopManualMode(self):
        # self.teleopDrive()

        if self.gamepad_pilote.getBButton():
            self.actionShoot.engage(self.actionShoot.TARGET_L1)
        elif self.gamepad_pilote.getAButton():
            self.actionShoot.engage(self.actionShoot.TARGET_L2)
        elif self.gamepad_pilote.getXButton():
            self.actionShoot.engage(self.actionShoot.TARGET_L3)
        elif self.gamepad_pilote.getYButton():
            self.actionShoot.engage(self.actionShoot.TARGET_L4)
        elif self.gamepad_pilote.getLeftBumper():
            self.actionIntake.engage()
        elif self.gamepad_pilote.getRightBumper():
            self.actionIntakeSortie.engage()
