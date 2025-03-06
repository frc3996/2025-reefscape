#!/usr/bin/env python3

"""
Patron pour une base pilotable Swerve Drive

Un NAVX est nécessaire, car le positionnement est absolue (field centric).
Le zéro est fait au démarrage du robot, donc il est important de bien le positionner.
Presser le bouton A va également refaire son zéro.

SWERVES (FALCONS x4) LIME LIGHT


"""

import math
from datetime import datetime
from time import time
from typing import Callable, Dict, override

import ntcore
import rev
import wpilib
import wpimath.geometry
from magicbot import MagicRobot
from navx import AHRS
from pathplannerlib.commands import Pose2d
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
from common.limelight_helpers import LimelightHelpers, PoseEstimate
from components.chariot import Chariot
# from components.climb import ActionClimb, Climb
from components.field import FieldLayout
from components.gyro import Gyro
from components.intake import ActionIntakeEntree, ActionIntakeSortie, Intake
from components.lift import Lift, LiftTarget
from components.limelight_me import LimeLightVision
from components.reefscape import Reefscape
from components.rikistick import RikiStick
from components.robot_actions import (ActionCycle, ActionCycleAutonomous,
                                      ActionIntake, ActionShoot, ActionStow)
from components.swervemodule import SwerveModule

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
    actionCycleAutonomous: ActionCycleAutonomous

    ##### HIGH Level components first (components that use components) #####
    # actionClimb: ActionClimb
    actionIntakeEntree: ActionIntakeEntree
    actionIntakeSortie: ActionIntakeSortie
    actionTrajectoryFollower: TrajectoryFollower

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

    # SwerveDrive
    drivetrain: swervedrive.SwerveDrive
    snapAngle: swervedrive.SnapAngle

    # FieldLayout
    field_layout: FieldLayout
    reefscape: Reefscape

    # Rikistick
    rikiStick: RikiStick

    # Lift
    lift: Lift

    # # climb
    # climb: Climb

    # Chariot
    chariot: Chariot

    # intake
    intake: Intake

    # Networktables pour de la configuration et retour d'information
    nt: ntcore.NetworkTable

    # Autonomous cycling toggle
    isAutoCycling: bool = True

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

        # NAVX
        # self.navx: AHRS = AHRS.create_spi()
        self.navx: AHRS = AHRS(AHRS.NavXComType.kUSB1)

        self.drivetrain_frontLeft = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_FL,
            constants.CANIds.SWERVE_ROTATE_FL,
            constants.CANIds.SWERVE_CANCODER_FL,
            1,
            rotation_zero=122,
            inverted=False,
        )
        self.drivetrain_frontRight = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_FR,
            constants.CANIds.SWERVE_ROTATE_FR,
            constants.CANIds.SWERVE_CANCODER_FR,
            2,
            rotation_zero=102,
            inverted=False,
        )
        self.drivetrain_backLeft = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_RL,
            constants.CANIds.SWERVE_ROTATE_RL,
            constants.CANIds.SWERVE_CANCODER_RL,
            3,
            rotation_zero=-100,
            inverted=False,
        )
        self.drivetrain_backRight = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_RR,
            constants.CANIds.SWERVE_ROTATE_RR,
            constants.CANIds.SWERVE_CANCODER_RR,
            4,
            rotation_zero=312,
            inverted=False,
        )

        # General
        self.gamepad_pilote = wpilib.XboxController(0)

        self.pdp = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)
        self.pdp.clearStickyFaults()

        self.limelightBack = LimeLightVision("limelight-back")
        self.limelightFront = LimeLightVision("limelight-front")

    @override
    def robotInit(self):
        return super().robotInit()

    @override
    def robotPeriodic(self) -> None:
        backMesurement = self.limelightBack.getVisionMesurement(self.drivetrain)
        _ = self.limelightFront.getVisionMesurement(
            self.drivetrain, backMesurement, True
        )

        self.drivetrain.updateOdometry()
        # self.drivetrain.log()

    @override
    def disabledInit(self) -> None:
        pass

    @override
    def disabledPeriodic(self):
        """Mets à jours le dashboard, même quand le robot est désactivé"""
        pass

    @override
    def autonomousInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot entre en mode autonome."""
        pass

    @override
    def teleopInit(self):
        """Cette fonction est appelée une seule fois lorsque le robot en
        tre en mode téléopéré."""
        self.pdp.clearStickyFaults()

    @override
    def teleopPeriodic(self) -> None:
        # Always drive
        self.teleopDrive()

        # Sub-modes
        if self.rikiStick.isEditMode():
            self.teleopTerrainEditMode()
        else:
            self.teleopManualOperations()
            # self.teleopAutonomousCycle()

    def teleopTerrainEditMode(self):
        assert self.rikiStick.isEditMode()
        if self.rikiStick.getKillSwitch():
            self.rikiStick.disableEditMode()
            self.reefscape.save(
                "terrain-" + datetime.now().strftime("%Y-%m-%d.%H.%M.%S") + ".json"
            )
            return
        team = self.rikiStick.getEditModeTeam()
        assert team != ""
        # Reefs
        for i in range(1, 13):
            if self.rikiStick.isReefButtonPressed(i):
                self.reefscape.setReefPose(team, i, self.drivetrain.getPose())
                return
        # Station slides
        for i in range(1, 3):
            if self.rikiStick.isStationButtonPressed(i):
                # 4 slides = A, B, X, Y
                slideID = 0
                if self.gamepad_pilote.getAButton():
                    slideID = 1
                elif self.gamepad_pilote.getBButton():
                    slideID = 2
                elif self.gamepad_pilote.getXButton():
                    slideID = 3
                elif self.gamepad_pilote.getYButton():
                    slideID = 4

                if slideID != 0:
                    self.reefscape.setStationSlidePose(
                        team, i, slideID, self.drivetrain.getPose()
                    )
                    return
        # Cages
        for i in range(1, 4):
            if self.rikiStick.isCageButtonPressed_EDIT_MODE(i):
                self.reefscape.setCagePose(team, i, self.drivetrain.getPose())
                return

    def teleopManualOperations(self):
        if (self.gamepad_pilote is None) or (self.gamepad_pilote.getButtonCount() <= 0):
            return

        # Shoot
        if self.gamepad_pilote.getAButton():
            self.snapAngle.engage(self.field_layout.getReefTargetPosition())
            self.actionShoot.start(LiftTarget.L1)
        elif self.gamepad_pilote.getBButton():
            self.snapAngle.engage(self.field_layout.getReefTargetPosition())
            self.actionShoot.start(LiftTarget.L2)
        elif self.gamepad_pilote.getXButton():
            self.snapAngle.engage(self.field_layout.getReefTargetPosition())
            self.actionShoot.start(LiftTarget.L3)
        elif self.gamepad_pilote.getYButton():
            self.snapAngle.engage(self.field_layout.getReefTargetPosition())
            self.actionShoot.start(LiftTarget.L4)

        # Intake coral
        if self.gamepad_pilote.getLeftTriggerAxis() > 0.5:
            stationPose = self.field_layout.getCoralTargetPosition()
            self.snapAngle.engage(
                stationPose.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(180))
            )
            self.actionIntake.engage()

        # Deposit coral
        if self.gamepad_pilote.getRightTriggerAxis() > 0.5:
            self.actionIntakeSortie.engage()

        # Manual climb
        if self.gamepad_pilote.getStartButton():
            pass
            # self.actionClimb.engage(forward ou +1 direction)
        elif self.gamepad_pilote.getBackButton():
            pass
            # self.actionClimb.engage(backward ou -1 direction)
        else:
            pass
            # self.actionClimb.done()

    def teleopAutonomousCycle(self):
        if self.rikiStick is None:
            self.isAutoCycling = False
        elif self.rikiStick.getKillSwitch():
            self.isAutoCycling = False
        elif abs(self.gamepad_pilote.getPOV() - 180) < 5:
            self.isAutoCycling = not self.isAutoCycling  # toggle

        if self.isAutoCycling:
            self.actionCycle.engage()

    def teleopDrive(self):
        leftY = gamepad_helper.apply_deadzone(
            self.leftYFilter.calculate(self.gamepad_pilote.getLeftY()), 0.2
        )
        leftX = gamepad_helper.apply_deadzone(
            self.leftXFilter.calculate(self.gamepad_pilote.getLeftX()), 0.2
        )
        rightX = gamepad_helper.apply_deadzone(
            self.rightXFilter.calculate(self.gamepad_pilote.getRightX()), 0.2
        )

        xSpeed = -1.0 * leftY * swervedrive.kMaxSpeed
        ySpeed = -1.0 * leftX * swervedrive.kMaxSpeed
        rot = -1.0 * rightX * swervedrive.kMaxAngularSpeed

        deltaHauteur = self.lift.get_lift_height() - self.lift.hauteurDeplacement
        if deltaHauteur > 0:
            scale = 1 - deltaHauteur / (
                self.lift.hauteurLevel4 - self.lift.hauteurDeplacement
            )
            xSpeed *= scale
            ySpeed *= scale

        self.drivetrain.drive(xSpeed, ySpeed, rot, True)
