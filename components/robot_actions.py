from dataclasses import field
import os
from typing import Callable, override, List

import wpilib
from magicbot import StateMachine, state, timed_state, tunable
from magicbot.state_machine import StateRef
from wpimath import controller
from wpimath.geometry import Pose2d

from autonomous.trajectory_follower_v3 import ActionPathPlannerV3
from common import tools
# from common.arduino_light import I2CArduinoLight, LedMode
from common.path_helper import PathHelper
from components.chariot import Chariot
from components.field import FieldLayout
from components.intake import ActionIntakeEntree, ActionIntakeSortie, Intake
from components.lift import Lift, LiftTarget
from components.limelight import LimeLightVision
from components.swervedrive import SwerveDrive
from components.reefscape import Reefscape


class ActionStow(StateMachine):
    lift: Lift

    @state(first=True)
    def position_all(self):
        """Premier etat, position la tete, et on s'assure que plu rien tourne"""
        self.lift.go_deplacement()
        self.done()


class ActionIntake(StateMachine):
    lift: Lift
    intake: Intake
    actionIntakeEntree: ActionIntakeEntree
    actionStow: ActionStow

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def move_lift(self):
        print("ActionIntake", "move_lift")
        if self.intake.piece_chargee():
            self.done()

        self.lift.go_intake()
        if self.lift.atGoal():
            self.next_state("start_intake")

    @state
    def start_intake(self):
        print("ActionIntake", "start_intake")
        self.actionIntakeEntree.engage()
        if self.intake.piece_chargee():
            self.next_state("finish")

    @state
    def finish(self, initial_call):
        print("ActionIntake", "finish")
        # TODO Flasher des lumieres????
        self.done()

    def done(self) -> None:
        print("ActionIntake", "done")
        self.actionStow.engage()
        return super().done()


class ActionShoot(StateMachine):
    lift: Lift
    intake: Intake
    actionIntakeSortie: ActionIntakeSortie
    actionStow: ActionStow
    chariot: Chariot
    current_target: LiftTarget = LiftTarget.DEPLACEMENT

    def __init__(self):
        self.ready_to_shoot: bool = False
        self.TARGETS: dict[LiftTarget, Callable[[Lift], None]] = {
            LiftTarget.DEPLACEMENT: Lift.go_deplacement,
            LiftTarget.L1: Lift.go_level1,
            LiftTarget.L2: Lift.go_level2,
            LiftTarget.L3: Lift.go_level3,
            LiftTarget.L4: Lift.go_level4,
            LiftTarget.INTAKE: Lift.go_intake,
        }

    def start(self, target: LiftTarget) -> None:
        assert target in self.TARGETS
        self.current_target = target
        return super().engage()

    @state(first=True)
    def move_lift(self, initial_call: bool):
        print("ActionShoot", "move_lift")
        self.ready_to_shoot = False
        if self.current_target not in self.TARGETS:
            print("INVALID TARGET FOR ActionShoot")
            self.done()

        if initial_call:
            self.TARGETS[self.current_target](self.lift)
            return

        if self.lift.atGoal():
            self.next_state("wait_release")
        else:
            print("ActionShoot: Waiting for lift")

    @state
    def wait_release(self):
        print("ActionShoot", "wait_release")
        self.ready_to_shoot = True

    @state(must_finish=True)
    def shoot_object(self, initial_call: bool):
        print("ActionShoot", "shoot_object")
        if initial_call:
            self.actionIntakeSortie.engage()

        if not self.actionIntakeSortie.is_executing:
            self.done()

    @override
    def done(self) -> None:
        print("ActionShoot", "done")
        if self.ready_to_shoot:
            self.ready_to_shoot = False
            self.next_state("shoot_object")
            return

        # TODO Flash limieres
        self.actionStow.engage()
        return super().done()


class ActionPathTester(StateMachine):
    drivetrain: SwerveDrive
    path_kp: tunable[int] = tunable(2)
    path_ki: tunable[int] = tunable(0)
    path_kd: tunable[int] = tunable(0)
    path_profile: tunable[int] = tunable(2)
    force_reset_pose: tunable[bool] = tunable(False)

    @override
    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def init(self):
        self.auto_path: PathHelper = PathHelper(
            self.drivetrain,
            "amp_to_1",
            kp=self.path_kp,
            ki=self.path_ki,
            kd=self.path_kd,
            profile_kp=self.path_profile,
        )
        self.auto_path.init_path(force_robot_starting_position=self.force_reset_pose)
        self.next_state("move")

    @state
    def move(self):
        self.auto_path.move_to_end()
        if self.auto_path.robot_reached_end_position():
            print("Reached end!")

    @override
    def done(self):
        super().done()

class ActionCycleBase(StateMachine):
    intake: Intake
    actionIntakeEntree: ActionIntakeEntree
    actionIntakeSortie: ActionIntakeSortie
    actionPathPlannerV3: ActionPathPlannerV3
    field_layout: FieldLayout

    ### MUST OVERRIDE in ActionCycle concrete classes ####
    def get_reef_position(self) -> Pose2d | None:
        raise NotImplementedError()
    def pop_reef_position(self) -> None:
        raise NotImplementedError()
    def get_coral_position(self) -> Pose2d | None:
        raise NotImplementedError()
    def pop_coral_position(self) -> None:
        raise NotImplementedError()
    ######################################################

    @state
    def move_reef(self):
        print("ActionCycle: MOVE_REEF")
        reefPosition = self.get_reef_position()
        if reefPosition is not None:
            self.actionPathPlannerV3.move(reefPosition)
            self.next_state("wait_move_reef")
            self.cage_time()

    @state
    def wait_move_reef(self):
        reefPosition = self.get_reef_position()
        if reefPosition is not None:
            self.actionPathPlannerV3.move(reefPosition)
            if not self.actionPathPlannerV3.is_executing:
                self.pop_reef_position()
                self.next_state("engage_deposit")
            self.cage_time()

    @state
    def engage_deposit(self):
        print("ActionCycle: engage_deposit")
        self.actionIntakeSortie.execute()
        self.next_state("wait_deposit")
        self.cage_time()

    @state
    def wait_deposit(self):
        self.actionIntakeSortie.execute()
        if not self.actionIntakeSortie.is_executing:
            self.next_state("move_coral")
        self.cage_time()

    @state
    def move_coral(self):
        print("ActionCycle: move_coral")
        coralPosition = self.get_coral_position()
        if coralPosition is not None:
            self.actionPathPlannerV3.move(coralPosition)
            self.next_state("wait_move_coral")
            self.cage_time()

    @state
    def wait_move_coral(self):
        coralPosition = self.get_coral_position()
        if coralPosition is not None:
            self.actionPathPlannerV3.move(coralPosition)
            if not self.actionPathPlannerV3.is_executing:
                self.pop_coral_position()
                self.next_state("engage_intake")
            self.cage_time()

    @state
    def engage_intake(self):
        print("ActionCycle: engage_intake")
        self.actionIntakeEntree.execute()
        self.next_state("wait_intake")
        self.cage_time()

    @state
    def wait_intake(self):
        print("ActionCycle: engage_intake")
        self.actionIntakeEntree.execute()
        if not self.actionIntakeEntree.is_executing:
            self.next_state("move_reef")
        self.cage_time()

    @state
    def move_cage(self):
        print("ActionCycle: move_cage")
        cagePosition: Pose2d | None = self.field_layout.getCagePosition()
        if cagePosition is None:
            self.next_state("start")
        else:
            self.actionPathPlannerV3.move(cagePosition)
        self.next_state("wait_move_cage")

    @state
    def wait_move_cage(self):
        cagePosition: Pose2d | None = self.field_layout.getCagePosition()
        if cagePosition is None:
            self.next_state("start")
        else:
            if not self.actionPathPlannerV3.is_executing:
                self.next_state("wait_for_input")
            else:
                self.actionPathPlannerV3.move(cagePosition)

    @state
    def wait_for_input(self):
        cagePosition: Pose2d | None = self.field_layout.getCagePosition()
        if cagePosition is None:
            self.next_state("start")

    def cage_time(self):
        if self.field_layout.getCagePosition() is not None:
            self.next_state("move_cage")

class ActionCycle(ActionCycleBase):
    @state(first=True)
    def start(self):
        print("ActionCycle: START")
        if self.intake.piece_chargee():
            self.next_state("move_reef")
        else:
            self.next_state("move_coral")
        self.cage_time()

    @override
    def get_reef_position(self) -> Pose2d | None:
        return self.field_layout.getReefPosition()
    @override
    def pop_reef_position(self) -> None:
        pass
    @override
    def get_coral_position(self) -> Pose2d | None:
        return self.field_layout.getCoralPosition()
    @override
    def pop_coral_position(self) -> None:
        pass

class ActionCycleAutonomous(ActionCycleBase):

    drivetrain: SwerveDrive
    reefscape : Reefscape
    autonomousActions : List[str] = []
    AUTONOMOUS_ACTIONS_FILE = "autonomous_actions.txt"

    def __init__(self):
        self.autonomousActions = []
        pathFichier = os.path.join(os.path.dirname(__file__), r"..", self.AUTONOMOUS_ACTIONS_FILE)
        try:
            with open(pathFichier, 'r') as fichier:
                for line in fichier:
                    line = line.strip()
                    if len(line) > 0 and line[0] != "#":
                        self.autonomousActions.append(line)
        except FileNotFoundError:
            print(self.AUTONOMOUS_ACTIONS_FILE + " not found.")
        if not self._validate_autonomous_actions():
            print("Invalid autonomous actions sequence. Autonomous actions will not be executed.")
            self.autonomousActions = []

    @state(first=True)
    def start(self):
        if len(self.autonomousActions) == 0:
            print("ActionCycleAutonomous: CANNOT START -- no actions")
        else:
            print("ActionCycleAutonomous: START")
            if self.autonomousActions[0][0] == "r":
                self.next_state("move_reef")
            elif self.autonomousActions[0][0] == "s":
                self.next_state("move_coral")
            else:
                print("ActionCycleAutonomous: CANNOT START -- invalid start action")

    def _validate_autonomous_actions(self) -> bool:
        if len(self.autonomousActions) == 0:
            return False
        for action in self.autonomousActions:
            match action[0]:
                case "r":
                    if int(action[1:]) not in range(1, 13):
                        print("Invalid reef: " + action)
                        return False
                case "s":
                    if int(action[1:]) not in range(1, 3):
                        print("Invalid coral station: " + action)
                        return False
                case "l":
                    if int(action[1:]) not in range(1, 5):
                        print("Invalid level: " + action)
                        return False
                case _:
                    print("Invalid action: " + action)
                    return False
        return True

    @override
    def get_reef_position(self) -> Pose2d | None:
        if len(self.autonomousActions) == 0:
            return None
        elif self.autonomousActions[0][0] != "r":
            return None
        else:
            reef = int(self.autonomousActions[0][1:])
            pose = self.reefscape.getReef(reef)
            # print("Reef " + str(reef) + " @ x: " + str(pose.translation().x) + " y: " + str(pose.translation().y))
            return pose

    @override
    def pop_reef_position(self) -> None:
        if len(self.autonomousActions) > 0:
            self.autonomousActions.pop(0)

    @override
    def get_coral_position(self) -> Pose2d | None:
        if len(self.autonomousActions) == 0:
            print("INVALID CORAL POSITION: EMPTY LIST")
            return None
        elif self.autonomousActions[0][0] != "s":
            print("INVALID CORAL POSITION: WRONG TYPE")
            return None
        else:
            station = int(self.autonomousActions[0][1:])
            pose = self.reefscape.getClosestCoralStationSlide(station, self.drivetrain.getPose())
            # print("Station " + str(station) + " @ x: " + str(pose.translation().x) + " y: " + str(pose.translation().y))
            return pose

    @override
    def pop_coral_position(self) -> None:
        if len(self.autonomousActions) > 0:
            self.autonomousActions.pop(0)
