from dataclasses import field
from typing import override

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
from components.lift import Lift
from components.limelight import LimeLightVision
from components.pixy import Pixy
from components.reefscape import CagePositionKeys
from components.swervedrive import SwerveDrive


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
        if self.intake.has_object():
            self.done()

        self.lift.go_intake()
        if self.lift.atGoal():
            self.next_state("start_intake")

    @state
    def start_intake(self):
        print("ActionIntake", "start_intake")
        self.actionIntakeEntree.engage()
        if self.intake.has_object():
            self.next_state("finish")

    @state
    def finish(self, initial_call):
        print("ActionIntake", "finish")
        # TODO Flasher des lumieres????
        self.actionStow.engage()
        self.done()

    def done(self) -> None:
        print("ActionIntake", "done")
        return super().done()


class ActionShoot(StateMachine):
    lift: Lift
    intake: Intake
    actionIntakeSortie: ActionIntakeSortie
    actionStow: ActionStow
    chariot: Chariot
    TARGET_L1 = 0
    TARGET_L2 = 1
    TARGET_L3 = 2
    TARGET_L4 = 3
    current_target = -1
    ready_to_shoot = False
    TARGETS = {
        TARGET_L1: Lift.go_level1,
        TARGET_L2: Lift.go_level2,
        TARGET_L3: Lift.go_level3,
        TARGET_L4: Lift.go_level4,
    }

    @override
    def engage(
        self, target, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        self.current_target = target
        return super().engage(initial_state, force)

    @state(first=True)
    def move_lift(self):
        print("ActionShoot", "move_lift")
        self.ready_to_shoot = False
        if self.current_target not in range(0, 4):
            print("INVALID TARGET FOR ActionShoot")
            self.done()

        self.TARGETS[self.current_target](self.lift)
        if self.lift.atGoal() and self.chariot.get_chariot_front_limit_switch():
            self.next_state("wait_release")
        elif not self.lift.atGoal():
            print("ActionShoot: Waiting for lift")
        elif not self.chariot.get_chariot_front_limit_switch():
            print("ActionShoot: Waiting for chariot")

    @state
    def wait_release(self):
        print("ActionShoot", "wait_release")
        self.ready_to_shoot = True

    @state(must_finish=True)
    def shoot_object(self, initial_call):
        print("ActionShoot", "shoot_object")
        if initial_call:
            self.actionIntakeSortie.engage()

        if not self.actionIntakeSortie.is_executing:
            self.done()

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


class ActionCycle(StateMachine):
    intake: Intake
    actionIntakeEntree: ActionIntakeEntree
    actionIntakeSortie: ActionIntakeSortie
    actionPathPlannerV3: ActionPathPlannerV3
    field_layout: FieldLayout

    @state(first=True)
    def start(self):
        print("ActionCycle: START")
        if self.intake.has_object():
            self.next_state("move_reef")
        else:
            self.next_state("move_coral")
        self.cage_time()

    @state
    def move_reef(self):
        print("ActionCycle: MOVE_REEF")
        self.actionPathPlannerV3.move(self.field_layout.getReefPosition())
        self.next_state("wait_move_reef")
        self.cage_time()

    @state
    def wait_move_reef(self):
        self.actionPathPlannerV3.move(self.field_layout.getReefPosition())
        if not self.actionPathPlannerV3.is_executing:
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
        self.actionPathPlannerV3.move(self.field_layout.getCoralPosition())
        self.next_state("wait_move_coral")
        self.cage_time()

    @state
    def wait_move_coral(self):
        self.actionPathPlannerV3.move(self.field_layout.getCoralPosition())
        if not self.actionPathPlannerV3.is_executing:
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
