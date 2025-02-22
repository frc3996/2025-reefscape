from dataclasses import field
from typing import override

import wpilib
from magicbot import StateMachine, state, timed_state, tunable
from magicbot.state_machine import StateRef
from wpimath import controller

from autonomous.trajectory_follower_v3 import ActionPathPlannerV3
from common import tools
# from common.arduino_light import I2CArduinoLight, LedMode
from common.path_helper import PathHelper
from components.field import FieldLayout
from components.intake import ActionIntakeEntree, ActionIntakeSortie, Intake
from components.limelight import LimeLightVision
from components.pixy import Pixy
from components.swervedrive import SwerveDrive
from components.lift import Lift

class ActionStow(StateMachine):
    is_sim: bool
    # intake: Intake

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @timed_state(first=True, must_finish=True, duration=2)
    def position_all(self):
        """Premier etat, position la tete, et on s'assure que plu rien tourne"""
        if self.is_sim:
            self.done()

        # TODO
        # self.intake.disable_intake()

    def done(self):
        super().done()


class ActionIntake(StateMachine):
    lift: Lift
    intake: Intake
    action_intake_entree: ActionIntakeEntree

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def move_lift(self):
        if self.intake.has_object():
            self.done()

        self.lift.go_intake()
        if self.lift.atGoal():
            self.next_state("start_intake")

    @state
    def start_intake(self):
        self.action_intake_entree.engage()
        if not self.action_intake_entree.is_executing:
            self.next_state("finish")

    @state
    def finish(self, initial_call):
        # TODO Flasher des lumieres????
        self.lift.go_deplacement()

    def done(self) -> None:
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

    @state
    def move_reef(self):
        print("ActionCycle: MOVE_REEF")
        self.actionPathPlannerV3.move(self.field_layout.getReefPosition())
        self.next_state("wait_move_reef")

    @state
    def wait_move_reef(self):
        self.actionPathPlannerV3.move(self.field_layout.getReefPosition())
        if not self.actionPathPlannerV3.is_executing:
            self.next_state("engage_deposit")

    @state
    def engage_deposit(self):
        print("ActionCycle: engage_deposit")
        self.actionIntakeSortie.execute()
        self.next_state("wait_deposit")

    @state
    def wait_deposit(self):
        self.actionIntakeSortie.execute()
        if not self.actionIntakeSortie.is_executing:
            self.next_state("move_coral")

    @state
    def move_coral(self):
        print("ActionCycle: move_coral")
        self.actionPathPlannerV3.move(self.field_layout.getCoralStation())
        self.next_state("wait_move_coral")

    @state
    def wait_move_coral(self):
        self.actionPathPlannerV3.move(self.field_layout.getCoralStation())
        if not self.actionPathPlannerV3.is_executing:
            self.next_state("engage_intake")

    @state
    def engage_intake(self):
        print("ActionCycle: engage_intake")
        self.actionIntakeEntree.execute()
        self.next_state("wait_intake")

    @state
    def wait_intake(self):
        print("ActionCycle: engage_intake")
        self.actionIntakeEntree.execute()
        if not self.actionIntakeEntree.is_executing:
            self.next_state("move_reef")
