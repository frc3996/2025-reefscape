import math

import numpy
import wpilib
import wpimath.units
from magicbot import StateMachine, feedback, state, timed_state, tunable
from magicbot.state_machine import StateRef
from wpimath import controller, geometry

import constants
from common import tools
from common.arduino_light import I2CArduinoLight, LedMode
from common.path_helper import PathHelper
from components.field import FieldLayout
from components.limelight import LimeLightVision
from components.pixy import Pixy
from components.lift import Lift
from components.intake import Intake
from components.swervedrive import SwerveDrive


class ActionStow(StateMachine):
    is_sim: bool
    intake: Intake

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
        self.intake.disable_intake()

    def done(self):
        super().done()


class ActionIntake(StateMachine):
    lift: Lift
    intake: Intake
    arduino_light: I2CArduinoLight
    drivetrain: SwerveDrive
    pixy: Pixy
    auto_intake_kp = tunable(0.001)  # For relative_rotate: 0.015
    auto_intake_ki = tunable(0)  # For relative_rotate: 0
    auto_intake_kd = tunable(0)  # For relative_rotate: 0
    auto_intake_pid = controller.PIDController(0, 0, 0)
    intake_target_angle = tunable(111)
    intake_target_speed = tunable(1.25)
    actionStow: ActionStow
    limelight_vision: LimeLightVision

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def position_lift(self):
        """Premier etat, position la tete"""
        if self.intake.has_object():
            self.limelight_vision.light_blink()
            self.next_state("finish")
            return

        self.arduino_light.set_leds(LedMode.BlinkSlow, 0, 255, 0)

        self.auto_intake_pid.setPID(
            self.auto_intake_kp,
            self.auto_intake_ki,
            self.auto_intake_kd,
        )
        self.auto_intake_pid.reset()

        self.lift.go_intake()
        self.next_state("start_intake")

    @state
    def start_intake(self, initial_call):
        if initial_call:
            self.timer = wpilib.Timer()
            self.timer.start()

        if tools.is_autonomous() and self.timer.get() > 2:
            self.next_state("finish")

        if self.intake.has_object():
            self.arduino_light.set_leds(LedMode.BlinkFast, 0, 255, 0)
            self.limelight_vision.light_blink()
            self.next_state("finish")

        if initial_call:
            self.intake.enable_intake()

        # # Automove to target
        if self.pixy.get_target_valid():
            offset = self.pixy.get_offset()
            res = tools.map_value(abs(offset), 0, 1000, 0, 0.5)
            fwd = self.intake_target_speed * (1 - res)
            error = self.auto_intake_pid.calculate(offset, 0)
        else:
            fwd = 0.35
            if tools.is_autonomous():
                fwd = 0.6
            error = 0
        self.drivetrain.set_robot_relative_automove_value(-fwd, -error)

    @state
    def finish(self, initial_call):
        if tools.is_autonomous():
            self.intake.jiggle()
            self.done()
            return
        if initial_call:
            self.lobras_head.set_angle(70)
            self.intake.jiggle()

    def done(self) -> None:
        self.arduino_light.set_leds(LedMode.Solid, 0, 255, 0)
        self.limelight_vision.light_off()
        if not tools.is_autonomous():
            self.actionStow.engage()
        return super().done()


class ActionPathTester(StateMachine):
    actionStow: ActionStow
    drivetrain: SwerveDrive
    path_kp = tunable(2)
    path_ki = tunable(0)
    path_kd = tunable(0)
    path_profile = tunable(2)
    force_reset_pose = tunable(False)

    def engage(
        self, initial_state: StateRef | None = None, force: bool = False
    ) -> None:
        return super().engage(initial_state, force)

    @state(first=True)
    def init(self):
        self.auto_path = PathHelper(
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

    def done(self):
        self.actionStow.engage()
        super().done()
