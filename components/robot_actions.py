import os
from dataclasses import field
from typing import Callable, List, override

import wpilib
from magicbot import StateMachine, state, timed_state, tunable
from magicbot.state_machine import StateRef
from wpimath import controller
from wpimath.geometry import Pose2d

from autonomous.trajectory_follower import TrajectoryFollower
from common import tools
# from common.arduino_light import I2CArduinoLight, LedMode
from components.chariot import Chariot
from components.field import FieldLayout
from components.intake import ActionIntakeEntree, ActionIntakeSortie, Intake
from components.lift import Lift, LiftTarget
from components.limelight import LimeLightVision
from components.reefscape import Reefscape
from components.swervedrive import SwerveDrive
from components.rikistick import RikiStick

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

    def start(self, target: LiftTarget, initial_state=None) -> None:
        assert target in self.TARGETS
        self.current_target = target
        return super().engage(initial_state)

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

        if self.lift.atGoal() and self.chariot.target_reached:
            self.next_state("wait_release")
        else:
            print("ActionShoot: Waiting for lift")

    @state
    def move_lift_auto(self, initial_call: bool):
        print("ActionShoot", "move_lift")
        self.ready_to_shoot = False
        if self.current_target not in self.TARGETS:
            print("INVALID TARGET FOR ActionShoot")
            self.done()

        if initial_call:
            self.TARGETS[self.current_target](self.lift)
            return

        if self.lift.atGoal() and self.chariot.target_reached:
            self.next_state("shoot_object")
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

class ActionCycleBase(StateMachine):
    intake: Intake
    actionTrajectoryFollower: TrajectoryFollower
    actionIntake: ActionIntake
    actionShoot: ActionShoot
    field_layout: FieldLayout
    rikiStick: RikiStick
    is_real: bool

    ### MUST OVERRIDE in ActionCycle concrete classes ####
    def get_reef_target(self) -> str:
        raise NotImplementedError()

    def pop_reef_target(self) -> None:
        raise NotImplementedError()

    def get_coral_target(self) -> str:
        raise NotImplementedError()

    def pop_coral_target(self) -> None:
        raise NotImplementedError()

    def get_level_target(self) -> LiftTarget:
        raise NotImplementedError()

    def pop_level_target(self) -> None:
        raise NotImplementedError()

    ######################################################

    @state
    def move_reef(self):
        print("ActionCycle: MOVE_REEF")
        reefPosition = self.get_reef_target()
        if reefPosition != "":
            self.actionTrajectoryFollower.move(reefPosition)
            self.next_state("wait_move_reef")

    @state
    def wait_move_reef(self):
        reefPosition = self.get_reef_target()
        if reefPosition != "":
            self.actionTrajectoryFollower.move(reefPosition)
            if not self.actionTrajectoryFollower.is_executing:
                self.pop_reef_target()
                self.next_state("engage_deposit")

    @state
    def engage_deposit(self):
        print("ActionCycle: engage_deposit")
        self.actionShoot.start(self.get_level_target(), "move_lift_auto")
        self.next_state("wait_deposit")

    @state
    def wait_deposit(self):
        self.actionShoot.engage("move_lift_auto")
        if (not self.is_real) or (not self.actionShoot.is_executing):
            self.next_state("move_coral")

    @state
    def move_coral(self):
        print("ActionCycle: move_coral")
        coralPosition = self.get_coral_target()
        if coralPosition != "":
            self.actionTrajectoryFollower.move(coralPosition)
            self.next_state("wait_move_coral")

    @state
    def wait_move_coral(self):
        coralPosition = self.get_coral_target()
        if coralPosition != "":
            self.actionTrajectoryFollower.move(coralPosition)
            if not self.actionTrajectoryFollower.is_executing:
                self.pop_coral_target()
                self.next_state("engage_intake")

    @state
    def engage_intake(self):
        print("ActionCycle: engage_intake")
        self.actionIntake.engage()
        self.next_state("wait_intake")

    @state
    def wait_intake(self):
        print("ActionCycle: engage_intake")
        self.actionIntake.engage()
        if (not self.is_real) or (not self.actionIntake.is_executing):
            self.next_state("move_reef")


class ActionCycle(ActionCycleBase):
    @state(first=True)
    def start(self):
        print("ActionCycle: START")
        if self.intake.piece_chargee():
            self.next_state("move_reef")
        else:
            self.next_state("move_coral")

    @override
    def get_reef_target(self) -> str:
        return self.field_layout.getReefTarget()

    @override
    def pop_reef_target(self) -> None:
        pass

    @override
    def get_coral_target(self) -> str:
        return self.field_layout.getCoralTarget()

    @override
    def pop_coral_target(self) -> None:
        pass

    @override
    def get_level_target(self) -> LiftTarget:
        return self.rikiStick.getLiftHeightTarget()

    @override
    def pop_level_target(self) -> None:
        pass

class ActionCycleAutonomous(ActionCycleBase):

    drivetrain: SwerveDrive
    reefscape: Reefscape
    AUTONOMOUS_ACTIONS_FILE = "autonomous_actions.txt"

    def __init__(self):
        super().__init__()
        self.autonomousActions: List[str] = list()

    def on_enable(self) -> None:
        super().on_enable()
        print("ActionCycleAutonomous: INIT")
        self.autonomousActions = []
        pathFichier = os.path.join(
            os.path.dirname(__file__), r"..", self.AUTONOMOUS_ACTIONS_FILE
        )
        try:
            with open(pathFichier, "r") as fichier:
                for line in fichier:
                    line = line.strip()
                    if len(line) > 0 and line[0] != "#":
                        self.autonomousActions.append(line)
        except FileNotFoundError:
            print(self.AUTONOMOUS_ACTIONS_FILE + " not found.")
        if not self._validate_autonomous_actions():
            print(
                "Invalid autonomous actions sequence."
            )

    @state(first=True)
    def start(self):
        if len(self.autonomousActions) == 0:
            print("ActionCycleAutonomous: CANNOT START -- no actions. Defaulting to reef.")
            self.next_state("move_reef")
        else:
            print("ActionCycleAutonomous: START")
            if self.autonomousActions[0][0] == "r":
                self.next_state("move_reef")
            elif self.autonomousActions[0][0] == "s":
                self.next_state("move_coral")
            elif self.autonomousActions[0][0] == "l":
                self.next_state("engage_deposit")
            else:
                print("ActionCycleAutonomous: CANNOT START -- invalid start action. Defaulting to reef.")
                self.next_state("move_reef")

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
                    if int(action[1]) not in range(1, 3) or int(action[3:]) not in range(1, 5):
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
    def get_reef_target(self) -> str:
        if len(self.autonomousActions) == 0:
            print("INVALID REEF TARGET -- EMPTY LIST")
            return self.__getAlliancePrefix() + "r12" # lets try! 
        elif self.autonomousActions[0][0] != "r":
            print("INVALID REEF POSITION -- WRONG TYPE: " + self.autonomousActions[0])
            return self.__getAlliancePrefix() + "r12" # lets try! 
        else:
            return self.__getAlliancePrefix() + self.autonomousActions[0]

    @override
    def pop_reef_target(self) -> None:
        if len(self.autonomousActions) > 0:
            self.autonomousActions.pop(0)

    @override
    def get_coral_target(self) -> str:
        if len(self.autonomousActions) == 0:
            print("INVALID CORAL POSITION -- EMPTY LIST")
            return self.__getAlliancePrefix() + "s2_2" # lets try! 
        elif self.autonomousActions[0][0] != "s":
            print("INVALID CORAL POSITION -- WRONG TYPE: " + self.autonomousActions[0])
            return self.__getAlliancePrefix() + "s2_2" # lets try! 
        else:
            return self.__getAlliancePrefix() + self.autonomousActions[0]

    @override
    def pop_coral_target(self) -> None:
        if len(self.autonomousActions) > 0:
            self.autonomousActions.pop(0)

    @override
    def get_level_target(self) -> LiftTarget:
        if len(self.autonomousActions) == 0:
            print("INVALID LIFT TARGET -- EMPTY LIST")
            return LiftTarget.L2 # lets try!
        elif self.autonomousActions[0][0] != "l":
            print("INVALID LIFT TARGET -- WRONG TYPE: " + self.autonomousActions[0])
            return LiftTarget.L2
        else:
            targetLevel = int(self.autonomousActions[0][1:])
            targetLevel = max(1, min(targetLevel, 4)) # clamp to [1, 4]
            return LiftTarget(targetLevel)

    @override
    def pop_level_target(self) -> None:
        if len(self.autonomousActions) > 0:
            self.autonomousActions.pop(0)

    def __getAlliancePrefix(self) -> str:
        if tools.is_red():
            return "r_"
        else:
            return "b_"
