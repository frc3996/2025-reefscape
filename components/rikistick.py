from typing import Callable

import wpilib
from magicbot import feedback, tunable

from components.lift import LiftTarget

RIKISTICK2_BUTTON_TO_REEF_MAP: dict[int, int] = {  # zero-based, voir +1 ci-dessous.
    1: 1,
    0: 2,
    8: 3,
    6: 4,
    10: 5,
    5: 6,
    9: 7,
    15: 8,
    14: 9,
    13: 10,
    4: 11,
    2: 12,
}

RIKISTICK1_BUTTON_TO_LIFTTARGET_MAP: dict[int, LiftTarget] = (
    {  # zero-based, voir +1 ci-dessous.
        2: LiftTarget.L1,
        15: LiftTarget.L2,
        14: LiftTarget.L3,
        13: LiftTarget.L4,
    }
)


class RikiStick:
    editMode_0Non_1Red_2Blue = tunable(0)
    reefTarget = 12  # [1, 12]
    stationTarget = 2  # [1, 2]
    liftHeightTarget: LiftTarget = LiftTarget.DEPLACEMENT

    def __init__(self) -> None:
        self.rikistick1: wpilib.Joystick = wpilib.Joystick(1)
        self.rikistick2: wpilib.Joystick = wpilib.Joystick(2)

    def isEditMode(self) -> bool:
        return self.editMode_0Non_1Red_2Blue != 0

    def disableEditMode(self):
        self.editMode_0Non_1Red_2Blue = 0

    def getEditModeTeam(self) -> str:
        match self.editMode_0Non_1Red_2Blue:
            case 1:
                return "r"
            case 2:
                return "b"
        return ""

    @feedback
    def getCoralStationTarget(self) -> int:
        assert self.stationTarget in range(1, 3)
        return self.stationTarget

    @feedback
    def getLiftHeightTargetStr(self) -> str:
        return self.getLiftHeightTarget().name

    def getLiftHeightTarget(self) -> LiftTarget:
        return self.liftHeightTarget

    @feedback
    def getReefTarget(self) -> int:
        assert self.reefTarget in range(1, 13)
        return self.reefTarget

    def getKillSwitch(self) -> bool:
        return self.rikistick1.getRawButton(5)

    def isReefButtonPressed(self, reef: int) -> bool:  # reef: [1, 12]
        for button in RIKISTICK2_BUTTON_TO_REEF_MAP:
            if self.rikistick2.getRawButton(button + 1):
                if RIKISTICK2_BUTTON_TO_REEF_MAP[button] == reef:
                    return True
        return False

    # En mode edit, les boutons du light target sont utilisés pour configurer la pose des cages
    def isCageButtonPressed_EDIT_MODE(self, cage: int) -> bool:  # cage: [1, 3]
        for button in RIKISTICK1_BUTTON_TO_LIFTTARGET_MAP:
            if self.rikistick1.getRawButton(button + 1):
                liftTarget = RIKISTICK1_BUTTON_TO_LIFTTARGET_MAP[button]
                if liftTarget == LiftTarget.L4 and cage == 1:
                    return True
                elif liftTarget == LiftTarget.L3 and cage == 2:
                    return True
                elif liftTarget == LiftTarget.L2 and cage == 3:
                    return True
        return False

    def isStationButtonPressed(self, station: int) -> bool:  # station: [1, 2]
        return self.rikistick1.getRawButton(station)

    def execute(self):
        # Reefs
        if (not self.rikistick2 is None) and (self.rikistick2.getButtonCount() > 0):
            for button in RIKISTICK2_BUTTON_TO_REEF_MAP:
                if self.rikistick2.getRawButton(button + 1):
                    self.reefTarget = RIKISTICK2_BUTTON_TO_REEF_MAP[button]
                    break

        if (not self.rikistick1 is None) and (self.rikistick1.getButtonCount() > 0):
            # Coral station
            for i in range(1, 3):
                if self.isStationButtonPressed(i):
                    self.stationTarget = i
                    break
            # Lift targets
            for button in RIKISTICK1_BUTTON_TO_LIFTTARGET_MAP:
                if self.rikistick1.getRawButton(button + 1):
                    self.liftHeightTarget = RIKISTICK1_BUTTON_TO_LIFTTARGET_MAP[button]
                    break
