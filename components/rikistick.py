import wpilib
from magicbot import feedback, tunable
from components.lift import LiftTarget

RIKISTICK2_BUTTON_TO_REEF_MAP = { # zero-based, voir +1 ci-dessous.
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
    2: 12
}

class RikiStick:

    # Terrain edit mode ("r" pour red terrain, "b" pour blue terrain, "" pour pas d'edit)
    editMode_0Non_1Red_2Blue = tunable(0)
    reefTarget = 1 # [1, 12]
    stationTarget = 1 # [1, 2]
    liftHeightTarget : LiftTarget = LiftTarget.DEPLACEMENT
    cageTarget = 0 # [0,3]

    def __init__(self) -> None:
        self.joystick: wpilib.Joystick = wpilib.Joystick(5)
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
        assert(self.stationTarget in range(1, 3))
        return self.stationTarget

    @feedback
    def getLiftHeightTargetStr(self) -> str:
        return self.getLiftHeightTarget().name

    def getLiftHeightTarget(self) -> LiftTarget:
        return self.liftHeightTarget

    def getCageTarget(self) -> int: # Zero si pas de cage cible
        return self.cageTarget

    @feedback
    def getReefTarget(self) -> int:
        assert(self.reefTarget in range(1, 13))
        return self.reefTarget
    
    def getKillSwitch(self) -> bool:
        return self.rikistick1.getRawButton(5)
    
    def isReefButtonPressed(self, reef: int) -> bool: # reef: [1, 12]
        for button in RIKISTICK2_BUTTON_TO_REEF_MAP:
            if self.rikistick2.getRawButton(button + 1):
                if RIKISTICK2_BUTTON_TO_REEF_MAP[button] == reef:
                    return True
        return False

    def isStationButtonPressed(self, station: int) -> bool: # station: [1, 2]
        return self.rikistick1.getRawButton(station)

    def isCageButtonPressed(self, cage: int) -> bool: # cage: [1, 3]
        if cage == 1 and self.rikistick1.getRawButtonPressed(14):
            return True
        elif cage == 2 and self.rikistick1.getRawButtonPressed(15):
            return True
        elif cage == 3 and self.rikistick1.getRawButtonPressed(16):
            return True
        return False

    def execute(self):
        # Reefs
        if (not self.rikistick2 is None) and (self.rikistick2.getButtonCount() != 0):
            for button in RIKISTICK2_BUTTON_TO_REEF_MAP:
                if self.rikistick2.getRawButton(button + 1):
                    self.reefTarget = RIKISTICK2_BUTTON_TO_REEF_MAP[button]
                    break

        if (not self.rikistick1 is None) and (self.rikistick1.getButtonCount() != 0):
            # Coral station
            for i in range(1, 3):
                if self.isStationButtonPressed(i):
                    self.stationTarget = i
                    break
            # Cage position
            for i in range(1, 4):
                if self.isCageButtonPressed(i):
                    self.cageTarget = i
                    break

    def execute_sim(self):
        if self.joystick is None or self.joystick.getButtonCount() == 0:
            return

        for i in range(1, 13):
            if self.joystick.getRawButton(i):
                self.reefTarget = i
                break
        if self.joystick.getRawButton(13):
            self.stationTarget = 1
        elif self.joystick.getRawButton(14):
            self.stationTarget = 2

        # # CagePosition
        # if self.joystick.getRawButton(15):
        #     self._cagePosition = 1
        # elif self.joystick.getRawButton(16):
        #     self._cagePosition = 2
        # elif self.joystick.getRawButton(17):
        #     self._cagePosition = 3
