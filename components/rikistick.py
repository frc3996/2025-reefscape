import wpilib
from magicbot import feedback, tunable

RIKISTICK2_BUTTON_TO_REEF_MAP = {
    2: 1,
    1: 2,
    9: 3,
    7: 4,
    11: 5,
    6: 6,
    10: 7,
    16: 8,
    15: 9,
    14: 10,
    5: 11,
    3: 12
}

def valtest(v : int):
    return v

class RikiStick:

    reefTarget = tunable(1)      # [1, 12]
    stationTarget = tunable(1)   # [1, 2]
    cageTarget = tunable(0)      # [1, 3]

    def __init__(self) -> None:
        self.joystick: wpilib.Joystick = wpilib.Joystick(5)
        self.rikistick1: wpilib.Joystick = wpilib.Joystick(1)
        self.rikistick2: wpilib.Joystick = wpilib.Joystick(2)

    @feedback
    def getCoralStationTarget(self) -> int:
        assert(self.stationTarget in range(1, 3))
        valtest(self.stationTarget)
        return self.stationTarget

    @feedback
    def getCageTarget(self) -> int: # Zero si pas de cage cible
        assert(self.cageTarget in range(0, 4))
        return self.cageTarget

    @feedback
    def getReefTarget(self) -> int:
        assert(self.reefTarget in range(1, 13))
        return self.reefTarget

    def execute(self):
        # Reefs
        if (not self.rikistick2 is None) and (self.rikistick2.getButtonCount() != 0):
            for button in RIKISTICK2_BUTTON_TO_REEF_MAP:
                if self.rikistick2.getRawButton(button):
                    self._reefTarget = RIKISTICK2_BUTTON_TO_REEF_MAP[button]
                    break

        if (not self.rikistick1 is None) and (self.rikistick1.getButtonCount() != 0):
            # Coral station
            if self.rikistick1.getRawButton(1):
                self._coralStation = 1
            elif self.rikistick1.getRawButton(2):
                self._coralStation = 2
            # Cage position
            if self.rikistick1.getRawButtonPressed(14):
                self._cagePosition = 1
            if self.rikistick1.getRawButtonPressed(15):
                self._cagePosition = 2
            if self.rikistick1.getRawButtonPressed(16):
                self._cagePosition = 3

    def execute_sim(self):
        if self.joystick is None or self.joystick.getButtonCount() == 0:
            return

        for i in range(1, 13):
            if self.joystick.getRawButton(i):
                self._reefPosition = i
                break
        if self.joystick.getRawButton(13):
            self._coralStation = 1
        elif self.joystick.getRawButton(14):
            self._coralStation = 2

        # # CagePosition
        # if self.joystick.getRawButton(15):
        #     self._cagePosition = 1
        # elif self.joystick.getRawButton(16):
        #     self._cagePosition = 2
        # elif self.joystick.getRawButton(17):
        #     self._cagePosition = 3
