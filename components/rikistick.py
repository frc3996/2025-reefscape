import wpilib

from components.reefscape import (CagePositionKeys, CoralStationKeys,
                                  ReefPositionsKeys)


class RikiStick:

    def __init__(self) -> None:
        self.joystick: wpilib.Joystick = wpilib.Joystick(3)
        self._coralStation: CoralStationKeys = CoralStationKeys.LEFT
        self._reefPosition: ReefPositionsKeys = ReefPositionsKeys.A
        self._cagePosition: CagePositionKeys = CagePositionKeys.MIDDLE

    @property
    def coralStation(self) -> str:
        return str(self._coralStation)

    @property
    def cagePosition(self) -> str:
        return str(self._cagePosition)

    @property
    def reefPosition(self) -> str:
        return str(self._reefPosition)

    def execute(self):
        # ReefBranch
        if self.joystick.getRawButton(1):
            self._reefPosition = ReefPositionsKeys.A
        elif self.joystick.getRawButton(2):
            self._reefPosition = ReefPositionsKeys.B
        elif self.joystick.getRawButton(3):
            self._reefPosition = ReefPositionsKeys.C
        elif self.joystick.getRawButton(4):
            self._reefPosition = ReefPositionsKeys.D
        elif self.joystick.getRawButton(5):
            self._reefPosition = ReefPositionsKeys.E
        elif self.joystick.getRawButton(6):
            self._reefPosition = ReefPositionsKeys.F
        elif self.joystick.getRawButton(7):
            self._reefPosition = ReefPositionsKeys.G
        elif self.joystick.getRawButton(8):
            self._reefPosition = ReefPositionsKeys.H
        elif self.joystick.getRawButton(9):
            self._reefPosition = ReefPositionsKeys.I
        elif self.joystick.getRawButton(10):
            self._reefPosition = ReefPositionsKeys.J
        elif self.joystick.getRawButton(11):
            self._reefPosition = ReefPositionsKeys.K
        elif self.joystick.getRawButton(12):
            self._reefPosition = ReefPositionsKeys.L

        # CoralStationSide
        if self.joystick.getRawButton(13):
            self._coralStation = CoralStationKeys.LEFT
        if self.joystick.getRawButton(14):
            self._coralStation = CoralStationKeys.RIGHT

        # # CagePosition
        # if self.joystick.getRawButton(15):
        #     self._cagePosition = CagePositionKeys.LEFT
        # if self.joystick.getRawButton(16):
        #     self._cagePosition = CagePositionKeys.MIDDLE
        # if self.joystick.getRawButton(17):
        #     self._cagePosition = CagePositionKeys.RIGHT
