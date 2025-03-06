import traceback

from magicbot import feedback
from navx import AHRS
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds


class Gyro:
    navx: AHRS
    is_real: bool

    def reset(self):
        self.navx.reset()

    def ok(self) -> bool:
        return self.navx is not None and self.navx.isConnected()

    def yawSpeed(self) -> float:
        if self.ok():
            return self.navx.getRawGyroZ()  # Unprocessed?
        else:
            return 0

    def getRotation2d(self) -> Rotation2d:
        # Get angle from navx!
        if self.navx.isConnected():
            # print(f"NAVX = {self.navx.getRotation2d()}")
            return self.navx.getRotation2d()

        if self.is_real:
            raise Exception(f"NAVX IS DISCONNECTED {traceback.extract_stack()}")
        else:
            return self.navx.getRotation2d()

    def execute(self):
        pass
