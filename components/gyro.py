from navx import AHRS
from wpimath.geometry import Rotation2d
from wpimath.kinematics import ChassisSpeeds


class Gyro:
    # navx: AHRS
    is_sim: bool

    def setup(self):
        self.sim_angle: Rotation2d = Rotation2d(0)

    def reset(self, current_rotation: Rotation2d):
        pass
        # if not self.is_sim:
        #     self.navx.reset()

    def update(self, chassis_speed: ChassisSpeeds):
        self.sim_angle = self.sim_angle + Rotation2d.fromDegrees(
            chassis_speed.omega * 1 * 20
        )

    def getRotation2d(self) -> Rotation2d:
        # Get angle from navx!
        if self.is_sim:
            return self.sim_angle

        # if self.navx.isConnected():
        #     return self.navx.getRotation2d()
        #Bidon
        return Rotation2d()
        # raise Exception("ABORT")

    def execute(self):
        pass
