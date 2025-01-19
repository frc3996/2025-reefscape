import wpilib  # Librairie de base de la FRC
from magicbot import feedback
import constants

class Pixy:
    def setup(self):
        self.offset = wpilib.AnalogInput(constants.AnalogIO.PIXY_OFFSET)
        self.isTarget = wpilib.DigitalInput(constants.DigitalIO.PIXY_VALID)

    # @feedback
    def get_target_valid(self):
        return self.isTarget.get()

    # @feedback
    def get_offset(self):
        """Returns offset value"""
        return self.offset.getValue() - 1350

    def targetReady(self, acceptable_offset=0.01):
        if self.get_target_valid() is False:
            return False

        if abs(self.get_offset()) < acceptable_offset:
            return True
        return False

    def execute(self):
        pass
