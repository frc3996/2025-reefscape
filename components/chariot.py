import rev
import wpilib
from magicbot import feedback, tunable, will_reset_to
from components.lift import Lift
import constants


TARGET_FRONT = 0
TARGET_BACK = 1
CURRENT_LIMIT_AMP = 5


class Chariot:
    chariot_speed = tunable(20)
    safe_lift_height = tunable(1)
    __target_reached = False
    __chariot_cmd = will_reset_to(0)
    __current_target = will_reset_to(TARGET_FRONT)
    __last_target = TARGET_FRONT
    lift: Lift

    def setup(self):
        """
        Appelé après l'injection
        """
        self.chariot_motor = rev.SparkMax(
            constants.CANIds.INTAKE_CHARIOT_MOTOR, rev.SparkMax.MotorType.kBrushless
        )
        self.chariot_motor_controller = self.chariot_motor.getClosedLoopController()
        self.chariot_motor_config = rev.SparkBaseConfig()
        self.chariot_motor_config.smartCurrentLimit(CURRENT_LIMIT_AMP)

        self.chariot_motor.configure(
            self.chariot_motor_config,
            self.chariot_motor.ResetMode.kResetSafeParameters,
            self.chariot_motor.PersistMode.kPersistParameters
        )

        self.chariot_front_limit_switch = wpilib.DigitalInput(constants.DigitalIO.INTAKE_FRONT_LIMIT_SWITCH)
        self.chariot_back_limit_switch = wpilib.DigitalInput(constants.DigitalIO.INTAKE_BACK_LIMIT_SWITCH)

    def move_intake_back_for_intake(self):
        if self.lift.get_lift_height() < self.safe_lift_height or self.lift.get_hauteur_cible():
            print("Cancelling to avoid head crash")
            return

        self.__current_target = TARGET_BACK

    @feedback
    def get_chariot_front_limit_switch(self) -> bool:
        return self.chariot_front_limit_switch.get()

    @feedback
    def get_chariot_back_limit_switch(self) -> bool:
        return self.chariot_back_limit_switch.get()

    @feedback
    def target_reached(self):
        return self.__target_reached

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        if self.__last_target != self.__current_target:
            self.__target_reached = False
            self.__last_target = self.__current_target

        if self.__current_target == TARGET_FRONT:
            if self.get_chariot_front_limit_switch():
                self.__target_reached = True
            else:
                self.__chariot_cmd = self.chariot_speed

        elif self.__current_target == TARGET_BACK:
            if self.get_chariot_back_limit_switch():
                self.__target_reached = True
            else:
                self.__chariot_cmd = -self.chariot_speed

        self.chariot_motor_controller.setReference(self.__chariot_cmd, self.chariot_motor.ControlType.kVelocity)