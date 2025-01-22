
  
# ################################################## Lift ################################################################

# - 1x string encodeur (analogue in)
# - 2x neo (1 "follower")
# - 4x limit switch (2 zero, 2 safety)

# def goL1:
#     return move lift(L1 pos)

# def goL2
#     return move lift(L2 pos)

# def goL3
#     return move lift(L3 pos)

# def LgoL4
#     return move lift(L4 pos)

# def setup():
#     créé pid avec string encoder

# def moveLift(pos):
#     set target
#     return pid reached

# def execute():
#     pid magic(target)

#     if target isGoingUp and safetyLimitSwitch: 
#         power 0
#     else:
#          moteur = pid magic

# def manualOffset():
#     if getControler(touche 1): offset + 0.1
#     if getControler(touche 2): offset - 0.1



import wpilib
import constants
from magicbot import tunable, will_reset_to, StateMachine
from magicbot.state_machine import state, timed_state


INTAKE_POSITION = 10


class Lift:
    max_speed = tunable(1)
    __target_speed = will_reset_to(0)

    def setup(self):
        """
        Appelé après l'injection
        """
        # self.lift_motor = wpilib.PWMMotorController("DemoMotor", constants.PWM_DEMOMOTOR)

    def set_speed(self, speed):
        """
        Fait tourner le moteur à la vitesse spécifiée
        """
        # S'assure que la vitesse maximale ne peut pas être dépassée
        self.__target_speed = speed

    def __move_to_position(self, position):
        """Move the lift to the proper position"""
        # TODO

    def go_intake(self):
        self.__move_to_position(INTAKE_POSITION)

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """

        # self.lift_motor.set(max(min(self.__target_speed, self.max_speed), -self.max_speed))


class LiftActions(StateMachine):

    def todo(self):
        pass