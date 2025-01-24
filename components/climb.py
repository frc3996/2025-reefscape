"""
Climber Code
By Rikitik 3996
Paul-Hubert
Rimouski, QC, Canada
2025

For
FIRST FRC Competition 2025
Reefscape

Presented by
HAAS
"""
import wpilib
import constants
import rev
from magicbot import tunable, will_reset_to
from magicbot.state_machine import state, timed_state
from magicbot import StateMachine


class Climb():
    max_speed = tunable(0.5)
    __climb_motor_speed = will_reset_to(0)
    # pneumatic_hub: wpilib.PneumaticHub

    def setup(self):
        """
        Appelé après l'injection
        """

        # Guide motor
        self.guide_motor = rev.CANSparkMax(constants.CANIds.CLIMB_GUIDE_MOTOR, rev.CANSparkMax.MotorType.kBrushless)
        self.guide_motor.setOpenLoopRampRate(0.5)

        # Main Climb
        self.climb_raise_motor_main = rev.CANSparkMax(constants.CANIds.CLIMB_RAISE_MOTOR_MAIN, rev.CANSparkMax.MotorType.kBrushless)
        self.climb_raise_motor_main.setOpenLoopRampRate(0.5)
        
        # Follower Climb
        self.climb_raise_motor_follower = rev.CANSparkMax(constants.CANIds.CLIMB_RAISE_MOTOR_FOLLOWER, rev.CANSparkMax.MotorType.kBrushless)
        self.climb_raise_motor_follower.follow(self.climb_raise_motor_main)

        # Limit switch
        self.cage_in_limitswitch_1 = wpilib.DigitalInput(constants.DigitalIO.CLIMB_CAGE_IN_LIMITSWITCH_1)
        self.cage_in_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.CLIMB_CAGE_IN_LIMITSWITCH_2)
        self.piston_out_limitswitch_1 = wpilib.DigitalInput(constants.DigitalIO.CLIMB_PISTON_OUT_LIMITSWITCH_1)
        self.piston_out_limitswitch_2 = wpilib.DigitalInput(constants.DigitalIO.CLIMB_PISTON_OUT_LIMITSWITCH_2)

        # Piston
        # self.piston_solenoid = self.pneumatic_hub.makeSolenoid(constants.SolenoidChannel.CLIMB_PISTON)

    def go_front(self):
        """Move motor shoot position"""
        # TODO

    def go_back(self):
        pass
        # TODO

    def enable_guide_motor(self):
        """Active les moteurs pour guider la cage"""
        # TODO

    def enable_climb_motor(self, speed):
        """Active les moteurs pour climb"""
        self.__climb_motor_speed = speed

    def open_pistons(self):
        """Ouvre les pistons"""
        # self.piston_solenoid.set(True)

    def close_pistons(self):
        """Ferme les pistons"""
        # self.piston_solenoid.set(False)

    def cageIn(self):
        """Retourne vrai si la switch1 OU 2 est activé"""
        #TODO
        # return intakeSwitch1 or intakeSwitch2

    def pistonDeployed(self):
        """Retroune vrai si la switch1 ET 2 sont activées"""
        # TODO
        # return pistonSwitch1 et pistonSwitch2

    def execute(self):
        """
        Cette fonction est appelé à chaque itération/boucle
        C'est ici qu'on doit écrire la valeur dans nos moteurs
        """
        self.climb_raise_motor_main.set(self.__climb_motor_speed)


class ClimbAction(StateMachine):
    climb: Climb

    def startClimb(self):
        self.engage("fetchCage")

    @state
    def fetchCage(self):
        self.climb.enable_guide_motor()
        self.climb.enable_climb_motor(0.1)
        self.climb.open_pistons()
        if self.climb.cageIn():
            self.next_state("startClimb")

    @timed_state(duration=0.5, next_state="restart")
    def lock(self):
        self.climb.close_pistons()
        if self.climb.pistonDeployed():
            self.next_state("motorClimb")
            
    @state
    def motorClimb(self):
        # Moteur à 100%, à l'infini
        self.climb.enable_climb_motor(1)
