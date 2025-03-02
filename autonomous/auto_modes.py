
from magicbot.state_machine import AutonomousStateMachine
from components.robot_actions import ActionCycleAutonomous
from magicbot import state

class RunAuto(AutonomousStateMachine):
    DEFAULT: bool = True
    MODE_NAME: str = "RunAuto"
    PATH_NAME: str = "RunAuto"
    actionCycleAutonomous: ActionCycleAutonomous

    def engage(self, initial_state = None, force = False):
        self.actionCycleAutonomous.engage()
        return super().engage(initial_state, force)

    @state(first=True)
    def run(self):
        self.actionCycleAutonomous.engage()
        pass

    @state
    def failed(self):
        """
        Cet état est appelé par défaut si un mode auto a échoué
        """
        self.next_state("finish")

    @state
    def finish(self):
        self.actionCycleAutonomous.done()
        self.done()
