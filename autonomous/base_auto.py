from magicbot.state_machine import AutonomousStateMachine, state


class BaseAuto(AutonomousStateMachine):
    """
    Cette classe est utilisée pour terminer les divers mode autonomes.
    l'état 'failed' ou 'finish' devrait être appelé une fois terminé.
    """

    # driver.flush() permet de réduire des problèmes potentiel sur la drive.
    @state
    def failed(self):
        """
        Cet état est appelé par défaut si un mode auto a échoué
        """
        self.next_state("finish")

    @state
    def finish(self):
        self.done()
