
from play_base import Play

from tactics.tactic_halt import TacticHalt

class PlayHalt(Play):
    def __init__(self):
        super(PlayHalt, self).__init__('PlayHalt')

        self.applicable = "HALT"
        self.done_aborted = "HALT"

        for role in self.roles:
            role.loop_enable = True
            role.behavior.add_child(
                    TacticHalt("TacticHalt", role.my_role))
