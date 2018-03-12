from play_base import Play
import constants

from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_keep import TacticKeep


class PlaySuperProtective(Play):
    '''
    an example of protective play
    '''
    def __init__(self, name='PlaySuperProtective'):
        super(PlaySuperProtective, self).__init__(name)

        self.applicable = "IN_PLAY"
        self.done_aborted = "IN_PLAY"

        keep_x = - constants.FieldHalfX + constants.RobotRadius * 2.0
        range_y = constants.FieldHalfY - 0.7
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[0].my_role, keep_x = keep_x,
                    range_high = range_y,
                    range_low = 0.2)
                )

        self.roles[1].loop_enable = True
        self.roles[1].behavior.add_child(
                TacticInterpose('TacticInterpose', self.roles[1].my_role, 
                    from_dist = 0.5)
                )

        self.roles[2].loop_enable = True
        self.roles[2].behavior.add_child(
                TacticKeep('TacticKeep', self.roles[2].my_role, keep_x = keep_x,
                    range_high = -0.2,
                    range_low = -range_y)
                )
    '''
    admiral can tune the behavior of play on every timestep of play selection
    '''
    def set_some_argument(self, argument):
        pass

