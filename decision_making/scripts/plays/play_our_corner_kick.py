
from play_base import Play

from tactics.tactic_setplay_shoot import TacticSetplayShoot
from tactics.tactic_corner_kick import TacticCornerKick
from tactics.tactic_keep import TacticKeep
from tactics.tactic_intersection import TacticIntersection
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_goalie import TacticGoalie
from tactics.tactic_receive_ball_at import TacticReceiveBallAt
from tactics.tactic_receive_ball_at_and_shoot import TacticReceiveBallAtAndShoot
from tactics.tactic_state import CornerKickState
from tactics.tactic_halt import TacticHalt
from pi_trees_lib.pi_trees_lib import Loop
from consai_msgs.msg import Pose
import constants

import weakref

class PlayOurCornerKick(Play):
    def __init__(self):
        super(PlayOurCornerKick, self).__init__('PlayOurCornerKick')

        self.applicable = "OUR_INDIRECT"
        self.done_aborted = None
        #self.assignment_type = "CLOSEST_BALL"

        self.state = CornerKickState()

        keep_x = -constants.FieldHalfX + constants.RobotRadius * 2.0
        self.roles[0].loop_enable = True
        self.roles[0].clear_behavior()
        self.roles[0].behavior.add_child(
                TacticGoalie('TacticGoalie', self.roles[0].my_role, keep_x=keep_x,
                    range_high = constants.GoalHalfSize,
                    range_low = -constants.GoalHalfSize)
                )

        abort_cornerkick_play_x = 1.2
        receive_field_separate_x = 2.6

        self.roles[1].loop_enable = True
        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticCornerKick('TacticCornerKick', self.roles[1].my_role, weakref.proxy(self.state))
                )
        l = Loop('loop', announce=False, iterations=-1)
        l.add_child(
                TacticReceiveBallAtAndShoot('TacticReceiveBallAtAndShoot', self.roles[1].my_role, weakref.proxy(self.state), \
                        Pose(2.0, 0.0, 0), \
                        [abort_cornerkick_play_x, -constants.FieldHalfY, receive_field_separate_x, +constants.FieldHalfY])
                        )
        self.roles[1].behavior.add_child(l)

        self.roles[2].loop_enable = True
        self.roles[2].clear_behavior()
        self.roles[2].behavior.add_child(
                TacticReceiveBallAtAndShoot('TacticReceiveBallAtAndShoot', self.roles[2].my_role, weakref.proxy(self.state), \
                        Pose(3.7, 2.1, 0), \
                        [receive_field_separate_x, 0, constants.FieldHalfX, constants.FieldHalfY])
                )

        self.roles[3].loop_enable = True
        self.roles[3].clear_behavior()
        self.roles[3].behavior.add_child(
                TacticReceiveBallAtAndShoot('TacticReceiveBallAtAndShoot', self.roles[3].my_role, weakref.proxy(self.state), \
                        Pose(2.9, -0.6, 0), \
                        [receive_field_separate_x, -constants.FieldHalfY, constants.FieldHalfX, 0])
                )

        self.roles[4].loop_enable = True
        self.roles[4].clear_behavior()
        self.roles[4].behavior.add_child(
                #TacticReceiveBallAt('TacticReceiveBallAt', self.roles[4].my_role, Pose(3.1, 1.2, 0))
                TacticHalt('TacticHalt', self.roles[4].my_role)
                )

        range_y = constants.FieldHalfY - 0.7
        pose1 = Pose(-2.5, range_y, 0)
        pose2 = Pose(-2.5, -range_y, 0)
        self.roles[5].loop_enable = True
        self.roles[5].clear_behavior()
        self.roles[5].behavior.add_child(
                TacticIntersection('TacticIntersection', self.roles[5].my_role,
                    pose1 = pose1, pose2 = pose2)
                )


