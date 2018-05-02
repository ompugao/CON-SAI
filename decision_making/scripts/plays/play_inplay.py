# -*- coding: utf-8 -*-
from play_stop import PlayStop

from tactics.tactic_inplay_shoot import TacticInplayShoot
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_keep import TacticKeep
from tactics.tactic_intersection import TacticIntersection
from tactics.tactic_interpose_receiver import TacticInterposeReceiver
from tactics.tactic_mark import TacticMark
import constants

from consai_msgs.msg import Pose
from field_analysis import FieldAnalysis

class PlayInPlay(PlayStop):
    def __init__(self):
        super(PlayInPlay, self).__init__('PlayInPlay')

        self.applicable = "IN_PLAY"
        self.done_aborted = "IN_PLAY"
        self.assignment_type = "CLOSEST_BALL"

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticInplayShoot('TacticInplayShoot', self.roles[1].my_role)
                )
        
        self.roles[2].clear_behavior() #これ追記したら動きました。(原因不明)
        self.roles[2].loop_enable = True
        self.roles[2].behavior.add_child(
                TacticInterposeReceiver('TacticInterposeReceiver', self.roles[2].my_role, base="ANALY_RECEIVE",to_dist = 0) #受け取る側のドリブラーをいじる。
                )
 

        self.roles[3].loop_enable = True
        self.roles[3].behavior.add_child(
                TacticInterposeReceiver('TacticInterposeReceiver', self.roles[3].my_role, base="ANALY_RECEIVE",to_dist = 0) #受け取る側のドリブラーをいじる。
                )
                # TacticKeep('TacticKeep', self.roles[3].my_role, keep_x = -2.0,
                #     range_high = range_y,
                #     range_low = 0.5)
                #)

        range_y = constants.FieldHalfY - 0.7
        self.roles[4].loop_enable = True
        self.roles[4].behavior.add_child(
                TacticInterposeReceiver('TacticInterposeReceiver', self.roles[4].my_role, base="ANALY_RECEIVE",to_dist = 0) #受け取る側のドリブラーをいじる。
                )
                # TacticKeep('TacticKeep', self.roles[4].my_role, keep_x = -2.0,
                #     range_high = -0.5,
                #     range_low = -range_y)
                # )

        pose1 = Pose(-2.5, range_y, 0)
        pose2 = Pose(-2.5, -range_y, 0)
        self.roles[5].loop_enable = True
        self.roles[5].behavior.add_child(
                TacticInterposeReceiver('TacticInterposeReceiver', self.roles[5].my_role, base="ANALY_RECEIVE",to_dist = 0) #受け取る側のドリブラーをいじる。
                )
                # TacticIntersection('TacticIntersection', self.roles[5].my_role,
                #     pose1 = pose1, pose2 = pose2)
                # )

        for i in range(6, len(self.roles)):
                self.roles[i].clear_behavior()

