# -*- coding: utf-8 -*-
from play_stop import PlayStop
from play_base import Play
from world_model import WorldModel
from tactics.tactic_mark import TacticMark
from tactics.tactic_inplay_shoot import TacticInplayShoot
from tactics.tactic_inplay_receive_and_shoot import TacticInplayReceiveAndShoot
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_keep import TacticKeep
from tactics.tactic_intersection import TacticIntersection
from tactics.tactic_interpose_receiver import TacticInterposeReceiver
from tactics.tactic_mark import TacticMark
from tactics.tactic_state import InplayMode, KickState
import constants
import weakref

from consai_msgs.msg import Pose
from field_analysis import FieldAnalysis

class PlayInPlay(Play):
    def __init__(self):
        super(PlayInPlay, self).__init__('PlayInPlay')

        self.applicable = "IN_PLAY"
        self.done_aborted = "IN_PLAY"
        self.assignment_type = "CLOSEST_BALL"


        self.roles[0].clear_behavior()
        self.roles[0].behavior.add_child(TacticKeep('TacticKeep', self.roles[0].my_role))

        self.inplay_mode = InplayMode()
        self.kick_state  = KickState()

        self.roles[1].clear_behavior()
        self.roles[1].behavior.add_child(
                TacticInplayReceiveAndShoot('TacticInplayReceiveAndShoot', self.roles[1].my_role, weakref.proxy(self.inplay_mode), weakref.proxy(self.kick_state), shoot_if_mode_is_true=True)
                )

        self.roles[2].clear_behavior() #これ追記したら動きました。(原因不明)
        self.roles[2].loop_enable = True
        self.roles[2].behavior.add_child(
                #TacticInterposeReceiver('TacticInterposeReceiver', self.roles[2].my_role, base="ANALY_RECEIVE",to_dist = 0) #受け取る側のドリブラーをいじる。
                TacticInplayReceiveAndShoot('TacticInplayReceiveAndShoot', self.roles[2].my_role, weakref.proxy(self.inplay_mode), weakref.proxy(self.kick_state), shoot_if_mode_is_true=False)
                )


        self.roles[3].loop_enable = True
        self.roles[3].behavior.add_child(TacticMark('TacticMark', self.roles[3].my_role,base='MarkBase',target='MarkTarget',from_dist=0.5))

        range_y = constants.FieldHalfY - 0.7
        pose1 = Pose(-2.0, range_y, 0)
        pose2 = Pose(-2.0, -range_y, 0)
        self.roles[4].loop_enable = True
        self.roles[4].behavior.add_child(
                TacticIntersection('TacticIntersection', self.roles[4].my_role,
                    pose1 = pose1, pose2 = pose2)
                )

        pose1 = Pose(-2.5, range_y, 0)
        pose2 = Pose(-2.5, -range_y, 0)
        self.roles[5].loop_enable = True
        self.roles[5].behavior.add_child(
                TacticIntersection('TacticIntersection', self.roles[5].my_role,
                    pose1 = pose1, pose2 = pose2)
                )

        for i in range(6, len(self.roles)):
                self.roles[i].clear_behavior()

