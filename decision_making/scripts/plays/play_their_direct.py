
from play_base import Play
from world_model import WorldModel
from tactics.tactic_mark import TacticMark
from tactics.tactic_goalie import TacticGoalie
from tactics.tactic_interpose import TacticInterpose
from tactics.tactic_position import TacticPosition
from consai_msgs.msg import Pose
import constants
import math
import numpy as np
import rospy

class PlayTheirDirect(Play):
    def __init__(self):
        super(PlayTheirDirect, self).__init__('PlayTheirDirect')

        self.applicable = "THEIR_DIRECT"
        self.done_aborted = "THEIR_DIRECT"

        keep_x = -constants.FieldHalfX + constants.RobotRadius * 2.0
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticGoalie('TacticGoalie', self.roles[0].my_role, keep_x=keep_x,
                    range_high = constants.GoalHalfSize,
                    range_low = -constants.GoalHalfSize)
                )

        # self.roles[1].loop_enable = True
        # self.roles[1].behavior.add_child(
        #         TacticInterpose('TacticInterpose', self.roles[1].my_role, 
        #             from_dist = 0.5)
        #         )
        # self.roles[2].loop_enable = True
        # self.roles[2].behavior.add_child(
        #         TacticInterpose('TacticInterpose', self.roles[2].my_role, 
        #             from_dist = 1.0)
        #         )

        #ranging
        ranging = np.zeros(6)
        ranging_sort=[0, 1, 2, 3, 4, 5]
        if WorldModel.get_pose('Enemy_1') is not None:
            for i in range(0,6):
                enemy_pose = WorldModel.get_pose('Enemy_%d'%(i))
                ranging[i]=math.hypot(enemy_pose.x-WorldModel.get_pose('CONST_OUR_GOAL').x, enemy_pose.y-WorldModel.get_pose('CONST_OUR_GOAL').y)+math.hypot(enemy_pose.x-WorldModel.get_pose('Ball').x, enemy_pose.y-WorldModel.get_pose('Ball').y)
                rospy.logerr('ranging'+str(i)+':'+str(ranging[i]))
            #sort_index
            ranging_sort = ranging.argsort()
            for i in range(0,6):
                rospy.logerr('sort'+str(i)+':'+str(ranging_sort[i]))
        #mark
        for i in range(1,4):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(TacticMark('TacticMark', self.roles[i].my_role,base='CONST_OUR_GOAL',target='Enemy_'+str(ranging_sort[i-1])))


        for i in range(4,6):
            x = -3.0
            y = 0.45 - 0.3*(i-2)
            theta = 0

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticPosition('TacticPosition', self.roles[i].my_role,
                        x, y ,theta))
