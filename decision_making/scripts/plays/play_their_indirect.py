
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

class PlayTheirIndirect(Play):
    def __init__(self):
        super(PlayTheirIndirect, self).__init__('PlayTheirIndirect')

        self.applicable = "THEIR_INDIRECT"
        self.done_aborted = "THEIR_INDIRECT"

        keep_x = -constants.FieldHalfX + constants.RobotRadius * 2.0
        self.roles[0].loop_enable = True
        self.roles[0].behavior.add_child(
                TacticGoalie('TacticGoalie', self.roles[0].my_role, keep_x=keep_x,
                    range_high = constants.GoalHalfSize,
                    range_low = -constants.GoalHalfSize)
                )

        if WorldModel.get_pose('Ball') is not None:
            self.roles[1].loop_enable = True
            self.roles[1].behavior.add_child(TacticMark('TacticMark', self.roles[1].my_role,base='CONST_OUR_GOAL',target='Ball'))

        # self.roles[2].loop_enable = True
        # self.roles[2].behavior.add_child(
        #         TacticInterpose('TacticInterpose', self.roles[2].my_role, 
        #             from_dist = 1.0)
        #         )

        num_enemies = 6
        if WorldModel.get_pose('Enemy_1') is not None and  WorldModel.get_pose('Ball') is not None:
            goal_pose = WorldModel.get_pose('CONST_OUR_GOAL')
            ball_pose = WorldModel.get_pose('Ball')
            ranging = []
            for i in range(0, num_enemies):
                enemy_pose = WorldModel.get_pose('Enemy_%d'%(i))
                if enemy_pose is None:
                    continue
                if math.hypot(enemy_pose.x-ball_pose.x, enemy_pose.y-ball_pose.y)<1:
                    ranging.append( (i, math.hypot(enemy_pose.x-goal_pose.x, enemy_pose.y-goal_pose.y) + 12) )
                else:
                    ranging.append( (i, math.hypot(enemy_pose.x-goal_pose.x, enemy_pose.y-goal_pose.y) ) )
                #rospy.logerr('ranging'+str(i)+':'+str(ranging[i]))
            for i in range(len(ranging), num_enemies):
                ranging.append( (i, np.inf) )
            #sort_index
            ranging_sort = map((lambda x: x[0]), sorted(ranging, key=(lambda x: x[1])))
            # for i in range(0,6):
            #     rospy.logerr('sort'+str(i)+':'+str(ranging_sort[i]))
        else:
            ranging_sort = np.array(range(num_enemies))
        #mark
        for i in range(2,4):
            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(TacticMark('TacticMark', self.roles[i].my_role,base='CONST_OUR_GOAL',target='Enemy_'+str(ranging_sort[i-2])))
    
        for i in range(4,6):
            x = -3.0
            y = 0.45 - 0.3*(i-2)
            theta = 0

            self.roles[i].loop_enable = True
            self.roles[i].behavior.add_child(
                    TacticPosition('TacticPosition', self.roles[i].my_role,
                        x, y ,theta))
