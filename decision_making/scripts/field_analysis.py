#!/usr/bin/env python
# -*- coding: utf-8 -*-import math
import tool
import constants
#from world_model import WorldModel
from consai_msgs.msg import Pose
import rospy
from geometry_msgs.msg import Point #constans.pyでこう書かれているから。

from collections import OrderedDict, defaultdict
from nav_msgs.msg import Odometry
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity
#from consai_msgs.msg import AnalyzedPose

class FieldAnalysis(object):
#変数定義
    analysis_area_score = [[0 for i in range(18)]for j in range(12)]#X軸12分割、Y軸18分割
        

    @classmethod
    def get_analysis_area_pose(cls, xnum,ynum,yaw):

        position = Pose((-4.5 + xnum*0.5),(3 - ynum*0.5),0) #エリアは50cm正方で分割
        yaw = 0
        return Pose(position.x, position.y, yaw)

    def get_analyzed_area_num(cls):
        xnum = 0
        ynum = 0
        yaw = 0
        return xnum,ynum,yaw

    @classmethod
    def update_field_analysis(cls): #ここで各エリアの点数付を行う。
        #FieldAnalysis.analysis_area_score[1][1] = 6
       # for i in range(12)
        #    for j in range(18)

        milkdata = FieldAnalysis.analysis_area_score[1][1]#logerrに表示する用の変数。
        #rospy.logerr(FieldAnalysis.get_analysis_area_pose(1,1))
        #rospy.logerr(WorldModel.get_enemy_pose(1))
       
       #rospy.logerr(WorldModel.get_enemy_pose(2))
       #rospy.logerr(WorldModel.get_enemy_pose(3))
       #rospy.logerr(WorldModel.get_enemy_pose(4))
       #rospy.logerr(WorldModel.get_enemy_pose(5))