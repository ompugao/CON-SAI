#!/usr/bin/env python
# -*- coding: utf-8 -*-import math
import tool
import constants
#from world_model import WorldModel
import rospy
from geometry_msgs.msg import Point #constans.pyでこう書かれているから。

from collections import OrderedDict, defaultdict
from nav_msgs.msg import Odometry
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity
import numpy as np


class FieldAnalysis(object):
    #定数定義
    xnum = 18 #X軸18分割、Y軸12分割
    ynum = 12
    #変数定義
    analysis_area_score = np.array([[0]*ynum for i in range(xnum)])

    @classmethod
    def dotLineDist(self,p, line): #安達くん作成クラス。線分と店の距離を出してくれるもの。
        "line: (p1, p2)"
        a, b = line
        if FieldAnalysis.dot(b-a, p-a) <= 0.0:
            return abs(p-a)
        if FieldAnalysis.dot(a-b, p-b) <= 0.0:
            return abs(p-b)
        return abs(FieldAnalysis.cross(b-a, p-a))/abs(b-a)

    @classmethod
    def get_analysis_area_pose(cls,area): #エリアを指定して、その座標を返す
        x = FieldAnalysis.get_analyzed_area_num()[0]
        y = FieldAnalysis.get_analyzed_area_num()[1]
        position = Pose((-4.5 + x*0.5),(3 - y*0.5),0) #エリアは50cm正方で分割
        yaw = 0
        return Pose(position.x, position.y,0)

    @classmethod
    def get_analyzed_area_num(cls): 
        #最もスコアの高いエリアを抽出
        most_area = 0
        for i in range(18):
            for j in range(12):
                if most_area <= FieldAnalysis.analysis_area_score[i][j]:
                    most_area = FieldAnalysis.analysis_area_score[i][j]
                    xnum = i
                    ynum = j
        yaw = 0
        return xnum,ynum,yaw

    @classmethod
    def update_field_analysis(cls): #ここで各エリアの点数付を行う
        FieldAnalysis.analysis_area_score[9][6] = 2#logerrに表示する用の変数
        #rospy.logerr(FieldAnalysis.get_analysis_area_pose(1,1))
        #rospy.logerr(WorldModel.get_enemy_pose(1))
        #rospy.logerr(WorldModel.get_enemy_pose(2))
        #rospy.logerr(WorldModel.get_enemy_pose(3))
        #rospy.logerr(WorldModel.get_enemy_pose(4))
        #rospy.logerr(WorldModel.get_enemy_pose(5))