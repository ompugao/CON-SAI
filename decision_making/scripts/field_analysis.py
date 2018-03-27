#!/usr/bin/env python
# -*- coding: utf-8 -*-import math
import math
import tool
import constants
import rospy
from geometry_msgs.msg import Point #constans.pyでこう書かれているから。

from collections import OrderedDict, defaultdict
from nav_msgs.msg import Odometry
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity
import numpy as np


class FieldAnalysis(object):
    #定数定義
    xnum = 18#X軸18分割、Y軸12分割
    ynum = 12
    FieldX = constants.FieldHalfX*2 #フィールド寸法
    FieldY = constants.FieldHalfY*2 #フィールド寸法

    #変数定義
    analysis_area_score = np.array([[0]*ynum for i in range(xnum)])

    @classmethod
    def dot(self,p1, p2):#安達くん作成クラス。
        return p1.real*p2.real + p1.imag*p2.imag

    @classmethod
    def cross(self,p1, p2):#安達くん作成クラス。
        return p1.real*p2.imag - p1.imag*p2.real

    @classmethod
    def dotLineDist(self,p, line): #安達くん作成クラス。線分と点の距離を出してくれるもの。
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
                if most_area < FieldAnalysis.analysis_area_score[i][j]:
                    most_area = FieldAnalysis.analysis_area_score[i][j]
                    xnum = i
                    ynum = j
        yaw = 0
        return xnum,ynum,yaw

    @classmethod
    def Score_Easytopass(cls):  #パスがしやすいエリアを評価。 #相手フィールドX軸一定(X=13) で、Y軸だけで評価。
        from world_model import WorldModel
        best_Dist = 0 #最も大きい値を入れたいので、初期値０
        best_Pos = Pose(0,0,0)
        
        i = 13 #X軸num とりあえず13に固定
        for k in range(1,12):
            nearest_Dist = 10 #最も小さい値を入れたいので、初期値とりあえず大きい数字を定義しただけ。
            nearest_enemynum = 0
            #rospy.logerr("oppppp")
            for enemy_num in range(6):
                #rospy.logerr(WorldModel.get_pose('Enemy_'+str(enemy_num)))
                Enemy_pose = complex(WorldModel.get_pose('Enemy_'+str(enemy_num)).x,WorldModel.get_pose('Enemy_'+str(enemy_num)).y)
                Dist = FieldAnalysis.dotLineDist(Enemy_pose,(WorldModel.get_pose("Ball").x+WorldModel.get_pose("Ball").y*1j,(-4.5 + i*0.5)+(3.0 - k*0.5)*1j))
                if nearest_Dist >= Dist:
                    nearest_Dist = Dist
                    nearest_enemynum = enemy_num
                    #rospy.logerr(nearest_Dist)
                #rospy.logerr(nearest_enemynum)
                #rospy.logerr(nearest_Dist)
            if best_Dist <= nearest_Dist:
                best_Dist = nearest_Dist
                best_Pos = Pose(i,k,0)
                #rospy.logerr(best_Dist)
                
        #rospy.logerr(best_Pos)
        FieldAnalysis.analysis_area_score[best_Pos.x][best_Pos.y] = FieldAnalysis.analysis_area_score[best_Pos.x][best_Pos.y]+1
        #rospy.logerr(FieldAnalysis.analysis_area_score[13][10])
        return None

    @classmethod
    def Score_Goal(cls):  #ボールとゴールの直線を評価して、敵がいなければシュート！
        from world_model import WorldModel
        i = 17
        for k in range(6,7):
            nearest_Dist = 10 #最も小さい値を入れたいので、初期値とりあえず大きい数字を定義しただけ。
            for enemy_num in range(6):
                #rospy.logerr(WorldModel.get_pose('Enemy_'+str(enemy_num)))
                Enemy_pose = complex(WorldModel.get_pose('Enemy_'+str(enemy_num)).x,WorldModel.get_pose('Enemy_'+str(enemy_num)).y)
                Dist = FieldAnalysis.dotLineDist(Enemy_pose,(WorldModel.get_pose("Ball").x+WorldModel.get_pose("Ball").y*1j,(-4.5 + i*0.5)+(3.0 - k*0.5)*1j))
                if nearest_Dist >= Dist:
                    nearest_Dist = Dist
            if nearest_Dist > 0.2:
                FieldAnalysis.analysis_area_score[i][k] = 5
        return None

    @classmethod
    def update_field_analysis(cls): #ここで各エリアの点数付を行う
        from world_model import WorldModel

        if WorldModel.get_pose('Role_1') is None:#最初データがヴィジョンから送られてこない。だからそれを無視するプログラム。無いとエラー出る。
            return None 
        #FieldAnalysis.analysis_area_score[9][6] = 2#logerrに表示する用の変数

        for i in range(18):
            for j in range(12):#12
                FieldAnalysis.analysis_area_score[i][j] = 0

        ##################実施評価の選択###############
        FieldAnalysis.Score_Easytopass()
        FieldAnalysis.Score_Goal()

        #rospy.logerr(FieldAnalysis.get_analysis_area_pose(1,1))
        #rospy.logerr(WorldModel.get_enemy_pose(1))
        #rospy.logerr(WorldModel.get_enemy_pose(2))
        #rospy.logerr(WorldModel.get_enemy_pose(3))
        #rospy.logerr(WorldModel.get_enemy_pose(4))
        #rospy.logerr(WorldModel.get_enemy_pose(5))