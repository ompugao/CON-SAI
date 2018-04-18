#!/usr/bin/env python
# -*- coding: utf-8 -*-
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

################## 評価グリッド関係 ##########################
    #定数定義 
    xgrid = 19 #X軸19分割、Y軸13分割
    ygrid = 13 
    FieldX = constants.FieldHalfX*2 #フィールド寸法
    FieldY = constants.FieldHalfY*2 #フィールド寸法

    #変数定義
    #area_score = np.array([[0]*ygrid for i in range(xgrid)])
    area_score = np.zeros([xgrid, ygrid])

    @classmethod
    def write_area_score(self,i, j,score): #スコア記述用
        FieldAnalysis.area_score[i+((FieldAnalysis.xgrid-1)/2), j+((FieldAnalysis.ygrid-1)/2)] = score
        return None

    @classmethod
    def read_area_score(self,i, j): #スコア読み込み用
        return FieldAnalysis.area_score[i+((FieldAnalysis.xgrid-1)/2), j+((FieldAnalysis.ygrid-1)/2)]
############################################################

##############　評価用　計算関係　#####################
    @classmethod
    def dot(self,p1, p2): #安達くん作成クラス。
        return p1.real*p2.real + p1.imag*p2.imag

    @classmethod
    def cross(self,p1, p2): #安達くん作成クラス。
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
###############################################################

############################ 評価関係　　##############################
    @classmethod
    def get_analysis_area_pose(cls,area): #エリアを指定して、その座標を返す
        x = FieldAnalysis.get_analyzed_area_num()[0]
        y = FieldAnalysis.get_analyzed_area_num()[1]
        position = Pose((x*0.5),(y*0.5),0) #エリアは50cm正方で分割
        yaw = 0
        return Pose(position.x, position.y,0)

    @classmethod
    def get_analyzed_area_num(cls):
        #最もスコアの高いエリアを抽出
        most_area = 0
        xnum = 0
        ynum = 0
        for i in range(-9,10,1):
            for j in range(-6,7,1):
                if most_area < FieldAnalysis.read_area_score(i,j):
                    most_area = FieldAnalysis.read_area_score(i,j)
                    xnum = i
                    ynum = j
        yaw = 0
        return xnum,ynum,yaw

    @classmethod
    def Score_Easytopass(cls):  #パスがしやすいエリアを評価。 #相手フィールドX軸一定(X=13) で、Y軸だけで評価。
        from world_model import WorldModel
        best_dist = 0 #最も大きい値を入れたいので、初期値０
        best_pos = Pose(0,0,0)
        
        i = 4 #X軸num とりあえず13に固定
        for k in range(-6,7,1):#1,12
            nearest_dist = 10 #最も小さい値を入れたいので、初期値とりあえず大きい数字を定義しただけ。
            nearest_enemynum = 0
            #rospy.logerr(k)
            for enemy_num in range(6):
                #rospy.logerr(WorldModel.get_pose('Enemy_'+str(enemy_num)))
                enemy_pose = WorldModel.get_pose('Enemy_'+str(enemy_num))
                if enemy_pose is None:
                    continue
                c_enemy_pose = complex(enemy_pose.x, enemy_pose.y)
                ball_pose = WorldModel.get_pose("Ball")
                dist = FieldAnalysis.dotLineDist(c_enemy_pose, (ball_pose.x + ball_pose.y*1j,(i*0.5)+(k*0.5)*1j))
                if nearest_dist > dist:
                    nearest_dist = dist
                    nearest_enemynum = enemy_num
                    #rospy.logerr(nearest_Dist)
                #rospy.logerr(nearest_enemynum)
                #rospy.logerr(nearest_Dist)
            if best_dist <= nearest_dist:
                best_dist = nearest_dist
                best_pos = Pose(i,k,0)
                rospy.logdebug("best distance %f"%(best_dist))
        
        #rospy.logerr(best_pos)
        FieldAnalysis.write_area_score(best_pos.x, best_pos.y, 3)
        #rospy.logerr(FieldAnalysis.analysis_area_score[13][10])
        return None

    @classmethod
    def Score_Goal(cls):  #ボールとゴールの直線を評価して、敵がいなければシュート！
        from world_model import WorldModel
        i = 9#18
        for k in range(-1,1,1):#6,7
            nearest_Dist = 10 #最も小さい値を入れたいので、初期値とりあえず大きい数字を定義しただけ。
            for enemy_num in range(6):
                #rospy.logerr(WorldModel.get_pose('Enemy_'+str(enemy_num)))
                enemy_pose = WorldModel.get_pose('Enemy_'+str(enemy_num))
                if enemy_pose is None:
                    continue
                c_enemy_pose = complex(enemy_pose.x, enemy_pose.y)
                ball_pose = WorldModel.get_pose('Ball')
                Dist = FieldAnalysis.dotLineDist(c_enemy_pose, (ball_pose.x+ball_pose.y*1j,(i*0.5)+(k*0.5)*1j))
                if nearest_Dist >= Dist:
                    nearest_Dist = Dist
            if nearest_Dist > 0.2:
                FieldAnalysis.write_area_score(i,k,5)
        return None
#####################################################################3

##################### 実行評価の選択　＆メイン処理(実質main関数)#########
    @classmethod
    def update_field_analysis(cls): #ここで各エリアの点数付を行う
        from world_model import WorldModel

        if WorldModel.get_pose('Role_1') is None:#最初データがヴィジョンから送られてこない。だからそれを無視するプログラム。無いとエラー出る。
            return None
        
        for i in range(-((FieldAnalysis.xgrid-1)/2),((FieldAnalysis.xgrid-1)/2)+1,1):
            for j in range(-((FieldAnalysis.ygrid-1)/2),((FieldAnalysis.ygrid-1)/2)+1,1):
        
                FieldAnalysis.write_area_score(i,j,0)
                #rospy.logerr(j)

        ##################実施評価の選択###############
        FieldAnalysis.Score_Easytopass()
        FieldAnalysis.Score_Goal()
