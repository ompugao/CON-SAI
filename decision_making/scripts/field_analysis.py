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
    xgrid = 25 #0.5間隔　X軸25分割、Y軸19分割
    ygrid = 19 
    FieldX = constants.FieldX #フィールド寸法
    FieldY = constants.FieldY #フィールド寸法

    #変数定義
    #area_score = np.array([[0]*ygrid for i in range(xgrid)])
    shoot_score = np.zeros([xgrid, ygrid]) #パスシュート用
    receive_score = np.zeros([xgrid, ygrid]) #パスレシーブ位置用

    @classmethod
    def write_area_score(self,area,i, j,score): #スコア記述用
        if area == 'SHOOT':
            FieldAnalysis.shoot_score[i+((FieldAnalysis.xgrid-1)/2), j+((FieldAnalysis.ygrid-1)/2)] = score
        elif area == 'RECEIVE':
            FieldAnalysis.receive_score[i+((FieldAnalysis.xgrid-1)/2), j+((FieldAnalysis.ygrid-1)/2)] = score
        return None

    @classmethod
    def read_area_score(self,area,i, j): #スコア読み込み用
        if area == 'SHOOT':
            return FieldAnalysis.shoot_score[i+((FieldAnalysis.xgrid-1)/2), j+((FieldAnalysis.ygrid-1)/2)]
        elif area == 'RECEIVE':
            return FieldAnalysis.receive_score[i+((FieldAnalysis.xgrid-1)/2), j+((FieldAnalysis.ygrid-1)/2)]
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

    @classmethod
    def cal_best_passposi(self,i,y1,y2): #xとy1~y2上の直線の中から、最も敵から遠いパスルートとなる座標選択
        from world_model import WorldModel
        best_dist = 0 #最も大きい値を入れたいので、初期値０
        best_pos = Pose(0,0,0)

        for k in range(y1,y2,1):
            nearest_dist = 10 #最も小さい値を入れたいので、初期値とりあえず大きい数字を定義しただけ。
            nearest_enemynum = 0
            #rospy.logerr(k)
            
            for enemy_num in range(len(WorldModel.enemy_assignments)):
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
            if best_dist <= nearest_dist:
                best_dist = nearest_dist
                best_pos = Pose(i,k,0)
                #rospy.logdebug("best distance %f"%(best_dist))
        
        return best_pos,best_dist


    
###############################################################

############################ 評価関係　　##############################
    @classmethod
    def get_best_receiving_pose(cls): #エリアを指定して、その座標を返す
        from world_model import WorldModel
        x = FieldAnalysis.get_analyzed_area_num('RECEIVE')[0]
        y = FieldAnalysis.get_analyzed_area_num('RECEIVE')[1]
        if (x == 12)and(-1 <= y <= 1) :
            position = Pose((x*0.5),(y*0.4),0) #ゴールポストギリギリを狙わないように修正
        else: 
            position = Pose((x*0.5),(y*0.5),0) #エリアは50cm正方で分割

        return Pose(position.x, position.y,0)

    @classmethod
    def get_best_passing_pose(cls): #エリアを指定して、その座標を返す
        from world_model import WorldModel
        x = FieldAnalysis.get_analyzed_area_num('SHOOT')[0]
        y = FieldAnalysis.get_analyzed_area_num('SHOOT')[1]
        if (x == 12)and(-1 <= y <= 1) : #ゴールへのシュート時
            position = Pose((x*0.5),(y*0.4),0) #ゴールポストギリギリを狙わないように修正            
        else: #パス時
            position = Pose((x*0.5),(y*0.5),0) #エリアは50cm正方で分割

        return Pose(position.x, position.y,0)

    @classmethod
    def get_analyzed_area_num(cls,area): #最もスコアの高いエリアを抽出
        from world_model import WorldModel
        if area == 'SHOOT':
            most_area = 0
            xnum = 0
            ynum = 0
            for i in range(-12,13,1):
                for j in range(-9,10,1):
                    if most_area < FieldAnalysis.read_area_score('SHOOT',i,j):
                        most_area = FieldAnalysis.read_area_score('SHOOT',i,j)
                        xnum = i
                        ynum = j
            yaw = 0
            ###########キックパワー調節############
            ball_pose = WorldModel.get_pose("Ball")
            #dist = math.sqrt(abs((xnum + ynum*1j) - (ball_pose.x + ball_pose.y*1j)))#ボールと座標の距離を出す
            dist = math.sqrt(pow(ball_pose.x-(xnum*0.5),2)+pow(ball_pose.y-(ynum*0.5),2))
            
            k_power = 0.8 #キックパワーのゲイン
            b_power = 1.6 #キックパワーのバイアス
            kick_power = (dist * k_power) + b_power
            #kick_power = 0.5 * dist #係数 * ボールの距離
            if kick_power > 8:    #最大powerが8
                kick_power = 8
            if xnum == 12:    #座標がゴールの場合は全力でシュート！！
                kick_power = 8
            
            WorldModel.commands['Role_1'].set_kick(kick_power)
            #rospy.logerr(kick_power)
            #rospy.logerr(dist)
        
        elif area == 'RECEIVE':
            most_area = 0
            xnum = 0
            ynum = 0
            for i in range(-12,13,1):
                for j in range(-9,10,1):
                    if most_area < FieldAnalysis.read_area_score('RECEIVE',i,j):
                        most_area = FieldAnalysis.read_area_score('RECEIVE',i,j)
                        xnum = i
                        ynum = j
            yaw = 0

        return xnum,ynum,yaw

    @classmethod
    def Score_Easytopass(cls,score):  #パスがしやすいエリアを評価。 #相手フィールドX軸一定(X=) で、Y軸だけで評価。
        from world_model import WorldModel
        best_dist = 0 #最も大きい値を入れたいので、初期値０
        best_pos = Pose(0,0,0)
        
        if score == 2:
            x =  0  #X軸num
            y1 = -6
            y2 = 6
            best_pos = FieldAnalysis.cal_best_passposi(x,y1,y2+1)[0]  
            FieldAnalysis.write_area_score('SHOOT',best_pos.x, best_pos.y,score)
            FieldAnalysis.write_area_score('RECEIVE',best_pos.x, best_pos.y,score) #シュート位置も受け取り位置も同じ
        
        elif score == 3:
            x =  6  #X軸num
            y1 = -6
            y2 = 6
            best_pos = FieldAnalysis.cal_best_passposi(x,y1,y2+1)[0]  
            FieldAnalysis.write_area_score('SHOOT',best_pos.x, best_pos.y,score)
            FieldAnalysis.write_area_score('RECEIVE',best_pos.x, best_pos.y,score) #シュート位置も受け取り位置も同じ
        
        if score == 4:
            x =  10  #X軸num

            pose_1=Pose(0,0,0)
            pose_2=Pose(0,0,0)
            dist_1=0
            dist_2=0

            #ゴール下側評価
            y1 = -6
            y2 = -3
            pose_1,dist_1 = FieldAnalysis.cal_best_passposi(x,y1,y2+1)
            #ゴール上側評価
            y1 = 3
            y2 = 6
            pose_2,dist_2 = FieldAnalysis.cal_best_passposi(x,y1,y2+1)

            if dist_1 > dist_2:
                best_pos = pose_1
            else :
                best_pos = pose_2

            FieldAnalysis.write_area_score('SHOOT',best_pos.x, best_pos.y,score)
            FieldAnalysis.write_area_score('RECEIVE',best_pos.x, best_pos.y,score) #シュート位置も受け取り位置も同じ
        
        #rospy.logdebug('best_pos x: %f, y: %f, theta %f'%(best_pos.x, best_pos.y, best_pos.theta))
        #rospy.logerr(FieldAnalysis.analysis_area_score[13][10])
        return None

    @classmethod
    def Score_Goal(cls):  #ボールとゴールの直線を評価して、敵がいなければシュート！
        from world_model import WorldModel
        i = 12#18
        for k in range(-1,1,1):#6,7
            nearest_Dist = 10 #最も小さい値を入れたいので、初期値とりあえず大きい数字を定義しただけ
            for enemy_num in range(len(WorldModel.enemy_assignments)):
                #rospy.logerr(WorldModel.get_pose('Enemy_'+str(enemy_num)))
                enemy_pose = WorldModel.get_pose('Enemy_'+str(enemy_num))
                if enemy_pose is None:
                    continue
                c_enemy_pose = complex(enemy_pose.x, enemy_pose.y)
                ball_pose = WorldModel.get_pose('Ball')
                Dist = FieldAnalysis.dotLineDist(c_enemy_pose, (ball_pose.x+ball_pose.y*1j,(i*0.5)+(k*0.4)*1j)) #ゴール端をシュート目標にしていたので、幅を0.4として修正
                if nearest_Dist >= Dist:
                    nearest_Dist = Dist
            if nearest_Dist > 0.2:
                FieldAnalysis.write_area_score('SHOOT',i,k, 6)
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
        
                FieldAnalysis.write_area_score('SHOOT',i,j,0)
                FieldAnalysis.write_area_score('RECEIVE',i,j,0)
                #rospy.logerr(j)

        ##################実施評価の選択###############
        FieldAnalysis.Score_Easytopass(2) #スコア2~4に当たる箇所のパス評価を行う。
        FieldAnalysis.Score_Easytopass(3)
        FieldAnalysis.Score_Easytopass(4)
        FieldAnalysis.Score_Goal()
