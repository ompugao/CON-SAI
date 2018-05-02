#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from collections import OrderedDict, defaultdict

from nav_msgs.msg import Odometry
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity
from consai_msgs.msg import RefereeTeamInfo

import tool
import constants
from proto.referee_pb2 import SSL_Referee
from command import Command
from observer import Observer
from field_analysis import FieldAnalysis

import math 
import numpy as np 


class WorldModel(object):
    _s = ['HALT', 'STOP', 'FORCE_START',
            'OUR_PRE_KICKOFF', 'OUR_KICKOFF_START',
            'OUR_PRE_PENALTY', 'OUR_PENALTY_START',
            'OUR_DIRECT', 'OUR_INDIRECT', 'OUR_TIMEOUT',
            'THEIR_PRE_KICKOFF', 'THEIR_KICKOFF_START',
            'THEIR_PRE_PENALTY', 'THEIR_PENALTY_START',
            'THEIR_DIRECT', 'THEIR_INDIRECT', 'THEIR_TIMEOUT',
            'BALL_IN_OUTSIDE', 'IN_PLAY',
            'BALL_IN_OUR_DEFENCE', 'BALL_IN_THEIR_DEFENCE']

    # テスト用のsituations
    _test = []
    for i in range(100):
        key = 'TEST' + str(i)
        _test.append(key)

    # KeyError を防ぐためdefaultdictを使う
    situations = defaultdict(lambda : False)
    for key in _s:
        situations[key] = False

    for key in _test:
        situations[key] = False

    _current_situation = 'HALT'
    _current_test = 'DUMMY_TEST'

    assignments = OrderedDict()
    enemy_assignments = OrderedDict()
    _threat_assignments = OrderedDict()

    for i in range(6):
        key = 'Role_' + str(i)
        assignments[key] = None

        key = 'Enemy_' + str(i)
        enemy_assignments[key] = None

        key = 'Threat_' + str(i)
        _threat_assignments[key] = None

    commands = {'Role_0' : Command(), 'Role_1' : Command(),
                'Role_2' : Command(), 'Role_3' : Command(),
                'Role_4' : Command(), 'Role_5' : Command()}

    _ball_odom = Odometry()

    _friend_goalie_id = 0
    _friend_color = 'blue'
    _friend_odoms = [Odometry()] * 12
    _existing_friends_id = [None] * 6
    _friend_team_info = RefereeTeamInfo()

    _enemy_goalie_id = 0
    _enemy_odoms = [Odometry()] * 12
    _existing_enemies_id = [None] * 6
    _enemy_team_info = RefereeTeamInfo()

    tf_listener = tf.TransformListener()

    _raw_refbox_command = None
    _refbox_command_changed = False
    _current_refbox_command = ''

    _refbox_dict_blue = {SSL_Referee.HALT : 'HALT', SSL_Referee.STOP : 'STOP',
            SSL_Referee.NORMAL_START : 'NORMAL_START', 
            SSL_Referee.FORCE_START : 'FORCE_START',
            SSL_Referee.PREPARE_KICKOFF_BLUE : 'OUR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_BLUE : 'OUR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_BLUE : 'OUR_DIRECT',
            SSL_Referee.INDIRECT_FREE_BLUE : 'OUR_INDIRECT',
            SSL_Referee.TIMEOUT_BLUE : 'OUR_TIMEOUT',
            SSL_Referee.PREPARE_KICKOFF_YELLOW : 'THEIR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_YELLOW : 'THEIR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_YELLOW : 'THEIR_DIRECT',
            SSL_Referee.INDIRECT_FREE_YELLOW : 'THEIR_INDIRECT',
            SSL_Referee.TIMEOUT_YELLOW : 'THEIR_TIMEOUT'}

    _refbox_dict_yellow = {SSL_Referee.HALT : 'HALT', SSL_Referee.STOP : 'STOP',
            SSL_Referee.NORMAL_START : 'NORMAL_START', 
            SSL_Referee.FORCE_START : 'FORCE_START',
            SSL_Referee.PREPARE_KICKOFF_BLUE : 'THEIR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_BLUE : 'THEIR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_BLUE : 'THEIR_DIRECT',
            SSL_Referee.INDIRECT_FREE_BLUE : 'THEIR_INDIRECT',
            SSL_Referee.TIMEOUT_BLUE : 'THEIR_TIMEOUT',
            SSL_Referee.PREPARE_KICKOFF_YELLOW : 'OUR_PRE_KICKOFF',
            SSL_Referee.PREPARE_PENALTY_YELLOW : 'OUR_PRE_PENALTY',
            SSL_Referee.DIRECT_FREE_YELLOW : 'OUR_DIRECT',
            SSL_Referee.INDIRECT_FREE_YELLOW : 'OUR_INDIRECT',
            SSL_Referee.TIMEOUT_YELLOW : 'OUR_TIMEOUT'}
    _refbox_dict = _refbox_dict_blue

    _observer = Observer()

    _ball_closest_frined_role = None
    _ball_closest_enemy_role = None


    @classmethod
    def update_world(cls):
        WorldModel._update_situation()
        rospy.logdebug('Referee: ' + WorldModel._current_refbox_command)
        rospy.logdebug('Situation: ' + WorldModel._current_situation)

        WorldModel._update_enemy_assignments()
        WorldModel._update_threat_assignments()

    
    @classmethod
    def update_assignments(cls, assignment_type=None):
        # IDが存在しないRoleをNoneにする
        IDs = WorldModel._existing_friends_id[:]
        
        for role, robot_id in WorldModel.assignments.items():
            if not robot_id in IDs:
                WorldModel.assignments[role] = None

        # IDsからすでにRoleが登録されてるIDを取り除く
        for robot_id in WorldModel.assignments.values():
            if robot_id in IDs:
                IDs.remove(robot_id)

        # Role_0にGoalie_IDを登録する
        if WorldModel._friend_goalie_id in IDs:
            IDs.remove(WorldModel._friend_goalie_id)
            WorldModel.assignments['Role_0'] = WorldModel._friend_goalie_id
        
        # 残ったIDを順番にRoleに登録する
        for role, robot_id in WorldModel.assignments.items():
            if IDs and role != 'Role_0' and robot_id is None:
                WorldModel.assignments[role] = IDs.pop(0)
        
        # IDが登録されてないRoleは末尾から詰める
        target_i = 1
        replace_i = 5
        while replace_i - target_i > 0:
            while replace_i > 2:
                if WorldModel.assignments['Role_' + str(replace_i)] is not None:
                    break
                replace_i -= 1

            target_role = 'Role_' + str(target_i)
            if WorldModel.assignments[target_role] is None:
                replace_role = 'Role_' + str(replace_i)
                replace_ID = WorldModel.assignments[replace_role]
                WorldModel.assignments[target_role] = replace_ID
                WorldModel.assignments[replace_role] = None

            target_i += 1

        # Ball holder のRoleとRole_1を入れ替える
        # Ball holderがRole_0だったら何もしない
        if assignment_type == 'CLOSEST_BALL':
            closest_role = WorldModel._update_closest_role(True)
            if closest_role and closest_role != 'Role_0':
                #swap
                WorldModel.assignments[closest_role], WorldModel.assignments['Role_1'] =\
                        WorldModel.assignments['Role_1'], WorldModel.assignments[closest_role]
                # closest_role をRole_1にもどす
                WorldModel._ball_closest_frined_role = 'Role_1'

    
    @classmethod
    def set_friend_color(cls, data):
        WorldModel._friend_color = data
        if data == 'blue':
            WorldModel._refbox_dict = WorldModel._refbox_dict_blue
        elif data == 'yellow':
            WorldModel._refbox_dict = WorldModel._refbox_dict_yellow


    @classmethod
    def set_friend_goalie_id(cls, robot_id):
        WorldModel._friend_goalie_id = robot_id


    @classmethod
    def set_blue_info(cls, team_info):
        if WorldModel._friend_color == 'blue':
            WorldModel._friend_team_info = team_info
        else:
            WorldModel._enemy_team_info = team_info
            WorldModel._enemy_goalie_id = team_info.goalie


    @classmethod
    def set_yellow_info(cls, team_info):
        if WorldModel._friend_color == 'yellow':
            WorldModel._friend_team_info = team_info
        else:
            WorldModel._enemy_team_info = team_info
            WorldModel._enemy_goalie_id = team_info.goalie


    def set_enemy_goalie_id(cls, robot_id):
        WorldModel._enemy_goalie_id = robot_id


    @classmethod
    def set_ball_odom(cls, msg):
        WorldModel._ball_odom = msg
        

    @classmethod
    def set_existing_friends_id(cls, data):
        WorldModel._existing_friends_id  = list(data)
        
    
    @classmethod
    def set_existing_enemies_id(cls, data):
        WorldModel._existing_enemies_id = list(data)
        

    @classmethod
    def set_friend_odom(cls, msg, robot_id):
        WorldModel._friend_odoms[robot_id] = msg


    @classmethod
    def set_enemy_odom(cls, msg, robot_id):
        WorldModel._enemy_odoms[robot_id] = msg


    @classmethod
    def set_refbox_command(cls, data):
        if WorldModel._raw_refbox_command != data and \
                data in WorldModel._refbox_dict:
            WorldModel._refbox_command_changed = True
            WorldModel._raw_refbox_command = data

    
    @classmethod
    def set_test_name(cls, data):
        WorldModel._current_test = data

    @classmethod 
    def dot(self,p1, p2): 
        return p1.real*p2.real + p1.imag*p2.imag 
 
    @classmethod 
    def cross(self,p1, p2): 
        return p1.real*p2.imag - p1.imag*p2.real 
 
    @classmethod 
    def dotLineDist(self,p, line): 
        "line: (p1, p2)" 
        a, b = line 
        if WorldModel.dot(b-a, p-a) <= 0.0: 
            #return abs(p-a) 
            return 12 
        if WorldModel.dot(a-b, p-b) <= 0.0: 
            return abs(p-b) 
        return abs(WorldModel.cross(b-a, p-a))/abs(b-a) 
 
    @classmethod 
    def KeepAnalysis(cls): 
        #p1,p2の初期定義 
        p1 = Pose(-constants.FieldHalfX + constants.RobotRadius * 2.0, -(constants.GoalHalfSize - constants.RobotRadius), 0) 
        p2 = Pose(-constants.FieldHalfX + constants.RobotRadius * 2.0, (constants.GoalHalfSize - constants.RobotRadius), 0) 
        if WorldModel.get_velocity('Ball') is None or WorldModel.get_velocity('Threat_0') is None: 
            base = WorldModel.get_pose('CONST_OUR_GOAL') 
            target = WorldModel.get_pose('Ball') 
            # rospy.logerr('0') 
        elif pow(WorldModel.get_velocity('Ball').x,2)+pow(WorldModel.get_velocity('Ball').y,2) > pow(2.5,2):#ボールに速度がある 
            #ボールの速度方向を計算 
            angleOfSpeed = math.atan2(WorldModel.get_velocity('Ball').y, WorldModel.get_velocity('Ball').x) 
            Length = 12 
            VelPointX = Length * math.cos(angleOfSpeed) + WorldModel.get_pose('Ball').x 
            VelPointY = Length * math.sin(angleOfSpeed) + WorldModel.get_pose('Ball').y 
            VelPoint = Pose(VelPointX, VelPointY, 0) 
            #速度拡大線とゴールの距離を計算 
            GoalDist = WorldModel.dotLineDist(WorldModel.get_pose('CONST_OUR_GOAL').x+WorldModel.get_pose('CONST_OUR_GOAL').y*1j,(WorldModel.get_pose('Ball').x+WorldModel.get_pose('Ball').y*1j,VelPoint.x+VelPoint.y*1j)) 
            if GoalDist <= constants.GoalHalfSize + 0.3:#シュートっぽい 
                base = VelPoint 
                target = WorldModel.get_pose('Ball') 
                # rospy.logerr(' v s') 
                #p1,p2の再計算 
                ballVelVector = Pose(VelPoint.x - WorldModel.get_pose('Ball').x, VelPoint.y - WorldModel.get_pose('Ball').y, 0) 
                ballKeepVector = Pose(WorldModel.get_pose('Role_0').x - WorldModel.get_pose('Ball').x, WorldModel.get_pose('Role_0').y - WorldModel.get_pose('Ball').y, 0) 
                ballNearVectorR = WorldModel.dot(ballVelVector.x+ballVelVector.y*1j,ballKeepVector.x+ballKeepVector.y*1j)/(pow(ballVelVector.x,2)+pow(ballVelVector.y,2)) 
                ballNearVectorX = ballNearVectorR*ballVelVector.x 
                ballNearVectorY = ballNearVectorR*ballVelVector.y 
                if ballNearVectorX+WorldModel.get_pose('Ball').x > -constants.FieldHalfX + constants.RobotRadius and abs(ballNearVectorY+WorldModel.get_pose('Ball').y)<constants.GoalHalfSize and abs(WorldModel.get_pose('Role_0').y)< constants.GoalHalfSize: 
                    p1 = Pose(WorldModel.get_pose('Ball').x+ballNearVectorR*ballVelVector.x,WorldModel.get_pose('Ball').y+ballNearVectorR*ballVelVector.y+0.1,0) 
                    p2 = Pose(WorldModel.get_pose('Ball').x+ballNearVectorR*ballVelVector.x,WorldModel.get_pose('Ball').y+ballNearVectorR*ballVelVector.y-0.1,0) 
                # p1=Pose(WorldModel.get_pose('Ball').x,WorldModel.get_pose('Ball').y+1,0) 
                # p2=Pose(WorldModel.get_pose('Ball').x,WorldModel.get_pose('Ball').y-1,0) 
            else:#パスっぽい 
                #ボールの速度線に最も近い敵を探索 
                MinEnemyDist = 15 
                for enemy_num in range (0,len(WorldModel.enemy_assignments)): 
                    EnemyDist = WorldModel.dotLineDist(WorldModel.get_pose('Enemy_'+str(enemy_num)).x+WorldModel.get_pose('Enemy_'+str(enemy_num)).y*1j,(WorldModel.get_pose('Ball').x+WorldModel.get_pose('Ball').y*1j,VelPoint.x+VelPoint.y*1j)) 
                    if EnemyDist < MinEnemyDist: 
                        MinEnemyDist = EnemyDist 
                        EnemyIndex = enemy_num 
                base = WorldModel.get_pose('CONST_OUR_GOAL') 
                target = WorldModel.get_pose('Enemy_'+str(EnemyIndex)) 
                # rospy.logerr(' v p '+str(WorldModel.get_velocity('Ball'))) 
        else:#ボールに速度がない 
            #脅威マシンの角度と角速度を計算 
            angleOfEnemy = WorldModel.get_pose('Threat_0').theta
            angleSpeedOfEnemy = WorldModel.get_velocity('Threat_0').theta
            #脅威マシンとゴールのなす角を計算 
            angleOfEnemyToGoal =  math.atan2(WorldModel.get_pose('CONST_OUR_GOAL').y-WorldModel.get_pose('Threat_0').y, WorldModel.get_pose('CONST_OUR_GOAL').x-WorldModel.get_pose('Threat_0').x) 
            #脅威マシンとボールのなす角を計算 
            angleOfEnemyToBall =  math.atan2(WorldModel.get_pose('Ball').y-WorldModel.get_pose('Threat_0').y, WorldModel.get_pose('Ball').x-WorldModel.get_pose('Threat_0').x) 
            #角度差を計算 
            differenceAngle = angleOfEnemy - angleOfEnemyToGoal 
            Length = 12 
            AnglePointX = Length * math.cos(angleOfEnemy) + WorldModel.get_pose('Threat_0').x 
            AnglePointY = Length * math.sin(angleOfEnemy) + WorldModel.get_pose('Threat_0').y 
            AnglePoint = Pose(AnglePointX, AnglePointY, 0) 
            # rospy.logerr('AnglePoint'+str(AnglePoint)) 
            #角度拡大線とゴールの距離を計算 
            GoalDist = WorldModel.dotLineDist(WorldModel.get_pose('CONST_OUR_GOAL').x+WorldModel.get_pose('CONST_OUR_GOAL').y*1j,(WorldModel.get_pose('Threat_0').x+WorldModel.get_pose('Threat_0').y*1j,AnglePoint.x+AnglePoint.y*1j)) 
            # rospy.logerr('GoalDist'+str(GoalDist)) 
            if GoalDist <= constants.GoalHalfSize :#今にもシュートをしたそうにこちらを見つめている 
                if abs(angleOfEnemyToGoal)<2.5:#正面からシュート（打ち分けがきく） 
                    d=66 
                    r=90 
                    angleOfCanShoot = math.acos(d/r) *3/4#正面のシュート領域の中央から3/4いないで撃ってくる想定
                    if angleOfEnemyToGoal<angleOfEnemy and angleOfEnemy<angleOfEnemyToBall:
                        base = AnglePoint 
                        target = WorldModel.get_pose('Ball') 
                    elif angleOfEnemyToGoal>angleOfEnemy and angleOfEnemy>angleOfEnemyToBall:
                        base = AnglePoint 
                        target = WorldModel.get_pose('Ball') 
                    elif angleOfEnemyToBall < angleOfEnemy - angleOfCanShoot: 
                        angleOfShoot = angleOfEnemyToBall + angleOfCanShoot 
                        ShootAnglePointX = Length * math.cos(angleOfShoot) + WorldModel.get_pose('Ball').x 
                        ShootAnglePointY = Length * math.sin(angleOfShoot) + WorldModel.get_pose('Ball').y 
                        ShootAnglePoint = Pose(ShootAnglePointX, ShootAnglePointY, 0) 
                        base = ShootAnglePoint 
                        target = WorldModel.get_pose('Ball') 
                    elif angleOfEnemyToBall > angleOfEnemy + angleOfCanShoot: 
                        angleOfShoot = angleOfEnemyToBall - angleOfCanShoot 
                        ShootAnglePointX = Length * math.cos(angleOfShoot) + WorldModel.get_pose('Ball').x 
                        ShootAnglePointY = Length * math.sin(angleOfShoot) + WorldModel.get_pose('Ball').y 
                        ShootAnglePoint = Pose(ShootAnglePointX, ShootAnglePointY, 0) 
                        base = ShootAnglePoint 
                        target = WorldModel.get_pose('Ball') 
                    else: 
                        base = AnglePoint 
                        target = WorldModel.get_pose('Ball') 
                    # rospy.logerr('nv s straight') 
                else:#横からシュート（打ち分けがきかない） 
                    base = WorldModel.get_pose('CONST_OUR_GOAL') 
                    target = WorldModel.get_pose('Ball') 
                    # rospy.logerr('nv s side') 
                    # p1=Pose(WorldModel.get_pose('Ball').x,WorldModel.get_pose('Ball').y+1,0) 
                    # p2=Pose(WorldModel.get_pose('Ball').x,WorldModel.get_pose('Ball').y-1,0) 
            elif GoalDist <= constants.GoalHalfSize + 0.5:#いつでもシュートをできそうにしている(1次シュート対策広めに取って2次シュート対策ディフェンダーに任せる) 
                base = WorldModel.get_pose('CONST_OUR_GOAL') 
                target = WorldModel.get_pose('Ball') 
                # rospy.logerr('nv s if') 
            else:#今すぐシュートはできない 
                #脅威マシン角度線に最も近い敵を探索 
                MinEnemyDist = 15 
                for enemy_num in range (0,len(WorldModel.enemy_assignments)): 
                    EnemyDist = WorldModel.dotLineDist(WorldModel.get_pose('Enemy_'+str(enemy_num)).x+WorldModel.get_pose('Enemy_'+str(enemy_num)).y*1j,(WorldModel.get_pose('Threat_0').x+WorldModel.get_pose('Threat_0').y*1j,AnglePoint.x+AnglePoint.y*1j)) 
                    if EnemyDist < MinEnemyDist: 
                        MinEnemyDist = EnemyDist 
                        EnemyIndex = enemy_num  
                #脅威マシンと推定パス対象マシンのなす角を計算 
                angleOfEnemyToEnemy = math.atan2(WorldModel.get_pose('Enemy_'+str(EnemyIndex)).y-WorldModel.get_pose('Threat_0').y, WorldModel.get_pose('Enemy_'+str(EnemyIndex)).x-WorldModel.get_pose('Threat_0').x) 
                #3つの角度を配列に代入 
                angles = [0,0,0] 
                angles[0] = angleOfEnemy 
                angles[1] = angleOfEnemyToGoal 
                angles[2] = angleOfEnemyToEnemy 
                #rospy.logerr('EnemyIndex: '+str(EnemyIndex)+'angleOfEnemy: '+str(angleOfEnemy)+' angleOfEnemyToGoal: '+str(angleOfEnemyToGoal)+' angleOfEnemyToEnemy: '+str(angleOfEnemyToEnemy)) 
                #3つの角度をソート 
                a=np.array(angles) 
                angles_sort=a.argsort() 
                if angles_sort[1] == 2:#真ん中がパス角のケース 
                    if (angles_sort[0] == 1 and angleSpeedOfEnemy < -0.01) or (angles_sort[2] == 1 and angleSpeedOfEnemy >= 0.01):#ゴール角最小で角速度大きくマイナスまたはゴール角最大で角速度大きくプラス（角速度大きくゴール方向）しきい値3radps 
                        base = WorldModel.get_pose('CONST_OUR_GOAL') 
                        target = WorldModel.get_pose('Ball') 
                        # rospy.logerr('nv ns mp asg') 
                    else:#パス角に収束しそう 
                        base = WorldModel.get_pose('CONST_OUR_GOAL') 
                        target = WorldModel.get_pose('Ball') 
                        #target = WorldModel.get_pose('Enemy_'+str(EnemyIndex)) 
                        # rospy.logerr('nv ns mp asp') 
                elif angles_sort[1] == 1:#真ん中がゴール角のケース 
                    base = WorldModel.get_pose('CONST_OUR_GOAL') 
                    target = WorldModel.get_pose('Ball') 
                    # rospy.logerr('nv ns mg') 
                else :#真ん中がスレッド角のケース(シュートもできるしパスもできる) 
                    if (angles_sort[0] == 1 and angleSpeedOfEnemy < 0.5) or (angles_sort[2] == 1 and angleSpeedOfEnemy >= -0.5):#ゴール角最小で角速度マイナスまたはゴール角最大で角速度プラス（角速度ゴール方向） 
                        base = WorldModel.get_pose('CONST_OUR_GOAL') 
                        target = WorldModel.get_pose('Ball') 
                        # rospy.logerr('nv ns me asg') 
                    elif (angles_sort[0] == 2 and angleSpeedOfEnemy < -0.5) or (angles_sort[2] == 2 and angleSpeedOfEnemy >= 0.5):#パス角最小で角速度マイナスまたはパス角最大で角速度プラス（角速度パス方向） 
                        base = WorldModel.get_pose('CONST_OUR_GOAL') 
                        target = WorldModel.get_pose('Ball') 
                        #target = WorldModel.get_pose('Enemy_'+str(EnemyIndex)) 
                        # rospy.logerr('nv ns me asp')        
                    else :
                        base = WorldModel.get_pose('CONST_OUR_GOAL') 
                        target = WorldModel.get_pose('Ball')   
 
        return base,target,p1,p2#ベースとターゲット入れ替えなう 
 
    @classmethod 
    def MarkAnalysis(cls): 
        if WorldModel.get_velocity('Ball') is None or WorldModel.get_velocity('Threat_0') is None: 
            base = 'CONST_OUR_GOAL'
            target = 'Threat_0'
        elif pow(WorldModel.get_velocity('Ball').x,2)+pow(WorldModel.get_velocity('Ball').y,2) > pow(2.5,2):#ボールに速度がある 
            #ボールの速度方向を計算 
            angleOfSpeed = math.atan2(WorldModel.get_velocity('Ball').y, WorldModel.get_velocity('Ball').x) 
            Length = 12 
            VelPointX = Length * math.cos(angleOfSpeed) + WorldModel.get_pose('Ball').x 
            VelPointY = Length * math.sin(angleOfSpeed) + WorldModel.get_pose('Ball').y 
            VelPoint = Pose(VelPointX, VelPointY, 0) 
            #速度拡大線とゴールの距離を計算 
            GoalDist = WorldModel.dotLineDist(WorldModel.get_pose('CONST_OUR_GOAL').x+WorldModel.get_pose('CONST_OUR_GOAL').y*1j,(WorldModel.get_pose('Ball').x+WorldModel.get_pose('Ball').y*1j,VelPoint.x+VelPoint.y*1j)) 
            if GoalDist <= constants.GoalHalfSize + 0.1:#シュートっぽい 
                base = 'CONST_OUR_GOAL' 
                target = 'Threat_0'
            else :#パスっぽい 
                #ボールの速度線に最も近い敵を探索 
                MinEnemyDist = 15 
                for enemy_num in range (0,len(WorldModel.enemy_assignments)): 
                    EnemyDist = WorldModel.dotLineDist(WorldModel.get_pose('Enemy_'+str(enemy_num)).x+WorldModel.get_pose('Enemy_'+str(enemy_num)).y*1j,(WorldModel.get_pose('Ball').x+WorldModel.get_pose('Ball').y*1j,VelPoint.x+VelPoint.y*1j)) 
                    if EnemyDist < MinEnemyDist: 
                        MinEnemyDist = EnemyDist 
                        EnemyIndex = enemy_num 
                base = 'Threat_0'
                target = 'Enemy_'+str(EnemyIndex)
        else :#ボールに速度がない 
            #脅威マシンの角度と角速度を計算 
            angleOfEnemy = WorldModel.get_pose('Threat_0').theta#敵なので反転np.pi- 
            angleSpeedOfEnemy = WorldModel.get_velocity('Threat_0').theta#- 
            #脅威マシンとゴールのなす角を計算 
            angleOfEnemyToGoal =  math.atan2(WorldModel.get_pose('CONST_OUR_GOAL').y-WorldModel.get_pose('Threat_0').y, WorldModel.get_pose('CONST_OUR_GOAL').x-WorldModel.get_pose('Threat_0').x) 
            #脅威マシンとボールのなす角を計算 
            angleOfEnemyToBall =  math.atan2(WorldModel.get_pose('Ball').y-WorldModel.get_pose('Threat_0').y, WorldModel.get_pose('Ball').x-WorldModel.get_pose('Threat_0').x) 
            #角度差を計算 
            differenceAngle = angleOfEnemy - angleOfEnemyToGoal 
            Length = 12 
            AnglePointX = Length * math.cos(angleOfEnemy) + WorldModel.get_pose('Threat_0').x 
            AnglePointY = Length * math.sin(angleOfEnemy) + WorldModel.get_pose('Threat_0').y 
            AnglePoint = Pose(AnglePointX, AnglePointY, 0) 
            # rospy.logerr('AnglePoint'+str(AnglePoint)) 
            #角度拡大線とゴールの距離を計算 
            GoalDist = WorldModel.dotLineDist(WorldModel.get_pose('CONST_OUR_GOAL').x+WorldModel.get_pose('CONST_OUR_GOAL').y*1j,(WorldModel.get_pose('Threat_0').x+WorldModel.get_pose('Threat_0').y*1j,AnglePoint.x+AnglePoint.y*1j)) 
            # rospy.logerr('GoalDist'+str(GoalDist)) 
            if GoalDist <= constants.GoalHalfSize+0.5:#シュートしそう 
                base = 'CONST_OUR_GOAL'
                target = 'Threat_0'
            else: 
                #脅威マシン角度線に最も近い敵を探索 
                MinEnemyDist = 15 
                for enemy_num in range (0,len(WorldModel.enemy_assignments)): 
                    EnemyDist = WorldModel.dotLineDist(WorldModel.get_pose('Enemy_'+str(enemy_num)).x+WorldModel.get_pose('Enemy_'+str(enemy_num)).y*1j,(WorldModel.get_pose('Threat_0').x+WorldModel.get_pose('Threat_0').y*1j,AnglePoint.x+AnglePoint.y*1j)) 
                    if EnemyDist < MinEnemyDist: 
                        MinEnemyDist = EnemyDist 
                        EnemyIndex = enemy_num  
                base = 'Threat_0'
                target = 'Enemy_'+str(EnemyIndex) 
        return base,target 


    @classmethod
    def get_pose(cls, name):
        pose = None

        if name == 'Ball':
            ball_pose = WorldModel._ball_odom.pose.pose.position
            pose = Pose(ball_pose.x, ball_pose.y, 0)

        elif name.startswith('Role'):
            robot_id = WorldModel.assignments[name]
            pose = WorldModel.get_friend_pose(robot_id)

        elif name.startswith('Enemy'):
            robot_id = WorldModel.enemy_assignments[name]
            pose = WorldModel.get_enemy_pose(robot_id)

        elif name.startswith('Threat'):
            robot_id = WorldModel._threat_assignments[name]
            pose = WorldModel.get_enemy_pose(robot_id)

        elif name.startswith('CONST'):
            if name not in constants.poses.keys():
                return pose  # None
            pose = constants.poses[name]

        elif name[:4]=='Keep': 
            if name == 'KeepBase': 
                pose=WorldModel.KeepAnalysis()[0] 
            elif name == 'KeepTarget': 
                pose=WorldModel.KeepAnalysis()[1] 
            elif name == 'KeepP1': 
                pose=WorldModel.KeepAnalysis()[2] 
            elif name == 'KeepP2': 
                pose=WorldModel.KeepAnalysis()[3] 

        elif name[:4]=='Mark': 
            if name == 'MarkBase': 
                pose = WorldModel.get_pose(WorldModel.MarkAnalysis()[0])
            elif name == 'MarkTarget':
                pose = WorldModel.get_pose(WorldModel.MarkAnalysis()[1])
        
        elif name[:5] == 'ANALY':#評価したエリアの座標
            if name == 'ANALY_PATH': #パスをシュートする
                pose = FieldAnalysis.get_analysis_area_pose('SHOOT')
            #rospy.logerr(pose)
            if name == 'ANALY_RECEIVE': #パスを受け取る
                pose = FieldAnalysis.get_analysis_area_pose('RECEIVE')


        #rospy.logerr(WorldModel.get_friend_pose(1))

        return pose

    @classmethod
    def get_velocity(cls, name):
        velocity = None

        if name == 'Ball':
            linear = WorldModel._ball_odom.twist.twist.linear
            velocity = Velocity(linear.x, linear.y, 0)

        elif name[:4] == 'Role':
            robot_id = WorldModel.assignments[name]
            velocity = WorldModel.get_friend_velocity(robot_id)

        elif name[:5] == 'Enemy':
            robot_id = WorldModel.enemy_assignments[name]
            velocity = WorldModel.get_enemy_velocity(robot_id)

        elif name[:6] == 'Threat':
            robot_id = WorldModel._threat_assignments[name]
            velocity = WorldModel.get_enemy_velocity(robot_id)

        return velocity

    
    @classmethod
    def get_friend_pose(cls, robot_id):

        if robot_id is None:
            return None

        position = WorldModel._friend_odoms[robot_id].pose.pose.position
        orientation = WorldModel._friend_odoms[robot_id].pose.pose.orientation
        yaw = tool.yawFromQuaternion(orientation)

        return Pose(position.x, position.y, yaw)
        

    @classmethod
    def get_enemy_pose(cls, robot_id):

        if robot_id is None:
            return None

        position = WorldModel._enemy_odoms[robot_id].pose.pose.position
        orientation = WorldModel._enemy_odoms[robot_id].pose.pose.orientation
        yaw = tool.yawFromQuaternion(orientation)

        return Pose(position.x, position.y, yaw)


    @classmethod
    def get_friend_velocity(cls, robot_id):

        if robot_id is None:
            return None

        linear = WorldModel._friend_odoms[robot_id].twist.twist.linear
        angular = WorldModel._friend_odoms[robot_id].twist.twist.angular

        return Velocity(linear.x, linear.y, angular.z)
        

    @classmethod
    def get_enemy_velocity(cls, robot_id):

        if robot_id is None:
            return None

        linear = WorldModel._enemy_odoms[robot_id].twist.twist.linear
        angular = WorldModel._enemy_odoms[robot_id].twist.twist.angular

        return Velocity(linear.x, linear.y, angular.z)


    @classmethod
    def _update_enemy_assignments(cls):
        # IDが存在しないRoleをNoneにする
        IDs = WorldModel._existing_enemies_id[:]
        
        for role, robot_id in WorldModel.enemy_assignments.items():
            if not robot_id in IDs:
                WorldModel.enemy_assignments[role] = None

        # IDsからすでにRoleが登録されてるIDを取り除く
        for robot_id in WorldModel.enemy_assignments.values():
            if robot_id in IDs:
                IDs.remove(robot_id)

        # Role_0にGoalie_IDを登録する
        if WorldModel._enemy_goalie_id in IDs:
            IDs.remove(WorldModel._enemy_goalie_id)
            WorldModel.enemy_assignments['Enemy_0'] = WorldModel._enemy_goalie_id
        
        # 残ったIDを順番にRoleに登録する
        for role, robot_id in WorldModel.enemy_assignments.items():
            if IDs and role != 'Enemy_0' and robot_id is None:
                WorldModel.enemy_assignments[role] = IDs.pop(0)
        
        # IDが登録されてないRoleは末尾から詰める
        target_i = 1
        replace_i = 5
        while replace_i - target_i > 0:
            while replace_i > 2:
                if WorldModel.enemy_assignments['Enemy_' + str(replace_i)] is not None:
                    break
                replace_i -= 1

            target_role = 'Enemy_' + str(target_i)
            if WorldModel.enemy_assignments[target_role] is None:
                replace_role = 'Enemy_' + str(replace_i)
                replace_ID = WorldModel.enemy_assignments[replace_role]
                WorldModel.enemy_assignments[target_role] = replace_ID
                WorldModel.enemy_assignments[replace_role] = None

            target_i += 1


    @classmethod
    def _update_situation(cls):
        if WorldModel._refbox_command_changed:
            WorldModel._refbox_command_changed = False

            # raw_refbox_commandをチームカラーによって見方/敵commandへ加工する
            refbox_command = WorldModel._refbox_dict[WorldModel._raw_refbox_command]

            # NORMAL_STARTはKICKOFFとPENALTYのトリガーになるため、その切り分けを行う
            if refbox_command == 'NORMAL_START':
                if WorldModel._current_situation == 'OUR_PRE_KICKOFF':
                    refbox_command = 'OUR_KICKOFF_START'

                elif WorldModel._current_situation == 'OUR_PRE_PENALTY':
                    refbox_command = 'OUR_PENALTY_START'

                elif WorldModel._current_situation == 'THEIR_PRE_KICKOFF':
                    refbox_command = 'THEIR_KICKOFF_START'

                elif WorldModel._current_situation == 'THEIR_PRE_PENALTY':
                    refbox_command = 'THEIR_PENALTY_START'

                else:
                    refbox_command = 'FORCE_START'

            WorldModel._set_current_situation(refbox_command)
            WorldModel._current_refbox_command = refbox_command


        ball_pose = WorldModel.get_pose('Ball')

        # ボールが動いたらインプレイ判定、refbox_commandを上書きする
        if WorldModel._current_refbox_command.endswith('START') or \
                WorldModel._current_refbox_command.endswith('DIRECT'):
            if WorldModel._observer.ball_is_moved(ball_pose):
                WorldModel._current_refbox_command = 'IN_PLAY'
        else:
            WorldModel._observer.set_ball_initial_pose(ball_pose)

        # current_refbox_commandがIN_PLAYのとき、ボール位置で戦況を判定する
        if WorldModel._current_refbox_command == 'IN_PLAY':
            WorldModel._set_current_situation('IN_PLAY')

            if WorldModel._observer.ball_is_in_defence_area(ball_pose, True):
                # 自分のディフェンスエリアに入ったか判定
                WorldModel._set_current_situation('BALL_IN_OUR_DEFENCE')

            elif WorldModel._observer.ball_is_in_defence_area(ball_pose, False):
                # 相手のディフェンスエリアに入ったか判定
                WorldModel._set_current_situation('BALL_IN_THEIR_DEFENCE')

        # ボールがフィールド外に出ることを判定
        # update_situationの最後に実行すること
        if WorldModel._observer.ball_is_in_field(ball_pose):
            if WorldModel._current_refbox_command != 'IN_PLAY':
                WorldModel._set_current_situation(WorldModel._current_refbox_command)
        else:
            WorldModel._set_current_situation('BALL_IN_OUTSIDE')

        # Test実行の判定
        if WorldModel._current_refbox_command != 'HALT' and \
                WorldModel._current_test in WorldModel.situations:
            WorldModel._set_current_situation(WorldModel._current_test)


    @classmethod
    def _set_current_situation(cls, situation):
        WorldModel.situations[WorldModel._current_situation] = False
        WorldModel._current_situation = situation
        WorldModel.situations[WorldModel._current_situation] = True

    @classmethod
    def get_current_situation(cls, ):
        return WorldModel._current_situation

    @classmethod
    def _update_closest_role(cls, is_friend_role=True):
        thresh_dist = 1000
        hysteresis = 0.2

        ball_pose = WorldModel.get_pose('Ball')
        closest_role = None

        prev_closest_role = None
        role_text = ''
        
        if is_friend_role:
            role_text = 'Role_'
            prev_closest_role = WorldModel._ball_closest_frined_role
        else:
            role_text = 'Enemy_'
            prev_closest_role = WorldModel._ball_closest_enemy_role

        for i in range(6):
            role = role_text + str(i)
            pose = WorldModel.get_pose(role)
            
            if pose is None:
                continue

            dist_to_ball = tool.getLength(pose, ball_pose)

            # ヒステリシスをもたせる
            if role == prev_closest_role:
                dist_to_ball -= hysteresis

            if dist_to_ball < thresh_dist:
                thresh_dist = dist_to_ball
                closest_role = role


        if is_friend_role:
            WorldModel._ball_closest_frined_role = closest_role
        else:
            WorldModel._ball_closest_enemy_role = closest_role
            
        return closest_role


    @classmethod
    def _update_threat_assignments(cls):
        # Ballに一番近いEnemyをThreat_0にする
        closest_role = WorldModel._update_closest_role(False)
        if closest_role:
            closest_id = WorldModel.enemy_assignments[closest_role]
            WorldModel._threat_assignments['Threat_0'] = closest_id

