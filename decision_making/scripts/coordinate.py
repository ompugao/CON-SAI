#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import tool
import constants
from world_model import WorldModel

from consai_msgs.msg import Pose

class Coordinate(object):

    def __init__(self):
        self.pose = Pose() # pos_x, pos_y, thta

        self._base = None
        self._target = None
        self._update_func = None

        # arrival parameters
        self._arrived_position_tolerance = 0.05 # unit:meter
        self._arrived_angle_tolerance = 15.0 * math.pi / 180.0

        # interpose
        self._to_dist = None
        self._from_dist = None


    def update(self):
        result = False
        if self._update_func:
            result = self._update_func()

        return result

    def set_interpose(self, base="CONST_OUR_GOAL", target="Ball", to_dist=None, from_dist=None):

        self._base = base
        self._target = target
        self._to_dist = to_dist
        self._from_dist = from_dist

        self._update_func = self._update_interpose
    

    def is_arrived(self, role):
        # robotが目標位置に到着したかを判断する
        # 厳し目につける

        role_pose = WorldModel.get_pose(role)

        if role_pose is None:
            return False

        arrived = False

        distance = tool.getSize(self.pose, role_pose)

        # 目標位置との距離、目標角度との差がtolerance以下であれば到着判定
        if distance < self._arrived_position_tolerance:
            diff_angle = self.pose.theta - role_pose.theta
            
            if tool.normalize(diff_angle) < self._arrived_angle_tolerance:
                arrived = True

        return arrived


    def _update_interpose(self):
        base_pose = WorldModel.get_pose(self._base)
        target_pose = WorldModel.get_pose(self._target)

        if base_pose is None or target_pose is None:
            return False

        angle_to_target = tool.getAngle(base_pose, target_pose)
        
        interposed_pose = Pose(0, 0, 0)
        if self._to_dist:
            trans = tool.Trans(base_pose, angle_to_target)
            tr_interposed_pose = Pose(self._to_dist, 0.0, 0)
            interposed_pose = trans.invertedTransform(tr_interposed_pose)
        elif self._from_dist:
            angle_to_base = tool.getAngle(target_pose, base_pose)
            trans = tool.Trans(target_pose, angle_to_base)
            tr_interposed_pose = Pose(self._from_dist, 0.0, 0)
            interposed_pose = trans.invertedTransform(tr_interposed_pose)

        interposed_pose.theta = angle_to_target

        self.pose = interposed_pose
        
        return True
