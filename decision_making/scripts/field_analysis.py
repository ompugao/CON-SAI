#!/usr/bin/env python
# -*- coding: utf-8 -*-import math
import tool
import constants
from world_model import WorldModel
from consai_msgs.msg import Pose
import rospy


from collections import OrderedDict, defaultdict
from nav_msgs.msg import Odometry
from consai_msgs.msg import Pose
from consai_msgs.msg import Pose as Velocity

class FieldAnalysis(object):
   @classmethod
   def update_field_analysis(cls):
       rospy.logerr(WorldModel.get_enemy_pose(1))