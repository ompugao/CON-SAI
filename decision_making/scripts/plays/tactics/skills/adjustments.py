
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel
import numpy as np
import tool

class WithKick(Task):
    def __init__(self, name, my_role, kick_power=6.0):
        super(WithKick, self).__init__(name)

        self._my_role = my_role
        self._kick_power = kick_power

    def run(self):
        WorldModel.commands[self._my_role].set_kick(self._kick_power)

        return TaskStatus.RUNNING

class WithKickIfLookingAt(Task):
    def __init__(self, name, my_role, target, angle_threshold = np.deg2rad(0.5), kick_power=6.0):
        super(WithKickIfLookingAt, self).__init__(name)
        self._my_role = my_role
        self._kick_power = kick_power
        self._target = target
        self._angle_threshold = angle_threshold

    def run(self):
        target_pose = WorldModel.get_pose(self._target)
        role_pose = WorldModel.get_pose(self._my_role)
        t = tool.Trans(role_pose, role_pose.theta)
        target_pose_local = t.transform(target_pose)

        diff_angle = tool.getAngleFromCenter(target_pose_local)
        if np.abs(diff_angle) < self._angle_threshold:
            WorldModel.commands[self._my_role].set_kick(self._kick_power)
        else:
            WorldModel.commands[self._my_role].set_kick(0)
        return TaskStatus.RUNNING

class NoNavigation(Task):
    def __init__(self, name, my_role):
        super(NoNavigation, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].navigation_enable = False

        return TaskStatus.RUNNING


class NoBallAvoidance(Task):
    def __init__(self, name, my_role):
        super(NoBallAvoidance, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].avoid_ball = False
        return TaskStatus.RUNNING


class NoDefenceAreaAvoidance(Task):
    def __init__(self, name, my_role):
        super(NoDefenceAreaAvoidance, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].avoid_defence_area = False

        return TaskStatus.RUNNING

class WithDefenceAreaAvoidance(Task):
    def __init__(self, name, my_role):
        super(WithDefenceAreaAvoidance, self).__init__(name)

        self._my_role = my_role

    def run(self):
        WorldModel.commands[self._my_role].avoid_defence_area = True

        return TaskStatus.RUNNING


class WithChip(Task):
    def __init__(self, name, my_role, kick_power=6.0):
        super(WithChip, self).__init__(name)

        self._my_role = my_role
        self._kick_power = kick_power


    def run(self):
        WorldModel.commands[self._my_role].set_kick(self._kick_power,True)

        return TaskStatus.RUNNING


class WithDribble(Task):
    def __init__(self, name, my_role, dribble_power=6.0):
        super(WithDribble, self).__init__(name)

        self._my_role = my_role
        self._dribble_power = dribble_power


    def run(self):
        WorldModel.commands[self._my_role].set_dribble(self._dribble_power)

        return TaskStatus.RUNNING

class FlexibleKick(Task):
    def __init__(self, name, my_role, kick_options_func):
        super(FlexibleKick, self).__init__(name)

        self._my_role = my_role
        self._kick_options_func = kick_options_func


    def run(self):
        kick_power, chipkick = self._kick_options_func()
        WorldModel.commands[self._my_role].set_kick(kick_power, chipkick)

        return TaskStatus.RUNNING
