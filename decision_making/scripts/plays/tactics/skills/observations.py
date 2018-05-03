
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel
import constants
import tool

class BallKicked(Task):
    def __init__(self, name, velocity_threshold=None):
        super(BallKicked, self).__init__(name)
        self.velocity_threshold = velocity_threshold

    def run(self):
        ball_velocity = WorldModel.get_velocity('Ball')
        if WorldModel._observer.ball_has_kicked(ball_velocity, self.velocity_threshold):
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING

class Triggered(Task):
    def __init__(self, name, trigger):
        super(Triggered, self).__init__(name)
        self.triggered = trigger

    def run(self):
        if self.triggered():
            print('triggered!')
            return TaskStatus.SUCCESS
        return TaskStatus.RUNNING

class BallInside(Task):
    def __init__(self, name, receiving_area):
        super(BallInside, self).__init__(name)
        self.receiving_area = receiving_area

    def run(self):
        ball_pose = WorldModel.get_pose('Ball')
        if ball_pose is None:
            return TaskStatus.RUNNING
        if self.receiving_area[0] < ball_pose.x < self.receiving_area[2] \
                and self.receiving_area[1] < ball_pose.y < self.receiving_area[3]:
            return TaskStatus.SUCCESS
        return TaskStatus.RUNNING

class BallReceived(Task):
    def __init__(self, name, my_role, dist_threshold, velocity_threshold):
        super(BallReceived, self).__init__(name)
        self.my_role = my_role
        self.dist_threshold = dist_threshold
        self.velocity_threshold = velocity_threshold

    def run(self):
        my_pose = WorldModel.get_pose(self.my_role)
        ball_pose = WorldModel.get_pose('Ball')
        if ball_pose is None:
            return TaskStatus.RUNNING
        ball_velocity = WorldModel.get_velocity('Ball')
        vel_abs = tool.getLengthFromCenter(ball_velocity)
        # print(tool.getLength(my_pose, ball_pose))
        # print(vel_abs)
        if (tool.getLength(my_pose, ball_pose) < self.dist_threshold + constants.BallRadius + constants.RobotRadius) \
                and vel_abs < self.velocity_threshold:
            return TaskStatus.SUCCESS
        return TaskStatus.RUNNING
