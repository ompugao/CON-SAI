
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

import sys,os
sys.path.append(os.pardir)
from world_model import WorldModel


class BallKicked(Task):
    def __init__(self, name):
        super(BallKicked, self).__init__(name)

    
    def run(self):
        ball_velocity = WorldModel.get_velocity('Ball')
        if WorldModel._observer.ball_has_kicked(ball_velocity):
            return TaskStatus.SUCCESS

        return TaskStatus.RUNNING

class Triggered(Task):
    def __init__(self, name, trigger):
        super(Triggered, self).__init__(name)
        self.triggered = trigger

    def run(self):
        if self.triggered():
            return TaskStatus.SUCCESS
        return TaskStatus.RUNNING

class BallInside(Task):
    def __init__(self, name, receiving_area):
        super(BallInside, self).__init__(name)
        self.receiving_area = receiving_area

    def run(self):
        ball_pose = WorldModel.get_pose('Ball')
        if self.receiving_area[0] < ball_pose.x < self.receiving_area[2] \
                and self.receiving_area[1] < ball_pose.y < self.receiving_area[3]:
            return TaskStatus.SUCCESS
        return TaskStatus.RUNNING
