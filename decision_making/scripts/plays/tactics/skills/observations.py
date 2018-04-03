
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

class CheckPassPossibility(Task):
    def __init__(self, name, target):
        """
        target :param str: target role/enemy/constants
        """
        super(CheckPassPossibility, self).__init__(name)
        self.target = target
    
    def run(self):
        """
        return success if it is *not* possible to pass the ball to the target,
        because of pi_trees_lib.Selector behaviour.
        """
        return TaskStatus.SUCCESS
