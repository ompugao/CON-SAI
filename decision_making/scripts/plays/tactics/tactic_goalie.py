
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *
from consai_msgs.msg import Pose
from skills.dynamic_drive import DynamicDrive
import math
from world_model import WorldModel
sys.path.append(os.pardir)
from coordinate import Coordinate
import constants
class TacticGoalie(ParallelAll):
    def __init__(self, name, my_role, target='Ball', keep_x=None, 
            keep_y=None, range_high=10.0, range_low=-10.0):
        super(TacticGoalie, self).__init__(name)

        self._coordinate = Coordinate()
        pose1 = Pose(keep_x, -(constants.GoalHalfSize - constants.RobotRadius), 0)
        pose2 = Pose(keep_x, (constants.GoalHalfSize - constants.RobotRadius), 0)
        self._coordinate.set_intersection(base='KeepBase', target='KeepTarget',pose1=WorldModel.get_pose('KeepP1'), pose2=WorldModel.get_pose('KeepP2'))

        self.add_child(DynamicDrive('DynamicDrive', my_role, self._coordinate,
            always_running = True))

