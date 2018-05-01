

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive

sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticReceiveBallAt(Selector):
    def __init__(self, name, my_role, pose):
        super(TacticReceiveBallAt, self).__init__(name)

        coord_receive = Coordinate()
        coord_receive.set_receive_ball(my_role, 1.0)
        self.add_child(DynamicDrive('drive_to_receive', my_role, coord_receive))

        coord_keep = Coordinate()
        coord_keep.set_position_looking_at_target(my_role, pose, "Ball")
        self.add_child(DynamicDrive('normal_position', my_role, coord_keep))

