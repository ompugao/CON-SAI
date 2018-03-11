import rospy

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from skills.observations import BallKicked
from skills.adjustments import WithKick, NoNavigation

from field_analysis import FieldAnalysis
from geometry_msgs.msg import Point

sys.path.append(os.pardir)
from coordinate import Coordinate
from consai_msgs.msg import Pose


class TacticInplayShoot(Selector):
    def __init__(self, name, my_role):
        super(TacticInplayShoot, self).__init__(name)

        coord = Coordinate()
        coord.set_receive_ball(my_role)

        self.add_child(DynamicDrive('drive_to_receive', my_role, coord))
        self.add_child(_Shoot('shoot',my_role))


class _Shoot(Sequence):
    def __init__(self, name, my_role):
        super(_Shoot, self).__init__(name)
 
        coord = Coordinate()
        #coord.set_approach_to_shoot(my_role, target=Point(0.0,0.0,0))#'CONST_OUR_GOAL'
        coord.set_approach_to_shoot(my_role, target='ANALY_PATH')
        DRIVE = ParallelOne('DRIVE')
        DRIVE.add_child(DynamicDrive('drive_to_ball', my_role, coord))
        DRIVE.add_child(NoNavigation('NoNavigation', my_role))

        SHOOT = ParallelOne('SHOOT')
        SHOOT.add_child(DynamicDrive('drive_to_shoot', my_role, coord, 
            always_running = True))
        SHOOT.add_child(WithKick('WithKick', my_role))
        SHOOT.add_child(NoNavigation('NoNavigation', my_role))
        SHOOT.add_child(BallKicked('BallKicked'))

        self.add_child(DRIVE)
        self.add_child(SHOOT)

