from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from skills.adjustments import WithKick, NoBallAvoidance
from skills.observations import BallKicked, Triggered

sys.path.append(os.pardir)
from coordinate import Coordinate


class TacticReceiveBallAtAndShoot(Sequence):
    def __init__(self, name, my_role, ref_cornerkick_state, normal_pose, receiving_area):
        super(TacticReceiveBallAtAndShoot, self).__init__(name)

        coord_keep = Coordinate()
        coord_keep.set_position_looking_at_target(my_role, normal_pose, "Ball")
        p = ParallelOne('stay_normal_position')
        p.add_child(DynamicDrive('normal_position', my_role, coord_keep, always_running=True))
        p.add_child(Triggered('trigger_kicked', ref_cornerkick_state))

        self.add_child(p)

        ###
        drive_and_shoot = Selector('shoot')
        coord_receive = Coordinate()
        coord_receive.set_receive_ball(my_role, 1.0)
        drive_and_shoot.add_child(DynamicDrive('drive_to_receive', my_role, coord_receive))


        shoot_seq = Sequence('shoot_seq')
        coord = Coordinate()
        # coord._tuning_param_x = 0.1
        # coord._tuning_param_y = 0.1
        coord.set_approach_to_shoot(my_role, target='CONST_THEIR_GOAL')

        drive = ParallelOne('drive')
        drive.add_child(DynamicDrive('drive_to_ball', my_role, coord))
        drive.add_child(NoBallAvoidance('NoBallAvoidance', my_role))

        shoot = ParallelOne('shoot')
        shoot.add_child(DynamicDrive('drive_to_shoot', my_role, coord, always_running = True))
        shoot.add_child(WithKick('WithKick', my_role))
        shoot.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        shoot.add_child(BallKicked('BallKicked'))

        shoot_seq.add_child(drive)
        shoot_seq.add_child(shoot)

        drive_and_shoot.add_child(shoot_seq)

        ###

        self.add_child(drive_and_shoot)


