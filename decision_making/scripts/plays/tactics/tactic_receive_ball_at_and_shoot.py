from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *
from pi_trees_lib_extensions import ConditionalSequence, OverwriteSequenceCondition, Print

from skills.dynamic_drive import DynamicDrive
from skills.adjustments import WithKick, NoBallAvoidance, WithDefenceAreaAvoidance
from skills.observations import BallKicked, Triggered, BallInside

from world_model import WorldModel

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
        coord_receive = Coordinate()
        coord_receive.set_receive_ball(my_role, 3.0, force_receive=True, target='CONST_THEIR_GOAL')
        def condition_generator():
            _receiving_area = receiving_area
            _my_role = my_role
            def condition():
                ball_pose = WorldModel.get_pose('Ball')
                if _receiving_area[0] < ball_pose.x < _receiving_area[2] \
                        and _receiving_area[1] < ball_pose.y < _receiving_area[3]:
                    print("%s will take ball!"%(_my_role))
                    return True
                return False
            return condition

        # XXX i guess this can be implemented using normal Tasks
        drive_and_shoot = ConditionalSequence('shoot_in_area', condition=condition_generator())
        drive_and_shoot.add_child(DynamicDrive('drive_to_receive', my_role, coord_receive))
        drive_and_shoot.add_child(OverwriteSequenceCondition('continue_after_receive', drive_and_shoot, (lambda: True)))


        shoot_seq = Sequence('shoot_seq')
        coord = Coordinate()
        # coord._tuning_param_x = 0.1
        # coord._tuning_param_y = 0.1
        coord.set_approach_to_shoot(my_role, target='CONST_THEIR_GOAL')

        drive = ParallelOne('drive')
        drive.add_child(DynamicDrive('drive_to_ball', my_role, coord))
        drive.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        drive.add_child(WithDefenceAreaAvoidance('AvoidDefenceArea', my_role))
        drive.add_child(Print('%s drive'%(my_role)))

        shoot = ParallelOne('shoot')
        shoot.add_child(DynamicDrive('drive_to_shoot', my_role, coord, always_running = True))
        shoot.add_child(WithKick('WithKick', my_role))
        shoot.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        shoot.add_child(WithDefenceAreaAvoidance('AvoidDefenceArea', my_role))
        shoot.add_child(BallKicked('BallKicked'))
        drive.add_child(Print('%s shoot'%(my_role)))

        shoot_seq.add_child(drive)
        shoot_seq.add_child(shoot)

        drive_and_shoot.add_child(shoot_seq)

        ###

        self.add_child(drive_and_shoot)


