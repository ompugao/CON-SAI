from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *
from pi_trees_lib_extensions import ConditionalSequence, OverwriteSequenceCondition, Print, ModeSelector, ToggleInplayMode, ResetState, TriggerKicked, DoNothing

from skills.dynamic_drive import DynamicDrive
from skills.adjustments import WithKick, WithKickIfLookingAt, NoBallAvoidance, WithDefenceAreaAvoidance, WithDribble
from skills.observations import BallKicked, Triggered, BallInside, BallReceived
from world_model import WorldModel

sys.path.append(os.pardir)
from coordinate import Coordinate

import numpy as np

class TacticInplayReceiveAndShoot(Sequence):
    def __init__(self, name, my_role, ref_mode, ref_kickstate, shoot_if_mode_is_true, *args, **kwargs):
        super(TacticInplayReceiveAndShoot, self).__init__(name, reset_after=True, *args, **kwargs)


        receive_seq = Sequence('receive_seq')

        coord_receiving = Coordinate()
        coord_receiving.set_dynamic_position_looking_at_target(my_role, 'ANALY_RECEIVE', 'Ball')
        p = ParallelOne('drive_to_receiving_position')
        p.add_child(DynamicDrive('drive_to_receiving_position', my_role, coord_receiving, always_running=True))
        p.add_child(Print('%s is moving to receiving position'%(my_role)))
        p.add_child(Triggered('trigger_kicked', ref_kickstate))

        receive_seq.add_child(p)

        coord_receive = Coordinate()
        coord_receive.set_receive_ball(my_role, 20.0, force_receive=True, target='CONST_THEIR_GOAL')
        p = ParallelOne('drive_to_receive')
        p.add_child(Print('%s is receiving'%(my_role)))
        p.add_child(DynamicDrive('drive_to_receive', my_role, coord_receive, always_running=True))
        p.add_child(BallReceived('receive_ball_condition', my_role, 0.08, 0.3))
        receive_seq.add_child(p)
        receive_seq.add_child(ToggleInplayMode('toggle_inplay_mode', ref_mode))
        receive_seq.add_child(ResetState('reset_state', ref_kickstate))

        ###
        shoot_seq = Sequence('pass_or_shoot_seq')

        coord = Coordinate()
        #coord.set_approach_to_shoot(my_role, target=Point(0.0,0.0,0))#'CONST_OUR_GOAL'
        coord.set_approach_to_shoot(my_role, target='ANALY_PATH')
        DRIVE = ParallelOne('DRIVE')
        DRIVE.add_child(WithKickIfLookingAt('WithKickIfLookingAt', my_role, 'ANALY_PATH', np.deg2rad(0.5)))
        DRIVE.add_child(DynamicDrive('drive_to_ball', my_role, coord))
        DRIVE.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        DRIVE.add_child(WithDribble('Dribbling', my_role))
        DRIVE.add_child(Print('%s is driving to shoot'%(my_role)))

        SHOOT = ParallelOne('SHOOT')
        SHOOT.add_child(DynamicDrive('drive_to_shoot', my_role, coord, always_running = True))
        SHOOT.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
        SHOOT.add_child(BallKicked('BallKicked', 0.05))
        SHOOT.add_child(WithDefenceAreaAvoidance('AvoidDefenceArea', my_role))
        SHOOT.add_child(WithDribble('Dribbling', my_role))
        SHOOT.add_child(Print('%s kicks!'%(my_role)))

        shoot_seq.add_child(DRIVE)
        shoot_seq.add_child(SHOOT)
        shoot_seq.add_child(TriggerKicked('trigger_kicked', ref_kickstate))

        shoot_seq.add_child(DoNothing('donothing', TaskStatus.RUNNING))


        children = None
        if shoot_if_mode_is_true:
            children = [shoot_seq, receive_seq]
        else:
            children = [receive_seq, shoot_seq]

        mode_selector = ModeSelector('shoot_receive_mode_selector', ref_mode, children=children)
        self.add_child(mode_selector)


