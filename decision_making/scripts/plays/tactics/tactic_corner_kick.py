
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *
from pi_trees_lib_extensions import ConditionalSequence, OverwriteSequenceCondition, TaskDone, SetTaskDone, ParallelOneIgnoringFailure
from skills.dynamic_drive import DynamicDrive
from skills.observations import BallKicked
from skills.adjustments import WithKick, NoBallAvoidance, FlexibleKick
import weakref

sys.path.append(os.pardir)
from coordinate import Coordinate
#from coordinate_cornerkick import CoordinateCornerKick
from world_model import WorldModel

import numpy as np

class TacticCornerKick(Sequence):
    def __init__(self, name, my_role, ref_cornerkick_state):
        super(TacticCornerKick, self).__init__(name)

        corner_kick = ParallelOneIgnoringFailure('corner_kick')
        task_done = TaskDone('has_done')
        select_passing_friend = RandomSelector('select_passing_friend')
        for i in range(0, 6):
            target_role = "Role_%d"%(i)
            def check_passing_possibility_generator(target_role):
                _target_role = target_role
                def check_passing_possibility(): # <- a closure
                    if my_role == _target_role:
                        # do not pass to myself
                        return False
                    target_pose = WorldModel.get_pose(_target_role)
                    if target_pose is None:
                        # non-existing target
                        return False
                    if target_pose.x < 0:
                        # do not pass to this target which is in our side of the field (i.e., goalie)
                        return False

                    # wait other robots ready
                    target_vel = WorldModel.get_velocity(_target_role)
                    if np.hypot(target_vel.x, target_vel.y) > 0.1:
                        return False

                    # check pass course
                    ball_pose = WorldModel.get_pose('Ball')
                    a                = np.array([target_pose.x - ball_pose.x, target_pose.y - ball_pose.y])
                    a_length         = np.linalg.norm(a)
                    squared_a_length = np.dot(a, a)
                    # XXX needs to check if other friends is in front of the target?
                    for name, robot_id in WorldModel.enemy_assignments.iteritems():
                        if robot_id is not None:
                            enemy_pose = WorldModel.get_pose(name)
                            b = np.array([enemy_pose.x - ball_pose.x, enemy_pose.y - ball_pose.y])
                            #b_length = np.linalg.norm(b)
                            adotb = np.dot(a, b)
                            squared_bsintheta = (np.dot(b, b) - (adotb / a_length)**2)
                            ratio = adotb / squared_a_length
                            #print("  target %s, enemy %s: %.5f, %.5f"%(_target_role, name, squared_bsintheta, ratio))
                            if squared_bsintheta < 0.04 and (0.7 < ratio < 0.99):
                                # if this enemy is very close to the target, we cannot pass the ball to it
                                #print("  dropped")
                                return False
                            else:
                                #print("  next")
                                pass
                    return True
                return check_passing_possibility

            def kick_options_func_generator(target_role):
                _target_role = target_role
                def kick_options_func():
                    target_pose = WorldModel.get_pose(_target_role)
                    ball_pose = WorldModel.get_pose('Ball')
                    a                = np.array([target_pose.x - ball_pose.x, target_pose.y - ball_pose.y])
                    a_length         = np.linalg.norm(a)
                    squared_a_length = np.dot(a, a)
                    # XXX needs to check if other friends is in front of the target?
                    b_needs_chipkick = False
                    for name, robot_id in WorldModel.enemy_assignments.iteritems():
                        if robot_id is not None:
                            enemy_pose = WorldModel.get_pose(name)
                            b = np.array([enemy_pose.x - ball_pose.x, enemy_pose.y - ball_pose.y])
                            #b_length = np.linalg.norm(b)
                            adotb = np.dot(a, b)
                            squared_bsintheta = (np.dot(b, b) - (adotb / a_length)**2)
                            ratio = adotb / squared_a_length
                            if squared_bsintheta < 0.01 and (0.0 < ratio < 0.8):
                                # if this enemy is very close to the target, we cannot pass the ball to it
                                b_needs_chipkick = True
                                break

                    kick_power = 0
                    if b_needs_chipkick:
                        # TODO measure a good chip kick -> flying distance model
                        kick_power = 0.9 * a_length
                    else:
                        kick_power = 6.0
                    return kick_power, b_needs_chipkick
                return kick_options_func


            # XXX normal 'Sequence' is not enough(?)
            seq = ConditionalSequence(name='pass_to_%d'%(i), condition=check_passing_possibility_generator(target_role)) #reset_after=True
            # coord = Coordinate()
            # coord.set_approach_to_shoot(my_role, target='CONST_THEIR_GOAL')
            coord1 = Coordinate()
            coord1.set_interpose(base='Ball', target=target_role, to_dist = -0.3)
            seq.add_child(DynamicDrive('drive_to_ball_back', my_role, coord1))

            kick = ParallelOne('kick')
            coord2 = Coordinate()
            coord2.set_interpose(base='Ball', target=target_role, to_dist = 0.0)
            kick.add_child(DynamicDrive('drive_to_ball', my_role, coord2))
            kick.add_child(FlexibleKick('FlexibleKick', my_role, kick_options_func_generator(target_role)))
            kick.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
            kick.add_child(BallKicked('BallKicked'))

            seq.add_child(kick)

            p = ParallelAll('parallelall')
            class TriggerKicked(Task):
                def __init__(self, name, ref_state):
                    super(TriggerKicked, self).__init__(name)
                    self.ref_state = ref_state
                def run(self,):
                    self.ref_state.set_kicked()
                    return TaskStatus.SUCCESS
            p.add_child(TriggerKicked('trigger_kicked', ref_cornerkick_state))
            #p.add_child(OverwriteSequenceCondition('continue_after_kick', seq, (lambda: True)))
            p.add_child(SetTaskDone('set_task_done', weakref.proxy(task_done)))
            seq.add_child(p)

            select_passing_friend.add_child(seq)
        corner_kick.add_child(select_passing_friend)
        corner_kick.add_child(task_done)
        self.add_child(corner_kick)

        ### 
