
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from skills.dynamic_drive import DynamicDrive
from skills.observations import BallKicked
from skills.adjustments import WithKick, NoBallAvoidance, FlexibleKick
from skills.observations import CheckPassPossibility

sys.path.append(os.pardir)
from coordinate import Coordinate
from world_model import WorldModel

import numpy as np

class ConditionalSequence(Task):
    def __init__(self, name, condition, *args, **kwargs):
        """
        condition :param callable: callable statement which decides if it should process its sequence
        """
        super(ConditionalSequence, self).__init__(name, *args, **kwargs)
        self.condition = condition
 
    def run(self):
        if self._announce:
            self.announce()

        if not self.condition():
            #print("%s bad"%(self.name,))
            if self.reset_after:
                self.reset()
            return TaskStatus.FAILURE
        #print("%s ok"%(self.name,))
        for c in self.children:

            if c.status == TaskStatus.SUCCESS:
                continue
            c.status = c.run()
            #print('run ' + c.name + ' ' + TaskStatus.to_s(c.status))

            if c.status != TaskStatus.SUCCESS:
                if c.status == TaskStatus.FAILURE:
                    if self.reset_after:
                        self.reset()
                        return TaskStatus.FAILURE
                return c.status

        if self.reset_after:
            self.reset()

        return TaskStatus.SUCCESS

class TacticCornerKick(Sequence):
    def __init__(self, name, my_role):
        super(TacticCornerKick, self).__init__(name)


        select_passing_friend = Selector('select_passing_friend')
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

                    #TODO wait other robots ready
                    target_vel = WorldModel.get_velocity(_target_role)
                    if np.hypot(target_vel.x, target_vel.y) > 0.1:
                        return False
                    #TODO check pass course
                    return True
                return check_passing_possibility

            def check_chipkick_necessity_generator():
                def check_chipkick_necessity():
                    return True
                return check_chipkick_necessity

            def kick_power_generator():
                def kick_power():
                    return 4.0
                return kick_power


            seq = ConditionalSequence(name='pass_to_%d'%(i), reset_after=True, condition=check_passing_possibility_generator(target_role))
            # coord = Coordinate()
            # coord.set_approach_to_shoot(my_role, target='CONST_THEIR_GOAL')
            coord1 = Coordinate()
            coord1.set_interpose(base='Ball', target=target_role, to_dist = -0.3)
            seq.add_child(DynamicDrive('drive_to_ball_back', my_role, coord1))

            kick = ParallelOne('kick')
            coord2 = Coordinate()
            coord2.set_interpose(base='Ball', target=target_role, to_dist = 0.0)
            kick.add_child(DynamicDrive('drive_to_ball', my_role, coord2))
            kick.add_child(FlexibleKick('FlexibleKick', my_role, check_chipkick_necessity_generator(), kick_power_generator()))
            kick.add_child(NoBallAvoidance('NoBallAvoidance', my_role))
            kick.add_child(BallKicked('BallKicked'))

            seq.add_child(kick)

            select_passing_friend.add_child(seq)
        self.add_child(select_passing_friend)

        ### 
