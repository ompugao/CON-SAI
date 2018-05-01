# -*- coding: utf-8 -*-
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

class ConditionalSequence(Task):
    def __init__(self, name, condition, *args, **kwargs):
        """
        condition :param callable: callable statement which decides if it should process its sequence
        NOTE:
        this task does not follow the convension of behaviour tree.
        Be careful about a condition of this task and the state of the behaviour tree which this task belongs to.
        """
        super(ConditionalSequence, self).__init__(name, *args, **kwargs)
        self._default_condition = condition
        self.set_condition(condition)
 
    def set_condition(self, condition):
        self.condition = condition

    def run(self):
        if self._announce:
            self.announce()

        if not self.condition():
            ##print("%s condition is bad"%(self.name,))
            if self.reset_after:
                self.reset()
            return TaskStatus.FAILURE
        #print("%s condition is ok"%(self.name,))
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

    def reset(self):
        super(ConditionalSequence, self).reset()
        self.condition = self._default_condition

class OverwriteSequenceCondition(Task):
    def __init__(self, name, conditional_seq, condition):
        super(OverwriteSequenceCondition, self).__init__(name)
        self.conditional_seq = conditional_seq
        self.condition = condition

    def run(self):
        self.conditional_seq.set_condition(self.condition)
        return TaskStatus.SUCCESS

class ParallelOneIgnoringFailure(Task):
    """
    TODO: write me
    """
    def __init__(self, name, *args, **kwargs):
        super(ParallelOneIgnoringFailure, self).__init__(name, *args, **kwargs)

        self.num_failure = 0

    def run(self):
        n_children = len(self.children)

        for c in self.children:
            # if c.status == TaskStatus.FAILURE:
            #     continue

            c.status = c.run()

            if c.status == TaskStatus.SUCCESS:
                if self.reset_after:
                    self.reset()
                return TaskStatus.SUCCESS

            if c.status == TaskStatus.FAILURE:
                self.num_failure += 1

        if self.num_failure == n_children:
            if self.reset_after:
                self.reset()
            return TaskStatus.FAILURE
        else:
            return TaskStatus.RUNNING

    def reset(self):
        super(ParallelOneIgnoringFailure, self).reset()
        self.num_failure = 0

class TaskDone(Task):
    def __init__(self, name):
        super(TaskDone, self).__init__(name)
        self._b_done = False

    def set_done(self, ):
        self._b_done = True

    def run(self):
        if self._b_done:
            return TaskStatus.SUCCESS
        return TaskStatus.RUNNING

class SetTaskDone(Task):
    def __init__(self, name, task_done):
        super(SetTaskDone, self).__init__(name)
        self.task_done = task_done
    def run(self):
        self.task_done.set_done()
        return TaskStatus.SUCCESS

class Print(Task):
    def __init__(self, name):
        super(Print, self).__init__(name)

    def run(self):
        rospy.loginfo('%s is running!'%(self.name))
        return TaskStatus.RUNNING
