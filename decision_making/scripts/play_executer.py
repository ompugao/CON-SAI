#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from world_model import WorldModel
from plays.play_book import *
from plays.test_book import *
from plays.play_dummy import PlayDummy
from game_evaluator import Admiral
import numpy as np

from field_analysis import FieldAnalysis

class PlayExecuter(object):

    def __init__(self):
        self._play_termination = True
        self._play = PlayDummy()
        self._play_past_time = 0.0
        self._admiral = Admiral()
        self.testbook = dict()
        test0 = Test0()
        test1 = Test1()
        self.testbook[test0.applicable] = test0
        self.testbook[test1.applicable] = test1
        self.playbook = PlayBook()
        self.playbook.register(PlayHalt())
        self.playbook.register(PlayOutside())
        self.playbook.register(PlayStop())
        self.playbook.register(PlayOurPreKickoff())
        self.playbook.register(PlayOurKickoffStart())
        self.playbook.register(PlayOurPrePenalty())
        self.playbook.register(PlayOurPenaltyStart())
        self.playbook.register(PlayForceStart())
        self.playbook.register(PlayInPlay())
        self.playbook.register(PlayIndirect())
        self.playbook.register(PlayDirect())
        self.playbook.register(PlayTheirPreKickoff())
        self.playbook.register(PlayTheirKickoffStart())
        self.playbook.register(PlayTheirIndirect())
        self.playbook.register(PlayTheirDirect())
        self.playbook.register(PlayTheirPrePenalty())
        self.playbook.register(PlayTheirPenaltyStart())
        self.playbook.register(PlayInPlayOurDefence())
        self.playbook.register(PlayInPlayTheirDefence())

    def update(self):
        WorldModel.update_world()

        self._select_play()

        WorldModel.update_assignments(self._play.assignment_type)

        self._execute_play()

        self._evaluate_play()

        FieldAnalysis.update_field_analysis()


    
    def _select_play(self):
        if self._play_termination:
            self._play.reset()

            self._play_past_time = rospy.get_time()
            self._play_termination = False

            rospy.logdebug('play reset')

        # select test play
        testplay = self.testbook.get(WorldModel.get_current_situation())
        if testplay is not None:
            self._play = testplay
            return

        # select play based on situation
        current_situation = WorldModel.get_current_situation()
        if not self._admiral.decide_situation(current_situation):
            self._admiral.reset()
            plays = self.playbook.get_plays(current_situation)
            if len(plays) > 0:
                self._play = plays[0].__class__()
            else:
                rospy.logwarn("No Play is registered to the situation, named %s", current_situation)
                self._play = PlayDummy()
        else:
            self._play = self._admiral.select_play(self._play, current_situation)

        # just in case
        if self._play is None:
            rospy.logwarn("play is None somehow! sets dummy!")
            self._play = PlayDummy()


    def _execute_play(self):
        for role in self._play.roles:
            status = role.behavior.run()
            role.behavior.set_status(status)

        text = "execute : " + self._play.name
        rospy.logdebug(text)


    def _evaluate_play(self):
        for role in self._play.roles:
            status = role.behavior.get_status()

            #rospy.logdebug("role %s status: %s"%(role.my_role, TaskStatus.to_s(status)))
            if role.loop_enable:
                if status == TaskStatus.SUCCESS or status == TaskStatus.FAILURE:
                    role.behavior.reset()
            else:
                if status == TaskStatus.SUCCESS or status == TaskStatus.FAILURE:
                    self._play_termination = True

        if self._play.timeout:
            if rospy.get_time() - self._play_past_time > self._play.timeout:
                self._play_termination = True

        # TODO(Asit) write recent_done termination 
        if (self._play.done and WorldModel.situations[self._play.done]) or \
                (self._play.done_aborted and 
                        not WorldModel.situations[self._play.done_aborted]):
            self._play_termination = True
