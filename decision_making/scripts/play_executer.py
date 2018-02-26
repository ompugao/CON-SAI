#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.task_setup import *

from world_model import WorldModel
from plays.play_book import PlayBook
from plays.test_book import TestBook
from plays.play_dummy import PlayDummy
from game_evaluator import Admiral
import numpy as np


class PlayExecuter(object):

    def __init__(self):
        self._play_termination = True
        self._play = PlayDummy()
        self._play_past_time = 0.0
        self._game_evaluator = Admiral()

    def update(self):
        WorldModel.update_world()

        self._select_play()

        WorldModel.update_assignments(self._play.assignment_type)

        self._execute_play()

        self._evaluate_play()

    
    def _select_play(self):
        if self._play_termination:
            self._play.reset()

            # Extract possible plays from playbook
            self.possible_plays = []
            for play in PlayBook.book:
                if WorldModel.situations[play.applicable]:
                    self.possible_plays.append(play)

            # playとtestは混ざらないようにWorldModelで調整している
            for test in TestBook.book:
                if WorldModel.situations[test.applicable]:
                    self.possible_plays.append(test)

            self._play_past_time = rospy.get_time()
            self._play_termination = False

            rospy.logdebug('play reset')

        if len(self.possible_plays) > 0:
            # XXX: needs hysteresis?
            aggressiveness = self._game_evaluator.evaluate()
            rospy.logdebug("agg: %f"%(aggressiveness,))
            mindiff = np.finfo(np.float32).max
            chosen_play = self._play
            for play in self.possible_plays:
                diff = np.abs(play.aggressiveness - aggressiveness)
                if diff < mindiff:
                    # choose the play whose aggressive ness is
                    # closest to the current game situation
                    chosen_play = play

            if chosen_play is not self._play:
                self._play.reset()
                self._play = chosen_play
        else:
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
