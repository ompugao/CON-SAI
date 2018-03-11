# -*- coding: utf-8 -*-
from world_model import WorldModel
import tool
import constants
import rospy
import enum
import numpy as np
from plays.play_book import *

class HysterisisState(object):
    def __init__(self, min_threshold, max_threshold):
        self.min_threshold = min_threshold
        self.max_threshold = max_threshold
        self.state = True

    def get_state(self, value):
        if self.state:
            if value < self.min_threshold:
                self.state = False
        else:
            if value > self.max_threshold:
                self.state = True
        return self.state

class GameAdvantage(object):
    PROTECTIVE = -2
    DEFFENSIVE = -1
    OFFENSIVE = 1
    AGGRESSIVE = 2


# XXX should be implemented in observer?
class Admiral(object):
    def __init__(self, ):
        self.risky_enemy_velocity_threshold = 1.0 #m/s
        self.ball_velocity_threshold = 1.0
        self.ball_in_our_goal_area_state = HysterisisState(1.2, 1.3)
        self.ball_in_their_goal_area_state = HysterisisState(1.2, 1.3)
        self.situations = ['IN_PLAY', 'BALL_IN_OUR_DEFENCE', 'BALL_IN_THEIR_DEFENCE']

    def decide_situation(self, situation):
        return situation in self.situations

    def select_play(self, current_situation):
        if current_situation is 'IN_PLAY':
            return self.select_play_inplay()
        elif current_situation is 'BALL_IN_OUR_DEFENCE':
            return self.select_play_ball_in_our_defence()
        elif current_situation is 'BALL_IN_THEIR_DEFENCE':
            return self.select_play_ball_in_their_defence()
        else:
            return PlayDummy()

    def select_play_inplay(self, ):


    def evaluate(self, ):
        """
        evaluate the current game status.
        Returns:
            float: safetyness value. bigger is safer, smaller is more risky
        """

        if self.ball_is_in_our_defence_area():
            return GameAdvantage.PROTECTIVE
        elif self.ball_is_in_their_defence_area():
            return GameAdvantage.AGGRESSIVE

        # XXX: needs to tune a coefficient balance
        evaluation = 0
        norm = len([0 for name, enemy_id in WorldModel._threat_assignments.iteritems() if enemy_id is not None])
        if norm is not 0:
            for name, enemy_id in WorldModel._threat_assignments.iteritems():
                if enemy_id is None:
                    continue
                # NOTE:  use vel.x because ball might be passed to the sideway
                # NOTE2: the direction to our side should be minus
                if WorldModel.get_enemy_velocity(enemy_id).x < (-self.risky_enemy_velocity_threshold):
                    evaluation -= WorldModel.get_enemy_velocity(enemy_id).x / (self.risky_enemy_velocity_threshold * norm)

        ball_vel_x = WorldModel.get_velocity('Ball').x
        coeff = 1.0
        if ball_vel_x < -self.ball_velocity_threshold:
            # the ball is coming!
            evaluation -= coeff * ball_vel_x

        if ball_vel_x > self.ball_velocity_threshold:
            # the ball goes toward their goal!
            evaluation += coeff * ball_vel_x

        return evaluation

    def ball_is_further_from(self, constant_pose_key, hysterisis_state):
        if constant_pose_key not in constants.poses.keys():
            return None

        ball_pose = WorldModel.get_pose('Ball')
        length_from_our_goal = tool.getLength(constants.poses[constant_pose_key], ball_pose)
        return hysterisis_state.get_state(length_from_our_goal)

    def ball_is_in_our_defence_area(self, ):
        return (not self.ball_is_further_from('CONST_OUR_GOAL', self.ball_in_our_goal_area_state))

    def ball_is_in_their_defence_area(self, ):
        return (not self.ball_is_further_from('CONST_THEIR_GOAL', self.ball_in_their_goal_area_state))

    def get_ball_holder(self, dist_threshold=0.5):
        ball_pose = WorldModel.get_pose('Ball')
        min_dist = np.finfo(np.float32).max
        min_robot_id = None

        def search_closest_robot(name, robot_id, min_dist, min_robot_id):
            if robot_id is not None:
                pose = WorldModel.get_enemy_pose(robot_id)
                dist = tool.getLength(pose, ball_pose)
                if dist < dist_threshold and dist < min_dist:
                    min_dist = dist
                    min_robot_id = robot_id
            return min_dist, min_robot_id

        for name, robot_id in WorldModel.assignments:
            min_dist, min_robot_id = search_closest_robot(name, robot_id, min_dist, min_robot_id)

        for name, robot_id in WorldModel.enemy_assignments:
            min_dist, min_robot_id = search_closest_robot(name, robot_id, min_dist, min_robot_id)
        
        if min_robot_id is not None:
            if min_robot_id in WorldModel.assignments.values():
                return constants.Teams.FRIEND
            else:
                return constants.Teams.ENEMY
        else:
            return constants.Teams.NOBODY
