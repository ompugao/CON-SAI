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

# class GameAdvantage(object):
#     PROTECTIVE = -2
#     DEFFENSIVE = -1
#     OFFENSIVE = 1
#     AGGRESSIVE = 2


# XXX should be implemented in observer?
class Admiral(object):
    def __init__(self, ):
        self.risky_enemy_velocity_threshold = 1.0 #m/s
        self.ball_velocity_threshold = 1.0
        self.ball_in_our_goal_area_state = HysterisisState(1.2, 1.3)
        self.ball_in_their_goal_area_state = HysterisisState(1.2, 1.3)
        self.situations = ['IN_PLAY', 'BALL_IN_OUR_DEFENCE', 'BALL_IN_THEIR_DEFENCE', 'OUR_DIRECT', 'OUR_INDIRECT']
        self.reset()


    def decide_situation(self, situation):
        return situation in self.situations

    def reset(self, ):
        self.last_situation = ""
        self.finish_keeping_play = (lambda : True)
        self.current_play = PlayDummy()
        import gc
        gc.collect()

    def select_play(self, current_situation):
        if self.current_play.name == "PlayDummy" or \
                (callable(self.finish_keeping_play) and self.finish_keeping_play()):
            self.reset()
            if current_situation is 'IN_PLAY':
                self.current_play, self.finish_keeping_play = self.select_play_inplay()
            elif current_situation is 'BALL_IN_OUR_DEFENCE':
                self.current_play, self.finish_keeping_play = self.select_play_ball_in_our_defence()
            elif current_situation is 'BALL_IN_THEIR_DEFENCE':
                self.current_play, self.finish_keeping_play = self.select_play_ball_in_their_defence()
            elif current_situation is 'OUR_INDIRECT':
                self.current_play, self.finish_keeping_play = self.select_play_our_indirect()
            elif current_situation is 'OUR_DIRECT':
                self.current_play, self.finish_keeping_play = self.select_play_our_direct()

        return self.current_play

    def select_play_inplay(self, ):
        ball_holder = self.get_ball_holder(dist_threshold=0.2)
        rospy.logdebug("current ball holder: %s"%(ball_holder,))
        if ball_holder is constants.Teams.ENEMY:
            play_super_protective = PlaySuperProtective()
            return play_super_protective, (lambda: True)
        return PlayInPlay(), (lambda: True)

    def select_play_ball_in_our_defence(self, ):
        return PlayInPlayOurDefence(), (lambda: True)

    def select_play_ball_in_their_defence(self, ):
        return PlayInPlayTheirDefence(), (lambda: True)

    def select_play_our_indirect(self, ):
        ball_pose = WorldModel.get_pose('Ball')
        if ball_pose.x > (0.7*constants.FieldHalfX) and np.abs(ball_pose.y) > (0.45*constants.FieldHalfY):
            def finish_corner_kick_play():
                p = WorldModel.get_pose('Ball')
                return p.x < (0.4 * constants.FieldHalfX)
            # XXX: add some extra modes?
            mode = np.sign(ball_pose.y)
            return PlayOurCornerKick(mode), finish_corner_kick_play
        return PlayIndirect(), (lambda: True)

    def select_play_our_direct(self, ):
        return PlayDirect(), (lambda: True)

    '''
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
    '''

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
        min_robot_name = None

        def search_closest_robot(name, robot_id, min_dist, min_robot_name):
            if robot_id is not None:
                pose = WorldModel.get_pose(name)
                dist = tool.getLength(pose, ball_pose)
                if dist < dist_threshold and dist < min_dist:
                    min_dist = dist
                    min_robot_name = name
            return min_dist, min_robot_name

        # rospy.loginfo("friend assignments %s"%(WorldModel.assignments.items()))
        # rospy.loginfo("enemy assignments %s"%(WorldModel.enemy_assignments.items()))
        for name, robot_id in WorldModel.assignments.iteritems():
            min_dist, min_robot_name = search_closest_robot(name, robot_id, min_dist, min_robot_name)

        for name, robot_id in WorldModel.enemy_assignments.iteritems():
            min_dist, min_robot_name = search_closest_robot(name, robot_id, min_dist, min_robot_name)
        
        if min_robot_name is not None:
            if min_robot_name in WorldModel.assignments.keys():
                return constants.Teams.FRIEND
            else:
                return constants.Teams.ENEMY
        else:
            return constants.Teams.NOBODY
