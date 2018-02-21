# -*- coding: utf-8 -*-
from world_model import WorldModel

class GameEvaluator(object):
    def __init__(self, ):
        self.risky_enemy_velocity_threshold = 1.0 #m/s
        self.ball_velocity_threshold = 1.0

    def evaluate(self, ):
        """
        evaluate the current game status.
        Returns:
            float: safetyness value in the range of [-1, 1]. bigger is safer, smaller is more risky
        """

        # XXX: needs to tune a coefficient balance
        evaluation = 0
        for name, enemy_id in WorldModel._threat_assignments:
            # ball might be passed to the sideway
            if WorldModel.get_enemy_velocity(enemy_id).x < - self.risky_enemy_velocity_threshold: # the direction to our side should be minus
                evaluation -= WorldModel.get_enemy_velocity(enemy_id).x / self.risky_enemy_velocity_threshold

        ball_vel_x = WorldMode.get_velocity('Ball').x
        if ball_vel_x < -self.ball_velocity_threshold:
            # the ball is coming!
            evaluation = WorldModel.get_enemy_velocity(enemy_id).x / self.ball_vel_x

        if ball_vel_x > self.ball_velocity_threshold:
            # the ball goes toward their goal!
            evaluation += WorldModel.get_enemy_velocity(enemy_id).x / self.ball_vel_x

        return evaluation
