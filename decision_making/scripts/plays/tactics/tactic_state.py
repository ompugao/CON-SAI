# -*- coding: utf-8 -*-

class CornerKickState(object):
    def __init__(self, ):
        self.kicked = False

    def set_kicked(self, state = True):
        self.kicked = state

    def __call__(self,):
        return self.kicked

