# -*- coding: utf-8 -*-

class KickState(object):
    def __init__(self, ):
        self.reset()

    def set_kicked(self, state = True):
        self.kicked = state

    def reset(self,):
        self.kicked = False

    def __call__(self,):
        return self.kicked

class CornerKickState(object):
    def __init__(self, ):
        self.reset()

    def set_kicked(self, state = True):
        self.kicked = state

    def reset(self,):
        self.kicked = False

    def __call__(self,):
        return self.kicked


class InplayMode(object):
    def __init__(self, ):
        self.mode = True

    def toggle(self,):
        self.mode = not self.mode

    def __call__(self,):
        return self.mode
