
from role_base import Role

class Play(object):
    def __init__(self, name):
        self.name = name
        self.applicable = None
        self.done = None
        self.done_aborted = None
        self.recent_done = None
        self.recent_done_aborted = None
        self.timeout = None
        self.assignment_type = None
        self.aggressiveness = 0.0

        self.roles = [
                Role("Role_0"), Role("Role_1"), Role("Role_2"),
                Role("Role_3"), Role("Role_4"), Role("Role_5")]
        
    def reset(self,):
        # 実行中のRoleをリセット
        for role in self._play.roles:
            role.behavior.reset()

