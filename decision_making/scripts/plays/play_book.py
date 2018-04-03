
from play_halt import PlayHalt
from play_outside import PlayOutside
from play_stop import PlayStop
from play_our_pre_kickoff import PlayOurPreKickoff
from play_our_kickoff_start import PlayOurKickoffStart
from play_our_pre_penalty import PlayOurPrePenalty
from play_our_penalty_start import PlayOurPenaltyStart
from play_force_start import PlayForceStart
from play_inplay import PlayInPlay
from play_indirect import PlayIndirect
from play_direct import PlayDirect
from play_their_pre_kickoff import PlayTheirPreKickoff
from play_their_kickoff_start import PlayTheirKickoffStart
from play_their_indirect import PlayTheirIndirect
from play_their_direct import PlayTheirDirect
from play_their_pre_penalty import PlayTheirPrePenalty
from play_their_penalty_start import PlayTheirPenaltyStart
from play_inplay_our_defence import PlayInPlayOurDefence
from play_inplay_their_defence import PlayInPlayTheirDefence
from play_super_protective import PlaySuperProtective
from play_our_corner_kick import PlayOurCornerKick
from play_dummy import PlayDummy

class PlayBook(object):
    def __init__(self, ):
        self.books = dict()

    def get_plays(self, situation):
        if not self.books.has_key(situation):
            return []
        else:
            return self.books[situation]
    def register(self, play):
        if not self.books.has_key(play.applicable):
            self.books[play.applicable] = []
        self.books[play.applicable].append(play)
