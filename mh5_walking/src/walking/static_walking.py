from .walking_base import WalkingBase

class StaticWalking(WalkingBase):

    def __init__(self):
        WalkingBase.__init__(self, name_space='static_walking')