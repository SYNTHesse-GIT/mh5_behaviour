import rospy
from .walking_base import WalkingBase
from .kinematics import Kinematics


class StaticWalking(WalkingBase):

    def __init__(self):
        WalkingBase.__init__(self, 'static_walking')
        self.K = Kinematics()
        self.K.reset()

    def readParams(self, name_space):
        WalkingBase.readParams(self, name_space)
        # static walking parameters
        self.legs0   = self.params.get('legs0', 0.95)     # initial pose legs
        self.arms0   = self.params.get('arms0', -0.75)    # initial pose arms
        self.swing_A = self.params.get('swing_A', 0.032)  # swing amplitute at torso
        self.step_L  = self.params.get('step_L', 0.050)   # one step length
        self.step_H  = self.params.get('step_H', 0.040)   # swing height for foot
        self.speed   = self.params.get('speed', 1.0)      # execution speed
        self.frecv   = self.params.get('frecv', 100)      # updates / second
        self.prep_t  = self.params.get('prep_t', 1.0)     # preparation duration [s]
        self.swing_t = self.params.get('swing_t', 0.5)    # lateral swing time [s]
        self.step_t  = self.params.get('step_t', 1.0)     # step time [s]
        self.arms_th = self.params.get('arm_th', 0.75)    # arm swing [-, + rad]

    def handleCommandCallback(self, msg):
        if msg.data == 'start':
            rospy.loginfo('starting walk')
            return
        if msg.data == 'stop':
            rospy.loginfo('stopping walk')
            return
        if msg.data == 'left':
            rospy.loginfo('walk left')
            return
        if msg.data == 'right':
            rospy.loginfo('walk right')
            return
        if msg.data == 'faster':
            rospy.loginfo('walking faster')
            return
        if msg.data == 'slower':
            rospy.loginfo('walking slower')
            return
        if msg.data.startswith('walk'):
            ns = int(msg.data.split()[1])
            rospy.loginfo(f'walking {ns} steps')
            return