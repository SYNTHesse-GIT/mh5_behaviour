import rospy
from .walking_base import WalkingBase
from .kinematics import Kinematics


class StaticWalking(WalkingBase):

    def __init__(self):
        WalkingBase.__init__(self, name_space='static_walking')
        self.K = Kinematics()
        self.K.reset()

    def readParams(self, name_space):
        WalkingBase.readParams(self, name_space)
        # static walking parameters
        self.legs0 = self.params.get(f'{name_space}/params/legs0', 0.95)
        self.arms0 = self.params.get(f'{name_space}/params/arms0', -0.75)
        self.swing_A = self.params.get(f'{name_space}/params/swing_A', 0.032)
        self.step_L = self.params.get(f'{name_space}/params/step_L', 0.050)
        self.step_H = self.params.get(f'{name_space}/params/step_H', 0.040)
        self.speed = self.params.get(f'{name_space}/params/speed', 1.0)
        self.frecv = self.params.get(f'{name_space}/params/frecv', 100)
        self.prep_t = self.params.get(f'{name_space}/params/prep_t', 2.0)
        self.swing_t = self.params.get(f'{name_space}/params/swing_t', 0.5)
        self.step_t = self.params.get(f'{name_space}/params/step_t', 1.0)
        self.arms_th = self.params.get(f'{name_space}/params/arm_th', 0.75)

    def startPose(self):
        # wait 2 s for the joint positions to be populated
        rospy.sleep(2)
        rospy.loginfo('Static walking: init pose starting...')
        steps = self.frecv * self.prep_t  # number of commands to send
        # start pose
        sp = {k: self.joint_states[k].p for k in self.joint_states.keys()}
        # end pose
        ep = {k: 0.0 for k in self.joint_states.keys()}
        ep.update(
            l_sho_p=self.arms0, r_sho_p=self.arms0,
            l_hip_p=self.legs0, l_kne_p=self.legs0 * 2, l_ank_p=self.legs0,
            r_hip_p=self.legs0, r_kne_p=self.legs0 * 2, r_ank_p=self.legs0)
        for step in range(int(steps)):
            for joint in ep.keys():
                self.joint_commands[joint] = sp[joint] + \
                        (ep[joint] - sp[joint]) * (step + 1) / steps
            self.publishCommands()
            rospy.sleep(1/self.frecv)

    def stopPose(self):
        rospy.loginfo('Static walking: stop pose starting...')
        steps = self.frecv * self.prep_t  # number of commands to send
        # start pose
        sp = {k: self.joint_states[k].p for k in self.joint_states.keys()}
        # end pose
        ep = {k: 0.0 for k in self.joint_states.keys()}
        ep.update(
            l_sho_p=self.arms0, r_sho_p=self.arms0,
            l_hip_p=1.69, l_kne_p=3.57, l_ank_p=1.92,
            r_hip_p=1.69, r_kne_p=3.57, r_ank_p=1.92)
        for step in range(int(steps)):
            for joint in ep.keys():
                self.joint_commands[joint] = sp[joint] + \
                        (ep[joint] - sp[joint]) * (step + 1) / steps
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('Static walking: stop pose complete')

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
