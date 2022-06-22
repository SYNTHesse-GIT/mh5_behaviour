from tracemalloc import start
import rospy
import numpy as np
import PyKDL as kdl
from .walking_base import WalkingBase, JS
from .kinematics import LinearPoser


class StaticWalking(WalkingBase):

    def __init__(self):
        WalkingBase.__init__(self, name_space='static_walking')

    def readParams(self, name_space):
        WalkingBase.readParams(self, name_space)
        # static walking parameters
        params = self.params.get('params', {})
        self.legs0 = params.get('legs0', 0.95)
        self.arms0 = params.get('arms0', -0.75)
        self.swing_A = params.get('swing_A', 0.032)
        self.step_L = params.get('step_L', 0.050)
        self.step_H = params.get( 'step_H', 0.040)
        self.speed = params.get('speed', 1.0)
        self.frecv = params.get('frecv', 100)
        self.prep_t = params.get('prep_t', 2.0)
        self.swing_t = params.get('swing_t', 0.5)
        self.step_t = params.get('step_t', 1.0)
        self.arms_th = params.get('arm_th', 0.75)

    def startPose(self):
        # wait 2 s for the joint positions to be populated
        rospy.sleep(2)
        rospy.loginfo('Static walking: init pose starting...')
        steps = self.frecv * self.prep_t  # number of commands to send
        # show start pose
        # for chain in ['LL', 'RL']:
        #     pos, rpy = self.K.EEPosRPY(chain)
        #     rospy.loginfo(f'{chain}-Q[]: {self.K.getJointValues(chain)}')
        #     rospy.loginfo(f'{chain}-Pos: {pos}')
        #     rospy.loginfo(f'{chain}-RPY: {rpy}')
        # start pose
        ll_sp = self.K.FK('LL')
        rl_sp = self.K.FK('RL')
        # rospy.loginfo(f'Beg Frame: Pos={ll_sp.p}, RPY={ll_sp.M.GetRPY()}')
        # end pose
        ll_ep = kdl.Frame(V=kdl.Vector(0.007, 0.0325, 0.03), R=kdl.Rotation.RPY(0,0,0))
        # ll_ep.p[2] += 0.05
        rl_ep = kdl.Frame(V=kdl.Vector(0.007, -0.0325, 0.03), R=kdl.Rotation.RPY(0,0,0))
        # rospy.loginfo(f'End Frame: Pos={ll_ep.p}, RPY={ll_ep.M.GetRPY()}')
        # rl_ep.p[2] += 0.05
        ll_iter = LinearPoser(ll_sp, ll_ep, int(steps))
        rl_iter = LinearPoser(rl_sp, rl_ep, int(steps))
        ll_q = self.K.getJointKDLArray('LL')
        rl_q = self.K.getJointKDLArray('RL')
        for (pose_ll, pose_rl) in zip(ll_iter, rl_iter):
            new_ll_q = self.K.IK('LL', pose_ll, ll_q)
            for i, jn in enumerate(self.K.getJointNames('LL')):
                pos = new_ll_q[i]
                # vel = abs(new_ll_q[i] - ll_q[i]) * self.frecv
                vel = 1 / self.frecv
                acc = vel / 4
                self.joint_commands[jn] = JS(pos, vel, acc)
            ll_q = new_ll_q
            new_rl_q = self.K.IK('RL', pose_rl, rl_q)
            for i, jn in enumerate(self.K.getJointNames('RL')):
                pos = new_rl_q[i]
                # vel = abs(new_rl_q[i] - rl_q[i]) * self.frecv
                vel = 1 / self.frecv
                acc = vel / 4
                self.joint_commands[jn] = JS(pos, vel, acc)
            rl_q = new_rl_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        # ep.update(
        #     l_sho_p=self.arms0, r_sho_p=self.arms0,
        #     l_hip_p=self.legs0, l_kne_p=self.legs0 * 2, l_ank_p=self.legs0,
        #     r_hip_p=self.legs0, r_kne_p=self.legs0 * 2, r_ank_p=self.legs0)
        # for step in range(int(steps)):
        #     for j in ep.keys():
        #         position = sp[j] + (ep[j] - sp[j]) * (step + 1) / steps
        #         velocity = (ep[j] - sp[j]) / steps
        #         acceleration = velocity / 4
        #         self.joint_commands[j] = JS(position, velocity, acceleration)
        #     self.publishCommands()
        #     rospy.sleep(1/self.frecv)

        rospy.loginfo('Static walking: start pose complete')

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
            for j in ep.keys():
                position = sp[j] + (ep[j] - sp[j]) * (step + 1) / steps
                velocity = (ep[j] - sp[j]) / steps
                acceleration = velocity / 4
                self.joint_commands[j] = JS(position, velocity, acceleration)
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
