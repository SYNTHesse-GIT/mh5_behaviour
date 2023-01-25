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
        self.step_H = params.get( 'step_H', 0.020)
        self.speed = params.get('speed', 1.0)
        self.frecv = params.get('frecv', 100)
        self.vel = params.get('vel', 1 / self.frecv)    # time control; velocity profile
        # self.acc = params.get('acc', self.vel/4)        # time control; acceleration profile
        self.acc = 0
        self.prep_t = params.get('prep_t', 2.0)
        self.swing_t = params.get('swing_t', 0.5)
        self.step_t = params.get('step_t', 1.0)
        self.arms_th = params.get('arm_th', 0.75)



    # def jntArrayDiff(self, arrA, arrB, factor=1.0):
    #     return [abs(a-b)*factor for (a, b) in zip(arrA, arrB)]

    def startPose(self):
        # wait 2 s for the joint positions to be populated
        rospy.sleep(2)
        rospy.loginfo('Static walking: init pose starting...')
        steps = int(self.frecv * self.prep_t)  # number of commands to send
        curr_l_q = self.K.getJointKDLArray('LL')
        curr_r_q = self.K.getJointKDLArray('RL')
        l_poser = LinearPoser(
            start_pose=self.K.FK('LL'),
            end_pose=kdl.Frame(kdl.Vector(0.0, 0.0315, 0.013)),
            steps=steps)
        r_poser = LinearPoser(
            start_pose=self.K.FK('RL'),
            end_pose=kdl.Frame(kdl.Vector(0.0, -0.0315, 0.013)),
            steps=steps)
        for (l_pose, r_pose) in zip(l_poser, r_poser):
            new_l_q = self.K.IK('LL', l_pose, curr_l_q)
            l_vel = self.jntArrayDiff(new_l_q, curr_l_q, self.frecv*0.9)
            self.setJointCommands('LL', new_l_q, l_vel)
            curr_l_q = new_l_q
            new_r_q = self.K.IK('RL', r_pose, curr_r_q)
            r_vel = self.jntArrayDiff(new_r_q, curr_r_q, self.frecv*0.9)
            self.setJointCommands('RL', new_r_q, r_vel)
            curr_r_q = new_r_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('Static walking: start pose complete')
        rospy.loginfo(f'LL Start Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Start Frame: \n{self.K.FK("RL")}')

    def stopPose(self):
        rospy.loginfo('Static walking: stop pose starting...')
        steps = int(self.frecv * self.prep_t)  # number of commands to send
        curr_l_q = self.K.getJointKDLArray('LL')
        curr_r_q = self.K.getJointKDLArray('RL')
        l_poser = LinearPoser(
            start_pose=self.K.FK('LL'),
            end_pose=kdl.Frame(kdl.Vector(0.007, 0.032, 0.0915)),
            steps=steps)
        r_poser = LinearPoser(
            start_pose=self.K.FK('RL'),
            end_pose=kdl.Frame(kdl.Vector(0.007, -0.032, 0.0915)),
            steps=steps)
        for (l_pose, r_pose) in zip(l_poser, r_poser):
            new_l_q = self.K.IK('LL', l_pose, curr_l_q)
            l_vel = self.jntArrayDiff(new_l_q, curr_l_q, self.frecv*0.9)
            self.setJointCommands('LL', new_l_q, l_vel)
            curr_l_q = new_l_q
            new_r_q = self.K.IK('RL', r_pose, curr_r_q)
            r_vel = self.jntArrayDiff(new_r_q, curr_r_q, self.frecv*0.9)
            self.setJointCommands('RL', new_r_q, r_vel)
            curr_r_q = new_r_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('Static walking: stop pose complete')
        rospy.loginfo(f'LL Start Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Start Frame: \n{self.K.FK("RL")}')
        # wait 2 more secs to finish all moves; 
        # after this the servos will be deactivated
        rospy.sleep(2)

    def start_walk(self, with_left=True):
        if with_left:
            supp, lift, fact = 'RL', 'LL', -1
        else:
            supp, lift, fact = 'LL', 'RL', 1

        # start positions
        supp_F = self.K.FK(supp)
        supp_0 = kdl.Frame(supp_F)  # make a copy
        supp_q = self.K.getJointKDLArray(supp)
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy
        lift_q = self.K.getJointKDLArray(lift)

        # half swing
        steps = int(self.swing_t * self.frecv * self.speed / 2.0)
        for s in np.linspace(0, self.swing_A*fact, steps):
            supp_F.p[1] = supp_0.p[1] - s
            new_supp_q = self.K.IK(supp, supp_F, supp_q)
            # self.K.setJointValues(supp, new_q)
            for i, jn in enumerate(self.K.getJointNames(supp)):
                self.joint_commands[jn] = JS(new_supp_q[i], self.vel, self.acc)
            supp_q = new_supp_q
            lift_F.p[1] = lift_0.p[1] - s
            new_lift_q = self.K.IK(lift, lift_F, lift_q)
            # self.K.setJointValues(lift, new_q)
            for i, jn in enumerate(self.K.getJointNames(lift)):
                self.joint_commands[jn] = JS(new_lift_q[i], self.vel, self.acc)
            lift_q = new_lift_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('start_walk')
        rospy.loginfo('After swing:')
        rospy.loginfo(f'LL Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Frame: \n{self.K.FK("RL")}')

        # half step
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy
        lift_q = self.K.getJointKDLArray(lift)
        steps = int(self.step_t * self.frecv * self.speed)
        dt = np.linspace(0, 2 * np.pi, steps)
        xs = dt * self.step_L / (2 * np.pi)
        zs = (1 - np.cos(dt))/2 * self.step_H
        for (x, z) in zip(xs, zs):
            # desired position
            lift_F.p[0] = lift_0.p[0] + x
            lift_F.p[2] = lift_0.p[2] + z
            new_lift_q = self.K.IK(lift, lift_F, lift_q)
            # self.K.setJointValues(lift, new_q)
            for i, jn in enumerate(self.K.getJointNames(lift)):
                self.joint_commands[jn] = JS(new_lift_q[i], self.vel, self.acc)
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('After step:')
        rospy.loginfo(f'LL Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Frame: \n{self.K.FK("RL")}')

    def propulsion(self, to_left=True):
        if to_left:
            supp, targ, fact = 'RL', 'LL', 1
        else:
            supp, targ, fact = 'LL', 'RL', -1

        # start positions
        supp_F = self.K.FK(supp)
        supp_0 = kdl.Frame(supp_F)  # make a copy
        supp_q = self.K.getJointKDLArray(supp)
        targ_F = self.K.FK(targ)
        targ_0 = kdl.Frame(targ_F)  # make a copy
        targ_q = self.K.getJointKDLArray(targ)

        # swing
        steps = int(self.swing_t * self.frecv * self.speed)
        dy = np.linspace(0, 2 * self.swing_A * fact, steps)
        dx = np.linspace(0, self.step_L, steps)
        for x, y in zip(dx, dy):
            supp_F.p[0] = supp_0.p[0] - x
            supp_F.p[1] = supp_0.p[1] - y
            new_supp_q = self.K.IK(supp, supp_F, supp_q)
            # self.K.setJointValues(supp, new_q)
            for i, jn in enumerate(self.K.getJointNames(supp)):
                self.joint_commands[jn] = JS(new_supp_q[i], self.vel, self.acc)
            supp_q = new_supp_q
            targ_F.p[0] = targ_0.p[0] - x
            targ_F.p[1] = targ_0.p[1] - y
            new_targ_q = self.K.IK(targ, targ_F, targ_q)
            # self.K.setJointValues(targ, new_q)
            for i, jn in enumerate(self.K.getJointNames(targ)):
                self.joint_commands[jn] = JS(new_targ_q[i], self.vel, self.acc)
            targ_q = new_targ_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('Propulsion:')
        rospy.loginfo(f'LL Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Frame: \n{self.K.FK("RL")}')

    def step(self, with_left=True):
        lift = 'LL' if with_left else 'RL'
        # start positions
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy
        lift_q = self.K.getJointKDLArray(lift)
        steps = int(self.step_t * self.frecv * self.speed)
        dt = np.linspace(0, 2 * np.pi, steps)
        xs = dt * self.step_L * 2 / (2 * np.pi)
        zs = (1 - np.cos(dt))/2 * self.step_H
        for (x, z) in zip(xs, zs):
            # desired position
            lift_F.p[0] = lift_0.p[0] + x
            lift_F.p[2] = lift_0.p[2] + z
            new_lift_q = self.K.IK(lift, lift_F, lift_q)
            # self.K.setJointValues(lift, new_q)
            for i, jn in enumerate(self.K.getJointNames(lift)):
                self.joint_commands[jn] = JS(new_lift_q[i], self.vel, self.acc)
            lift_q = new_lift_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('Step:')
        rospy.loginfo(f'LL Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Frame: \n{self.K.FK("RL")}')

    def stop_walk(self, with_left=True):
        if with_left:
            supp, lift, fact = 'RL', 'LL', -1
        else:
            supp, lift, fact = 'LL', 'RL', 1

        # start positions
        supp_F = self.K.FK(supp)
        supp_0 = kdl.Frame(supp_F)  # make a copy
        supp_q = self.K.getJointKDLArray(supp)
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy
        lift_q = self.K.getJointKDLArray(lift)

        # half step
        # lift_F = self.K.FK(lift)
        # lift_0 = kdl.Frame(lift_F)  # make a copy
        steps = int(self.step_t * self.frecv * self.speed)
        dt = np.linspace(0, 2 * np.pi, steps)
        xs = dt * self.step_L / (2 * np.pi)
        zs = (1 - np.cos(dt))/2 * self.step_H
        for (x, z) in zip(xs, zs):
            # desired position
            lift_F.p[0] = lift_0.p[0] + x
            lift_F.p[2] = lift_0.p[2] + z
            new_lift_q = self.K.IK(lift, lift_F, lift_q)
            # self.K.setJointValues(lift, new_q)
            for i, jn in enumerate(self.K.getJointNames(lift)):
                self.joint_commands[jn] = JS(new_lift_q[i], self.vel, self.acc)
            lift_q = new_lift_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('stop_walk:')
        rospy.loginfo('Step:')
        rospy.loginfo(f'LL Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Frame: \n{self.K.FK("RL")}')

        # half swing
        steps = int(self.swing_t * self.frecv * self.speed / 2.0)
        for s in np.linspace(0, self.swing_A*fact, steps):
            supp_F.p[1] = supp_0.p[1] + s
            new_supp_q = self.K.IK(supp, supp_F, supp_q)
            # self.K.setJointValues(supp, new_q)
            for i, jn in enumerate(self.K.getJointNames(supp)):
                self.joint_commands[jn] = JS(new_supp_q[i], self.vel, self.acc)
            supp_q = new_supp_q
            lift_F.p[1] = lift_0.p[1] + s
            new_lift_q = self.K.IK(lift, lift_F, lift_q)
            # self.K.setJointValues(lift, new_q)
            for i, jn in enumerate(self.K.getJointNames(lift)):
                self.joint_commands[jn] = JS(new_lift_q[i], self.vel, self.acc)
            lift_q = new_lift_q
            self.publishCommands()
            rospy.sleep(1/self.frecv)
        rospy.loginfo('swing:')
        rospy.loginfo(f'LL Frame: \n{self.K.FK("LL")}')
        rospy.loginfo(f'RL Frame: \n{self.K.FK("RL")}')

    def handleCommandCallback(self, msg):
        if msg.data == 'start':
            rospy.loginfo('starting walk')
            self.start_walk()
            self.propulsion()
            self.step(with_left=False)
            self.propulsion(to_left=False)
            self.stop_walk()
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
