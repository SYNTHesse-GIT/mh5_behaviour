import rospy
import numpy as np
import PyKDL as kdl
from sensor_msgs.msg import JointState

from kinematics import Kinematics


class MH5Walking_Controller():

    def __init__(self, params={}):
        self.legs0 = params.get('legs0', 0.95)       # initial pose legs
        self.arms0 = params.get('arms0', -0.75)      # initial pose arms
        self.swing_A = params.get('swing_A', 0.032)  # swing amplitute at torso
        self.step_L = params.get('step_L', 0.050)    # one step length
        self.step_H = params.get('step_H', 0.040)    # swing height for foot
        self.speed = params.get('speed', 1.0)        # execution speed
        self.frecv = params.get('frecv', 100)        # updates / second
        self.prep_t = params.get('prep_t', 1.0)      # preparation duration [s]
        self.swing_t = params.get('swing_t', 0.5)    # lateral swing time [s]
        self.step_t = params.get('step_t', 1.0)      # step time [s]

        self.arms_th = params.get('arm_th', 0.75)    # arm swing [-, + rad]

        self.K = Kinematics()
        self.K.reset()
        self.pub = rospy.Publisher(
            name='joint_states',
            data_class=JointState,
            queue_size=1,
            # latch=True
        )

    def publish_states(self):
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = self.K.getJointNames()
        msg.position = self.K.getJointValues()
        self.pub.publish(msg)

    def prepare_walk(self, legs_angle=None, arms_angle=None):
        legs0 = legs_angle if legs_angle is not None else self.legs0
        arms0 = arms_angle if arms_angle is not None else self.arms0
        # does not use IK as the knee joint will prefere to move backwards
        steps = int(self.frecv * self.prep_t * self.speed)
        leg_pos = np.linspace(0, legs0, steps)
        arm_pos = np.linspace(0, arms0, steps)
        for leg_p, arm_p in zip(leg_pos, arm_pos):
            q = kdl.JntArray(self.K.getJointKDLArray('LL'))
            q[1], q[2], q[4] = leg_p, 2 * leg_p, leg_p
            self.K.setJointValues('LL', q)
            self.K.setJointValues('RL', q)
            q = kdl.JntArray(self.K.getJointKDLArray('LA'))
            q[0] = arm_p
            self.K.setJointValues('LA', q)
            self.K.setJointValues('RA', q)
            self.publish_states()
            rospy.sleep(1.0/self.frecv)

    def start_walk(self, with_left=True):
        if with_left:
            supp, lift, fact = 'RL', 'LL', -1
        else:
            supp, lift, fact = 'LL', 'RL', 1

        # start positions
        supp_F = self.K.FK(supp)
        supp_0 = kdl.Frame(supp_F)  # make a copy
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy

        # half swing
        steps = int(self.swing_t * self.frecv * self.speed / 2.0)
        for s in np.linspace(0, self.swing_A*fact, steps):
            supp_F.p[1] = supp_0.p[1] - s
            new_q = self.K.IK(supp, supp_F)
            self.K.setJointValues(supp, new_q)
            lift_F.p[1] = lift_0.p[1] - s
            new_q = self.K.IK(lift, lift_F)
            self.K.setJointValues(lift, new_q)
            self.publish_states()
            rospy.sleep(1/self.frecv)

        # half step
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy
        steps = int(self.step_t * self.frecv * self.speed)
        dt = np.linspace(0, 2 * np.pi, steps)
        xs = dt * self.step_L / (2 * np.pi)
        zs = (1 - np.cos(dt))/2 * self.step_H
        for (x, z) in zip(xs, zs):
            # desired position
            lift_F.p[0] = lift_0.p[0] + x
            lift_F.p[2] = lift_0.p[2] + z
            new_q = self.K.IK(lift, lift_F)
            self.K.setJointValues(lift, new_q)
            self.publish_states()
            rospy.sleep(1/self.frecv)

    def propulsion(self, to_left=True):
        if to_left:
            supp, targ, fact = 'RL', 'LL', 1
        else:
            supp, targ, fact = 'LL', 'RL', -1

        # start positions
        supp_F = self.K.FK(supp)
        supp_0 = kdl.Frame(supp_F)  # make a copy
        targ_F = self.K.FK(targ)
        targ_0 = kdl.Frame(targ_F)  # make a copy

        # swing
        steps = int(self.swing_t * self.frecv * self.speed)
        dy = np.linspace(0, 2 * self.swing_A * fact, steps)
        dx = np.linspace(0, self.step_L, steps)
        for x, y in zip(dx, dy):
            supp_F.p[0] = supp_0.p[0] - x
            supp_F.p[1] = supp_0.p[1] - y
            new_q = self.K.IK(supp, supp_F)
            self.K.setJointValues(supp, new_q)
            targ_F.p[0] = targ_0.p[0] - x
            targ_F.p[1] = targ_0.p[1] - y
            new_q = self.K.IK(targ, targ_F)
            self.K.setJointValues(targ, new_q)
            self.publish_states()
            rospy.sleep(1/self.frecv)

    def step(self, with_left=True):
        lift = 'LL' if with_left else 'RL'
        # start positions
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy

        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy
        steps = int(self.step_t * self.frecv * self.speed)
        dt = np.linspace(0, 2 * np.pi, steps)
        xs = dt * self.step_L * 2 / (2 * np.pi)
        zs = (1 - np.cos(dt))/2 * self.step_H
        for (x, z) in zip(xs, zs):
            # desired position
            lift_F.p[0] = lift_0.p[0] + x
            lift_F.p[2] = lift_0.p[2] + z
            new_q = self.K.IK(lift, lift_F)
            self.K.setJointValues(lift, new_q)
            self.publish_states()
            rospy.sleep(1/self.frecv)

    def stop_walk(self, with_left=True):
        if with_left:
            supp, lift, fact = 'RL', 'LL', -1
        else:
            supp, lift, fact = 'LL', 'RL', 1

        # start positions
        supp_F = self.K.FK(supp)
        supp_0 = kdl.Frame(supp_F)  # make a copy
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy

        # half step
        lift_F = self.K.FK(lift)
        lift_0 = kdl.Frame(lift_F)  # make a copy
        steps = int(self.step_t * self.frecv * self.speed)
        dt = np.linspace(0, 2 * np.pi, steps)
        xs = dt * self.step_L / (2 * np.pi)
        zs = (1 - np.cos(dt))/2 * self.step_H
        for (x, z) in zip(xs, zs):
            # desired position
            lift_F.p[0] = lift_0.p[0] + x
            lift_F.p[2] = lift_0.p[2] + z
            new_q = self.K.IK(lift, lift_F)
            self.K.setJointValues(lift, new_q)
            self.publish_states()
            rospy.sleep(1/self.frecv)

        # half swing
        steps = int(self.swing_t * self.frecv * self.speed / 2.0)
        for s in np.linspace(0, self.swing_A*fact, steps):
            supp_F.p[1] = supp_0.p[1] + s
            new_q = self.K.IK(supp, supp_F)
            self.K.setJointValues(supp, new_q)
            lift_F.p[1] = lift_0.p[1] + s
            new_q = self.K.IK(lift, lift_F)
            self.K.setJointValues(lift, new_q)
            self.publish_states()
            rospy.sleep(1/self.frecv)
