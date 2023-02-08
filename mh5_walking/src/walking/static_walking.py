import rospy
import PyKDL as kdl

from .walking_base import WalkingBase
from .kinematics import LinearPoser, SinPoser


class StaticWalking(WalkingBase):

    def __init__(self):
        WalkingBase.__init__(self, name_space='static_walking')

    def read_params(self, name_space):
        super().read_params(name_space)
        params = self.params.get('params', {})
        self.dsp_duration = params.get('dsp_duration', 0.5)
        self.ssp_duration = params.get('ssp_duration', 1.5)
        self.swing_direction = params.get('swing_direction', -1)
        self.swing_y = params.get('swing_y', 0.04)
        self.step_x = params.get('step_x', 0.05)
        self.step_z = params.get('step_z', 0.03)
        self.steps = 0

    def run_standing(self):
        """
        This should normally check a topic for commands to start walking.
        Here we simply wait for a while and move to Double Support Phase.
        """
        rospy.sleep(2)
        self.start_dsp()

    def start_dsp(self):
        rospy.loginfo('Starting DSP...')
        self.phase_total_steps = int(self.dsp_duration / self.dt)
        self.phase_step = 1
        self.frame_iters = []
        llP = kdl.Vector(
            self.stand_x,
            self.foot_spread - self.swing_direction * self.swing_y,
            self.stand_z)
        self.frame_iters.append(LinearPoser(
            self.K.left_leg, self.phase_total_steps, kdl.Frame(llP)))
        rlP = kdl.Vector(
            self.stand_x,
            - self.foot_spread - self.swing_direction * self.swing_y,
            self.stand_z)
        self.frame_iters.append(LinearPoser(
            self.K.right_leg, self.phase_total_steps, kdl.Frame(rlP)))
        super().start_dsp()

    def run_dsp(self):
        if self.phase_step <= self.phase_total_steps:
            for frame_iter in self.frame_iters:
                frame = frame_iter.frame(self.phase_step)
                new_qs = frame_iter.chain.ik(frame)
                vels = self.jntArrayDiff(new_qs, frame_iter.qs, 1.0/self.dt)
                self.set_joint_commands(frame_iter.chain, new_qs, vels)
                frame_iter.qs = new_qs
            self.publish_commands()
            self.phase_step += 1
        else:
            self.start_ssp()

    def start_ssp(self):
        rospy.loginfo('Starting SSP...')
        self.phase_total_steps = int(self.ssp_duration / self.dt)
        self.phase_step = 1
        self.frame_iters = []
        llP = kdl.Vector(self.K.left_leg.fk().p)
        rlP = kdl.Vector(self.K.right_leg.fk().p)
        if self.swing_direction == -1:
            # swing left leg
            llP.x(rlP.x() + self.step_x)
            self.frame_iters.append(
                SinPoser(
                    chain=self.K.left_leg,
                    steps=self.phase_total_steps,
                    lift=self.step_z,
                    end_frame=kdl.Frame(llP)
                )
            )
        elif self.swing_direction == 1:
            # swing right leg
            rlP.x(llP.x() + self.step_x)
            self.frame_iters.append(
                SinPoser(
                    chain=self.K.right_leg,
                    steps=self.phase_total_steps,
                    lift=self.step_z,
                    end_frame=kdl.Frame(rlP)
                )
            )
        # llP = kdl.Vector(
        #     self.stand_x,
        #     self.foot_spread * (1 - self.swing_direction),
        #     self.stand_z)
        # self.frame_iters.append(LinearPoser(
        #     self.K.left_leg, self.phase_total_steps, kdl.Frame(llP)))
        # rlP = kdl.Vector(
        #     self.stand_x,
        #     - self.foot_spread * (1 + self.swing_direction),
        #     self.stand_z)
        # self.frame_iters.append(LinearPoser(
        #     self.K.right_leg, self.phase_total_steps, kdl.Frame(rlP)))
        super().start_ssp()

    def run_ssp(self):
        if self.phase_step <= self.phase_total_steps:
            for frame_iter in self.frame_iters:
                frame = frame_iter.frame(self.phase_step)
                new_qs = frame_iter.chain.ik(frame)
                vels = self.jntArrayDiff(new_qs, frame_iter.qs, 1.0/self.dt)
                self.set_joint_commands(frame_iter.chain, new_qs, vels)
                frame_iter.qs = new_qs
            self.publish_commands()
            self.phase_step += 1
        else:
            self.steps += 1
            if self.steps <= 3:
                self.swing_direction *= -1
            else:
                # stop
                self.swing_direction = 0
            self.start_dsp()
