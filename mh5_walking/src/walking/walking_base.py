from collections import namedtuple

import rospy

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from controller_manager_msgs.srv import SwitchControllerRequest
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import PyKDL as kdl

from .kinematics import Kinematics, LinearPoser


# joint state
JS = namedtuple('JS', ['p', 'v', 'a', 'l'], defaults=[0, 0, 0, 0])


class WalkingBase:

    def __init__(self, name_space):
        self.name_space = name_space
        self.controllers = {}
        self.joint_states = {}
        self.joint_commands = {}
        self.K = Kinematics(joint_states=self.joint_states)
        # self.K.reset()
        rospy.loginfo('waiting for controller_manager services...')
        rospy.wait_for_service(
            'controller_manager/list_controllers', timeout=5)
        list_controllers = rospy.ServiceProxy(
            'controller_manager/list_controllers', ListControllers)
        self.available_controllers = list_controllers().controller
        rospy.wait_for_service(
            'controller_manager/switch_controller', timeout=5)
        self.switch_controller = rospy.ServiceProxy(
            'controller_manager/switch_controller', SwitchController)

        rospy.loginfo('found these controllers:')
        for c in self.available_controllers:
            rospy.loginfo(f'Controller: {c.name}, state: {c.state}')
        self.read_params(name_space)
        self.start_subscribers(name_space)
        self.start_controllers()
        # wait for the joint_states messages as they are needed to initialize
        # the pose iterators
        rospy.sleep(2)

        self.state = None
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.run)

        rospy.sleep(2)
        self.start_pose()

        # self.state = "Standing"
        # self.start_standing()
        rospy.on_shutdown(self.on_shutdown)

    def read_params(self, name_space):
        # read controllers parameters
        self.params = rospy.get_param(name_space)
        if 'using' not in self.params:
            message = 'no controllers specified in parameters (using:)'
            rospy.logerror(message)
            raise ValueError(message)
        self.using_controllers = self.params['using']
        # wlaking parameters
        params = self.params.get('params', {})
        self.dt = params.get('dt', 0.05)
        self.prep_t = params.get('prep_t', 1.0)
        self.foot_spread = params.get('foot_spread', 0.033)
        self.stand_z = params.get('stand_z', 0.013)
        self.stand_x = params.get('stand_x', 0.005)
        self.sit_z = params.get('sit_z', 0.091)
        self.sit_x = params.get('sit_x', 0.007)
        # self.step_size = params.get('step_size', 0.050)
        self.phase_total_steps = None
        self.phase_step = None

    def start_subscribers(self, name):
        rospy.loginfo('Strating subscribers...')
        self.command_sub = rospy.Subscriber(
            f'{name}/command', String, self.handle_command_callback)
        self.state_sub = rospy.Subscriber(
            'joint_states', JointState, self.handle_joint_state_callback)

    def handle_command_callback(self, msg):
        pass

    def handle_joint_state_callback(self, msg):
        """Handles the `joint_states` messages by updating the `joint_states`
        dictionary. Only the joints that are present in the message are
        updated.

        Parameters
        ----------
        msg : `JointState`
            A message with joint state information.
        """
        for i, joint_name in enumerate(msg.name):
            self.joint_states[joint_name] = JS(
                p=msg.position[i], v=msg.velocity[i], l=msg.effort[i])

    def start_controllers(self):
        rospy.loginfo('Starting controllers:')
        for controller in self.available_controllers:
            if controller.name in self.using_controllers:
                if controller.state == 'running':
                    # attempting to start an already running controller will
                    # issue an error...
                    rospy.loginfo(
                        f'controller {controller.name} already running...')
                else:
                    # attempt to start controller
                    req = SwitchControllerRequest(
                        start_controllers=[controller.name],
                        timeout=5,
                        strictness=2)
                    resp = self.switch_controller(req)
                    if not resp.ok:
                        # if we failed to start one of the controllers abort
                        # and stop all already started controllers then raise
                        # and exception
                        msg = f'failed to start controller {controller.name}'
                        rospy.logerr(msg)
                        self.stopControllers()
                        raise RuntimeError(msg)
                    else:
                        rospy.loginfo(f'controller {controller.name} started')
                # for a running controllers register a `command` publisher and
                # store the joint in order they are handled by controller
                pub = rospy.Publisher(
                    f'{controller.name}/command',
                    JointTrajectoryPoint,
                    queue_size=1)
                self.controllers[controller.name] = {
                    'res': rospy.get_param(f'{controller.name}/joints'),
                    'pub': pub
                }

    def stop_controllers(self):
        for controller in self.controllers.keys():
            req = SwitchControllerRequest(
                stop_controllers=[controller], timeout=5, strictness=2)
            resp = self.switch_controller(req)
            if resp.ok:
                rospy.loginfo(f'controller {controller} stopped')
            else:
                rospy.loginfo(f'failed to stop controller {controller}')

    def run(self, event=None):
        # print(f'in RUN; state: {self.state} ')
        if self.state == 'Starting':
            self.run_start_pose()
        elif self.state == "Standing":
            self.run_standing()
        elif self.state == "DoubleSupport":
            self.run_dsp()
        elif self.state == "SingleSupport":
            self.run_ssp()
        elif self.state == 'Stopping':
            self.run_stop_pose()
        # else:
        #     raise Exception(f"Unknown state: {self.state}")

    def start_pose(self):
        rospy.loginfo('Starting inital pose...')
        self.phase_total_steps = int(self.prep_t / self.dt)
        self.phase_step = 1
        self.frame_iters = []
        llP = kdl.Vector(self.stand_x, self.foot_spread, self.stand_z)
        self.frame_iters.append(LinearPoser(
            self.K.left_leg, self.phase_total_steps, kdl.Frame(llP)))
        rlP = kdl.Vector(self.stand_x, -self.foot_spread, self.stand_z)
        self.frame_iters.append(LinearPoser(
            self.K.right_leg, self.phase_total_steps, kdl.Frame(rlP)))
        self.state = 'Starting'

    def run_start_pose(self):
        if self.phase_step <= self.phase_total_steps:
            for frame_iter in self.frame_iters:
                frame = frame_iter.frame(self.phase_step)
                new_qs = frame_iter.chain.ik(frame)
                vels = self.jntArrayDiff(new_qs, frame_iter.qs, 1.0/self.dt)
                self.set_joint_commands(frame_iter.chain, new_qs, vels)
                frame_iter.qs = new_qs
            # print(self.joint_commands)
            self.publish_commands()
            self.phase_step += 1
        else:
            self.start_standing()

    def start_standing(self):
        self.state = 'Standing'

    def run_standing(self):
        pass

    def start_dsp(self):
        self.state = "DoubleSupport"

    def run_dsp(self):
        pass

    def start_ssp(self):
        self.state = "SingleSupport"

    def run_ssp(self):
        pass

    def stop_pose(self):
        rospy.loginfo('Starting sitting pose...')
        self.phase_total_steps = int(self.prep_t / self.dt)
        self.phase_step = 1
        self.frame_iters = []
        llP = kdl.Vector(self.sit_x, self.foot_spread, self.sit_z)
        self.frame_iters.append(LinearPoser(
            self.K.left_leg, self.phase_total_steps, kdl.Frame(llP)))
        rlP = kdl.Vector(self.sit_x, -self.foot_spread, self.sit_z)
        self.frame_iters.append(LinearPoser(
            self.K.right_leg, self.phase_total_steps, kdl.Frame(rlP)))
        self.state = 'Stopping'

    def run_stop_pose(self):
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
            self.state = 'Sitting'

    def jntArrayDiff(self, arrA, arrB, factor=1.0):
        return [abs(a-b)*factor for (a, b) in zip(arrA, arrB)]

    def set_joint_commands(self, chain_name, qs, vels=None, accs=None):
        if vels is None:
            vels = [0.0] * qs.rows()
        if accs is None:
            accs = [0.0] * qs.rows()
        for i, jn in enumerate(chain_name.names):
            self.joint_commands[jn] = JS(p=qs[i], v=vels[i], a=accs[i])

    def publish_commands(self):
        """Publish the `command` messages for each if the used controllers based
        on the requested poisitions for the joints. For each controller the
        command array is constructed based on the order of the joints specified
        for that controller. The requests are taken from the `joint_commands`
        dictionary that is prepared by the walking algorithms. If a joint does
        not have a request command position the value is taken from the current
        joint state in `joint_states`.
        """
        for controller in self.controllers.values():
            command = JointTrajectoryPoint()
            command.positions = [0.0] * len(controller['res'])
            command.velocities = [0.0] * len(controller['res'])
            command.accelerations = [0.0] * len(controller['res'])
            for i, joint_name in enumerate(controller['res']):
                command.positions[i] = self.joint_commands[joint_name].p
                command.velocities[i] = self.joint_commands[joint_name].v
                command.accelerations[i] = self.joint_commands[joint_name].a
            controller['pub'].publish(command)

    def on_shutdown(self):
        self.stop_pose()
        while self.state != 'Sitting':
            rospy.sleep(.5)
        self.stop_controllers()
