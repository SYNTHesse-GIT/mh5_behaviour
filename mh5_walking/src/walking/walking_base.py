from collections import namedtuple

import rospy
import PyKDL as kdl

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, \
                                        SwitchControllerRequest
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from .kinematics import Kinematics, LinearPoser

# joint state
JS = namedtuple('JS', ['p', 'v', 'a', 'l'], defaults=[0, 0, 0, 0])


# class WalkingFSM:

#     def __init__(self):
#         self.state = None
#         self.next_foot = 0         # no swing foot yet
#         self.start_standing()

#     def run(self):
#         if self.state == "Standing":
#             self.run_standing()
#         elif self.state == "DoubleSupport":
#             self.run_double_support()
#         elif self.state == "SingleSupport":
#             self.run_single_support()
#         else:
#             raise Exception(f"Unknown state: {self.state}")

#     def start_standing(self):
#         self.should_start_walking = False
#         self.state = "Standing"
#         self.run_standing()

#     def run_standing(self):
#         if self.should_start_walking:
#             self.should_start_walking = False
#             self.start_double_support()

#     def start_double_support(self):
#         self.state = "DoubleSupport"
#         # mpc
#         self.run_double_support()

#     def run_double_support(self):


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

        rospy.loginfo([(c.name, c.state) for c in self.available_controllers])
        self.readParams(name_space)
        self.startSubscribers(name_space)
        self.startControllers()
        # self.startPose()
        self.state = "Standing"
        self.start_standing()
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.run)
        rospy.on_shutdown(self.on_shutdown)

    def readParams(self, name_space):
        # read controllers parameters
        self.params = rospy.get_param(name_space)
        if 'using' not in self.params:
            message = 'no controllers specified in parameters (using:)'
            rospy.logerror(message)
            raise ValueError(message)
        self.using_controllers = self.params['using']
        params = self.params.get('params', {})
        self.dt = params.get('dt', 0.05)
        self.foot_spread = params.get('foot_spread', 0.033)
        self.step_size = params.get('step_size', 0.050)
        self.stand_z = params.get('stand_z', 0.013)
        self.stanz_x = params.get('stand_x', 0.005)
        self.prep_t = params.get('prep_t', 1.0)
        self.phase_total_steps = None
        self.phase_step = None

    def startSubscribers(self, name):
        self.command = rospy.Subscriber(
            f'{name}/command', String, self.handleCommandCallback)
        self.state = rospy.Subscriber(
            'joint_states', JointState, self.handleJointStateCallback)

    def handleCommandCallback(self, msg):
        pass

    def handleJointStateCallback(self, msg):
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

    def startControllers(self):
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

    def stopControllers(self):
        for controller in self.controllers.keys():
            req = SwitchControllerRequest(
                stop_controllers=[controller], timeout=5, strictness=2)
            resp = self.switch_controller(req)
            if resp.ok:
                rospy.loginfo(f'controller {controller} stopped')
            else:
                rospy.loginfo(f'failed to stop controller {controller}')

    def run(self):
        if self.state == "Standing":
            self.run_standing()
        elif self.state == "DoubleSupport":
            self.run_double_support()
        elif self.state == "SingleSupport":
            self.run_single_support()
        else:
            raise Exception(f"Unknown state: {self.state}")

    def start_standing(self):
        self.phase_total_steps = int(self.prep_t / self.dt)
        self.phase_step = 1
        self.frame_iters = []
        llP = kdl.Vector(self.stand_x, self.foot_spread, self.stand_z)
        self.frame_iters.append(LinearPoser(
            self.K.left_leg, self.phase_total_steps, kdl.Frame(llP)))
        rlP = kdl.Vector(self.stand_x, -self.foot_spread, self.stand_z)
        self.frame_iters.append(LinearPoser(
            self.K.right_leg, self.phase_total_steps, kdl.Frame(rlP)))

    def run_standing(self):
        if self.phase_step <= self.phase_total_steps:
            for frame_iter in self.frame_iters:
                frame = frame_iter.frame(self.phase_step)
                qs = frame_iter.chain.ik(frame)


    def start_double_support(self):
        pass

    def run_double_support(self):
        pass

    def start_single_support(self):
        pass

    def run_single_support(self):
        pass

    def startPose(self):
        pass

    def stopPose(self):
        pass

    def publishCommands(self):
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
                if joint_name in self.joint_commands:
                    command.positions[i] = self.joint_commands[joint_name].p
                    command.velocities[i] = self.joint_commands[joint_name].v
                    command.accelerations[i] = self.joint_commands[joint_name].a
                else:
                    command.positions[i] = self.joint_states[joint_name].p
                    command.velocities[i] = self.joint_states[joint_name].v
                    command.accelerations[i] = 0
            controller['pub'].publish(command)

    def on_shutdown(self):
        self.stopPose()
        self.stopControllers()
