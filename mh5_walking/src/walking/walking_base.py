from collections import namedtuple

import rospy

from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, \
                                        SwitchControllerRequest
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from .kinematics import Kinematics

JS = namedtuple('JS', ['p', 'v', 'l'])


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
        self.startPose()
        rospy.on_shutdown(self.on_shutdown)

    def readParams(self, name_space):
        # read controllers parameters
        self.params = rospy.get_param(name_space)
        if 'using' not in self.params:
            message = 'no controllers specified in parameters (using:)'
            rospy.logerror(message)
            raise ValueError(message)
        self.using_controllers = self.params['using']

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
                    Float64MultiArray,
                    queue_size=5)
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
            command = Float64MultiArray()
            command.data = [0.0] * len(controller['res'])
            for index, joint_name in enumerate(controller['res']):
                command.data[index] = self.joint_commands.get(
                    joint_name, self.joint_states[joint_name].p)
            controller['pub'].publish(command)

    def on_shutdown(self):
        self.stopPose()
        self.stopControllers()
