#!/usr/bin/env python3

from collections import namedtuple

import rospy
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

import numpy as np

# joint state: position, vaelocity, acceleration, load
JS = namedtuple('JS', ['p', 'v', 'a', 'l'], defaults=[0, 0, 0, 0])

class ControllerHandler:

    def __init__(self):
        rospy.loginfo('waiting for controller_manager services...')
        rospy.wait_for_service('controller_manager/list_controllers', timeout=5)
        list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
        self.available_controllers = list_controllers().controller
        rospy.wait_for_service('controller_manager/switch_controller', timeout=5)
        self.switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
        rospy.loginfo('found these controllers:')
        for c in self.available_controllers:
            rospy.loginfo(f'Controller: {c.name}, state: {c.state}')
        self.controllers = {}

    def start_controllers(self, controller_names):
        rospy.loginfo('Starting controllers:')
        for controller in self.available_controllers:
            if controller.name in controller_names:
                if controller.state == 'running':
                    rospy.loginfo(f'controller {controller.name} already running...')
                else:
                    req = SwitchControllerRequest(
                        start_controllers=[controller.name],
                        timeout=5,
                        strictness=2)
                    resp = self.switch_controller(req)
                    if not resp.ok:
                        msg = f'failed to start controller {controller.name}'
                        rospy.logerr(msg)
                        self.stopControllers(controllers)
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
            req = SwitchControllerRequest(stop_controllers=[controller], timeout=5, strictness=2)
            resp = self.switch_controller(req)
            if resp.ok:
                rospy.loginfo(f'controller {controller} stopped')
            else:
                rospy.loginfo(f'failed to stop controller {controller}')

    def publish_commands(self, joint_commands):
        for controller in self.controllers.values():
            command = JointTrajectoryPoint()
            command.positions = [0.0] * len(controller['res'])
            command.velocities = [0.0] * len(controller['res'])
            command.accelerations = [0.0] * len(controller['res'])
            for i, joint_name in enumerate(controller['res']):
                command.positions[i] = joint_commands[joint_name].p
                command.velocities[i] = joint_commands[joint_name].v
                command.accelerations[i] = joint_commands[joint_name].a
            controller['pub'].publish(command)

class Demo():

    def __init__(self):
        self.ch = ControllerHandler()
        self.ch.start_controllers(['arms_pos_controller', 'head_pos_controller'])
        rospy.on_shutdown(self.on_shutdown)
        rospy.loginfo('Strating subscribers...')
        self.joint_states = {}
        self.joint_commands = {}
        self.joints_head = ['head_y', 'head_p']
        self.joints_left = ['l_sho_p', 'l_sho_r',  'l_elb_y',  'l_elb_p']
        self.joints_right = ['r_sho_p', 'r_sho_r',  'r_elb_y',  'r_elb_p']
        self.state_sub = rospy.Subscriber('joint_states', JointState, self.handle_joint_state_callback)
        rospy.sleep(1)  # wait for the joints to become active
        self.start_pose()
        self.phase = 'head'
        self.start_head()

    def on_shutdown(self):
        self.start_pose()
        self.ch.stop_controllers()

    def handle_joint_state_callback(self, msg):
        for i, joint_name in enumerate(msg.name):
            self.joint_states[joint_name] = JS(
                p=msg.position[i], v=msg.velocity[i], l=msg.effort[i])

    def start_pose(self):
        vel = 0.1
        rospy.loginfo('Starting pose...')
        for joint, position in zip(self.joints_head, [0,0]):
            self.joint_commands[joint] = JS(position, vel, 0)
        for joint, position in zip(self.joints_left, [0, 0, 0, 0]):
            self.joint_commands[joint] = JS(position, vel, 0)
        for joint, position in zip(self.joints_right, [0, 0, 0, 0]):
            self.joint_commands[joint] = JS(position, vel, 0)
        self.ch.publish_commands(self.joint_commands)
        rospy.sleep(2)

    def start_head(self):
        moves = 100
        head_y = np.random.random_sample() * 2 * 0.5 - 0.5
        head_p = np.random.random_sample() * (-0.6) + 0.2
        head_y_array = np.linspace(self.joint_states['head_y'].p, head_y, moves)
        head_p_array = np.linspace(self.joint_states['head_p'].p, head_p, moves)
        self.positions = [head_y_array, head_p_array]
        self.index = 0

    def run_head(self):
        for joint, positions in zip(self.joints_head, self.positions):
            self.joint_commands[joint] = JS(positions[self.index], 0, 0)
        self.ch.publish_commands(self.joint_commands)
        self.index +=1
        if self.index == len(positions):
            if np.random.random_sample() > 0.25:
                self.start_head()
            else:
                self.start_pose()
                self.start_wave()
                self.phase = 'wave'

    def start_wave(self):
        moves = 100
        # raise arms
        sho_p = np.linspace(0, 1.5, moves)
        sho_r = np.linspace(0, 0.7, moves)
        elb_y = [0] * moves
        elb_p = [0] * moves
        # lower arms
        np.append(sho_p, np.linspace(1.5, 0, moves))
        np.append(sho_r, np.linspace(0.7, 0, moves))
        np.append(elb_y, [0] * moves)
        np.append(elb_p, [0] * moves)
        # motion path
        self.positions = [sho_p, sho_r, elb_y, elb_p, sho_p, sho_r, elb_y, elb_p]
        self.index = 0

    def run_wave(self):
        for joint, positions in zip(self.joints_left + self.joints_right, self.positions):
            self.joint_commands[joint] = JS(positions[self.index], 0, 0)
        self.ch.publish_commands(self.joint_commands)
        self.index +=1
        if self.index == len(positions):
                self.start_pose()
                self.start_head()
                self.phase = 'head'

    def run_demo(self):
        if self.phase == 'head':
            self.run_head()
        elif self.phase == 'wave':
            self.run_wave()


if __name__ == '__main__':

    rospy.init_node('mh5_demo')

    demo = Demo()

    r = rospy.Rate(50) # 10Hz
    while not rospy.is_shutdown():
        demo.run_demo()
        r.sleep()


