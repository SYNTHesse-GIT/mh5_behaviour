
from threading import stack_size
import numpy as np
import rospy
import kdl_parser_py.urdf
import PyKDL as kdl


class Chain:

    def __init__(self, kdl_chain, joint_states,
                 FK=kdl.ChainFkSolverPos_recursive,
                 IK=kdl.ChainIkSolverPos_LMA):
        self.chain = kdl_chain
        self.joint_states = joint_states
        self.FK = FK(self.chain)
        self.IK = IK(self.chain)
        names = []
        for seg in range(kdl_chain.getNrOfSegments()):
            joint = kdl_chain.getSegment(seg).getJoint()
            if joint.getType() != kdl.Joint.JointType.Fixed:
                names.append(joint.getName())
        self.names = names
        self.size = len(self.names)

    def __len__(self):
        return self.size

    @property
    def q(self):
        return [self.joint_states[name].p for name in self.names]

    def as_numpy(self):
        return np.array(self.q)

    def jntArray(self):
        ja = kdl.JntArray(self.size)
        for i, name in enumerate(self.names):
            ja[i] = self.joint_states[name].p
        return ja

    def fk(self):
        ja = self.jntArray()
        F = kdl.Frame()
        self.FK.JntToCart(ja, F)
        return F

    def ik(self, frame, q=None):
        if q is None:
            q = self.jntArray()
        result = kdl.JntArray(self.size)
        self.IK.CartToJnt(q, frame, result)
        return result


class Kinematics():

    def __init__(self,
                 joint_states,
                 param_name='robot_description'):

        (ok, self.robot) = kdl_parser_py.urdf.treeFromParam(param_name)

        if not ok:
            msg = 'failed to read robot description from parameter server'
            rospy.logerr(msg)
            raise RuntimeError(msg)

        # ch_par = [
        #     ('LL', 'left_landing'),
        #     ('RL', 'right_landing'),
        #     ('LA', 'left_hand'),
        #     ('RA', 'right_hand'),
        #     ('HD', 'left_camera')]
        # self.chains = {}
        # for ch_name, ch_end in ch_par:
        #     chain = self.robot.getChain("base_link", ch_end)
        #     self.chains[ch_name] = {
        #         'chain': chain,
        #         'joints': self.__get_joint_names_from_chain(chain),
        #         'FK': kdl.ChainFkSolverPos_recursive(chain),
        #         'IK': kdl.ChainIkSolverPos_LMA(chain)
        #     }
        self.left_leg = Chain(self.robot.getChain("base_link", "left_landing"), joint_states)
        self.right_leg = Chain(self.robot.getChain("base_link", "right_landing"), joint_states)
        self.left_arm = Chain(self.robot.getChain("base_link", "left_hand"), joint_states)
        self.right_arm = Chain(self.robot.getChain("base_link", "right_hand"), joint_states)
        self.head = Chain(self.robot.getChain("base_link", "left_camera"), joint_states)
        self.joint_states = joint_states

    # def getChains(self):
    #     return self.chains.keys()

    # def getNrOfJoints(self, chain=None):
    #     if chain is not None:
    #         return self.chains[chain]['chain'].getNrOfJoints()
    #     else:
    #         nr = 0
    #         for ch in self.chains.keys():
    #             nr += self.getNrOfJoints(ch)
    #         return nr

    # def getJointNames(self, chain=None):
    #     if chain is not None:
    #         return self.chains[chain]['joints']
    #     else:
    #         return [name
    #                 for chain in self.chains.values()
    #                 for name in chain['joints']]

    # def getJointValues(self, chain=None):
    #     if chain is not None:
    #         return [self.joint_states[j].p for j in self.chains[chain]['joints']]
    #     else:
    #         return [value
    #                 for ch in self.chains.keys()
    #                 for value in self.getJointValues(ch)]

    # def setJointValues(self, chain, q, vel, acc, joint_commands):
    #     # assert isinstance(values, kdl.JntArray)
    #     # # we need to re-allocate because = assigns the link
    #     # self.chains[chain]['values'] = kdl.JntArray(values)
    #     for i, jn in enumerate(self.getJointNames(chain)):
    #             joint_commands[jn].p = q[i]
    #             joint_commands[jn].p = vel
    #             joint_commands[jn].p = acc

    # def reset(self, chain=None):
    #     if chain is not None:
    #         nrJ = self.getNrOfJoints(chain)
    #         self.chains[chain]['values'] = kdl.JntArray(nrJ)
    #     else:
    #         for ch in self.chains.keys():
    #             self.reset(ch)

    # def getJointKDLArray(self, chain):
    #     jnt_array = kdl.JntArray(len(self.chains[chain]['joints']))
    #     for i, value in enumerate(self.getJointValues(chain)):
    #         jnt_array[i] = value
    #     return jnt_array

    # def getJointNamesAndValues(self, chain=None):
    #     return zip(self.getJointNames(chain), self.getJointValues(chain))

    # def FK(self, chain):
    #     fk = self.chains[chain]['FK']
    #     ja = self.getJointKDLArray(chain)
    #     F = kdl.Frame()
    #     fk.JntToCart(ja, F)
    #     return F

    # def IK(self, chain, frame, q=None):
    #     # IK is from current position
    #     ik = self.chains[chain]['IK']
    #     if q is None:
    #         q = self.getJointKDLArray(chain)
    #     result = kdl.JntArray(self.getNrOfJoints(chain))
    #     ik.CartToJnt(q, frame, result)
    #     return result

    # def EEPos(self, chain):
    #     return list(self.FK(chain).p)

    # def EERPY(self, chain):
    #     return list(self.FK(chain).M.GetGetRPY())

    # def EEPosRPY(self, chain):
    #     frame = self.FK(chain)
    #     return list(frame.p), list(frame.M.GetRPY())

    # def __get_joint_names_from_chain(self, chain):
    #     names = []
    #     for seg in range(chain.getNrOfSegments()):
    #         joint = chain.getSegment(seg).getJoint()
    #         if joint.getType() != kdl.Joint.JointType.Fixed:
    #             names.append(joint.getName())
    #     return names


class LinearPoser:

    def __init__(self, chain, steps, end_frame, start_frame=None):
        self.chain = chain
        if start_frame is None:
            start_frame = chain.fk()
        self.start_xyz = np.array(start_frame.p)
        self.start_rpy = np.array(start_frame.M.GetRPY())
        self.end_xyz = np.array(end_frame.p)
        self.end_rpy = np.array(end_frame.M.GetRPY())
        self.d_xyz = (self.end_xyz - self.start_xyz) / steps
        self.d_rpy = (self.end_rpy - self.start_rpy) / steps
        self.steps = steps
        self.qs = chain.q

    def frame(self, step):
        assert self.steps >= step > 0
        xyz = self.start_xyz + step * self.d_xyz
        rpy = self.start_rpy + step * self.d_rpy
        return kdl.Frame(
                V=kdl.Vector(xyz[0], xyz[1], xyz[2]),
                R=kdl.Rotation.RPY(rpy[0], rpy[1], rpy[2]))
