
from threading import stack_size
import numpy as np
import rospy
import kdl_parser_py.urdf
import PyKDL as kdl


class Kinematics():

    def __init__(self,
                 joint_states,
                 param_name='robot_description'):

        (ok, self.robot) = kdl_parser_py.urdf.treeFromParam(param_name)

        if not ok:
            msg = 'failed to read robot description from parameter server'
            rospy.logerr(msg)
            raise RuntimeError(msg)

        ch_par = [
            ('LL', 'left_landing'),
            ('RL', 'right_landing'),
            ('LA', 'left_hand'),
            ('RA', 'right_hand'),
            ('HD', 'left_camera')]
        self.chains = {}
        for ch_name, ch_end in ch_par:
            chain = self.robot.getChain("base_link", ch_end)
            self.chains[ch_name] = {
                'chain': chain,
                'joints': self.__get_joint_names_from_chain(chain),
                'FK': kdl.ChainFkSolverPos_recursive(chain),
                'IK': kdl.ChainIkSolverPos_LMA(chain)
            }
        self.joint_states = joint_states

    def getChains(self):
        return self.chains.keys()

    def getNrOfJoints(self, chain=None):
        if chain is not None:
            return self.chains[chain]['chain'].getNrOfJoints()
        else:
            nr = 0
            for ch in self.chains.keys():
                nr += self.getNrOfJoints(ch)
            return nr

    def getJointNames(self, chain=None):
        if chain is not None:
            return self.chains[chain]['joints']
        else:
            return [name
                    for chain in self.chains.values()
                    for name in chain['joints']]

    def getJointValues(self, chain=None):
        if chain is not None:
            return [self.joint_states[j].p for j in self.chains[chain]['joints']]
        else:
            return [value
                    for ch in self.chains.keys()
                    for value in self.getJointValues(ch)]

    # def setJointValues(self, chain, values):
    #     assert isinstance(values, kdl.JntArray)
    #     # we need to re-allocate because = assigns the link
    #     self.chains[chain]['values'] = kdl.JntArray(values)

    # def reset(self, chain=None):
    #     if chain is not None:
    #         nrJ = self.getNrOfJoints(chain)
    #         self.chains[chain]['values'] = kdl.JntArray(nrJ)
    #     else:
    #         for ch in self.chains.keys():
    #             self.reset(ch)

    def getJointKDLArray(self, chain):
        jnt_array = kdl.JntArray(len(self.chains[chain]['joints']))
        for i, value in enumerate(self.getJointValues(chain)):
            jnt_array[i] = value
        return jnt_array

    def getJointNamesAndValues(self, chain=None):
        return zip(self.getJointNames(chain), self.getJointValues(chain))

    def FK(self, chain):
        fk = self.chains[chain]['FK']
        ja = self.getJointKDLArray(chain)
        F = kdl.Frame()
        fk.JntToCart(ja, F)
        return F

    def IK(self, chain, frame, q=None):
        # IK is from current position
        ik = self.chains[chain]['IK']
        if q is None:
            q = self.getJointKDLArray(chain)
        result = kdl.JntArray(self.getNrOfJoints(chain))
        ik.CartToJnt(q, frame, result)
        return result

    def EEPos(self, chain):
        return list(self.FK(chain).p)

    def EERPY(self, chain):
        return list(self.FK(chain).M.GetGetRPY())

    def EEPosRPY(self, chain):
        frame = self.FK(chain)
        return list(frame.p), list(frame.M.GetRPY())

    def __get_joint_names_from_chain(self, chain):
        names = []
        for seg in range(chain.getNrOfSegments()):
            joint = chain.getSegment(seg).getJoint()
            if joint.getType() != kdl.Joint.JointType.Fixed:
                names.append(joint.getName())
        return names


class LinearPoser:

    def __init__(self, start_pose, end_pose, steps):
        self.start_xyz = np.array(start_pose.p)
        self.start_rpy = np.array(start_pose.M.GetRPY())
        self.end_xyz = np.array(end_pose.p)
        self.end_rpy = np.array(end_pose.M.GetRPY())
        self.d_xyz = (self.end_xyz - self.start_xyz) / steps
        self.d_rpy = (self.end_rpy - self.start_rpy) / steps
        self.steps = steps

    def __iter__(self):
        for i in range(self.steps):
            xyz = self.start_xyz + (i+1) * self.d_xyz
            rpy = self.start_rpy + (i+1) * self.d_rpy
            yield kdl.Frame(
                V=kdl.Vector(xyz[0], xyz[1], xyz[2]),
                R=kdl.Rotation.RPY(rpy[0], rpy[1], rpy[2])
            )
