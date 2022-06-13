
import rospy
import kdl_parser_py.urdf
import PyKDL as kdl


class Kinematics():

    def __init__(self, param_name='robot_description'):
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
                'values': kdl.JntArray(chain.getNrOfJoints()),
                'FK': kdl.ChainFkSolverPos_recursive(chain),
                'IK': kdl.ChainIkSolverPos_LMA(chain)
            }

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
            return list(self.chains[chain]['values'])
        else:
            return [value
                    for ch in self.chains.keys()
                    for value in self.getJointValues(ch)]

    def setJointValues(self, chain, values):
        assert isinstance(values, kdl.JntArray)
        # we need to re-allocate because = assigns the link
        self.chains[chain]['values'] = kdl.JntArray(values)

    def reset(self, chain=None):
        if chain is not None:
            nrJ = self.getNrOfJoints(chain)
            self.chains[chain]['values'] = kdl.JntArray(nrJ)
        else:
            for ch in self.chains.keys():
                self.reset(ch)

    def getJointKDLArray(self, chain):
        return self.chains[chain]['values']

    def getJointNamesAndValues(self, chain=None):
        return zip(self.getJointNames(chain), self.getJointValues(chain))

    def FK(self, chain):
        fk = self.chains[chain]['FK']
        ja = self.chains[chain]['values']
        F = kdl.Frame()
        fk.JntToCart(ja, F)
        return F

    def IK(self, chain, frame):
        # IK is from current position
        ik = self.chains[chain]['IK']
        q = self.chains[chain]['values']
        result = kdl.JntArray(self.getNrOfJoints(chain))
        ik.CartToJnt(q, frame, result)
        return result

    def EEPos(self, chain):
        return list(self.FK(chain).p)

    def EERPY(self, chain):
        return list(self.FK(chain).M.GetEulerZYX())

    def __get_joint_names_from_chain(self, chain):
        names = []
        for seg in range(chain.getNrOfSegments()):
            joint = chain.getSegment(seg).getJoint()
            if joint.getType() != kdl.Joint.JointType.Fixed:
                names.append(joint.getName())
        return names
