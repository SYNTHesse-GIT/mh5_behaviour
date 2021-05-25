import yaml
from os.path import join

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class Pose():
    """A `Pose` groups together the joints and the positions needed to
    produce a specific robot pose.
    """

    def __init__(self):
        #: str: Pose name
        self.name = ''

        #: list of str: Joints used by the `Pose`. If the source XACRO did not
        #: use ``joints=`` the `Pose` will inherit by default all the joints
        #: defined in the portfolio
        self.joints = []

        #: list of float: Positions for each of the joints associated with
        #: this `Pose`. Each position matches the order of joints and is expressed
        #: in the unit of measures defined by the portfolio.
        self.positions = []

    @classmethod
    def from_xml(cls, xml_elem, portfolio):
        """Initializes the `Pose` from an XML element tree structure.

        Parameters
        ----------
        xml_elem : xml.etree.ElementTree.ElementTree
            The XML element tree that contains the structure of the `Pose`
        portfolio : :obj:`Portfolio`
            The top `Portfolio` object that ownes this `Pose`

        Returns
        -------
        :obj:`Pose`
            The initiazed `Pose` object.

        Raises
        ------
        ValueError
            If values included in the XML are incorrect. More details are
            provided in the exception text.
        AssertError
            If certain attributes are not included in the XML. More details
            are provided in the exception text.
        """
        pose = Pose()
        
        err = f'>>> Portfolio {portfolio.name} '
        assert xml_elem.tag == 'pose', err + 'only <pose> tags should be used in <poses>'

        assert 'name' in xml_elem.attrib, err + '"name" attribute expected in <pose> definition'
        pose.name = xml_elem.attrib['name']

        err = err + f'scene {pose.name} '
        assert 'positions' in xml_elem.attrib, err + '"positions" attribute expected'
        positions = xml_elem.attrib['positions']
        assert isinstance(positions, str), err + '"positions" should be a string of positions'
        positions = positions.split()
        try:
            pose.positions = [float(pos) for pos in positions]
        except:
            raise ValueError(err + 'failed to convert positions to floats') from None

        if 'joints' in xml_elem.attrib:
            assert isinstance(xml_elem.attrib['joints'], str), err + '"joints" should be a string of joint names'
            pose.joints = xml_elem.attrib['joints'].split()
        else:
            pose.joints = portfolio.joints
        assert len(pose.joints) == len(pose.positions), err + 'list of "joints" has different length than "positions"'

        return pose


class Scene():

    def __init__(self):
        self.name = ''
        self.poses = []
        self.durations = []


    @classmethod
    def from_xml(cls, xml_elem, portfolio):

        scene = Scene()

        err = f'>>> Portfolio {portfolio.name} '
        assert xml_elem.tag == 'scene', err + 'only <scene> tags should be used in <scenes>'

        assert 'name' in xml_elem.attrib, err + '"name" attribute expected in <scene> definition'
        scene.name = xml_elem.attrib['name']

        for pose in list(xml_elem):
            
            err = err + f'scene {scene.name} '
            assert pose.tag == 'pose', err + 'only <pose> tags should be used in <scene>'
            
            assert 'name' in pose.attrib, err + '"name" for pose missing'
            p_name = pose.attrib['name']
            assert p_name in portfolio.poses, err + f'pose {p_name} unknown'
            scene.poses.append(p_name)

            if 'duration' in pose.attrib:
                try:
                    scene.durations.append(float(pose.attrib['duration']))
                except:
                    raise ValueError(err + 'failed to convert duration to float') from None
            else:
                if portfolio.duration:
                    scene.durations.append(portfolio.duration)
                else:
                    raise ValueError(err + 'no duration specified')
            
        return scene


class Script():

    def __init__(self):
        self.name = ''
        self.scenes = []
        self.inverse = []
        self.repeat = []

    @classmethod
    def from_xml(cls, xml_elem, portfolio):

        script = Script()

        err = f'>>> Portfolio {portfolio.name} '
        assert xml_elem.tag == 'script', err + 'only <script> tags should be used in <scripts>'

        assert 'name' in xml_elem.attrib, err + '"name" attribute expected in <script> definition'
        script.name = xml_elem.attrib['name']

        for scene in list(xml_elem):
            
            err = err + f'script {script.name} '
            assert scene.tag == 'scene', err + 'only <scene> tags should be used in <script>'
            
            assert 'name' in scene.attrib, err + '"name" for scene missing'
            s_name = scene.attrib['name']
            assert s_name in portfolio.scenes, err + f'scene {s_name} unknown'
            script.scenes.append(s_name)

            if 'inverse' in scene.attrib:
                if scene.attrib['inverse'] in ['False', 'false']:
                    script.inverse.append(False)
                elif scene.attrib['inverse'] in ['True', 'true']:
                    script.inverse.append(True)
                else:
                    raise ValueError(err + '"inverse" cannot be parse')
            else:
                script.inverse.append(False)
            
            if 'repeat' in scene.attrib:
                try:
                    script.repeat.append(int(scene.attrib['repeat']))
                except:
                    raise ValueError(err + '"repeat" should be an "int"') from None
            else:
                script.repeat.append(1)

        return script


class Portfolio():
    """A portfolio of scripts.

    A Portfolio is a collection of scripts that can share certain
    elements like scenes and poses.

    A portfolio is composed of the following elements:

    

    """
        
    def __init__(self):
        #: the name of the Portfolio
        self.name = 'dummy'

        #: Unit of measurements for positions angles. Only ``rad`` and ``deg``
        #: allowed and by default it will be ``rad`` if no attribute is
        #: specified in the souce XACRO.
        #: :type str
        self.units = 'rad'

        #: The default list of joints to be used across the scripts in this
        #: portfolio. The order of joints is important as all the ``positions``
        #: sppecifications will assume the same order. A portfolio can use
        #: a subset of all the joints of the robot and only the joints
        #: specified here will be passed when constructing the communcation
        #: messages with the robots' controllers.
        #: :type list(str)
        self.joints = []

        #: The default duration (in seconds) for poses in scenes. If scenes
        #: do not specify a ``duration=`` attribute, they will inherit
        #: automatically this duration from the portfolio.
        #: :type float
        self.duration = None

        #: A dictionary of Pose() defined in this portfolio.
        #: type dict{name: Pose}
        self.poses = {}

        #: A dictionary of Scene() defined in this portfolio.
        #: type dict{name: Scene}
        self.scenes = {}

        #: A dictionary of Script() defined in this portfolio.
        #: type dict{name: Script}
        self.scripts = {}


    @classmethod
    def from_xml(cls, xml_elem):
        """Constructs a ``Portfolio`` object by reading an XML defintion
        and parsing it.

        Parameters
        ----------
        xml_elem : xml.etree.ElementTree.ElementTree
            The XML element with the structure of the portfolio.

        Raises
        ------
        ValueError:
            If the data is incorrect. Additional details are provided in 
            the exception text.
        AssertionError:
            If attributes are missing or missmatched with the expected ones. 

        """
        portfolio = Portfolio()
        
        assert 'name' in xml_elem.attrib, '>>> Missing "name" attribute in portfolio defintion'
        portfolio.name = xml_elem.attrib['name']
        
        portfolio.units = xml_elem.attrib.get('units', 'rad')
        # check units
        assert portfolio.units in ['rad', 'deg'], f'>>> Portfolio file {portfolio.name} units should be "rad" or "deg" only'

        if 'joints' in xml_elem.attrib:
            if isinstance(xml_elem.attrib['joints'], str):
                portfolio.joints = xml_elem.attrib['joints'].split()
            else:
                raise ValueError(f'>>> Portfolio {portfolio.name} "joints" should be a string of joint names')

        if 'duration' in xml_elem.attrib:
            try:
                portfolio.duration = float(xml_elem.attrib['duration'])
            except:
                raise ValueError(f'>>> Portfolio {portfolio.name} default duration should be "int" or "float"') from None

        for child in xml_elem:

            if child.tag == 'poses':
                for pose_xml in child:
                    pose = Pose.from_xml(pose_xml, portfolio)
                    portfolio.poses[pose.name] = pose

            if child.tag == 'scenes':
                for scene_xml in child:
                    scene = Scene.from_xml(scene_xml, portfolio)
                    portfolio.scenes[scene.name] = scene

            if child.tag == 'scripts':
                for script_xml in child:
                    script = Script.from_xml(script_xml, portfolio)
                    portfolio.scripts[script.name] = script

        return portfolio


    def get_script_names(self):
        return list(self.scripts.keys())


    def to_joint_trajectory_goal(self, script_name, speed):
        if not script_name in self.scripts:
            return None

        running_duration = 0
        pos = {}

        if speed <= 0.0:
            speed = 1.0
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joints

        for joint in self.joints:
            pos[joint] = 0.0

        if self.units == 'deg':
            factor = 57.295779513
        else:
            factor = 1.0

        script = self.scripts[script_name]
        for scene_name, inverse, repeat in zip(script.scenes, script.inverse, script.repeat):
            # TODO handle inverse

            for _ in range(repeat):
                
                scene = self.scenes[scene_name]
                for pose_name, duration in zip(scene.poses, scene.durations):

                    duration = duration / speed

                    pose = self.poses[pose_name]
                    for joint, position in zip(pose.joints, pose.positions):
                        pos[joint] = position

                    point = JointTrajectoryPoint()
                    point.positions = [pos[joint] / factor for joint in self.joints]
                    point.time_from_start = rospy.Duration.from_sec(running_duration + duration)
                    goal.trajectory.points.append(point)

                    running_duration = running_duration + duration

        return goal
