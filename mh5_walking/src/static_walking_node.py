#!/usr/bin/env python3

import rospy

from walking import StaticWalking


if __name__ == '__main__':

    rospy.init_node('mh5_static_walking')
    walker = StaticWalking()

    rospy.spin()
