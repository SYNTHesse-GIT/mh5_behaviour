#!/usr/bin/env python3

import os
import rospy
import rospkg



from director import Director


if __name__ == '__main__':

    rospy.init_node('mh5_director')

    director = Director()

    director.load_scripts()
    director.setup_services()
    director.setup_action_client()

    # main loop
    rospy.loginfo('[mh5_director] wating for orders')
    rospy.spin()
