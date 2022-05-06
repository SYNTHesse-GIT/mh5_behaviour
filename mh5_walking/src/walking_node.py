#!/usr/bin/env python3

import rospy

from walking_controller import MH5Walking_Controller


if __name__ == "__main__":

    rospy.init_node('test_walking', log_level=rospy.INFO)
    p = {
        'speed': 2.0,
        'prep_th': 0.95,
    }

    controller = MH5Walking_Controller(params=p)
    controller.publish_states()
    rospy.sleep(2)
    controller.prepare_walk()
    use_left = True
    controller.start_walk(with_left=use_left)
    controller.propulsion(to_left=use_left)
    use_left = not use_left
    for _ in range(3):
        controller.step(with_left=use_left)
        controller.propulsion(to_left=use_left)
        use_left = not use_left
    controller.stop_walk(with_left=use_left)
