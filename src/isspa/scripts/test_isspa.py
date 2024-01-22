#! /usr/bin/env python

import rospy
from isspa.logging_output import logger

if __name__ == '__main__':
    
    rospy.init_node('test_python_library')
    logger.info("This is an info message")
    logger.warning("This is a warning message")

    while not rospy.is_shutdown():
        rospy.sleep(1)

    