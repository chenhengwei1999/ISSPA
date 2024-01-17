#!/usr/bin/env python

import rospy
from logging_output import logger

if __name__ == '__main__':
    rospy.init_node('test_logger')
    rospy.loginfo('test logger')

    logger.info('this is info')
    logger.error("this is error")
    logger.warning("this is warning")
    logger.debug("this is debug")
    logger.critical("this is critical")

    print('test logger')
    rospy.spin()
