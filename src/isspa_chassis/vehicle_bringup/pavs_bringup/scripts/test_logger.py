#!/usr/bin/env python

import rospy
from logging_output import logger

if __name__ == '__main__':
    rospy.init_node('test_logger')
    rospy.loginfo('test logger')
    logger.info('test logger')
    logger.fatal("this is fatal")
    logger.error("this is error")
    print('test logger')
    rospy.spin()
