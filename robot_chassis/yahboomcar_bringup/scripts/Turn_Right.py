#!/usr/bin/env python2
# coding:utf-8
import math
import tf
import rospy
import numpy as np
from time import sleep
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from math import radians, copysign, sqrt, pow
from geometry_msgs.msg import Twist, Point, Quaternion
from transform_utils import quat_to_angle, normalize_angle
from yahboomcar_bringup.cfg import PatrolParamConfig
#from dynamic_reconfigure.server import Server
#import dynamic_reconfigure.client
RAD2DEG = 180 / math.pi

class YahboomCarPatrol():
    def __init__(self):
        #rospy.on_shutdown(self.cancel)
        #self.r = rospy.Rate(20)
        self.moving = True
        self.Joy_active = False
        self.command_src = "finish"
        self.warning = 1
        self.SetLoop = False
        self.Linear = 0.5
        self.Angular = 1.0
        self.Length = 0.2 #1.0
        self.Angle = 360.0
        self.LineScaling = 1.1
        self.RotationScaling = 0.75
        self.LineTolerance = 0.1
        self.RotationTolerance = 0.3
        self.ResponseDist = 0.6
        self.LaserAngle = 20
        self.Command = "Square"
        self.circle_adjust = rospy.get_param('~circle_adjust', 2.0)
        self.Switch = False
        self.tf_listener = tf.TransformListener()
        self.odom_frame = rospy.get_param('~odom_frame', '/odom')
        self.base_frame = rospy.get_param('~base_frame', '/base_footprint')
        #Server(PatrolParamConfig, self.dynamic_reconfigure_callback)
        self.pub_cmdVel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        #self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.registerScan, queue_size=1)
        #self.sub_Joy = rospy.Subscriber('/JoyState', Bool, self.JoyStateCallback)
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(60.0))
        #self.dyn_client = dynamic_reconfigure.client.Client("YahboomCarPatrol", timeout=60)
        #rospy.loginfo("Bring up rqt_reconfigure to control the Robot.")
        print(1111)
        
    def Turn(self):
        #sleep(0.5)
        advancing = self.advancing(0.75)
        sleep(0.1)#0.5
        print ("advancing: ", advancing)
        print("rrr")
        self.Spin(-90)
        print("ddddd")
        sleep(0.1)
        self.advancing(0.1)
        self.Command = "finish"
        return

    def advancing(self, target_distance):
        position = self.get_position()
        x_start, y_start = position.x, position.y
        # print ("x_start: {}, y_start: {}".format(x_start, y_start))
        while not rospy.is_shutdown():
            #print("ddd")
            #self.r.sleep()
            move_cmd = Twist()
            position = self.get_position()
            
            # Compute the Euclidean distance from the target point
            distance = sqrt(pow((position.x - x_start), 2) +
                            pow((position.y - y_start), 2))
            # rospy.loginfo(position)
            distance *= self.LineScaling
            # How close are we?
            error = distance - target_distance
            # print ("advancing target_distance: {},distance: {},error: {}".format(target_distance, distance,error))
            # If not, move in the appropriate direction
            move_cmd.linear.x = self.Linear
            # move_cmd.linear.x = copysign(self.Linear, -1 * error)
            if abs(error) < self.LineTolerance : return True
            if self.Joy_active:
                if self.moving == True:
                    self.pub_cmdVel.publish(Twist())
                    self.moving = False
                continue
            else: self.pub_cmdVel.publish(move_cmd)
            self.moving = True
        return False
    
    def Spin(self, angle):
        
        target_angle = radians(angle)
        odom_angle = self.get_odom_angle()
        last_angle = odom_angle
        turn_angle = 0
        # Alternate directions between tests
        while not rospy.is_shutdown():
            #self.r.sleep()
            # Get the current rotation angle from tf
            
            odom_angle = self.get_odom_angle()
            # Compute how far we have gone since the last measurement
            delta_angle = self.RotationScaling * normalize_angle(odom_angle - last_angle)
            # Add to our total angle so far
            turn_angle += delta_angle
            # Compute the new error
            error = target_angle - turn_angle
            # Store the current angle for the next comparison
            last_angle = odom_angle
            # print("Spin target_angle: {},turn_angle: {},error: {}".format(target_angle, turn_angle, abs(error)))
            move_cmd = Twist()
            if abs(error) < self.RotationTolerance:
                self.pub_cmdVel.publish(Twist())
                return True
            if self.Joy_active :
                if self.moving == True:
                    self.pub_cmdVel.publish(Twist())
                    self.moving = False
                    
                continue
            else:
                
                if self.Command == "Circle":
                    length = self.Linear * self.circle_adjust / self.Length
                    move_cmd.linear.x = self.Linear
                    move_cmd.angular.z = copysign(length, error)
                elif self.Command == "Square":
                    length = self.Linear * self.circle_adjust / self.Length
                    move_cmd.linear.x = self.Linear
                    move_cmd.angular.z = copysign(length, error)
                else:
                    print("cccc")
                    move_cmd.angular.z = copysign(self.Angular, error)
                self.pub_cmdVel.publish(move_cmd)
            self.moving = True
        # Stop the robot
        self.pub_cmdVel.publish(Twist())
        
        return True

    def get_odom_angle(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        # Convert the rotation from a quaternion to an Euler angle
        return quat_to_angle(Quaternion(*rot))
    
    def get_position(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        return Point(*trans)
    
    def cancel(self):
        # Always stop the robot when shutting down the node
        rospy.loginfo("Stopping the robot...")
        #self.pub_cmdVel.publish(Twist())
        #self.pub_cmdVel.unregister()
        #self.sub_scan.unregister()
        #self.sub_Joy.unregister()
        #self.tf_listener.clear()
        rospy.sleep(1)
        
    def process(self):
        print(11)
        while not rospy.is_shutdown():
            self.Turn()
            print(0)
            
            