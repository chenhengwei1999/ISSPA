#!/usr/bin/env python
# encoding: utf-8
import sys
import math
import rospy
import random
import threading
from math import pi
from time import sleep
from sensor_msgs.msg import Imu, MagneticField, JointState
from stm32_com import PAVS
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float32, Int32, Bool
from dynamic_reconfigure.server import Server
from pavs_bringup.cfg import PIDparamConfig

from logging_output import logger

car_type_dic={
    'R2':5,
    'X3':1,
    'NONE':-1
}


class pavs_driver:
    def __init__(self):
        global car_type_dic
        rospy.on_shutdown(self.cancel)
        # 弧度转角度
        # Radians turn angle
        self.RA2DE = 180 / pi
        self.car = PAVS()
        self.car_type = rospy.get_param("~car_type","R2")#default R2
        
        self.car.set_car_type(5)
        self.imu_link = rospy.get_param("~imu_link", "imu_link")
        self.Prefix = rospy.get_param("~prefix", "")
        self.xlinear_limit = rospy.get_param('~xlinear_speed_limit', 1.0)
        self.ylinear_limit = rospy.get_param('~ylinear_speed_limit', 1.0)
        self.angular_limit = rospy.get_param('~angular_speed_limit', 1.0)
        self.nav_use_rotvel = rospy.get_param('~nav_use_rotvel', False)

        self.sub_cmd_vel = rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback, queue_size=1)
        self.sub_RGBLight = rospy.Subscriber("RGBLight", Int32, self.RGBLightcallback, queue_size=100)
        self.sub_Buzzer = rospy.Subscriber("Buzzer", Bool, self.Buzzercallback, queue_size=100)
        
        self.EdiPublisher = rospy.Publisher('edition', Float32, queue_size=100)
        self.volPublisher = rospy.Publisher('voltage', Float32, queue_size=100)
        self.staPublisher = rospy.Publisher('joint_states', JointState, queue_size=100)
        self.velPublisher = rospy.Publisher("/pub_vel", Twist, queue_size=100)
        self.imuPublisher = rospy.Publisher("/pub_imu", Imu, queue_size=100)
        self.magPublisher = rospy.Publisher("/pub_mag", MagneticField, queue_size=100)
        self.dyn_server = Server(PIDparamConfig, self.dynamic_reconfigure_callback)
        self.car.create_receive_threading()
        self.rate = rospy.Rate(10)

    def cancel(self):
        self.velPublisher.unregister()
        self.imuPublisher.unregister()
        self.EdiPublisher.unregister()
        self.volPublisher.unregister()
        self.staPublisher.unregister()
        self.magPublisher.unregister()
        self.sub_cmd_vel.unregister()
        self.sub_RGBLight.unregister()
        self.sub_Buzzer.unregister()
        # Always stop the robot when shutting down the node
        rospy.loginfo("Close the robot...")
        rospy.sleep(1)

    def pub_data(self):
        # 发布小车运动速度、陀螺仪数据、电池电压
        ## Publish the speed of the car, gyroscope data, and battery voltage
        while not rospy.is_shutdown():
            #sleep(0.1)
            imu = Imu()
            twist = Twist()
            battery = Float32()
            edition = Float32()
            mag = MagneticField()
            state = JointState()
            state.header.stamp = rospy.Time.now()
            state.header.frame_id = "joint_states"
            if len(self.Prefix)==0:
                state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
                              "front_right_steer_joint", "front_right_wheel_joint"]
            else:
                state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
                              self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
            edition.data = self.car.get_version()
            battery.data = self.car.get_battery_voltage()
            ax, ay, az = self.car.get_accelerometer_data()
            gx, gy, gz = self.car.get_gyroscope_data()
            mx, my, mz = self.car.get_magnetometer_data()
            vx, vy, angular = self.car.get_motion_data()
            
            # 发布陀螺仪的数据
            # Publish gyroscope data
            imu.header.stamp = rospy.Time.now()
            imu.header.frame_id = self.imu_link
            imu.linear_acceleration.x = ax
            imu.linear_acceleration.y = ay
            imu.linear_acceleration.z = az
            imu.angular_velocity.x = gx
            imu.angular_velocity.y = gy
            imu.angular_velocity.z = gz
            mag.header.stamp = rospy.Time.now()
            mag.header.frame_id = self.imu_link
            mag.magnetic_field.x = mx
            mag.magnetic_field.y = my
            mag.magnetic_field.z = mz
            # 将小车当前的线速度和角速度发布出去
            # Publish the current linear vel and angular vel of the car
            twist.linear.x = vx    #velocity in axis 
            #twist.linear.y = vy*1000   #steer angle
            twist.linear.y = vy   #steer angle
            #twist.angular.z = angular
            twist.linear.z = angular    #this is invalued
            self.velPublisher.publish(twist)
            # print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
            # print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
            # print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
            # rospy.loginfo("battery: {}".format(battery))
            # rospy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
            self.imuPublisher.publish(imu)
            self.magPublisher.publish(mag)
            self.volPublisher.publish(battery)
            self.EdiPublisher.publish(edition)
            
            self.rate.sleep()
            #turn to radis
            steer_radis = vy*1000.0*3.1416/180.0
            state.position = [0, 0, steer_radis, 0, steer_radis, 0]
            if not vx == angular == 0:
                i = random.uniform(-3.14, 3.14)
                state.position = [i, i, steer_radis, i, steer_radis, i]
            self.staPublisher.publish(state)

    def RGBLightcallback(self, msg):
        # 流水灯控制，服务端回调函数 RGBLight control
        '''
        effect=[0, 6]，0：停止灯效，1：流水灯，2：跑马灯，3：呼吸灯，4：渐变灯，5：星光点点，6：电量显示
        speed=[1, 10]，数值越小速度变化越快。
        '''
        if not isinstance(msg, Int32): return
        # print ("RGBLight: ", msg.data)
        for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        # 蜂鸣器控制  Buzzer control
        if not isinstance(msg, Bool): return
        # print ("Buzzer: ", msg.data)
        if msg.data:
            for i in range(3): self.car.set_beep(1)
        else:
            for i in range(3): self.car.set_beep(0)

    def cmd_vel_callback(self, msg):
        # 小车运动控制，订阅者回调函数
        # Car motion control, subscriber callback function
        if not isinstance(msg, Twist): return
        # 下发线速度和角速度
        # Issue linear vel and angular vel
        vx = msg.linear.x
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
        vy = msg.linear.y
        angular = msg.angular.z     # wait for change

        if vx < 0:
            logger.debug("vx is negative")
            logger.debug("vx: {}, vy: {}, angular: {}".format(vx, vy, angular))
            angular = -angular
            logger.debug("angular has been changed to: {}".format(angular))

        # 小车运动控制,vel: ±1, angular: ±5
        # Trolley motion control,vel=[-1, 1], angular=[-5, 5]
        # rospy.loginfo("cmd_velx: {}, cmd_vely: {}, cmd_ang: {}".format(vx, vy, angular))
        '''if self.nav_use_rotvel == True:
            #in navigation, we use angular info to instead steer rot
            vy = angular/1000.0/3.1416*180.0  #Radian system
            angular = 0.0
        else:
            vy = 0.0'''
        #self.car.set_car_motion(vx*0.8, vy, angular)
        self.car.set_car_motion(vx*0.9, vy, angular)
        #rospy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)

    def dynamic_reconfigure_callback(self, config, level):
        # self.car.set_pid_param(config['Kp'], config['Ki'], config['Kd'])
        # print("PID: ", config['Kp'], config['Ki'], config['Kd'])
        self.linear_max = config['linear_max']
        self.linear_min = config['linear_min']
        self.angular_max = config['angular_max']
        self.angular_min = config['angular_min']
        return config

if __name__ == '__main__':
    rospy.init_node("driver_node", anonymous=False)
    try:
        logger.info("Welcome to the PAVS experimental platform!")
        logger.info("PAVS's Chassis is Opening...")
        driver = pavs_driver()
        driver.pub_data()
        rospy.spin()
    except:
        rospy.loginfo("Final!!!")
