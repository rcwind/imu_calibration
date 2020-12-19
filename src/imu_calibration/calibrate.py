#! /usr/bin/python

from __future__ import with_statement

import roslib; roslib.load_manifest('imu_calibration')
import yaml
import PyKDL
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from imu_calibration.msg import ScanAngle
from math import *
import threading
import dynamic_reconfigure.client
import os
import subprocess
import yaml

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0*pi
    while res < -pi:
        res += 2.0*pi
    return res


class CalibrateRobot:
    def __init__(self):
        self.lock = threading.Lock()

        self.sub_imu  = rospy.Subscriber('imu', Imu, self.imu_cb)

        self.sub_odom = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.sub_scan = rospy.Subscriber('scan_angle', ScanAngle, self.scan_cb)
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.imu_time = rospy.Time()
        self.odom_time = rospy.Time()
        self.scan_time = rospy.Time()
        
        # params
        self.inital_wall_angle = rospy.get_param("inital_wall_angle", 0.1)
        self.imu_calibrate_time = rospy.get_param("imu_calibrate_time", 10.0)
        self.imu_angle = 0
        self.imu_time = rospy.Time.now()
        self.scan_angle = 0
        self.scan_time = rospy.Time.now()
        self.odom_angle = 0
        self.odom_time = rospy.Time.now()

    def calibrate(self, speed, imu_drift=0):
        # rotate 360 degrees
        (imu_start_angle, odom_start_angle, scan_start_angle, 
         imu_start_time, odom_start_time, scan_start_time) = self.sync_timestamps()
        last_angle = odom_start_angle
        turn_angle = 0
        rospy.loginfo(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
        rospy.loginfo("rotation speed = %f(%fdeg/s)"%(speed, speed*180/pi))
        while turn_angle < 2*pi:
            if rospy.is_shutdown():
                return
            cmd = Twist()
            cmd.angular.z = speed
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
            with self.lock:
                delta_angle = normalize_angle(self.odom_angle - last_angle)
            turn_angle += delta_angle
            last_angle = self.odom_angle
        self.cmd_pub.publish(Twist())

        rospy.sleep(3)
        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time) = self.sync_timestamps()

        scan_delta = 2*pi + normalize_angle(scan_end_angle - scan_start_angle)

        imu_delta = 2*pi + normalize_angle(imu_end_angle - imu_start_angle) - imu_drift*(imu_end_time - imu_start_time).to_sec()
        odom_delta = 2*pi + normalize_angle(odom_end_angle - odom_start_angle)

        imu_result = imu_delta/scan_delta
        odom_result = odom_delta/scan_delta

        rospy.loginfo('Imu error: %f%%'%(100.0*(imu_result-1.0)))
        rospy.loginfo('Odom error: %f%%'%(100.0*(odom_result-1.0)))

        return (imu_result, odom_result)

    def imu_drift(self):
        # estimate imu drift
        rospy.loginfo('Estimating imu drift')
        (imu_start_angle, odom_start_angle, scan_start_angle, 
         imu_start_time, odom_start_time, scan_start_time) = self.sync_timestamps()
        rospy.sleep(self.imu_calibrate_time)
        (imu_end_angle, odom_end_angle, scan_end_angle,
         imu_end_time, odom_end_time, scan_end_time) = self.sync_timestamps()

        imu_drift = normalize_angle(imu_end_angle - imu_start_angle) / ((imu_end_time - imu_start_time).to_sec())
        rospy.loginfo(' ... imu drift is %f deg/s'%(imu_drift*180.0/pi))
        return imu_drift


    def align(self):
        self.sync_timestamps()
        rospy.loginfo("Aligning base with wall")
        with self.lock:
            angle = self.scan_angle
        cmd = Twist()

        rospy.loginfo("wall angle = %f(%fdeg)"%(angle, angle*180/pi))
        while (angle - pi/2) < -self.inital_wall_angle or (angle - pi/2) > self.inital_wall_angle:
            if rospy.is_shutdown():
                exit(0)
            if angle > 0:
                cmd.angular.z = -0.1
            else:
                cmd.angular.z = 0.1
            self.cmd_pub.publish(cmd)
            rospy.sleep(0.1)
            with self.lock:
                angle = self.scan_angle

    def sync_timestamps(self, start_time=None):
        if not start_time:
            start_time = rospy.Time.now() + rospy.Duration(1)
        while not rospy.is_shutdown():
            rospy.sleep(0.2)
            with self.lock:
                if self.imu_time < start_time :
                    rospy.loginfo("Still waiting for imu")
                elif self.odom_time < start_time:
                    rospy.loginfo("Still waiting for odom")
                elif self.scan_time < start_time:
                    rospy.loginfo("Still waiting for scan")
                else:
                    return (self.imu_angle, self.odom_angle, self.scan_angle,
                            self.imu_time, self.odom_time, self.scan_time)
        exit(0)
        

    def imu_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.orientation)
            self.imu_angle = angle
            self.imu_time = msg.header.stamp

    def odom_cb(self, msg):
        with self.lock:
            angle = quat_to_angle(msg.pose.pose.orientation)
            self.odom_angle = angle
            self.odom_time = msg.header.stamp

    def scan_cb(self, msg):
        with self.lock:
            angle = msg.scan_angle
            self.scan_angle = angle
            self.scan_time = msg.header.stamp

    def scale_filt(data_list):
        if len(data_list)==0:
            return data_list
        if len(data_list)>2:
            for scale in data_list:
                if (scale - 1.0) > 0.1 or (scale - 1.0) < -0.1:
                    rospy.logwarn("drop scale value %f"%scale)
                    data_list.remove(scale)
            return data_list
        elif len(data_list)<=2:
            return data_list

    def scale_average(data_list):
        if len(data_list)==0:
            return 0
        if len(data_list)>2:
            rospy.logwarn("drop min scale value %f"%min(data_list))
            rospy.logwarn("drop max scale value %f"%max(data_list))
            data_list.remove(min(data_list))
            data_list.remove(max(data_list))
            average_data = sum(data_list)/len(data_list)
            return average_data
        elif len(data_list)<=2:
            average_data = sum(data_list)/len(data_list)
            return average_data

def main():
    rospy.init_node('scan_to_angle')
    robot = CalibrateRobot()
    imu_res = 1.0

    imu_drift = robot.imu_drift()
    imu_corr = []
    odom_corr = []
    for speed in (0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4):
        robot.align()
        rospy.sleep(3)
        (imu, odom) = robot.calibrate(speed, imu_drift)
        rospy.loginfo("gyro_scale_correction = %f"%imu)
        rospy.loginfo("odom_angular_scale_correction = %f"%odom)
        rospy.loginfo("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<")
        if imu:
            imu_corr.append(imu)
        odom_corr.append(odom)

    imu_corr = robot.scale_filt(imu_corr)
    #  odom_corr = robot.filt(odom_corr)

    if  len(imu_corr > 0):
        imu_res = sum(imu_corr)/len(imu_corr)
        #  imu_res = robot.scale_average(imu_corr) 
        rospy.loginfo("final gyro_scale_correction parameter is %f"%imu_res)
    else:
        rospy.logerr("no matched gyro_scale_correction parameter, please check the imu")

    odom_res = sum(odom_corr)/len(odom_corr)
    #  odom_res = robot.scale_average(odom_corr) 
    rospy.loginfo("final odom_angular_scale_correction parameter is %f"%odom_res)

if __name__ == '__main__':
    main()

