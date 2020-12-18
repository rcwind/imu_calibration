#! /usr/bin/python

from __future__ import with_statement

import roslib; roslib.load_manifest('imu_calibration')
import yaml
import rospy
from sensor_msgs.msg import LaserScan
from imu_calibration.msg import ScanAngle
from math import *


class ScanToAngle:
    def __init__(self):
        self.min_angle = rospy.get_param('min_angle', -0.3)
        self.max_angle = rospy.get_param('max_angle', 0.3)
        self.pub = rospy.Publisher('scan_angle', ScanAngle)
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_cb)


    def scan_cb(self, msg):
        angle = msg.angle_min
        d_angle = msg.angle_increment
        sum_x = 0
        sum_y = 0
        sum_xx = 0
        sum_xy = 0
        num = 0
        for r in msg.ranges:
            if angle > self.min_angle and angle < self.max_angle and r < msg.range_max:
                x = sin(angle) * r
                y = cos(angle) * r
                sum_x += x
                sum_y += y
                sum_xx += x*x
                sum_xy += x*y
                num += 1
            angle += d_angle
        if num > 0:
            angle=atan2((-sum_x*sum_y+num*sum_xy)/(num*sum_xx-sum_x*sum_x), 1)
            res = ScanAngle()
            res.header = msg.header
            res.scan_angle = angle
            self.pub.publish(res)
        else:
            rospy.logerr("Please point me at a wall.")

def main():
    rospy.init_node('scan_to_angle')
    s = ScanToAngle()
    rospy.spin()


if __name__ == '__main__':
    main()
