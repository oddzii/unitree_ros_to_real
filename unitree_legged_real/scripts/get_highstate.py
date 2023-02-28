#!/usr/bin/env python3
import rospy
from unitree_legged_msgs.msg import HighState
import math

class GetHighState():
    def __init__(self):
        rospy.init_node('Gat_High_state', anonymous=True)
        self.sub = rospy.Subscriber('high_state', HighState, self.callback)
        self.data = []
        self.velocity_x = 0
        self.velocity_y = 0
        self.yawspeed = 0
        self.yawangle = 0
        self.motorState = 0 
    def callback(self,msg):
        self.data = msg
        self.velocity_x = self.data.velocity[0]
        self.velocity_y = self.data.velocity[1]
        self.motorState = self.data.motorState[17]
        self.yawspeed = self.data.yawSpeed
        self.yawangle = self.data.imu.rpy[2]

    def get_value(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            print(self.motorState)
            # print(self.yawangle*180/math.pi)
            r.sleep()

if __name__ == '__main__':
    try:
        task = GetHighState()
        task.get_value()
    except rospy.ROSInterruptException:
        pass
