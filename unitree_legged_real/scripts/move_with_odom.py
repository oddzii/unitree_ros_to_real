#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class move_with_odom():

    def __init__(self):
        rospy.init_node('move_with_odom', anonymous=True)
        self.cmd_vel = Twist()
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size= 1)

    def Move_Odom(self,meter): 
        self.move_dis_init = rospy.wait_for_message('/moved_distance', Float64)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.move_dist = rospy.wait_for_message('/moved_distance', Float64)
            self.meter = abs(meter)
            if self.move_dist.data - self.move_dis_init.data  < self.meter:
                if meter > 0:
                    self.cmd_vel.linear.x = 0.2
                else:
                    self.cmd_vel.linear.x = -0.2
            else:
                self.cmd_vel.linear.x = 0
                self.pub.publish(self.cmd_vel)
                rospy.loginfo("Move Done")
                break
            self.pub.publish(self.cmd_vel)
            print("dist Move: {} meter".format(self.move_dist.data - self.move_dis_init.data))
            r.sleep()

if __name__ == '__main__':
    try:
        move = move_with_odom()
        move.Move_Odom(-0.9)
    except rospy.ROSInterruptException:
        pass
