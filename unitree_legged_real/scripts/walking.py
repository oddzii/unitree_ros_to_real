#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def publish_cmd_vel():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('publish_cmd_vel', anonymous=True)
    rate = rospy.Rate(10) # 10 Hz
    while not rospy.is_shutdown():
        linear_velocity_x, linear_velocity_y, angular_velocity = map(float, input("Enter linear velocities along x and y-axis and angular velocity (m/s and rad/s): ").split())
        linear_velocity_x = max(min(linear_velocity_x, 0.5), -0.5)
        linear_velocity_y = max(min(linear_velocity_y, 0.5), -0.5)
        angular_velocity = max(min(angular_velocity, 1), -1)
        velocity_msg = Twist()
        velocity_msg.linear.x = linear_velocity_x
        velocity_msg.linear.y = linear_velocity_y
        velocity_msg.angular.z = angular_velocity
        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_cmd_vel()
    except rospy.ROSInterruptException:
        pass
