#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Pose ,Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from unitree_legged_msgs.msg import HighState
from tf.transformations import quaternion_from_euler
from math import cos, sin
import tf
from sensor_msgs.msg import JointState

class Odometry3DOF():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = 0
        # Initialize the node and publisher
        rospy.init_node("odometry_node")
        self.pub = rospy.Publisher("odom", Odometry, queue_size=10)
        rospy.Subscriber('high_state', HighState, self.callback)
        self.odom_broadcaster = tf.TransformBroadcaster()

        
        self.joint_names = ['FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint', 'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint', 'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint']
        self.joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def callback(self, data):
    	# Get joint state 
        self.joint_state = [data.motorState[0].q, data.motorState[1].q, data.motorState[2].q, data.motorState[3].q, data.motorState[4].q, data.motorState[5].q, data.motorState[6].q, data.motorState[7].q, data.motorState[8].q, data.motorState[9].q, data.motorState[10].q, data.motorState[11].q]
        # Get the current time
        current_time = rospy.Time.now()
	
        # Calculate the time step dt
        if self.last_time == 0:
            dt = 0
        else:
            dt = (current_time - self.last_time).to_sec()

        # Get the linear velocity in x and y direction
        vx = data.velocity[0]
        vy = data.velocity[1]

        # Get the angular velocity around the z-axis
        vth = data.yawSpeed

        # Calculate the change in x, y, and th
        delta_x = (vx * cos(self.th) - vy * sin(self.th)) * dt
        delta_y = (vx * sin(self.th) + vy * cos(self.th)) * dt
        delta_th = vth * dt

        # Update the position and orientation
        self.x = self.x + delta_x
        self.y = self.y + delta_y
        self.th = self.th + delta_th
        
        odom_quat = quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        #set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))
        #set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))
        self.pub.publish(odom)

        # Update the last time
        self.last_time = current_time
        
        #update joint state
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_state
        self.publisher.publish(joint_state)

if __name__ == "__main__":
    try:
        odom = Odometry3DOF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass