#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class initPose(Node):
	def __init__(self):
		super().__init__('init_pose')
		global init_msg
		self.pub_ = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
		self.sub_ = self.create_subscription(Odometry, "/odom", self.callback, 10)
		init_msg = PoseWithCovarianceStamped()
		init_msg.header.frame_id = "map"
		
	def callback(self, msg:Odometry):
		init_msg.pose.pose.position.x = msg.pose.pose.position.x
		init_msg.pose.pose.position.y = msg.pose.pose.position.y
		init_msg.pose.pose.orientation.x = msg.pose.pose.orientation.x
		init_msg.pose.pose.orientation.y = msg.pose.pose.orientation.y
		init_msg.pose.pose.orientation.z = msg.pose.pose.orientation.z
		init_msg.pose.pose.orientation.w = msg.pose.pose.orientation.w
		self.get_logger().info("Setting initial pose...")
		self.pub_.publish(init_msg)
		self.get_logger().info("Initial pose is set.")

def main(args=None):
	rclpy.init(args=args)
	node1 = initPose()
	rclpy.spin_once(node1)
	rclpy.shutdown()

if __name__ == '__main__':
	main()