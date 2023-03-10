#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from orb_slam_msgs.srv import ScaleFactor

from nav_msgs.msg import Odometry
from orb_slam_msgs.msg import TrackResult

class MonoScale(Node):
    def __init__(self):
        super().__init__("mono_scale")
        self.nodename = "mono_scale"
        self.get_logger().info(f"{self.nodename} started")
        self.check_distance = float(self.declare_parameter('check_distance', 0.5).value)

        # subscriptions
        self.create_subscription(TrackResult, "mono/track", self.track_result_callback, 1)
        self.create_subscription(Odometry, "odom", self.odom_callback, 1)
        self.cli = self.create_client(ScaleFactor, 'set_scale_factor')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.scale_factor_req = ScaleFactor.Request()

        self.tcw_start = False
        self.tcw_pose_last = 0
        self.curr_map_id = 0
        self.odom_start = False
        self.checking = True

    def track_result_callback(self, msg):
        if not self.tcw_start:
            self.get_logger().info("track_result_start")
            self.tcw_pos0 = msg.pose.position
            self.curr_map_id = msg.map_id
            self.tcw_start = True
        else:
            self.tcw_pos_last = msg.pose.position
            self.curr_map_id = msg.map_id

    def distance(self, p1, p2):
        d = np.square(p1.x - p2.x) + np.square(p1.y - p2.y) + np.square(p1.z - p2.z)
        d = np.sqrt(d)
        return d

    def send_request(self, d):
        self.scale_factor_req.scale_factor = d
        self.future = self.cli.call_async(self.scale_factor_req)
        # rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def odom_callback(self, msg):
        if self.odom_start:
            p = msg.pose.pose.position
            od = self.distance(p, self.odom0)
            # self.get_logger().info(f"distance: {od}, odom.x: {p.x}")
            if od > self.check_distance:
                self.tcw_start = False
                self.tcw_pose_last = 0
                self.curr_map_id = 0
                self.odom_start = False
                self.checking = True
                twc_d = self.distance(self.tcw_pos0, self.tcw_pos_last)
                self.get_logger().info(f"odom: {od}, tcw: {twc_d}, rate: {od/twc_d}")
                res = self.send_request(od/twc_d)
                self.get_logger().info(f"response: {res}")
        elif self.tcw_start:
            self.odom_start = True
            self.odom0 = msg.pose.pose.position

def main(args=None):
    rclpy.init(args=args)
    try:
        mono_scale = MonoScale()
        rclpy.spin(mono_scale)
    except rclpy.exceptions.ROSInterruptException:
        pass

    mono_scale.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
