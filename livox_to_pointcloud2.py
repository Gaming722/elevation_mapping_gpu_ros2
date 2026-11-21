#!/usr/bin/env python3
"""Publish MID-360 scans as PointCloud2 in the original LiDAR frame."""

import argparse
import math
import struct

import rclpy
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster


def convert(scan):
    cloud = PointCloud2()
    cloud.header = scan.header
    cloud.height = 1
    cloud.width = len(scan.points)
    cloud.fields = [
        PointField(name=name, offset=4 * index, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z", "intensity"))
    ]
    point = struct.Struct("<ffff")
    data = bytearray(point.size * cloud.width)
    for index, sample in enumerate(scan.points):
        point.pack_into(data, index * point.size, sample.x, sample.y, sample.z, sample.reflectivity)
    cloud.point_step = point.size
    cloud.row_step = len(data)
    cloud.data = data
    return cloud


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--lidar-in-imu", type=float, nargs=7,
        metavar=("TX", "TY", "TZ", "QX", "QY", "QZ", "QW"),
        help="Calibrated pose of the LiDAR in lidar_base, in meters and quaternion xyzw. "
             "Provide only if this static TF is not published elsewhere.",
    )
    parser.add_argument("--input", default="/livox/lidar")
    parser.add_argument("--output", default="/elevation_input/livox_points")
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    if args.lidar_in_imu:
        if not all(math.isfinite(value) for value in args.lidar_in_imu):
            parser.error("the extrinsic must contain finite values")
        if not math.isclose(math.hypot(*args.lidar_in_imu[3:]), 1.0, abs_tol=1e-6):
            parser.error("the extrinsic quaternion must be normalized")

    rclpy.init()
    node = rclpy.create_node("elevation_livox_bridge")
    publisher = node.create_publisher(PointCloud2, args.output, qos_profile_sensor_data)
    broadcaster = StaticTransformBroadcaster(node) if args.lidar_in_imu else None
    lidar_frame = None

    def on_scan(scan):
        nonlocal lidar_frame
        frame = scan.header.frame_id
        if not frame or not scan.points:
            return
        if broadcaster and frame != lidar_frame:
            if lidar_frame is not None or frame == "lidar_base":
                node.get_logger().error(f"Unexpected LiDAR frame {frame!r}", throttle_duration_sec=5)
                return
            transform = TransformStamped()
            transform.header.stamp = node.get_clock().now().to_msg()
            transform.header.frame_id = "lidar_base"
            transform.child_frame_id = frame
            t = transform.transform.translation
            q = transform.transform.rotation
            t.x, t.y, t.z, q.x, q.y, q.z, q.w = args.lidar_in_imu
            broadcaster.sendTransform(transform)
            lidar_frame = frame
        publisher.publish(convert(scan))

    subscription = node.create_subscription(CustomMsg, args.input, on_scan, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
