#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct


class RGBToIntensityConverter(Node):
    def __init__(self):
        super().__init__('rgb_to_intensity_converter')

        #self.sub = self.create_subscription(PointCloud2, '/filtered_points', self.cloud_callback, 10)
        self.sub = self.create_subscription(PointCloud2, '/ml_/pointcloud', self.cloud_callback, 10)
        self.pub = self.create_publisher(PointCloud2, 'velodyne_points', 1)

    def cloud_callback(self, cloud_msg):
        try:
            # Get cloud data as structured array
            field_names = [field.name for field in cloud_msg.fields]
            #print(field_names)
            cloud_data = pc2.read_points(cloud_msg, field_names=field_names, skip_nans=True)

            # Convert to numpy array
            np_cloud = np.array(list(cloud_data))
            print("np_cloud.shape", np_cloud.shape)
            # Extract XYZ
            xyz = np_cloud[:, :3]
            print("np_cloud", np_cloud)
            #print("xyz",xyz)


            # Extract RGB and convert to intensity
            if 'rgb' in field_names:
                rgb_index = field_names.index('rgb')
                rgb = np_cloud[:, rgb_index].copy()
                #print("rgb :",rgb)
                #print("xyz.shape",xyz.shape)
                #print("rgb.shape",rgb.shape)
                # RGB 데이터를 바이트 배열로 변환
                rgb_bytes = rgb.view(np.uint8).reshape(-1, 8)  # 64비트(8바이트)로 가정

                # RGB 채널 추출 (마지막 3바이트 사용)
                r = rgb_bytes[:, 5].astype(np.float32)
                g = rgb_bytes[:, 6].astype(np.float32)
                b = rgb_bytes[:, 7].astype(np.float32)
                #print("r g b",r,g,b)
                #print("r.shape",r.shape)
                intensity = ((r.astype(np.float32) + g.astype(np.float32) + b.astype(np.float32)) / 3.0)
            else:
                intensity = np.zeros(len(xyz), dtype=np.float32)

            #print("intensity",intensity)
            #print("intensity.shape",intensity.shape)


            new_points = np.column_stack((xyz, intensity))
            #print("cloud_msg.header",cloud_msg.header)
            # Create new PointCloud2 message
            new_cloud_msg = pc2.create_cloud(cloud_msg.header, [
                PointField(name='x', offset=0, datatype=PointField.FLOAT64, count=1),
                PointField(name='y', offset=4, datatype=PointField.FLOAT64, count=1),
                PointField(name='z', offset=8, datatype=PointField.FLOAT64, count=1),
                PointField(name='intensity', offset=12, datatype=PointField.FLOAT64, count=1)
            ], new_points)
            new_cloud_msg.is_dense = True
            print("cloud_msg.point_step", cloud_msg.point_step)
            print("cloud_msg.is_dense", cloud_msg.is_dense)

            print("new cloud_msg.point_step", new_cloud_msg.point_step)
            print("new_cloud_msg.is_dense", new_cloud_msg.is_dense)
            #print("header.frame_id",header.frame_id)
            # Publish the new point cloud
            self.pub.publish(new_cloud_msg)
            self.get_logger().info("Successfully converted and published point cloud in Velodyne format")

        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")


def main(args=None):
    rclpy.init(args=args)
    converter = RGBToIntensityConverter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
