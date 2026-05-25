#!/usr/bin/env python3
"""Quick script to analyze z-values in /utlidar/cloud"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import sys

class CloudChecker(Node):
    def __init__(self):
        super().__init__('cloud_checker')
        self.sub = self.create_subscription(PointCloud2, '/utlidar/cloud', self.cb, 10)
        self.done = False

    def cb(self, msg):
        if self.done:
            return
        self.done = True
        
        # Parse point cloud to get z values
        z_vals = []
        x_vals = []
        y_vals = []
        
        # Find x, y, z field offsets
        x_off = y_off = z_off = -1
        for f in msg.fields:
            if f.name == 'x': x_off = f.offset
            elif f.name == 'y': y_off = f.offset
            elif f.name == 'z': z_off = f.offset
        
        if z_off < 0:
            print("No z field found!")
            print("Fields:", [(f.name, f.offset, f.datatype) for f in msg.fields])
            return
        
        print(f"frame_id: {msg.header.frame_id}")
        print(f"num points: {msg.width * msg.height}")
        print(f"fields: {[(f.name, f.offset, f.datatype) for f in msg.fields]}")
        print(f"point_step: {msg.point_step}")
        
        for i in range(msg.width * msg.height):
            off = i * msg.point_step
            x = struct.unpack_from('f', msg.data, off + x_off)[0]
            y = struct.unpack_from('f', msg.data, off + y_off)[0]
            z = struct.unpack_from('f', msg.data, off + z_off)[0]
            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)
        
        z_vals.sort()
        n = len(z_vals)
        
        print(f"\n=== Z value statistics (in utlidar_lidar frame) ===")
        print(f"  min:    {z_vals[0]:.4f}")
        print(f"  5%:     {z_vals[int(n*0.05)]:.4f}")
        print(f"  25%:    {z_vals[int(n*0.25)]:.4f}")
        print(f"  median: {z_vals[int(n*0.5)]:.4f}")
        print(f"  75%:    {z_vals[int(n*0.75)]:.4f}")
        print(f"  95%:    {z_vals[int(n*0.95)]:.4f}")
        print(f"  max:    {z_vals[-1]:.4f}")
        
        # Histogram
        print(f"\n=== Z value histogram ===")
        bins = {}
        for z in z_vals:
            b = round(z * 10) / 10  # 0.1m bins
            bins[b] = bins.get(b, 0) + 1
        for b in sorted(bins.keys()):
            bar = '#' * min(bins[b] // 5, 60)
            print(f"  z={b:+.1f}m: {bins[b]:5d} {bar}")
        
        print(f"\n=== Ground detection check ===")
        ground = [z for z in z_vals if -0.5 <= z <= -0.05]
        obstacle = [z for z in z_vals if -0.05 < z <= 0.5]
        other = [z for z in z_vals if z < -0.5 or z > 0.5]
        print(f"  Ground [-0.5, -0.05]:  {len(ground)} points ({100*len(ground)/n:.1f}%)")
        print(f"  Obstacle [-0.05, 0.5]: {len(obstacle)} points ({100*len(obstacle)/n:.1f}%)")
        print(f"  Other:                 {len(other)} points ({100*len(other)/n:.1f}%)")
        
        sys.exit(0)

rclpy.init()
node = CloudChecker()
rclpy.spin(node)
