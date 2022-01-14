#!/usr/bin/env python
# Sample code to publish a PointCloud2 with python
import struct

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class GeneratePointCloud:
    def __init__(self):
        self.pub = rospy.Publisher("point_cloud", PointCloud2, queue_size=2)

        i = 0
        while not rospy.is_shutdown():
            t0 = rospy.Time.now()
            self.pub_cube(i % 16 + 1)
            rospy.logdebug((rospy.Time.now() - t0).to_sec())
            i += 1
            rospy.sleep(1.0)

    def pub_cube(self, lim=8):
        points = []
        for i in range(lim):
            for j in range(lim):
                for k in range(lim):
                    x = float(i) / lim
                    y = float(j) / lim
                    z = float(k) / lim
                    pt = [x, y, z, 0]
                    r = np.uint8(x * 255.0)
                    g = np.uint8(y * 255.0)
                    b = np.uint8(z * 255.0)
                    a = 255
                    # print(r, g, b, a)
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    # print(hex(rgb))
                    pt[3] = rgb
                    points.append(pt)

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.UINT32, 1),
                  # PointField('rgba', 12, PointField.UINT32, 1),
                  ]

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "map"
        pc2 = point_cloud2.create_cloud(header, fields, points)
        pc2.header.stamp = rospy.Time.now()
        self.pub.publish(pc2)


if __name__ == '__main__':
    rospy.init_node('generate_point_cloud')
    generate_point_cloud = GeneratePointCloud()
    rospy.spin()
