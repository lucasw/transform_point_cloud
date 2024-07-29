#!/usr/bin/env python
# Lucas Walter
# aggregate every n points clouds

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class AggregatePointCloud:
    def __init__(self):
        self.pub = rospy.Publisher("point_cloud_aggregated", PointCloud2, queue_size=2)
        self.num = rospy.get_param("~num", 12)
        self.count = 0
        self.cloud_out = None
        self.sub = rospy.Subscriber("point_cloud", PointCloud2,
                                    self.point_cloud_callback, queue_size=2)

    def point_cloud_callback(self, msg):
        field_names = [field.name for field in msg.fields]
        # rospy.loginfo_throttle(1.0, f"{msg.fields} {field_names}")
        cloud_data = list(pc2.read_points(msg, skip_nans=True, field_names=field_names))
        if self.cloud_out is None:
            self.cloud_out = cloud_data
        else:
            self.cloud_out += cloud_data

        self.count += 1
        if self.count % self.num == 0:
            rospy.loginfo_throttle(4.0, len(self.cloud_out))
            cloud = pc2.create_cloud(msg.header, msg.fields, self.cloud_out)
            self.pub.publish(cloud)
            self.cloud_out = None


if __name__ == '__main__':
    rospy.init_node('aggregate_point_cloud')
    aggregate_point_cloud = AggregatePointCloud()
    rospy.spin()
