/* Copyright 2018 Lucas Walter

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


class TransformPointCloud
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string target_frame_;
  double timeout_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // ROS_INFO_STREAM(msg->header.frame_id << " " << target_frame_);
    #if 0
    // convert to pcl_ros and print a few points
    for (size_t i = 0; i < 8 && i < msg->points.size(); ++i)
    {
      ROS_INFO_STREAM(i << " " << msg->
    }
    #endif
    geometry_msgs::TransformStamped transform;
    try
    {
      // This preserves the world coordinates of the point cloud but shifts
      // the parent to target_frame_
      transform = tf_buffer_.lookupTransform(target_frame_, msg->header.frame_id,
      // This shifts the point cloud to be relative to the target_frame
      // by the same amount it used to be relative to msg->header.frame_id,
      // the parent will still be msg->header.frame_id when done.
      // transform = tf_buffer_.lookupTransform(msg->header.frame_id, target_frame_,
          msg->header.stamp, ros::Duration(timeout_));
      sensor_msgs::PointCloud2 cloud_out;
      tf2::doTransform(*msg, cloud_out, transform);
      pub_.publish(cloud_out);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
  }
public:
  TransformPointCloud() :
    target_frame_("map"),
    timeout_(1.0),
    tf_listener_(tf_buffer_)
  {
    ros::param::get("~frame", target_frame_);
    ros::param::get("~timeout", timeout_);
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud_transformed", 3);
    sub_ = nh_.subscribe("point_cloud", 1, &TransformPointCloud::pointCloudCallback, this);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transform_point_cloud");
  TransformPointCloud transform_point_cloud;
  ros::spin();
}
