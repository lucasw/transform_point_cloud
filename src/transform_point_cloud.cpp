/* Copyright 2018 Lucas Walter

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <transform_point_cloud/LookupTransformConfig.h>

class TransformPointCloud
{
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  std::string target_frame_;
  double timeout_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  boost::recursive_mutex dr_mutex_;
  transform_point_cloud::LookupTransformConfig config_;
  typedef dynamic_reconfigure::Server<transform_point_cloud::LookupTransformConfig> ReconfigureServer;
  boost::shared_ptr< ReconfigureServer > server_;
  void drCallback(transform_point_cloud::LookupTransformConfig& config,
    uint32_t level)
  {
    config_ = config;
  }

  void printPointCloud(const sensor_msgs::PointCloud2& pc, const size_t max_count = 8)
  {
    // https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(pc, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(pc, "z");
    size_t count = 0;
    for (; (iter_x != iter_x.end()) && (iter_y != iter_y.end()) && (iter_z != iter_z.end()) &&
         (count < max_count);
         ++iter_x, ++iter_y, ++iter_z, ++count)
    {
      ROS_DEBUG_STREAM(count << " " << *iter_x << " " << *iter_y << " " << *iter_z);
    }
  }

  void scalePointCloud(sensor_msgs::PointCloud2& pc,
      const double sx, const double sy, const double sz)
  {
    if ((sx == 1.0) && (sy == 1.0) && (sz == 1.0))
      return;
    // https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");
    size_t count = 0;
    for (; (iter_x != iter_x.end()) && (iter_y != iter_y.end()) && (iter_z != iter_z.end());
         ++iter_x, ++iter_y, ++iter_z, ++count)
    {
      *iter_x *= sx;
      *iter_y *= sy;
      *iter_z *= sz;
    }
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    // TODO(lucasw) actually subscribe and unsubscribe in config callback?
    if (!config_.enable)
      return;

    if (msg->width * msg->height < 1) {
      ROS_WARN_STREAM("empty point cloud " << msg->width << " " << msg->height);
      return;
    }

    // ROS_INFO_STREAM(msg->header.frame_id << " " << target_frame_);
    ROS_DEBUG_STREAM("input:");
    printPointCloud(*msg, 8);
    geometry_msgs::TransformStamped transform;
    try
    {
      // (target_frame, pc frame) preserves the world coordinates of the point cloud but shifts
      // the parent to target_frame_
      // (pc_frame, target_frame) shifts the point cloud to be relative to the target_frame
      // by the same amount it used to be relative to msg->header.frame_id,
      // but the frame will still be msg->header.frame_id when done.
      const std::string target_frame = (config_.target_frame == "") ? msg->header.frame_id : config_.target_frame;
      const std::string source_frame = (config_.source_frame == "") ? msg->header.frame_id : config_.source_frame;
      transform = tf_buffer_.lookupTransform(
          target_frame,
          source_frame,
          msg->header.stamp + ros::Duration(config_.offset_lookup_time),
          ros::Duration(config_.timeout));
      sensor_msgs::PointCloud2 cloud_out;
      tf2::doTransform(*msg, cloud_out, transform);
      scalePointCloud(cloud_out, config_.scale_x, config_.scale_y, config_.scale_z);
      ROS_DEBUG_STREAM("output:");
      // TODO(lwalter) should the scaling be done on the output?
      printPointCloud(cloud_out, 8);
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
    // nh_("~"),
    tf_listener_(tf_buffer_)
  {
    server_.reset(new ReconfigureServer(dr_mutex_));  //, nh_));
    dynamic_reconfigure::Server<transform_point_cloud::LookupTransformConfig>::CallbackType cbt =
        boost::bind(&TransformPointCloud::drCallback, this, boost::placeholders::_1, boost::placeholders::_2);
    server_->setCallback(cbt);

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
