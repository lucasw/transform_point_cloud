# transform_point_cloud

transform a pointcloud2 with tf2, dynamic reconfigure to change the lookup transform parameters.

Demonstration:

https://github.com/lucasw/dynamic_reconfigure_tools is required.

```
roslaunch transform_point_cloud demo.launch
```
![rviz and rqt demo.launch](data/transform_point_cloud.png)

In the image above the point cloud originally has map for a frame_id, but the parameters transform the points to relative to frame2 by the same amount, with frame1 as the new frame_id.
