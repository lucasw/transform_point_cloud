use nalgebra::point; // , Rotation, Rotation3};
use roslibrust_util::{geometry_msgs::TransformStamped, sensor_msgs::PointCloud2};
use tf_roslibrust::{transforms::isometry_from_transform, LookupTransform, TfError, TfListener};

pub fn transform_point_cloud(cloud_in: PointCloud2, tfs: TransformStamped) -> PointCloud2 {
    let stamp = cloud_in.header.stamp.clone();
    // TODO(lucasw) wanr if child_frame_id != cloud_in.header.frame_id?
    let target_frame = tfs.header.frame_id;
    let cloud_in: ros_pointcloud2::PointCloud2Msg = cloud_in.into(); // .try_into_iter().unwrap();
    log::debug!("{} to {}", cloud_in.header.frame_id, target_frame);
    let cloud_to_target = isometry_from_transform(&tfs.transform);
    // TODO(lucasw) once this is a function if this fails then
    // return the error
    let points_in: Vec<ros_pointcloud2::prelude::PointXYZ> = cloud_in.try_into_vec().unwrap();
    let mut points_out = Vec::new();
    for pt_in in points_in {
        // .iter().take(10) {
        let pt_in = point![pt_in.x as f64, pt_in.y as f64, pt_in.z as f64];
        let pt_out = cloud_to_target * pt_in;
        points_out.push(ros_pointcloud2::prelude::PointXYZ::new(
            pt_out.x as f32,
            pt_out.y as f32,
            pt_out.z as f32,
        ));
        // log::info!("{pt:?}");
    }
    let pc_out = ros_pointcloud2::PointCloud2Msg::try_from_vec(points_out).unwrap();
    let mut pc_out_msg: PointCloud2 = pc_out.into();
    pc_out_msg.header.stamp = stamp;
    pc_out_msg.header.frame_id = target_frame.to_string();

    pc_out_msg
}

/// return either a point cloud to publish in the first slot, or return the input point cloud
/// unchanged
pub fn transform_point_cloud_with_listener(
    cloud_in: PointCloud2,
    listener: &TfListener,
    target_frame: &str,
) -> (Option<PointCloud2>, Option<PointCloud2>) {
    // TODO(lucasw) turn this into a function, return Some with enum that is the
    // transformed cloud to be published, the unused cloud to be re-queued,
    // an error (because there was a different error and don't
    // try transforming the same cloud later)
    let res = listener.lookup_transform(
        target_frame,
        &cloud_in.header.frame_id,
        Some(cloud_in.header.stamp.clone()),
    );
    match res {
        Ok(tfs) => {
            let pc_out_msg = transform_point_cloud(cloud_in, tfs);
            (Some(pc_out_msg), None)
        }
        Err(err) => match err {
            TfError::AttemptedLookupInFuture(_, _, _) => {
                // print!("-");
                // try to process again later
                (None, Some(cloud_in))
            }
            _ => {
                // give up on this point cloud with any other error
                log::warn!("lookup err, discarding point cloud {err:?}");
                (None, None)
            }
        },
    }
}
