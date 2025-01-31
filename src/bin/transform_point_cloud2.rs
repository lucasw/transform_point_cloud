/*!
Take in a camera info and target frame and find the intersection of the boundary
of the camera fov and the xy plane in the target frame, publish out as a marker and polygon
based on camera_info_to_plane.py/.cpp and renamed to avoid rosrun confusion with the C++ node

cargo run --release -- _target_frame:=base_link point_cloud_in:=/sensor/point_cloud
*/

use clap::command;
use nalgebra::point; // , Rotation, Rotation3};
use roslibrust::ros1::NodeHandle;
use roslibrust_util::sensor_msgs;
use std::collections::HashMap;
use tf_roslibrust::{
    // tf_util,
    transforms::isometry_from_transform,
    LookupTransform,
    TfError,
    TfListener,
};
use tokio::time::Duration;

/// return true if the cloud ought to be retried later
fn transform_point_cloud(
    cloud_in: sensor_msgs::PointCloud2,
    listener: &TfListener,
    target_frame: &str,
) -> (
    Option<sensor_msgs::PointCloud2>,
    Option<sensor_msgs::PointCloud2>,
) {
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
            let stamp = cloud_in.header.stamp.clone();
            let cloud_in: ros_pointcloud2::PointCloud2Msg = cloud_in.into(); // .try_into_iter().unwrap();
            log::info!("{} to {}", cloud_in.header.frame_id, target_frame);
            let cloud_to_target = isometry_from_transform(&tfs.transform);
            // TODO(lucasw) once this is a function if this fails then
            // return the error
            let points_in: Vec<ros_pointcloud2::prelude::PointXYZ> =
                cloud_in.try_into_vec().unwrap();
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
            let mut pc_out_msg: sensor_msgs::PointCloud2 = pc_out.into();
            pc_out_msg.header.stamp = stamp;
            pc_out_msg.header.frame_id = target_frame.to_string();
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

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {
    simple_logger::SimpleLogger::new()
        .with_level(log::LevelFilter::Info)
        .init()?;

    // don't want params and remaps to be mutable after init is done
    let (nh, _full_node_name, params, remaps) = {
        let mut params = HashMap::<String, String>::new();
        params.insert("_name".to_string(), "transform_point_cloud".to_string());
        params.insert("target_frame".to_string(), "odom".to_string());
        let mut remaps = HashMap::<String, String>::new();
        remaps.insert("point_cloud_in".to_string(), "point_cloud_in".to_string());
        remaps.insert("point_cloud_out".to_string(), "point_cloud_out".to_string());

        let (_ns, full_node_name, remaining_args) =
            roslibrust_util::get_params_remaps(&mut params, &mut remaps);

        // using clap only for version reporting currently
        let _matches = command!().get_matches_from(remaining_args);

        let ros_master_uri =
            std::env::var("ROS_MASTER_URI").unwrap_or("http://localhost:11311".to_string());
        let nh = NodeHandle::new(&ros_master_uri, &full_node_name).await?;
        log::info!("connected to roscore at {ros_master_uri}");

        (nh, full_node_name, params, remaps)
    };

    let target_frame = params.get("target_frame").unwrap();

    let point_cloud_out_topic = remaps.get("point_cloud_out").unwrap();
    log::info!("publishing on '{point_cloud_out_topic}'");
    let transformed_point_cloud_pub = nh
        .advertise::<sensor_msgs::PointCloud2>(point_cloud_out_topic, 3, false)
        .await?;

    // TODO(lucasw) don't need channels yet but later switch away from tokio::select and use
    // channels between threads/tasks?
    // let (pc_tx, pc_rx) = tokio::sync::oneshot::channel();
    let mut clouds_in = std::collections::VecDeque::new();

    let point_cloud_in_topic = remaps.get("point_cloud_in").unwrap();
    log::info!("subscribing to '{point_cloud_in_topic}'");
    let mut point_cloud_sub = nh
        .subscribe::<sensor_msgs::PointCloud2>(point_cloud_in_topic, 10)
        .await?;

    let listener = TfListener::new(&nh).await;

    // TODO(lucasw) make this a queue
    let mut update_interval = tokio::time::interval(Duration::from_millis(50));
    let mut count = 0;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                log::info!("ctrl-c exiting");
                break;
            }
            // TODO(lucasw) not sure if this works properly inside a select
            // Need a ReceivedQueue in roslibrust_util along the lines of one that
            // only stores the latest
            rv = point_cloud_sub.next() => {
                // print!("c");
                // let t0 = tf_util::duration_now();
                match rv {
                    Some(Ok(pc2_msg)) => {
                        count += 1;
                        if count % 200 == 0 {
                            log::info!("{} bytes, {} x {}", pc2_msg.data.len(), pc2_msg.width, pc2_msg.height);
                        }
                        // TODO(lucasw) this doesn't work, maybe didn't implement the From
                        // properly- or the macro generated sensor_msgs::PointCloud2 inside
                        // ros_pointcloud2 not the same as the one generated here?  Or they can't
                        // be generated twice like that?
                        // the error mentions ros_pointcloud2::prelude::sensor_msgs;
                        // let pc2_msg1: ros_pointcloud2::prelude::sensor_msgs::PointCloud2 = pc2_msg;
                        // tx.send(pc);
                        clouds_in.push_back(pc2_msg);
                    },
                    Some(Err(error)) => {
                        log::info!("rx error: {error}");
                    },
                    None => (),
                }
            }
            _ = update_interval.tick() => {
                let cloud_in = clouds_in.pop_front();
                match cloud_in {
                    None => {},
                    Some(cloud_in) => {
                        let (pc_out_msg, retry_cloud_in) = transform_point_cloud(cloud_in, &listener, target_frame);
                        if let Some(retry_cloud_in) = retry_cloud_in {
                            clouds_in.push_front(retry_cloud_in);
                        }
                        if let Some(pc_out_msg) = pc_out_msg {
                            transformed_point_cloud_pub.publish(&pc_out_msg).await?;
                            // log::info!("{cloud_to_target:?}");
                            log::info!("clouds left: {}", clouds_in.len());
                        }
                    },
                }  // handle input cloud
            },  // update
        } // tokio select loop
    }

    Ok(())
}
