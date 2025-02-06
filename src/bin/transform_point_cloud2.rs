/*!
Take in a camera info and target frame and find the intersection of the boundary
of the camera fov and the xy plane in the target frame, publish out as a marker and polygon
based on camera_info_to_plane.py/.cpp and renamed to avoid rosrun confusion with the C++ node

cargo run --release -- _target_frame:=base_link point_cloud_in:=/sensor/point_cloud
*/

use clap::command;
use roslibrust::ros1::NodeHandle;
use roslibrust_util::sensor_msgs;
use std::collections::HashMap;
use tf_roslibrust::TfListener;
use tokio::time::Duration;

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
        .advertise::<sensor_msgs::PointCloud2>(point_cloud_out_topic, 30, false)
        .await?;

    // TODO(lucasw) don't need channels yet but later switch away from tokio::select and use
    // channels between threads/tasks?
    // let (pc_tx, pc_rx) = tokio::sync::oneshot::channel();
    let mut clouds_in = std::collections::VecDeque::new();

    let point_cloud_in_topic = remaps.get("point_cloud_in").unwrap();
    log::info!("subscribing to '{point_cloud_in_topic}'");
    let mut point_cloud_sub = nh
        .subscribe::<sensor_msgs::PointCloud2>(point_cloud_in_topic, 30)
        .await?;

    let listener = TfListener::new(&nh).await;

    // TODO(lucasw) make this a queue
    let mut update_interval = tokio::time::interval(Duration::from_millis(10));
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
                        if count % 600 == 0 {
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
                log::debug!("clouds left: {}", clouds_in.len());
                for _ in 0..20 {
                    let cloud_in = clouds_in.pop_front();
                    match cloud_in {
                        None => {
                            break;
                        },
                        Some(cloud_in) => {
                            let (pc_out_msg, retry_cloud_in) = transform_point_cloud::transform_point_cloud_with_listener(cloud_in, &listener, target_frame);
                            if let Some(retry_cloud_in) = retry_cloud_in {
                                clouds_in.push_front(retry_cloud_in);
                            }
                            if let Some(pc_out_msg) = pc_out_msg {
                                transformed_point_cloud_pub.publish(&pc_out_msg).await?;
                                // log::info!("{cloud_to_target:?}");
                            }
                        },
                    }  // handle input cloud
                }
            },  // update
        } // tokio select loop
    }

    Ok(())
}
