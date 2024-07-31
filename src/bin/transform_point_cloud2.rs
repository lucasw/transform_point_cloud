/// Take in a camera info and target frame and find the intersection of the boundary
/// of the camera fov and the xy plane in the target frame, publish out as a marker and polygon
/// based on camera_info_to_plane.py/.cpp and renamed to avoid rosrun confusion with the C++ node

use nalgebra::point;  // , Rotation, Rotation3};
use roslibrust::ros1::{NodeHandle, Publisher};
use std::collections::HashMap;
use tf_roslibrust::{
    TfError,
    TfListener,
    // tf_util,
    transforms::isometry_from_transform,
};
use tokio::time::Duration;

// this is already done in ros_pointcloud2, but that was fine in the case of tf_roslibrust and
// tf_demo- is there something in the structure of ros_pointcloud2 that breaks this?
roslibrust_codegen_macro::find_and_generate_ros_messages!();

// copy paste from ros_pointcloud2/rpcl2/src/ros.rs,
// but couldn't get that to work
impl From<sensor_msgs::PointCloud2> for ros_pointcloud2::PointCloud2Msg {
    fn from(msg: sensor_msgs::PointCloud2) -> Self {
        Self {
            header: ros_pointcloud2::ros::HeaderMsg {
                seq: msg.header.seq,
                stamp: ros_pointcloud2::ros::TimeMsg {
                    sec: msg.header.stamp.secs as i32,
                    nanosec: msg.header.stamp.nsecs,
                },
                frame_id: msg.header.frame_id,
            },
            dimensions: ros_pointcloud2::CloudDimensions {
                width: msg.width,
                height: msg.height,
            },
            fields: msg
                .fields
                .into_iter()
                .map(|field| ros_pointcloud2::ros::PointFieldMsg {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            endian: if msg.is_bigendian {
                ros_pointcloud2::Endian::Big
            } else {
                ros_pointcloud2::Endian::Little
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            dense: if msg.is_dense {
                ros_pointcloud2::Denseness::Dense
            } else {
                ros_pointcloud2::Denseness::Sparse
            },
        }
    }
}

impl From<ros_pointcloud2::PointCloud2Msg> for sensor_msgs::PointCloud2 {
    fn from(msg: ros_pointcloud2::PointCloud2Msg) -> Self {
        sensor_msgs::PointCloud2 {
            header: std_msgs::Header {
                seq: msg.header.seq,
                stamp: roslibrust_codegen::Time {
                    secs: msg.header.stamp.sec as u32,
                    nsecs: msg.header.stamp.nanosec,
                },
                frame_id: msg.header.frame_id,
            },
            height: msg.dimensions.height,
            width: msg.dimensions.width,
            fields: msg
                .fields
                .into_iter()
                .map(|field| sensor_msgs::PointField {
                    name: field.name,
                    offset: field.offset,
                    datatype: field.datatype,
                    count: field.count,
                })
                .collect(),
            is_bigendian: if msg.endian == ros_pointcloud2::Endian::Big {
                true
            } else {
                false
            },
            point_step: msg.point_step,
            row_step: msg.row_step,
            data: msg.data,
            is_dense: if msg.dense == ros_pointcloud2::Denseness::Dense {
                true
            } else {
                false
            },
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), anyhow::Error> {

    // need to have leading slash on node name and topic to function properly
    // so figure out namespace then prefix it to name and topics

    // string parameters
    let mut param_str = HashMap::<String, String>::new();
    param_str.insert("target_frame".to_string(), "map".to_string());
    // need extra leading _ for name and ns
    param_str.insert("_name".to_string(), "transform_point_cloud2".to_string());
    param_str.insert("_ns".to_string(), "".to_string());

    // TODO(lucasw) can an existing rust arg handling library handle the ':=' ros cli args?
    let args = std::env::args();
    let mut args2 = Vec::new();
    for arg in args {
        let key_val: Vec<&str> = arg.split(":=").collect();
        if key_val.len() != 2 {
            args2.push(arg);
            continue;
        }

        let (mut key, val) = (key_val[0].to_string(), key_val[1].to_string());
        if !key.starts_with("_") {
            println!("unused arg pair {key}:={val}- need to prefix name with underscore");
            continue;
        }
        key.replace_range(0..1, "");

        if param_str.contains_key(&key) {
            param_str.insert(key, val);
        } else {
            println!("unused '{key}' '{val}'");
        }
    }
    println!("{args2:?}");
    println!("{param_str:?}");

    let ns = param_str.remove("_ns").unwrap();
    let target_frame = param_str.remove("target_frame").unwrap();

    let full_node_name = &format!(
        "/{}/{}",
        &ns,
        &param_str["_name"],
        ).replace("//", "/");
    println!("{}", format!("full ns and node name: {full_node_name}"));

    let nh = NodeHandle::new(&std::env::var("ROS_MASTER_URI")?, full_node_name)
        .await?;

    let transformed_point_cloud_pub: Publisher<sensor_msgs::PointCloud2> = nh.advertise(&format!("{}/point_cloud_transformed_rust", ns.as_str()), 3).await?;

    // TODO(lucasw) don't need channels yet but later switch away from tokio::select and use
    // channels between threads/tasks?
    // let (pc_tc, pc_rx) = tokio::sync::oneshot::channel();
    let mut clouds_in = std::collections::VecDeque::new();

    // TODO(lucasw) remember leading ns or it won't work
    let mut point_cloud_sub = nh.subscribe::<sensor_msgs::PointCloud2>(&format!("{}/point_cloud", ns.as_str()), 10).await?;

    let mut listener = TfListener::new(&nh).await;

    // TODO(lucasw) make this a queue
    let mut update_interval = tokio::time::interval(Duration::from_millis(50));

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                println!("ctrl-c exiting");
                break;
            }
            // TODO(lucasw) move this into listener
            rv = listener._dynamic_subscriber.next() => {
                print!(".");
                match rv {
                    Some(Ok(tfm)) => {
                        listener.update_tf(tfm);  // .await;
                    },
                    Some(Err(error)) => {
                        // probably can't keep up with tf publish rate
                        println!("{error}");
                    },
                    None => (),
                }
            }
            rv = listener._static_subscriber.next() => {
                print!("+");
                match rv {
                    Some(Ok(tfm)) => {
                        listener.update_tf_static(tfm);  // .await;
                    },
                    Some(Err(error)) => {
                        panic!("{error}");
                    },
                    None => (),
                }
            }
            rv = point_cloud_sub.next() => {
                // print!("c");
                // let t0 = tf_util::duration_now();
                match rv {
                    Some(Ok(pc2_msg)) => {
                        println!("{} bytes, {} x {}", pc2_msg.data.len(), pc2_msg.width, pc2_msg.height);
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
                        println!("rx error: {error}");
                    },
                    None => (),
                }
            }
            _ = update_interval.tick() => {
                let cloud_in = clouds_in.pop_front();
                match cloud_in {
                    None => {},
                    Some(cloud_in) => {
                        // TODO(lucasw) turn this into a function, return Some with enum that is the
                        // transformed cloud to be published, the unused cloud to be re-queued,
                        // an error (because there was a different error and don't
                        // try transforming the same cloud later)
                        let res = listener.lookup_transform(
                            &target_frame,
                            &cloud_in.header.frame_id,
                            None,
                        );
                        match res {
                            Ok(tfs) => {
                                let stamp = cloud_in.header.stamp.clone();
                                let cloud_in: ros_pointcloud2::PointCloud2Msg = cloud_in.into();  // .try_into_iter().unwrap();
                                println!("{} to {}", cloud_in.header.frame_id, target_frame);
                                let cloud_to_target = isometry_from_transform(&tfs.transform);
                                // TODO(lucasw) once this is a function if this fails then
                                // return the error
                                let points_in: Vec<ros_pointcloud2::prelude::PointXYZ> = cloud_in.try_into_vec().unwrap();
                                let mut points_out = Vec::new();
                                for pt_in in points_in {  // .iter().take(10) {
                                    let pt_in = point![pt_in.x as f64, pt_in.y as f64, pt_in.z as f64];
                                    let pt_out = cloud_to_target * pt_in;
                                    points_out.push(ros_pointcloud2::prelude::PointXYZ::new(
                                            pt_out.x as f32,
                                            pt_out.y as f32,
                                            pt_out.z as f32,
                                    ));
                                    // println!("{pt:?}");
                                }
                                let pc_out = ros_pointcloud2::PointCloud2Msg::try_from_vec(points_out).unwrap();
                                let mut pc_out_msg: sensor_msgs::PointCloud2 = pc_out.into();
                                pc_out_msg.header.stamp = stamp;
                                pc_out_msg.header.frame_id = target_frame.clone();
                                transformed_point_cloud_pub.publish(&pc_out_msg).await?;

                                // println!("{cloud_to_target:?}");
                                println!("clouds left: {}", clouds_in.len());
                            },
                            Err(err) => match err {
                                TfError::AttemptedLookUpInFuture(_, _) => {
                                    print!("-");
                                    // try to process again later
                                    clouds_in.push_front(cloud_in);
                                },
                                _ => {
                                    // give up on this point cloud with any other error
                                    println!("lookup err, discarding point cloud {err:?}");
                                },
                            },
                        }
                    },
                }  // handle input cloud
            },  // update
        }  // tokio select loop
    }

    Ok(())
}
