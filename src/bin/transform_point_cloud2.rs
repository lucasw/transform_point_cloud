/// Take in a camera info and target frame and find the intersection of the boundary
/// of the camera fov and the xy plane in the target frame, publish out as a marker and polygon
/// based on camera_info_to_plane.py/.cpp and renamed to avoid rosrun confusion with the C++ node

// use nalgebra::{Point3, Rotation, Rotation3};
use roslibrust::ros1::{NodeHandle, Publisher};
use std::collections::HashMap;
use tf_roslibrust::{
    // TfError,
    TfListener,
    // tf_util,
    // transforms::isometry_from_transform,
};
use tokio::time::Duration;

// this is already done in ros_pointcloud2, but that was fine in the case of tf_roslibrust and
// tf_demo- is there something in the structure of ros_pointcloud2 that breaks this?
roslibrust_codegen_macro::find_and_generate_ros_messages!();

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
                        let pc: ros_pointcloud2::PointCloud2Msg = pc2_msg.into();  // .try_into_iter().unwrap();
                        // println!("{pc:?}");
                    },
                    Some(Err(error)) => {
                        println!("rx error: {error}");
                    },
                    None => (),
                }
            }
            _ = update_interval.tick() => {
            },  // update
        }  // tokio select loop
    }

    Ok(())
}
