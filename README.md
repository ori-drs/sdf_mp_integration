# sdf_mp_integration

## To do

- [ ] Document the benchmarking results and commmands
## Benchmarking commands

To play the cow and lady dataset:
rosbag play /home/mark/code/sdf_package_testing/src/data.bag

Voxblox:
roslaunch voxblox_ros cow_and_lady_dataset_esdf.launch

FIESTA

GPU-Voxels



## Benchmarking commands



## Example planning goals
rostopic pub -1 /full_goal sdf_mp_integration/WholeBodyPose '{arm: [0.2, 1, 1, 1.57, 1], base:{header:{stamp: now, frame_id: odom}, pose:{position:{x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}'

rostopic pub -1 /full_goal sdf_mp_integration/WholeBodyPose '{arm: [0, 0, 0, 1.57, 0], base:{header:{stamp: now, frame_id: odom}, pose:{position:{x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}'

rostopic pub -1 /start_moving_obstacle 0.5




Good grasping pose: whole_body.move_to_joint_positions({"arm_lift_joint": 0.0, "arm_flex_joint": -0.9, "arm_roll_joint": 0, "wrist_flex_joint": -0.7, "wrist_roll_joint": 0.0})
[0.0, 0.9, 0.0, 0.7, 0.0]

rostopic pub -1 /full_goal sdf_mp_integration/WholeBodyPose '{arm: [0.0, 0.9, 0.0, 0.7, 0.0], base:{header:{stamp: now, frame_id: odom}, pose:{position:{x: 1.84, y: -0.65, z: 0}, orientation: {x: 0, y: 0, z: -0.062, w: 0.998}}}}'

rostopic pub -1 /full_goal sdf_mp_integration/WholeBodyPose '{arm: [0.12, 1.31, 0.81, 0.74, 0.37], base:{header:{stamp: now, frame_id: odom}, pose:{position:{x: 1.84, y: -0.65, z: 0}, orientation: {x: 0, y: 0, z: -0.062, w: 0.998}}}}'