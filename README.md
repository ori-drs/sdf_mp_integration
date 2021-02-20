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
