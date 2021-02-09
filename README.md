# sdf_mp_integration

## To do

- [ ] Create a test to ensure the sdf for each package is as expected
    - [x] GPU-Voxels
    - [ ] FIESTA
    - [ ] Voxblox

Note that the test is already implemented in GPU-Voxels: a box from 0-10 in a 64x64x64 env. Results in txt file.

- [x] Look to parallelise GPU-Voxels inverse calculation
- [ ] Record benchmarking results
    - [ ] Calculation of the SDF
    - [ ] Including the gradient calculation in GPU-Voxels


## Benchmarking commands

To play the cow and lady dataset:
rosbag play /home/mark/code/sdf_package_testing/src/data.bag

Voxblox:
roslaunch voxblox_ros cow_and_lady_dataset_esdf.launch


rostopic pub -1 /full_goal sdf_mp_integration/WholeBodyPose '{arm: [0.2, 1, 1, 1.57, 1], base:{header:{stamp: now, frame_id: odom}, pose:{position:{x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}'
rostopic pub -1 /full_goal sdf_mp_integration/WholeBodyPose '{arm: [0, 0, 0, 1.57, 0], base:{header:{stamp: now, frame_id: odom}, pose:{position:{x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}}}'
