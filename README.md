# mcl_3dl

> [!CAUTION]
> This package should be used with [dddmr_lego_loam_bor ](https://github.com/dddmobilerobot/dddmr_lego_loam_bor), because it loads pose graph generated using [dddmr_lego_loam_bor ](https://github.com/dddmobilerobot/dddmr_lego_loam_bor) to localize instead of using a point cloud file. Therefore, our algorithm can run in a huge map without computation overhead.

This repo is based on the [mcl_3dl ](https://github.com/at-wat/mcl_3dl), and has been modified for ground vehicles. We did not fork the repo due to significant modifications, variant features and frameworks are implemented.

The original author deserve all the credits, we just stand on the shoulders of giants.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_mcl_3dl/dddmr_mcl_3dl.gif" width="640" height="400"/>
</p>

The following features are provided and are different from original version:

- ROS2 version of mcl_3dl, the algorithm is modified and designed for ground vehicles.
- Particle filter is updated based on travelling distance/rotation, so the computation are minized when robot is idling.
- Feature selection is based on [dddmr_lego_loam_bor ](https://github.com/dddmobilerobot/dddmr_lego_loam_bor).
- Submap concept is introduced, the computation requirement is significaly reduced, our version can localize the robot in 500mx500m map on a jetson Orin nano, larger map is also possible.
- Particle rating mechanisms are redesigned for mobile robots, the ground point cloud is used to constraint the robot on the ground.
  - Euclidean Cluster Extraction is used to normalized the rating, we rate each particle by using clusters instead of using points. Because the far object comprise only few points, the points base rating can not effectively use this far object. Instead, our cluster-based rating will be able to compensate this situation.
  - The normal vector of features is used to normalized the rating. When the robot travels between long walls, virtual slipping will occur (the robot think it is not moving at all), the normal of the feature is tracked to compensate virtual slipping condition.

## RUN The Demo
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack6).
```
cd ~
git clone https://github.com/dddmobilerobot/dddmr_navigation.git
cd ~/dddmr_navigation && git submodule update --init dddmr_docker src/dddmr_mcl_3dl src/dddmr_rviz_tools
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Download essential files
Pose graph (3.3MB) and a bag file (1.2GB) will be download to run the demo. Noted that the bag file is not the one we used to generate the map, the pose graph is generated using another bag file.
```
cd ~/dddmr_navigation/src/dddmr_mcl_3dl && ./download_files.bash
```
### 3. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. Then we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Play mapping using bag files
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
