# mcl_3dl
This repo is based on the [mcl_3dl ](https://github.com/at-wat/mcl_3dl), and has been modified for ground vehicles. We did not fork the repo due to significant modifications, variant features and frameworks are implemented.

The original author deserve all the credits, we just stand on the shoulders of giants.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_mcl_3dl/dddmr_mcl_3dl.gif" width="700" height="420"/>
</p>

The following features are provided and are different from original version:

- ROS2 version (Humble) of mcl_3dl particularly for ground vehicles.
- Particle filter is updated based on travelling distance/rotation, so the computation are minized when robot is idling.
- Feature selection is based on [dddmr_lego_loam_bor ](https://github.com/dddmobilerobot/dddmr_lego_loam_bor).
- Submap concept is introduced, the computer requirement is significaly reduced, our version can be run in jetson Orin Nano on a 500mx500m map.
- Particle rating mechanism is redesigned for mobile robots, the ground point cloud is used to constraint the robot on the ground.