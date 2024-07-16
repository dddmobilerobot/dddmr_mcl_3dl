# mcl_3dl

> [!CAUTION]
> This package should be used with [dddmr_lego_loam_bor ](https://github.com/dddmobilerobot/dddmr_lego_loam_bor), because it loads pose graph from [dddmr_lego_loam_bor ](https://github.com/dddmobilerobot/dddmr_lego_loam_bor) to localize instead of using a point cloud file.

This repo is based on the [mcl_3dl ](https://github.com/at-wat/mcl_3dl), and has been modified for ground vehicles. We did not fork the repo due to significant modifications, variant features and frameworks are implemented.

The original author deserve all the credits, we just stand on the shoulders of giants.

<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/dddmr_mcl_3dl/dddmr_mcl_3dl.gif" width="640" height="400"/>
</p>

The following features are provided and are different from original version:

- ROS2 version (Humble) of mcl_3dl particularly for ground vehicles.
- Particle filter is updated based on travelling distance/rotation, so the computation are minized when robot is idling.
- Feature selection is based on [dddmr_lego_loam_bor ](https://github.com/dddmobilerobot/dddmr_lego_loam_bor).
- Submap concept is introduced, the computation requirement is significaly reduced, our version can localize the robot in 500mx500m map on a jetson Orin nano, larger map is also possible.
- Particle rating mechanisms are redesigned for mobile robots, the ground point cloud is used to constraint the robot on the ground.
  - Euclidean Cluster Extraction is used to normalized the rating, we rate each particle by using clusters instead of using points. Because the far object comprise only few points, the points base rating can not effectively use this far object. Instead, our cluster-based rating will be able to compensate this situation.
  - The normal vector of features is used to normalized the rating. When the robot travels between long walls, virtual slipping will occur (the robot think it is not moving at all), the normal of the feature is tracked to compensate virtual slipping condition.
