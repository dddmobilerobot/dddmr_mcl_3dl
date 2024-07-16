/*
 * Copyright (c) 2018, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its 
 *       contributors may be used to endorse or promote products derived from 
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H
#define MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H

#include "utilities.h"

#include <pcl/point_types.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <mcl_3dl/pf.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2/LinearMath/Transform.h"

namespace mcl_3dl
{

struct LidarMeasurementResult
{
  float likelihood;
  float quality;

  LidarMeasurementResult(const float likelihood_value, const float quality_value)
    : likelihood(likelihood_value)
    , quality(quality_value)
  {
  }
};

class LidarMeasurementModelLikelihood
{

private:

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_;

  int num_points_;
  int num_points_default_;
  int num_points_global_;

  int threshold_for_trusted_ground_;
  double radius_of_ground_search_;

  float match_dist_min_;
  float match_dist_flat_;

public:

  void loadConfig(
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter);
  void setGlobalLocalizationStatus(
      const int num_particles,
      const int current_num_particles);
  LidarMeasurementResult measure(
      pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree,
      pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree_ground,
      pcl::PointCloud<pcl::Normal>& normals,
      std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr> pcl_segmentations,
      const State6DOF& s) const;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H
