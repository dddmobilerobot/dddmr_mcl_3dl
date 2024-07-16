/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
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


#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
/*voxel*/
#include <pcl/filters/voxel_grid.h>

class PCLPublisherNode : public rclcpp::Node {

  public:
    PCLPublisherNode(std::string name);

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;

    double map_rotate_around_x_, map_rotate_around_y_, map_rotate_around_z_;
    double map_translate_x_, map_translate_y_, map_translate_z_;
    double ground_rotate_around_x_, ground_rotate_around_y_, ground_rotate_around_z_;
    double ground_translate_x_, ground_translate_y_, ground_translate_z_;
    
    double map_down_sample_, ground_down_sample_;

    std::string package_share_directory_, map_dir_, ground_dir_, global_frame_;
};

PCLPublisherNode::PCLPublisherNode(std::string name): Node(name){

  package_share_directory_ = ament_index_cpp::get_package_share_directory("mcl_3dl");

  this->declare_parameter("map_dir", rclcpp::ParameterValue(""));
  rclcpp::Parameter map_dir = this->get_parameter("map_dir");
  map_dir_ = map_dir.as_string();
  RCLCPP_INFO(this->get_logger(), "map_dir: %s", map_dir_.c_str());

  this->declare_parameter("ground_dir", rclcpp::ParameterValue(""));
  rclcpp::Parameter ground_dir = this->get_parameter("ground_dir");
  ground_dir_ = ground_dir.as_string();
  RCLCPP_INFO(this->get_logger(), "ground_dir: %s", ground_dir_.c_str());

  this->declare_parameter("global_frame", rclcpp::ParameterValue(""));
  rclcpp::Parameter global_frame = this->get_parameter("global_frame");
  global_frame_ = global_frame.as_string();
  RCLCPP_INFO(this->get_logger(), "global_frame: %s", global_frame_.c_str());

  this->declare_parameter("map_rotate_around_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_rotate_around_x = this->get_parameter("map_rotate_around_x");
  map_rotate_around_x_ = map_rotate_around_x.as_double();
  RCLCPP_INFO(this->get_logger(), "map_rotate_around_x: %.2f", map_rotate_around_x_);

  this->declare_parameter("map_rotate_around_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_rotate_around_y = this->get_parameter("map_rotate_around_y");
  map_rotate_around_y_ = map_rotate_around_y.as_double();
  RCLCPP_INFO(this->get_logger(), "map_rotate_around_y: %.2f", map_rotate_around_y_);

  this->declare_parameter("map_rotate_around_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_rotate_around_z = this->get_parameter("map_rotate_around_z");
  map_rotate_around_z_ = map_rotate_around_z.as_double();
  RCLCPP_INFO(this->get_logger(), "map_rotate_around_z: %.2f", map_rotate_around_z_);

  this->declare_parameter("map_translate_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_translate_x = this->get_parameter("map_translate_x");
  map_translate_x_ = map_translate_x.as_double();
  RCLCPP_INFO(this->get_logger(), "map_translate_x: %.2f", map_translate_x_);

  this->declare_parameter("map_translate_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_translate_y = this->get_parameter("map_translate_y");
  map_translate_y_ = map_translate_y.as_double();
  RCLCPP_INFO(this->get_logger(), "map_translate_y: %.2f", map_translate_y_);

  this->declare_parameter("map_translate_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_translate_z = this->get_parameter("map_translate_z");
  map_translate_z_ = map_translate_z.as_double();
  RCLCPP_INFO(this->get_logger(), "map_translate_z: %.2f", map_translate_z_);

  this->declare_parameter("ground_rotate_around_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter ground_rotate_around_x = this->get_parameter("ground_rotate_around_x");
  ground_rotate_around_x_ = ground_rotate_around_x.as_double();
  RCLCPP_INFO(this->get_logger(), "ground_rotate_around_x: %.2f", ground_rotate_around_x_);

  this->declare_parameter("ground_rotate_around_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter ground_rotate_around_y = this->get_parameter("ground_rotate_around_y");
  ground_rotate_around_y_ = ground_rotate_around_y.as_double();
  RCLCPP_INFO(this->get_logger(), "ground_rotate_around_y: %.2f", ground_rotate_around_y_);

  this->declare_parameter("ground_rotate_around_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter ground_rotate_around_z = this->get_parameter("ground_rotate_around_z");
  ground_rotate_around_z_ = ground_rotate_around_z.as_double();
  RCLCPP_INFO(this->get_logger(), "ground_rotate_around_z: %.2f", ground_rotate_around_z_);

  this->declare_parameter("ground_translate_x", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter ground_translate_x = this->get_parameter("ground_translate_x");
  ground_translate_x_ = ground_translate_x.as_double();
  RCLCPP_INFO(this->get_logger(), "ground_translate_x: %.2f", ground_translate_x_);

  this->declare_parameter("ground_translate_y", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter ground_translate_y = this->get_parameter("ground_translate_y");
  ground_translate_y_ = ground_translate_y.as_double();
  RCLCPP_INFO(this->get_logger(), "ground_translate_y: %.2f", ground_translate_y_);

  this->declare_parameter("ground_translate_z", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter ground_translate_z = this->get_parameter("ground_translate_z");
  ground_translate_z_ = ground_translate_z.as_double();
  RCLCPP_INFO(this->get_logger(), "ground_translate_z: %.2f", ground_translate_z_);

  this->declare_parameter("map_down_sample", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter map_down_sample = this->get_parameter("map_down_sample");
  map_down_sample_ = map_down_sample.as_double();
  RCLCPP_INFO(this->get_logger(), "map_down_sample: %.2f", map_down_sample_);

  this->declare_parameter("ground_down_sample", rclcpp::ParameterValue(0.0));
  rclcpp::Parameter ground_down_sample = this->get_parameter("ground_down_sample");
  ground_down_sample_ = ground_down_sample.as_double();
  RCLCPP_INFO(this->get_logger(), "ground_down_sample: %.2f", ground_down_sample_);
  
  //@ Latched topic, Create a publisher using the QoS settings to emulate a ROS1 latched topic
  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapground",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ground (new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (map_dir_, *map_cloud) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read map pcd.");
    return;
  }
  else{
    map_cloud->header.frame_id = global_frame_;

    //@  METHOD #2: Using a Affine3f. This method is easier and less error prone

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 0.0 meters on the axis.
    transform_2.translation() << map_translate_x_, map_translate_y_, map_translate_z_;

    // The same rotation matrix as before; theta radians around X axis
    if(fabs(map_rotate_around_x_)>0.01)
      transform_2.rotate (Eigen::AngleAxisf (map_rotate_around_x_, Eigen::Vector3f::UnitX()));
    if(fabs(map_rotate_around_y_)>0.01)
      transform_2.rotate (Eigen::AngleAxisf (map_rotate_around_y_, Eigen::Vector3f::UnitY()));
    if(fabs(map_rotate_around_z_)>0.01)
      transform_2.rotate (Eigen::AngleAxisf (map_rotate_around_z_, Eigen::Vector3f::UnitZ()));  

    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud(*map_cloud, *map_cloud, transform_2);
    RCLCPP_INFO(this->get_logger(), "Map pointcloud size: %lu", map_cloud->points.size());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (map_cloud);
    sor.setLeafSize (map_down_sample_, map_down_sample_, map_down_sample_);
    sor.filter (*map_cloud);
    map_cloud->is_dense = false;

    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*map_cloud, *map_cloud, ind);
    RCLCPP_INFO(this->get_logger(), "Map pointcloud size after down size: %lu", map_cloud->points.size());
    sensor_msgs::msg::PointCloud2 ros_msg_map_cloud;
    pcl::toROSMsg(*map_cloud, ros_msg_map_cloud);
    pub_map_->publish(ros_msg_map_cloud);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ> (ground_dir_, *map_ground) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read ground pcd.");
  }
  else{
    map_ground->header.frame_id = global_frame_;

    //@METHOD #2: Using a Affine3f. This method is easier and less error prone
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

    // Define a translation of 0.0 meters on the axis.
    transform_2.translation() << ground_translate_x_, ground_translate_y_, ground_translate_z_;

    // The same rotation matrix as before; theta radians around X axis
    if(fabs(ground_rotate_around_x_)>0.01)
      transform_2.rotate (Eigen::AngleAxisf (ground_rotate_around_x_, Eigen::Vector3f::UnitX()));
    if(fabs(ground_rotate_around_y_)>0.01)
      transform_2.rotate (Eigen::AngleAxisf (ground_rotate_around_y_, Eigen::Vector3f::UnitY()));
    if(fabs(ground_rotate_around_z_)>0.01)
      transform_2.rotate (Eigen::AngleAxisf (ground_rotate_around_z_, Eigen::Vector3f::UnitZ()));  

    // You can either apply transform_1 or transform_2; they are the same
    pcl::transformPointCloud (*map_ground, *map_ground, transform_2);
    RCLCPP_INFO(this->get_logger(), "Ground pointcloud size: %lu", map_ground->points.size());
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (map_ground);
    sor.setLeafSize (ground_down_sample_, ground_down_sample_, ground_down_sample_);
    sor.filter (*map_ground);
    map_ground->is_dense = false;

    std::vector<int> ind;
    pcl::removeNaNFromPointCloud(*map_ground, *map_ground, ind);
    RCLCPP_INFO(this->get_logger(), "Ground pointcloud size after down size: %lu", map_ground->points.size());
    sensor_msgs::msg::PointCloud2 ros_msg_map_ground;
    pcl::toROSMsg(*map_ground, ros_msg_map_ground);
    pub_ground_->publish(ros_msg_map_ground);
  }
}

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  auto PP = std::make_shared<PCLPublisherNode>("pcl_publisher");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(PP);
  executor.spin();
  rclcpp::shutdown();  
  return 0;
}
