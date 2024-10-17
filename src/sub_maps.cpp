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

#include <mcl_3dl/sub_maps.h>
namespace mcl_3dl
{
SubMaps::SubMaps(std::string name) : Node(name), is_current_ready_(false), 
  prepare_warm_up_(false), is_warm_up_ready_(false), is_initial_(false){
  
  access_ = new sub_maps_mutex_t();

  declare_parameter("pose_graph_dir", rclcpp::ParameterValue(""));
  this->get_parameter("pose_graph_dir", pose_graph_dir_);
  RCLCPP_INFO(this->get_logger(), "pose_graph_dir: %s", pose_graph_dir_.c_str());

  declare_parameter("sub_map_search_radius", rclcpp::ParameterValue(50.0));
  this->get_parameter("sub_map_search_radius", sub_map_search_radius_);
  RCLCPP_INFO(this->get_logger(), "sub_map_search_radius: %.1f", sub_map_search_radius_);

  declare_parameter("sub_map_warmup_trigger_distance", rclcpp::ParameterValue(20.0));
  this->get_parameter("sub_map_warmup_trigger_distance", sub_map_warmup_trigger_distance_);
  RCLCPP_INFO(this->get_logger(), "sub_map_warmup_trigger_distance: %.1f", sub_map_warmup_trigger_distance_);

  declare_parameter("complete_map_voxel_size", rclcpp::ParameterValue(0.3f));
  this->get_parameter("complete_map_voxel_size", complete_map_voxel_size_);
  RCLCPP_INFO(this->get_logger(), "complete_map_voxel_size: %.2f", complete_map_voxel_size_);


  //@ Latched topic, Create a publisher using the QoS settings to emulate a ROS1 latched topic
  pub_sub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapground",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_map_warmup_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapcloud_warmup",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_sub_ground_warmup_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sub_mapground_warmup",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapcloud",
              rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("mapground",
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());  

  timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  readPoseGraph();

  warm_up_timer_ = this->create_wall_timer(200ms, std::bind(&SubMaps::warmUpThread, this), timer_group_);
}

SubMaps::~SubMaps(){
  delete access_;
}

void SubMaps::readPoseGraph(){

  /*
  pcd_poses_ is original data. It is map frame to base_link, each pcd is base_link frame
  */

  pcd_poses_.reset(new pcl::PointCloud<PointTypePose>());
  if (pcl::io::loadPCDFile<PointTypePose> (pose_graph_dir_ + "/poses.pcd", *pcd_poses_) == -1) //* load the file
  {
    RCLCPP_ERROR(this->get_logger(), "Read poses PCD file fail: %s", pose_graph_dir_.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "Poses read: %lu", pcd_poses_->points.size());
  
  for(unsigned int it=0; it<pcd_poses_->points.size(); it++){
    //@ something like 0_feature.pcd
    std::string feature_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_feature.pcd";
    pcl::PointCloud<pcl_t>::Ptr a_feature_pcd(new pcl::PointCloud<pcl_t>());
    if (pcl::io::loadPCDFile<pcl_t> (feature_file_dir, *a_feature_pcd) == -1) //* load the file
    {
      RCLCPP_ERROR(this->get_logger(), "Read feature PCD file fail: %s", feature_file_dir.c_str());
    }
    
    cornerCloudKeyFrames_baselink_.push_back(a_feature_pcd);

    std::string ground_file_dir = pose_graph_dir_ + "/pcd/" + std::to_string(it) + "_ground.pcd";
    pcl::PointCloud<pcl_t>::Ptr a_ground_pcd(new pcl::PointCloud<pcl_t>());
    if (pcl::io::loadPCDFile<pcl_t> (ground_file_dir, *a_ground_pcd) == -1) //* load the file
    {
      RCLCPP_ERROR(this->get_logger(), "Read ground PCD file fail: %s", ground_file_dir.c_str());
    }
    surfCloudKeyFrames_baselink_.push_back(a_ground_pcd);
  }

  pcl::PointCloud<pcl_t>::Ptr map_cloud (new pcl::PointCloud<pcl_t>);
  pcl::PointCloud<pcl_t>::Ptr map_ground (new pcl::PointCloud<pcl_t>);  
  poses_ = *pcd_poses_;
  //@----- update keyframe poses ---
  cornerCloudKeyFrames_.clear();
  pcl::PointCloud<pcl_t>::Ptr poses_pcl_t(new pcl::PointCloud<pcl_t>());
  for(unsigned int it=0; it<poses_.points.size(); it++){
    //@ something like 0_feature.pcd
    pcl::PointCloud<pcl_t>::Ptr a_feature_pcd(new pcl::PointCloud<pcl_t>());
    pcl::PointCloud<pcl_t> a_feature_pcd_baselink = *cornerCloudKeyFrames_baselink_[it];
    geometry_msgs::msg::TransformStamped trans_m2b;
    Eigen::Affine3d trans_m2b_af3;
    trans_m2b.transform.translation.x = poses_.points[it].x;
    trans_m2b.transform.translation.y = poses_.points[it].y;
    trans_m2b.transform.translation.z = poses_.points[it].z;
    tf2::Quaternion q;
    q.setRPY( poses_.points[it].roll, poses_.points[it].pitch, poses_.points[it].yaw);
    trans_m2b.transform.rotation.x = q.x(); trans_m2b.transform.rotation.y = q.y();
    trans_m2b.transform.rotation.z = q.z(); trans_m2b.transform.rotation.w = q.w();
    trans_m2b_af3 = tf2::transformToEigen(trans_m2b);
    //@transform to map frame
    pcl::transformPointCloud(a_feature_pcd_baselink, *a_feature_pcd, trans_m2b_af3);
    cornerCloudKeyFrames_.push_back(a_feature_pcd);
    *map_cloud += (*a_feature_pcd);

    pcl::PointCloud<pcl_t>::Ptr a_ground_pcd(new pcl::PointCloud<pcl_t>());
    pcl::PointCloud<pcl_t> a_ground_pcd_baselink = *surfCloudKeyFrames_baselink_[it];
    pcl::transformPointCloud(a_ground_pcd_baselink, *a_ground_pcd, trans_m2b_af3);
    surfCloudKeyFrames_.push_back(a_ground_pcd);
    *map_ground += (*a_ground_pcd);

    pcl_t pt;
    pt.x = poses_.points[it].x;
    pt.y = poses_.points[it].y;
    pt.z = poses_.points[it].z;
    poses_pcl_t->push_back(pt);
  }

  kdtree_poses_.reset(new pcl::KdTreeFLANN<pcl_t>());
  kdtree_poses_->setInputCloud(poses_pcl_t);

  RCLCPP_INFO(this->get_logger(), "\033[1;32mPose graph is loaded.\033[0m ");

  RCLCPP_INFO(this->get_logger(), "Map pointcloud size: %lu", map_cloud->points.size());

  //@ euc to filter noisy data
  pcl::PointCloud<pcl_t>::Ptr map_cloud_after_euc (new pcl::PointCloud<pcl_t>);
  pcl::search::KdTree<pcl_t>::Ptr pc_kdtree (new pcl::search::KdTree<pcl_t>);
  pc_kdtree->setInputCloud (map_cloud);
  std::vector<pcl::PointIndices> cluster_indices_segmentation;
  pcl::EuclideanClusterExtraction<pcl_t> ec_segmentation;
  ec_segmentation.setClusterTolerance (0.2);
  ec_segmentation.setMinClusterSize (1);
  ec_segmentation.setMaxClusterSize (map_cloud->points.size());
  ec_segmentation.setSearchMethod (pc_kdtree);
  ec_segmentation.setInputCloud (map_cloud);
  ec_segmentation.extract (cluster_indices_segmentation);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_segmentation.begin (); it != cluster_indices_segmentation.end (); ++it)
  {
    if(it->indices.size()<10)
      continue;

    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      map_cloud_after_euc->push_back(map_cloud->points[(*pit)]);
    } 
  }

  pcl::VoxelGrid<pcl_t> sor_map;
  sor_map.setInputCloud (map_cloud_after_euc);
  sor_map.setLeafSize (complete_map_voxel_size_, complete_map_voxel_size_, complete_map_voxel_size_);
  sor_map.filter (*map_cloud_after_euc);
  map_cloud_after_euc->is_dense = false;
  std::vector<int> ind_map;
  pcl::removeNaNFromPointCloud(*map_cloud_after_euc, *map_cloud_after_euc, ind_map);
  RCLCPP_INFO(this->get_logger(), "Map pointcloud size after down size: %lu", map_cloud_after_euc->points.size());
  sensor_msgs::msg::PointCloud2 map_pc;
  pcl::toROSMsg(*map_cloud_after_euc, map_pc);
  map_pc.header.frame_id = "map";
  pub_map_->publish(map_pc);

  //@ inflate ground to deal with sparse issue
  pcl::PointCloud<pcl_t>::Ptr ground_cloud_after_patched (new pcl::PointCloud<pcl_t>);
  pcl::search::KdTree<pcl_t>::Ptr pc_ground_kdtree (new pcl::search::KdTree<pcl_t>);
  pc_ground_kdtree->setInputCloud (map_ground);
  for(auto it=map_ground->points.begin();it!=map_ground->points.end();it++){

    //@ kd tree search nearby points
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(pc_ground_kdtree->radiusSearch ((*it), 0.3, pointIdxRadiusSearch, pointRadiusSquaredDistance)<10){
      ground_cloud_after_patched->push_back((*it));
      continue;
    }
    
    pcl::PointCloud<pcl_t>::Ptr a_ground_patch (new pcl::PointCloud<pcl_t>);
    for(auto m_id=pointIdxRadiusSearch.begin(); m_id!=pointIdxRadiusSearch.end(); m_id++){
      a_ground_patch->push_back(map_ground->points[(*m_id)]);
    }
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl_t> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (a_ground_patch);
    seg.segment (*inliers, *coefficients);
    // rasanc Ax+By+Cz+D = 0
    RCLCPP_DEBUG(this->get_logger(), "coef: %f, %f, %f, %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
    /*Find min-max xy*/
    pcl_t min_pt, max_pt;
    pcl::getMinMax3D(*a_ground_patch, min_pt, max_pt);

    for(double xi=min_pt.x; xi<max_pt.x; xi+=0.05){
      for(double yi=min_pt.y; yi<max_pt.y; yi+=0.05){
        pcl_t ipt;
        ipt.x = xi;
        ipt.y = yi;      
        ipt.z = (-coefficients->values[3]-coefficients->values[0]*xi-coefficients->values[1]*yi)/coefficients->values[2];
        double dz = (*it).z - ipt.z;
        if(fabs(dz)>0.03)
          continue;
        ground_cloud_after_patched->push_back(ipt);
      }
    }
  }
  //@ patch along the pose
  for(unsigned int it=0; it<poses_.points.size(); it++){
    //@ something like 0_feature.pcd
    geometry_msgs::msg::TransformStamped trans_m2b;
    Eigen::Affine3d trans_m2b_af3;
    trans_m2b.transform.translation.x = poses_.points[it].x;
    trans_m2b.transform.translation.y = poses_.points[it].y;
    trans_m2b.transform.translation.z = poses_.points[it].z;

    //@ find closest point from the pose to ground, and shit its z
    pcl_t pose_pt; pose_pt.x = poses_.points[it].x; pose_pt.y = poses_.points[it].y; pose_pt.z = poses_.points[it].z;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(pc_ground_kdtree->radiusSearch (pose_pt, 1.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)<0){
      continue;
    }
    trans_m2b.transform.translation.z = map_ground->points[pointIdxRadiusSearch[0]].z;

    tf2::Quaternion q;
    q.setRPY( poses_.points[it].roll, poses_.points[it].pitch, poses_.points[it].yaw);
    trans_m2b.transform.rotation.x = q.x(); trans_m2b.transform.rotation.y = q.y();
    trans_m2b.transform.rotation.z = q.z(); trans_m2b.transform.rotation.w = q.w();
    trans_m2b_af3 = tf2::transformToEigen(trans_m2b);

    tf2::Quaternion rotation(trans_m2b.transform.rotation.x, trans_m2b.transform.rotation.y, trans_m2b.transform.rotation.z, trans_m2b.transform.rotation.w);
    tf2::Vector3 vector(0, 0, 1);
    tf2::Vector3 pose_normal = tf2::quatRotate(rotation, vector);

    double d = -trans_m2b.transform.translation.x*pose_normal[0]-trans_m2b.transform.translation.y*pose_normal[1]-trans_m2b.transform.translation.z*pose_normal[2];
    for(double xi=poses_.points[it].x-0.5; xi<poses_.points[it].x+0.5; xi+=0.05){
      for(double yi=poses_.points[it].y-0.5; yi<poses_.points[it].y+0.5; yi+=0.05){
        pcl_t ipt;
        ipt.x = xi;
        ipt.y = yi;      
        ipt.z = (-d-pose_normal[0]*xi-pose_normal[1]*yi)/pose_normal[2];
        double dz = trans_m2b.transform.translation.z - ipt.z;
        if(fabs(dz)>0.03)
          continue;
        ground_cloud_after_patched->push_back(ipt);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Ground pointcloud size: %lu", ground_cloud_after_patched->points.size());
  pcl::VoxelGrid<pcl_t> sor_ground;
  sor_ground.setInputCloud (ground_cloud_after_patched);
  sor_ground.setLeafSize (complete_map_voxel_size_, complete_map_voxel_size_, complete_map_voxel_size_);
  sor_ground.filter (*ground_cloud_after_patched);
  ground_cloud_after_patched->is_dense = false;
  std::vector<int> ind_ground;
  pcl::removeNaNFromPointCloud(*ground_cloud_after_patched, *ground_cloud_after_patched, ind_ground);
  RCLCPP_INFO(this->get_logger(), "Ground pointcloud size after down size: %lu", ground_cloud_after_patched->points.size());
  sensor_msgs::msg::PointCloud2 ground_pc;
  pcl::toROSMsg(*ground_cloud_after_patched, ground_pc);
  ground_pc.header.frame_id = "map";
  pub_ground_->publish(ground_pc);
  
  RCLCPP_INFO(this->get_logger(), "\033[1;32mMap and Ground published.\033[0m ");

  is_initial_ = true;
}

void SubMaps::warmUpThread(){
  
  if(!is_initial_)
    return;
  
  if(!is_current_ready_){
    mcl_3dl::pcl_t target_pose;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    target_pose.x = robot_pose_.pose.pose.position.x;
    target_pose.y = robot_pose_.pose.pose.position.y;
    target_pose.z = robot_pose_.pose.pose.position.z;
    if(kdtree_poses_->radiusSearch(target_pose, sub_map_search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance,0)<1)
    {
      is_current_ready_ = false;
      return;
    }
    map_current_.reset(new pcl::PointCloud<pcl_t>());
    ground_current_.reset(new pcl::PointCloud<pcl_t>());
    for(auto it=pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end(); it++){
      *map_current_ += (*cornerCloudKeyFrames_[*it]);
      *ground_current_ += (*surfCloudKeyFrames_[*it]);
    }

    kdtree_ground_current_.setInputCloud(ground_current_);
    kdtree_map_current_.setInputCloud(map_current_);   
    //@Normal estimation for ground
    pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n;
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    tree->setInputCloud (ground_current_);
    n.setInputCloud (ground_current_);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (normals_ground_current_);

    sensor_msgs::msg::PointCloud2 map_pc;
    pcl::toROSMsg(*map_current_, map_pc);
    map_pc.header.frame_id = "map";
    pub_sub_map_->publish(map_pc);

    sensor_msgs::msg::PointCloud2 ground_pc;
    pcl::toROSMsg(*ground_current_, ground_pc);
    ground_pc.header.frame_id = "map";
    pub_sub_ground_->publish(ground_pc);
    
    current_sub_map_pose_ = robot_pose_;
    RCLCPP_INFO(this->get_logger(), "All essential KD-tree are generated.");
    
    is_current_ready_ = true;
  }
  
  else if(prepare_warm_up_ && !is_warm_up_ready_){
    mcl_3dl::pcl_t target_pose;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    target_pose.x = warm_up_pose_.pose.pose.position.x;
    target_pose.y = warm_up_pose_.pose.pose.position.y;
    target_pose.z = warm_up_pose_.pose.pose.position.z;
    RCLCPP_INFO(this->get_logger(), "Prepare warm up at: %.2f, %.2f, %.2f", target_pose.x, target_pose.y, target_pose.z);
    if(kdtree_poses_->radiusSearch(target_pose, sub_map_search_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance,0)<1)
    {
      prepare_warm_up_ = false;
      return;
    }
    map_warmup_.reset(new pcl::PointCloud<pcl_t>());
    ground_warmup_.reset(new pcl::PointCloud<pcl_t>());
    for(auto it=pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end(); it++){
      *map_warmup_ += (*cornerCloudKeyFrames_[*it]);
      *ground_warmup_ += (*surfCloudKeyFrames_[*it]);
    }
    kdtree_ground_warmup_.setInputCloud(ground_warmup_);
    kdtree_map_warmup_.setInputCloud(map_warmup_);   
    //@Normal estimation for ground
    pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n;
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    tree->setInputCloud (ground_warmup_);
    n.setInputCloud (ground_warmup_);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (normals_ground_warmup_);
    sensor_msgs::msg::PointCloud2 map_pc;
    pcl::toROSMsg(*map_warmup_, map_pc);
    map_pc.header.frame_id = "map";
    pub_sub_map_warmup_->publish(map_pc);

    sensor_msgs::msg::PointCloud2 ground_pc;
    pcl::toROSMsg(*ground_warmup_, ground_pc);
    ground_pc.header.frame_id = "map";
    pub_sub_ground_warmup_->publish(ground_pc);
    
    RCLCPP_INFO(this->get_logger(), "All essential Warmup KD-tree are generated.");
    prepare_warm_up_ = false;
    is_warm_up_ready_ = true;
  }
}

bool SubMaps::isWarmUpReady(){
  return is_warm_up_ready_;
}

void SubMaps::swapKdTree(){
  
  map_current_.reset(new pcl::PointCloud<pcl_t>());
  ground_current_.reset(new pcl::PointCloud<pcl_t>());  
  map_current_ = map_warmup_;
  ground_current_ = ground_warmup_;
  kdtree_map_current_ = kdtree_map_warmup_;
  kdtree_ground_current_ = kdtree_ground_warmup_;
  normals_ground_current_ = normals_ground_warmup_;
  current_sub_map_pose_ = warm_up_pose_;

  sensor_msgs::msg::PointCloud2 map_pc;
  pcl::toROSMsg(*map_warmup_, map_pc);
  map_pc.header.frame_id = "map";
  pub_sub_map_->publish(map_pc);

  sensor_msgs::msg::PointCloud2 ground_pc;
  pcl::toROSMsg(*ground_warmup_, ground_pc);
  ground_pc.header.frame_id = "map";
  pub_sub_ground_->publish(ground_pc);

  is_warm_up_ready_ = false;
  RCLCPP_INFO(this->get_logger(), "KD-tree swapped.");
}

void SubMaps::setInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose){
  RCLCPP_INFO(this->get_logger(), "Receive initial pose at: %.2f, %.2f, %.2f", pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z);
  prepare_warm_up_ = true;
  warm_up_pose_ = pose;
}

void SubMaps::setPose(const geometry_msgs::msg::PoseWithCovarianceStamped pose){
  robot_pose_ = pose;
  double dx = pose.pose.pose.position.x - current_sub_map_pose_.pose.pose.position.x;
  double dy = pose.pose.pose.position.y - current_sub_map_pose_.pose.pose.position.y;
  double dz = pose.pose.pose.position.z - current_sub_map_pose_.pose.pose.position.z;
  if(!is_warm_up_ready_ && sqrt(dx*dx + dy*dy + dz*dz)>=sub_map_warmup_trigger_distance_){
    prepare_warm_up_ = true;
    warm_up_pose_ = pose;
    RCLCPP_INFO(this->get_logger(), "Ready to warm up");
  }
}

}