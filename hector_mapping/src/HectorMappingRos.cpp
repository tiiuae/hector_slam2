//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "hector_mapping/HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "hector_mapping/HectorDrawings.h"
#include "hector_mapping/HectorDebugInfoProvider.h"
#include "hector_mapping/HectorMapMutex.h"

#include <pluginlib/class_list_macros.hpp>

#ifndef TF2_SCALAR_H
typedef btScalar tfScalar;
#endif

namespace hector_mapping
{

/*Constructor//{*/
HectorMappingRos::HectorMappingRos(rclcpp::NodeOptions options) : Node("hector_mapping_ros", options) {

  std::string mapTopic_ = "map";

  p_pub_drawings_               = this->declare_parameter("pub_drawings", false);
  p_pub_debug_output_           = this->declare_parameter("pub_debug_output", false);
  p_pub_map_odom_transform_     = this->declare_parameter("pub_map_odom_transform", true);
  p_pub_odometry_               = this->declare_parameter("pub_odometry", false);
  p_advertise_map_service_      = this->declare_parameter("advertise_map_service", true);
  p_scan_subscriber_queue_size_ = this->declare_parameter("scan_subscriber_queue_size", 5);

  p_map_resolution_       = this->declare_parameter("map_resolution", 0.025);
  p_map_size_             = this->declare_parameter("map_size", 1024);
  p_map_start_x_          = this->declare_parameter("map_start_x", 0.5);
  p_map_start_y_          = this->declare_parameter("map_start_y", 0.5);
  p_map_multi_res_levels_ = this->declare_parameter("map_multi_res_levels", 3);

  p_update_factor_free_     = this->declare_parameter("update_factor_free", 0.4);
  p_update_factor_occupied_ = this->declare_parameter("update_factor_occupied", 0.9);

  p_map_update_distance_threshold_ = this->declare_parameter("map_update_distance_thresh", 0.4);
  p_map_update_angle_threshold_    = this->declare_parameter("map_update_angle_thresh", 0.9);

  p_scan_topic_        = this->declare_parameter("scan_topic", std::string("scan"));
  p_sys_msg_topic_     = this->declare_parameter("sys_msg_topic", std::string("syscommand"));
  p_pose_update_topic_ = this->declare_parameter("pose_update_topic", std::string("poseupdate"));

  p_use_tf_scan_transformation_ = this->declare_parameter("use_tf_scan_transformation", true);
  p_use_tf_pose_start_estimate_ = this->declare_parameter("use_tf_pose_start_estimate", false);
  p_map_with_known_poses_       = this->declare_parameter("map_with_known_poses", false);

  p_base_frame_ = this->declare_parameter("base_frame", std::string("base_link"));
  p_map_frame_  = this->declare_parameter("map_frame", std::string("map"));
  p_odom_frame_ = this->declare_parameter("odom_frame", std::string("odom"));

  p_pub_map_scanmatch_transform_           = this->declare_parameter("pub_map_scanmatch_transform", true);
  p_tf_map_scanmatch_transform_frame_name_ = this->declare_parameter("tf_map_scanmatch_transform_frame_name", std::string("scanmatcher_frame"));

  p_timing_output_ = this->declare_parameter("output_timing", false);

  p_map_pub_period_ = this->declare_parameter("map_pub_period", 2.0);

  double tmp            = 0.0;
  tmp                   = this->declare_parameter("laser_min_dist", 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp * tmp);

  tmp                   = this->declare_parameter("laser_max_dist", 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp * tmp);

  tmp                  = this->declare_parameter("laser_z_min_value", -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  tmp                  = this->declare_parameter("laser_z_max_value", 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  debugInfoProvider     = 0;
  hectorDrawings        = 0;
  lastGetMapUpdateIndex = 100;
  /* tfB_                  = 0; */
  map__publish_thread_ = 0;
  initial_pose_set_    = false;

  if (p_pub_drawings_) {
    RCLCPP_INFO(this->get_logger(), "HectorSM publishing debug drawings");
    hectorDrawings = std::make_shared<HectorDrawings>();
  }

  if (p_pub_debug_output_) {
    RCLCPP_INFO(this->get_logger(), "HectorSM publishing debug info");
    debugInfoProvider = std::make_shared<HectorDebugInfoProvider>();
  }

  if (p_pub_odometry_) {
    odometryPublisher_ = this->create_publisher<nav_msgs::msg::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = std::make_unique<hectorslam::HectorSlamProcessor>(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_,
                                                                    Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings,
                                                                    debugInfoProvider);

  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels     = 1;

  for (int i = 0; i < mapLevels; ++i) {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0) {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_          = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mapTopicStr, 1);
    tmp.mapMetadataPublisher_  = this->create_publisher<nav_msgs::msg::MapMetaData>(mapMetaTopicStr, 1);

    if ((i == 0) && p_advertise_map_service_) {
      tmp.dynamicMapServiceServer_ = this->create_service<nav_msgs::srv::GetMap>("dynamic_map", &HectorMappingRos::mapCallback);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if (i == 0) {
      mapPubContainer[i].mapMetadataPublisher_->publish(mapPubContainer[i].map_.map.info);
    }
  }

  RCLCPP_INFO(this->get_logger(), "HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  RCLCPP_INFO(this->get_logger(), "HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  RCLCPP_INFO(this->get_logger(), "HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  RCLCPP_INFO(this->get_logger(), "HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(p_scan_topic_, p_scan_subscriber_queue_size_,
                                                                           std::bind(&HectorMappingRos::scanCallback, this, std::placeholders::_1));
  sysMsgSubscriber_ =
      this->create_subscription<std_msgs::msg::String>(p_sys_msg_topic_, 2, std::bind(&HectorMappingRos::sysMsgCallback, this, std::placeholders::_1));

  poseUpdatePublisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(p_pose_update_topic_, 1);
  posePublisher_       = this->create_publisher<geometry_msgs::msg::PoseStamped>("slam_out_pose", 1);

  scan_point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("slam_cloud", 1);

  /* tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(); */
  tfB_ = nullptr;
  /* ROS_ASSERT(tfB_); */
  /* TODO: replacement in ros2 */

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_->subscribe(this, "initialpose");
  initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));

  map__publish_thread_ = std::make_unique<boost::thread>(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = rclcpp::Time(0, 0);

  // | ---------------------- listener ----------------------- |

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

} /*//}*/

/* HectorMappingRos::~HectorMappingRos() { */

/*   /1* if (hectorDrawings) *1/ */
/*   /1*   delete hectorDrawings; *1/ */

/*   /1* if (debugInfoProvider) *1/ */
/*   /1*   delete debugInfoProvider; *1/ */

/*   /1* if (tfB_) *1/ */
/*   /1*   delete tfB_; *1/ */

/*   /1* if (map__publish_thread_) *1/ */
/*   /1*   delete map__publish_thread_; *1/ */
/* } */

void HectorMappingRos::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  if (hectorDrawings) {
    hectorDrawings->setTime(scan->header.stamp);
  }

  rclcpp::Time startTime = rclcpp::Node::now();

  if (!p_use_tf_scan_transformation_) {
    if (rosLaserScanToDataContainer(scan, laserScanContainer, slamProcessor->getScaleToMap())) {
      slamProcessor->update(laserScanContainer, slamProcessor->getLastScanMatchPose());
    }
  } else {
    rclcpp::Duration dur(0.5);

    try {
      laser_transform_ = tf_buffer_->lookupTransform(p_base_frame_, scan->header.frame_id, scan->header.stamp);
    }
    catch (tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "[HectorMappingRos]: '%s'", ex.what());
    }

    // projector_.transformLaserScanToPointCloud(p_base_frame_ ,scan, pointCloud,tf_);
    projector_.projectLaser(*scan, laser_point_cloud_, 30.0);

    if (this->count_subscribers("slam_cloud") > 0) {
      scan_point_cloud_publisher_->publish(laser_point_cloud_);
    }

    Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

    if (rosPointCloudToDataContainer(laser_point_cloud_, laserTransform, laserScanContainer, slamProcessor->getScaleToMap())) {
      if (initial_pose_set_) {
        initial_pose_set_ = false;
        startEstimate     = initial_pose_;
      } else if (p_use_tf_pose_start_estimate_) {

        try {
          tf::StampedTransform stamped_pose;

          tf_.waitForTransform(p_map_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
          tf_.lookupTransform(p_map_frame_, p_base_frame_, scan.header.stamp, stamped_pose);

          tfScalar yaw, pitch, roll;
          stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);

          startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(), stamped_pose.getOrigin().getY(), yaw);
        }
        catch (tf::TransformException e) {
          ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_.c_str());
          startEstimate = slamProcessor->getLastScanMatchPose();
        }

      } else {
        startEstimate = slamProcessor->getLastScanMatchPose();
      }


      if (p_map_with_known_poses_) {
        slamProcessor->update(laserScanContainer, startEstimate, true);
      } else {
        slamProcessor->update(laserScanContainer, startEstimate);
      }
    }

    /* } else { */
    /*   RCLCPP_INFO(this->get_logger(), "lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_.c_str(), */
    /*               scan.header.frame_id.c_str()); */
    /*   return; */
    /* } */
  }

  if (p_timing_output_) {
    rclcpp::Duration duration = rclcpp::Node::now() - startTime;
    RCLCPP_INFO(this->get_logger(), "HectorSLAM Iter took: %f milliseconds", duration.toSec() * 1000.0f);
  }

  // If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
  if (p_map_with_known_poses_) {
    return;
  }

  poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

  poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
  posePublisher_.publish(poseInfoContainer_.getPoseStamped());

  if (p_pub_odometry_) {
    nav_msgs::Odometry tmp;
    tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

    tmp.header         = poseInfoContainer_.getPoseWithCovarianceStamped().header;
    tmp.child_frame_id = p_base_frame_;
    odometryPublisher_.publish(tmp);
  }

  if (tfB_ == nullptr) {  // initialize with shared_pointer to 'this' Node
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }

  if (p_pub_map_odom_transform_) {
    tf::StampedTransform odom_to_base;

    try {
      tf_.waitForTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
      tf_.lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, odom_to_base);
    }
    catch (tf::TransformException e) {
      ROS_ERROR("Transform failed during publishing of map_odom transform: %s", e.what());
      odom_to_base.setIdentity();
    }
    map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
    tfB_->sendTransform(tf::StampedTransform(map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
  }

  if (p_pub_map_scanmatch_transform_) {
    tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_));
  }
}

void HectorMappingRos::sysMsgCallback(const std_msgs::msg::String::SharedPtr string) {
  RCLCPP_INFO(this->get_logger(), "HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset") {
    RCLCPP_INFO(this->get_logger(), "HectorSM reset");
    slamProcessor->reset();
  }
}

bool HectorMappingRos::mapCallback(const std::shared_ptr<nav_msgs::srv::GetMap::Request> req, std::shared_ptr<nav_msgs::srv::GetMap::Response> res) {
  RCLCPP_INFO(this->get_logger(), "HectorSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex) {
  nav_msgs::GetMap::Response& map_(mapPublisher.map_);

  // only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex()) {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    // std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex) {
      mapMutex->lockMap();
    }

    for (int i = 0; i < size; ++i) {
      if (gridMap.isFree(i)) {
        data[i] = 0;
      } else if (gridMap.isOccupied(i)) {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex) {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map_.map);
}

bool HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::msg::LaserScan::SharedPtr scan, hectorslam::DataContainer& dataContainer,
                                                   float scaleToMap) {
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i) {
    float dist = scan.ranges[i];

    if ((dist > scan.range_min) && (dist < maxRangeForContainer)) {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }

  return true;
}

bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform,
                                                    hectorslam::DataContainer& dataContainer, float scaleToMap) {
  size_t size = pointCloud.points.size();
  // RCLCPP_INFO(this->get_logger(),"size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos(laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y()) * scaleToMap);

  for (size_t i = 0; i < size; ++i) {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x * currPoint.x + currPoint.y * currPoint.y;

    if ((dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_)) {

      if ((currPoint.x < 0.0f) && (dist_sqr < 0.50f)) {
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_) {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(), pointPosBaseFrame.y()) * scaleToMap);
      }
    }
  }

  return true;
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap) {
  Eigen::Vector2f mapOrigin(gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength() * 0.5f;

  map_.map.info.origin.position.x    = mapOrigin.x();
  map_.map.info.origin.position.y    = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width  = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings,
debugInfoProvider);
}
*/


void HectorMappingRos::publishMapLoop(double map_pub_period) {
  ros::Rate r(1.0 / map_pub_period);
  while (ros::ok()) {
    // ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime(ros::Time::now());
    // publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    // publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0], slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    // ros::WallDuration t2 = ros::WallTime::now() - t1;

    // std::cout << "time s: " << t2.toSec();
    // RCLCPP_INFO(this->get_logger(),"HectorSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map) {
}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  initial_pose_set_ = true;

  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  initial_pose_ = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(pose.getRotation()));
  RCLCPP_INFO(this->get_logger(), "Setting initial pose with world coords x: %f y: %f yaw: %f", initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}

}  // namespace hector_mapping

PLUGINLIB_EXPORT_CLASS(hector_mapping::HectorMappingRos, nodelet::Nodelet);
