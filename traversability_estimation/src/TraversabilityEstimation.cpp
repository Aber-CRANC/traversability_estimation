/*
 * TraversabilityEstimation.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"

#include <traversability_interfaces/msg/traversability_result.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <utility>

#include "traversability_estimation/common.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace traversability_estimation
{

TraversabilityEstimation::TraversabilityEstimation(const rclcpp::NodeOptions & options)
: Node("traversability_estimation", options),
  getImageCallback_(false),
  traversabilityMap_(options),
  acceptGridMapToInitTraversabilityMap_(false),
  updateDuration_(0.0),
  traversabilityType_("traversability"),
  slopeType_("traversability_slope"),
  stepType_("traversability_step"),
  roughnessType_("traversability_roughness"),
  robotSlopeType_("robot_slope"),
  useRawMap_(false)
{
  RCLCPP_DEBUG(get_logger(), "Traversability estimation node started.");
  readParameters();
  traversabilityMap_.createLayers(useRawMap_);
  // submapClient_ = create_client<grid_map_msgs::srv::GetGridMap>(submapServiceName_);

  // if (updateDuration_.nanoseconds() != 0) {
  //   create_wall_timer(
  //     std::chrono::duration<double, std::nano>(updateDuration_.nanoseconds()),
  //     std::bind(&TraversabilityEstimation::updateTraversability, this));
  // } else {
  //   RCLCPP_WARN(get_logger(), "Update rate is zero. No traversability map will be published.");
  // }

  // Services offered
  // loadElevationMapService_ = create_service<grid_map_msgs::srv::ProcessFile>(
  //   "load_elevation_map",
  //   std::bind(&TraversabilityEstimation::loadElevationMap, this, _1, _2));
  // updateTraversabilityService_ = create_service<grid_map_msgs::srv::GetGridMapInfo>(
  //   "update_traversability",
  //   std::bind(&TraversabilityEstimation::updateServiceCallback, this, _1, _2));
  // getTraversabilityService_ = create_service<grid_map_msgs::srv::GetGridMap>(
  //   "get_traversability",
  //   std::bind(&TraversabilityEstimation::getTraversabilityMap, this, _1, _2));
  // footprintPathService_ = create_service<traversability_interfaces::srv::CheckFootprintPath>(
  //   "check_footprint_path",
  //   std::bind(&TraversabilityEstimation::checkFootprintPath, this, _1, _2));
  // TODO(SivertHavso): Add back the saveToBagService
  // updateParameters_ = create_service<std_srvs::srv::Empty>(
  //   "update_parameters",
  //   std::bind(&TraversabilityEstimation::updateParameter, this, _1, _2));
  // traversabilityFootprint_ = create_service<std_srvs::srv::Empty>(
  //   "traversability_footprint",
  //   std::bind(&TraversabilityEstimation::traversabilityFootprint, this, _1, _2));
  // TODO(SivertHavso): Add back the saveToBagService
  // saveToBagService_ = create_service<grid_map_msgs::srv::ProcessFile>(
  //   "save_traversability_map_to_bag",
  //   std::bind(&TraversabilityEstimation::saveToBag, this, _1, _2));

  // Subscriptions
  // imageSubscriber_ = create_subscription<sensor_msgs::msg::Image>(
  //   imageTopic_, 1,
  //   std::bind(&TraversabilityEstimation::imageCallback, this, _1));

  if (acceptGridMapToInitTraversabilityMap_) {
    gridMapToInitTraversabilityMapSubscriber_ =
      create_subscription<grid_map_msgs::msg::GridMap>(
      gridMapToInitTraversabilityMapTopic_, 1,
      std::bind(
        &TraversabilityEstimation::gridMapToInitTraversabilityMapCallback, this,
        _1));
  }

  elevationMapLayers_.push_back("elevation");
  if (!useRawMap_) {
    elevationMapLayers_.push_back("upper_bound");
    elevationMapLayers_.push_back("lower_bound");
  } else {
    elevationMapLayers_.push_back("variance");
    elevationMapLayers_.push_back("horizontal_variance_x");
    elevationMapLayers_.push_back("horizontal_variance_y");
    elevationMapLayers_.push_back("horizontal_variance_xy");
    elevationMapLayers_.push_back("time");
  }
}

TraversabilityEstimation::~TraversabilityEstimation()
{
}

bool TraversabilityEstimation::readParameters()
{
  // Read boolean to switch between raw and fused map.
  declare_parameter("use_raw_map", false);
  declare_parameter("submap_service", "get_grid_map");
  declare_parameter("min_update_rate", 1.0);
  declare_parameter("image_topic", "image_elevation");
  declare_parameter("min_height", 0.0);
  declare_parameter("max_height", 1.0);
  declare_parameter("image_position_x", 0.0);
  declare_parameter("image_position_y", 0.0);
  declare_parameter("robot_frame_id", "base_link");
  declare_parameter("robot", "robot");
  declare_parameter("package", "traversability_estimation");
  declare_parameter("map_center_x", 0.0);
  declare_parameter("map_center_y", 0.0);
  declare_parameter("map_length_x", 5.0);
  declare_parameter("map_length_y", 5.0);
  declare_parameter("footprint_yaw", M_PI_2);
  declare_parameter("grid_map_to_initialize_traversability_map.enable", false);
  declare_parameter(
    "grid_map_to_initialize_traversability_map.grid_map_topic_name",
    "initial_elevation_map");

  get_parameter("use_raw_map", useRawMap_);
  get_parameter("submap_service", submapServiceName_);
  get_parameter("image_topic", imageTopic_);
  get_parameter("min_height", imageMinHeight_);
  get_parameter("max_height", imageMaxHeight_);
  get_parameter("image_position_x", imagePosition_.x());
  get_parameter("image_position_y", imagePosition_.y());
  get_parameter("robot_frame_id", robotFrameId_);
  get_parameter("robot", robot_);
  get_parameter("package", package_);
  get_parameter(
    "grid_map_to_initialize_traversability_map.enable",
    acceptGridMapToInitTraversabilityMap_);
  get_parameter(
    "grid_map_to_initialize_traversability_map.grid_map_topic_name",
    gridMapToInitTraversabilityMapTopic_);


  double updateRate;
  get_parameter("min_update_rate", updateRate);
  if (updateRate != 0.0) {
    updateDuration_ = rclcpp::Duration(1e9 / updateRate);
  } else {
    updateDuration_ = rclcpp::Duration(0);
  }

  grid_map::Position mapCenter;
  get_parameter("map_center_x", mapCenter.x());
  get_parameter("map_center_y", mapCenter.y());
  get_parameter("map_length_x", mapLength_.x());
  get_parameter("map_length_y", mapLength_.y());
  get_parameter("footprint_yaw", footprintYaw_);

  submapPoint_.header.frame_id = robotFrameId_;
  submapPoint_.point.x = mapCenter.x();
  submapPoint_.point.y = mapCenter.y();
  submapPoint_.point.z = 0.0;

  return true;
}

// bool TraversabilityEstimation::loadElevationMap(
//   const grid_map_msgs::srv::ProcessFile_Request::SharedPtr request,
//   grid_map_msgs::srv::ProcessFile_Response::SharedPtr response)
// {
//   RCLCPP_INFO(get_logger(), "TraversabilityEstimation: loadElevationMap");
//   if (request->file_path.empty() || request->topic_name.empty()) {
//     RCLCPP_WARN(
//       get_logger(), "Fields 'file_path' and 'topic_name' in service request must be filled in.");
//     response->success = static_cast<unsigned char>(false);
//     return true;
//   }

//   grid_map::GridMap map;
//   if (!grid_map::GridMapRosConverter::loadFromBag(request->file_path, request->topic_name, map)) {
//     RCLCPP_ERROR(
//       get_logger(),
//       "Cannot find bag '%s' or topic '%s' of the elevation map!",
//       request->file_path.c_str(),
//       request->topic_name.c_str());
//     response->success = static_cast<unsigned char>(false);
//   } else {
//     map.setTimestamp(get_clock()->now().nanoseconds());
//     if (!initializeTraversabilityMapFromGridMap(map)) {
//       RCLCPP_ERROR_STREAM(
//         get_logger(),
//         "loadElevationMap: it was not possible to load elevation map from bag with path '" <<
//           request->file_path << "' and topic '" << request->topic_name << "'.");
//       response->success = static_cast<unsigned char>(false);
//     } else {
//       response->success = static_cast<unsigned char>(true);
//     }
//   }

//   return true;
// }

void TraversabilityEstimation::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr image)
{
  if (!getImageCallback_) {
    grid_map::GridMapRosConverter::initializeFromImage(
      *image, imageResolution_, imageGridMap_,
      imagePosition_);
    RCLCPP_INFO(
      get_logger(),
      "Initialized map with size %f x %f m (%i x %i cells).",
      imageGridMap_.getLength().x(), imageGridMap_.getLength().y(),
      imageGridMap_.getSize()(0), imageGridMap_.getSize()(1));
    imageGridMap_.add("upper_bound", 0.0);  // TODO(Unknown): Add value for layers.
    imageGridMap_.add("lower_bound", 0.0);
    imageGridMap_.add(
      "uncertainty_range",
      imageGridMap_.get("upper_bound") - imageGridMap_.get("lower_bound"));
    getImageCallback_ = true;
  }
  grid_map::GridMapRosConverter::addLayerFromImage(
    *image, "elevation", imageGridMap_,
    imageMinHeight_, imageMaxHeight_);
  // grid_map_msgs::msg::GridMap::UniquePtr elevationMap;
  traversabilityMap_.setElevationMap(imageGridMap_);
}

// bool TraversabilityEstimation::updateServiceCallback(
//   grid_map_msgs::srv::GetGridMapInfo_Request::SharedPtr,
//   grid_map_msgs::srv::GetGridMapInfo_Response::SharedPtr response)
// {
//   if (updateDuration_.nanoseconds() == 0) {
//     updateTraversability();
//   }
//   // Wait until traversability map is computed.
//   if (!traversabilityMap_.traversabilityMapInitialized()) {
//     RCLCPP_WARN(
//       get_logger(),
//       "Cannot respond to update traversability map request because the map is not initialized");
//   }

//   grid_map_msgs::msg::GridMap msg;
//   grid_map::GridMap traversabilityMap = traversabilityMap_.getTraversabilityMap();

//   // response->info.header.frame_id = traversabilityMap_.getMapFrameId();
//   // response->info.header.stamp = ros::Time::now();
//   response->info.resolution = traversabilityMap.getResolution();
//   response->info.length_x = traversabilityMap.getLength()[0];
//   response->info.length_y = traversabilityMap.getLength()[1];
//   geometry_msgs::msg::Pose pose;
//   grid_map::Position position = traversabilityMap.getPosition();
//   pose.position.x = position[0];
//   pose.position.y = position[1];
//   pose.orientation.w = 1.0;
//   response->info.pose = pose;

//   return true;
// }

bool TraversabilityEstimation::updateTraversability(grid_map_msgs::msg::GridMap& elevationMap)
{
  if (!getImageCallback_) {
    RCLCPP_DEBUG(get_logger(), "Checking if service %s is ready.", submapServiceName_.c_str());
    if (!submapClient_->wait_for_service(std::chrono::duration<double, std::nano>(2.0 * 1e9))) {
      return false;
    }
    RCLCPP_DEBUG(get_logger(), "Sending request to %s.", submapServiceName_.c_str());
    if (requestElevationMap(elevationMap)) {
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(), periodThrottledConsoleMessages, "Failed to retrieve elevation grid map.");
      return;
    }
  } else {
    if (!traversabilityMap_.computeTraversability()) {return;}
  }
}

// bool TraversabilityEstimation::updateParameter(
//   std_srvs::srv::Empty_Request::SharedPtr,
//   std_srvs::srv::Empty_Response::SharedPtr) // response)
// {
// // Load parameters file.
// string path = ros::package::getPath(package_);
// string path_filter_parameter = path + "/config/" + robot_ + "_filter_parameter.yaml";
// string path_footprint_parameter = path + "/config/" + robot_ + "_footprint_parameter.yaml";
// // Filter parameters
// string commandString =
//   "rosparam load " + path_filter_parameter + " /traversability_estimation";
// const char * command_filter = commandString.c_str();
// if (system(command_filter) != 0) {
//   RCLCPP_ERROR("Can't update parameter.");
//   return false;
// }
// // Footprint parameters.
// commandString = "rosparam load " + path_footprint_parameter + " /traversability_estimation";
// const char * command_footprint = commandString.c_str();
// if (system(command_footprint) != 0) {
//   RCLCPP_ERROR("Can't update parameter.");
//   return false;
// }

// if (!traversabilityMap_.updateFilter()) {return false;}
//   RCLCPP_WARN(get_logger(), "updateParameter service not supported.");
//   return false;
// }

// bool TraversabilityEstimation::requestElevationMap(grid_map_msgs::msg::GridMap& map)
// {
//   submapPoint_.header.stamp = get_clock()->now();
//   geometry_msgs::msg::PointStamped submapPointTransformed;

//   try {
//     submapPointTransformed = transformBuffer_->transform(
//       submapPoint_,
//       traversabilityMap_.getMapFrameId());
//   } catch (tf2::TransformException & ex) {
//     RCLCPP_ERROR(get_logger(), "%s", ex.what());
//     return false;
//   }

//   std::shared_ptr<grid_map_msgs::srv::GetGridMap> submapService;
//   auto request = std::make_shared<grid_map_msgs::srv::GetGridMap_Request>();
//   request->position_x = submapPointTransformed.point.x;
//   request->position_y = submapPointTransformed.point.y;
//   request->length_x = mapLength_.x();
//   request->length_y = mapLength_.y();
//   request->layers = elevationMapLayers_;

//   using ServiceResponseFuture =
//     rclcpp::Client<grid_map_msgs::srv::GetGridMap>::SharedFutureWithRequest;
//   auto future_result = submapClient_->async_send_request(
//     request,
//     [map](ServiceResponseFuture future) {
//       auto response_pair = future.get();
//       map = response_pair.second->map;
//     });

//   return true;
// }

// bool TraversabilityEstimation::traversabilityFootprint(
//   std_srvs::srv::Empty_Request::SharedPtr,
//   std_srvs::srv::Empty_Response::SharedPtr)
// {
//   if (!traversabilityMap_.traversabilityFootprint(footprintYaw_)) {return false;}

//   return true;
// }

// bool TraversabilityEstimation::checkFootprintPath(
//   traversability_interfaces::srv::CheckFootprintPath_Request::SharedPtr request,
//   traversability_interfaces::srv::CheckFootprintPath_Response::SharedPtr response)
// {
//   const int nPaths = request->path.size();
//   if (nPaths == 0) {
//     RCLCPP_WARN_THROTTLE(
//       get_logger(),
//       *get_clock(), periodThrottledConsoleMessages, "No footprint path available to check!");
//     return false;
//   }

//   traversability_interfaces::msg::TraversabilityResult::UniquePtr result;
//   traversability_interfaces::msg::FootprintPath::UniquePtr path;
//   for (int j = 0; j < nPaths; j++) {
//     *path = request->path[j];
//     if (!traversabilityMap_.checkFootprintPath(std::move(path), std::move(result), true)) {
//       return false;
//     }
//     response->result.push_back(*result);
//   }

//   return true;
// }

// bool TraversabilityEstimation::getTraversabilityMap(
//   grid_map_msgs::srv::GetGridMap_Request::SharedPtr request,
//   grid_map_msgs::srv::GetGridMap::Response::SharedPtr response)
// {
//   grid_map::Position requestedSubmapPosition(request->position_x, request->position_y);
//   grid_map::Length requestedSubmapLength(request->length_x, request->length_y);
//   // grid_map_msgs::msg::GridMap::UniquePtr msg;
//   grid_map::GridMap map, subMap;
//   map = traversabilityMap_.getTraversabilityMap();
//   bool isSuccess;
//   subMap = map.getSubmap(requestedSubmapPosition, requestedSubmapLength, isSuccess);
//   if (request->layers.empty()) {
//     response->map = *grid_map::GridMapRosConverter::toMessage(subMap);
//   } else {
//     std::vector<std::string> layers;
//     for (const auto & layer : request->layers) {
//       layers.push_back(layer);
//     }
//     response->map = *grid_map::GridMapRosConverter::toMessage(subMap, layers);
//   }
//   return isSuccess;
// }

// bool TraversabilityEstimation::saveToBag(
//   grid_map_msgs::srv::ProcessFile_Request::SharedPtr request,
//   grid_map_msgs::srv::ProcessFile_Response::SharedPtr response)
// {
//   RCLCPP_INFO(get_logger(), "Save to bag.");
//   if (request->file_path.empty() || request->topic_name.empty()) {
//     RCLCPP_WARN(
//       get_logger(), "Fields 'file_path' and 'topic_name' in service request must be filled in.");
//     response->success = static_cast<unsigned char>(false);
//     return true;
//   }

//   response->success = static_cast<unsigned char>(
//     grid_map::GridMapRosConverter::saveToBag(
//       traversabilityMap_.getTraversabilityMap(),
//       request->file_path, request->topic_name));
//   return true;
// }

void TraversabilityEstimation::initializeTraversabilityMapFromGridMap(std::unique_ptr<grid_map::GridMap> gridMap)
{
  if (traversabilityMap_.traversabilityMapInitialized()) {
    RCLCPP_WARN(
      get_logger(),
      "Received grid map message cannot be used to initialize"
      " the traversability map, because current traversability map has been already initialized.");
    return;
  }

  // grid_map::GridMap mapWithCheckedLayers = gridMap;
  for (const auto & layer : elevationMapLayers_) {
    if (!gridMap->exists(layer)) {
      gridMap->add(layer, 0.0);
      RCLCPP_INFO_STREAM(
        get_logger(),
        "[initializeTraversabilityMapFromGridMap]: Added layer '" << layer <<
          "'.");
    }
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "Map frame id: " << gridMap->getFrameId());
  for (const auto & layer : gridMap->getLayers()) {
    RCLCPP_DEBUG_STREAM(get_logger(), "Map layers: " << layer);
  }
  RCLCPP_DEBUG_STREAM(get_logger(), "Map size: " << gridMap->getLength());
  RCLCPP_DEBUG_STREAM(get_logger(), "Map position: " << gridMap->getPosition());
  RCLCPP_DEBUG_STREAM(get_logger(), "Map resolution: " << gridMap->getResolution());

  // grid_map_msgs::GridMap message;
  traversabilityMap_.setElevationMap(*gridMap);
  if (!traversabilityMap_.computeTraversability()) {
    RCLCPP_WARN(
      get_logger(),
      "initializeTraversabilityMapFromGridMap: cannot compute traversability.");
    return;
  }
  return;
}

void TraversabilityEstimation::gridMapToInitTraversabilityMapCallback(
  const grid_map_msgs::msg::GridMap::ConstSharedPtr message)
{
  auto gridMap = std::make_unique<grid_map::GridMap>();
  grid_map::GridMapRosConverter::fromMessage(*message, *gridMap);
  initializeTraversabilityMapFromGridMap(std::move(gridMap));
  // if (!initializeTraversabilityMapFromGridMap(std::move(gridMap))) {
  //   RCLCPP_ERROR(
  //     get_logger(),
  //     "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: "
  //     "It was not possible to use received grid map message to initialize traversability map.");
  // } else {
  //   RCLCPP_INFO(
  //     get_logger(),
  //     "[TraversabilityEstimation::gridMapToInitTraversabilityMapCallback]: "
  //     "Traversability Map initialized using received grid map on topic '%s'.",
  //     gridMapToInitTraversabilityMapTopic_.c_str());
  // }
}

}  // namespace traversability_estimation

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
RCLCPP_COMPONENTS_REGISTER_NODE(traversability_estimation::TraversabilityEstimation)
