/*
 * RoughnessFilter.cpp
 *
 *  Created on: Mar 13, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/RoughnessFilter.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

namespace filters
{

template<typename T>
RoughnessFilter<T>::RoughnessFilter()
: criticalValue_(0.3),
  estimationRadius_(0.3),
  type_("traversability_roughness")
{
}

template<typename T>
RoughnessFilter<T>::~RoughnessFilter()
{
}

template<typename T>
bool RoughnessFilter<T>::configure()
{
  auto logger = rclcpp::get_logger("traversability_rougness_filter");

  if (!FilterBase<T>::getParam(std::string("critical_value"), criticalValue_)) {
    RCLCPP_ERROR(logger, "RoughnessFilter did not find param critical_value");
    return false;
  }

  if (criticalValue_ < 0.0) {
    RCLCPP_ERROR(logger, "Critical roughness must be greater than zero");
    return false;
  }

  RCLCPP_DEBUG(logger, "Critical roughness = %f", criticalValue_);

  if (!FilterBase<T>::getParam(std::string("estimation_radius"), estimationRadius_)) {
    RCLCPP_ERROR(logger, "RoughnessFilter did not find param estimation_radius");
    return false;
  }

  if (estimationRadius_ < 0.0) {
    RCLCPP_ERROR(logger, "Roughness estimation radius must be greater than zero");
    return false;
  }

  RCLCPP_DEBUG(logger, "Roughness estimation radius = %f", estimationRadius_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    RCLCPP_ERROR(logger, "RoughnessFilter did not find param map_type");
    return false;
  }

  RCLCPP_DEBUG(logger, "Roughness map type = %s", type_.c_str());

  return true;
}

template<typename T>
bool RoughnessFilter<T>::update(const T & mapIn, T & mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);
  double roughnessMax = 0.0;

  for (grid_map::GridMapIterator iterator(mapOut);
    !iterator.isPastEnd(); ++iterator)
  {
    // Check if this is an empty cell (hole in the map).
    if (!mapOut.isValid(*iterator, "surface_normal_x")) {continue;}

    // Prepare data computation.
    const int maxNumberOfCells = ceil(pow(2 * estimationRadius_ / mapOut.getResolution(), 2));
    Eigen::MatrixXd points(3, maxNumberOfCells);

    // Requested position (center) of circle in map.
    grid_map::Position center;
    mapOut.getPosition(*iterator, center);

    // Gather surrounding data.
    size_t nPoints = 0;
    for (grid_map::CircleIterator submapIterator(mapOut, center, estimationRadius_);
      !submapIterator.isPastEnd(); ++submapIterator)
    {
      if (!mapOut.isValid(*submapIterator, "elevation")) {continue;}
      Eigen::Vector3d point;
      mapOut.getPosition3("elevation", *submapIterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }

    const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;

    // Compute standard deviation of submap.
    double normalX = mapOut.at("surface_normal_x", *iterator);
    double normalY = mapOut.at("surface_normal_y", *iterator);
    double normalZ = mapOut.at("surface_normal_z", *iterator);
    double planeParameter = mean.x() * normalX + mean.y() * normalY + mean.z() * normalZ;
    double sum = 0.0;
    for (size_t i = 0; i < nPoints; i++) {
      double dist = normalX * points(0, i) + normalY * points(1, i) +
        normalZ * points(2, i) - planeParameter;
      sum += pow(dist, 2);
    }
    double roughness = sqrt(sum / (nPoints - 1));

    if (roughness < criticalValue_) {
      mapOut.at(type_, *iterator) = 1.0 - roughness / criticalValue_;
    } else {
      mapOut.at(type_, *iterator) = 0.0;
    }

    if (roughness > roughnessMax) {roughnessMax = roughness;}
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("traversability_slope_filter"), "roughness max = %f",
    roughnessMax);

  return true;
}

}  // namespace filters

PLUGINLIB_EXPORT_CLASS(
  filters::RoughnessFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
