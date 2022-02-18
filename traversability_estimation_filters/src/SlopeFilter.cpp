/*
 * SlopeFilter.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "filters/SlopeFilter.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

#include <string>

// using namespace grid_map;

namespace filters
{

template<typename T>
SlopeFilter<T>::SlopeFilter()
: criticalValue_(M_PI_4),
  type_("traversability_slope")
{
}

template<typename T>
SlopeFilter<T>::~SlopeFilter()
{
}

template<typename T>
bool SlopeFilter<T>::configure()
{
  auto logger = rclcpp::get_logger("traversability_slope_filter");

  if (!FilterBase<T>::getParam(std::string("critical_value"), criticalValue_)) {
    RCLCPP_ERROR(logger, "SlopeFilter did not find param critical_value");
    return false;
  }

  if (criticalValue_ > M_PI_2 || criticalValue_ < 0.0) {
    RCLCPP_ERROR(logger, "Critical slope must be in the interval [0, PI/2]");
    return false;
  }

  RCLCPP_DEBUG(logger, "critical Slope = %f", criticalValue_);

  if (!FilterBase<T>::getParam(std::string("map_type"), type_)) {
    RCLCPP_ERROR(logger, "SlopeFilter did not find param map_type");
    return false;
  }

  RCLCPP_DEBUG(logger, "Slope map type = %s", type_.c_str());

  return true;
}

template<typename T>
bool SlopeFilter<T>::update(const T & mapIn, T & mapOut)
{
  // Add new layer to the elevation map.
  mapOut = mapIn;
  mapOut.add(type_);

  double slope, slopeMax = 0.0;

  for (grid_map::GridMapIterator iterator(mapOut);
    !iterator.isPastEnd(); ++iterator)
  {
    // Check if there is a surface normal (empty cell).
    if (!mapOut.isValid(*iterator, "surface_normal_z")) {continue;}

    // Compute slope from surface normal z
    slope = acos(mapOut.at("surface_normal_z", *iterator));

    if (slope < criticalValue_) {
      mapOut.at(type_, *iterator) = 1.0 - slope / criticalValue_;
    } else {
      mapOut.at(type_, *iterator) = 0.0;
    }

    if (slope > slopeMax) {slopeMax = slope;}
  }

  RCLCPP_DEBUG(rclcpp::get_logger("traversability_slope_filter"), "slope max = %f", slopeMax);

  return true;
}

}  // namespace filters

PLUGINLIB_EXPORT_CLASS(
  filters::SlopeFilter<grid_map::GridMap>,
  filters::FilterBase<grid_map::GridMap>)
