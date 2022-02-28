/*
 * traversability_estimation_node.cpp
 *
 *  Created on: Oct 22, 2014
 *      Author: Ralf Kaestner, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "traversability_estimation/TraversabilityEstimation.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto traversabilityEstimation = std::make_shared<traversability_estimation::TraversabilityEstimation>(options);

  executor.add_node(traversabilityEstimation);

  executor.spin();

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
