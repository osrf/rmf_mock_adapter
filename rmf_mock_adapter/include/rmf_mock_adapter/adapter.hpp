/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef RMF_MOCK_ADAPTER__ADAPTER_HPP
#define RMF_MOCK_ADAPTER__ADAPTER_HPP

#include <rmf_traffic/agv/Planner.hpp>

namespace rmf_mock_adapter {

//==============================================================================
/// Implement this class to receive robot commands from RMF
class RobotCommandHandle
{
public:

  /// Have the robot follow a new path. If it was already following a path, then
  /// it should immediately switch over to this one.
  ///
  /// \param[in] waypoints
  ///   The sequence of waypoints to follow. When the robot arrives at a
  ///   waypoint in this sequence, it should wait at that waypoint until the
  ///   clock reaches the time() field of the waypoint. This is important
  ///   because the waypoint timing is used to avoid traffic conflicts with
  ///   other vehicles.
  ///
  /// \param[in] path_finished_callback
  ///   Trigger this callback when the robot is done following the new path.
  virtual void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      std::function<void()> path_finished_callback) = 0;

  /// Have the robot begin a pre-defined docking procedure. Implement this
  /// function as a no-op if your robots do not perform docking procedures.
  ///
  /// \param[in] dock_name
  ///   The predefined name of the docking procedure to use.
  ///
  /// \param[in] docking_finished_callback
  ///   Trigger this callback when the docking is finished.
  virtual void dock(
      const std::string& dock_name,
      std::function<void()> docking_finished_callback) = 0;

};

//==============================================================================
/// You will be given an instance of this class every time you add a new robot
/// to your fleet. Use that instance to send updates to RoMi-H about your
/// robot's state.
///
/// To remove the available of this robot, simply allow the instance to expire.
class RobotUpdateHandle
{
public:

  /// Tell the RMF schedule that the robot has been delayed.
  void add_delay(rmf_traffic::Duration duration);

  /// Tell the RMF schedule that the robot was interrupted and needs a new plan.
  void interrupted();

  /// Update the current position of the robot by specifying the waypoint that
  /// the robot is on and its orientation.
  void update_position(
      std::size_t waypoint,
      double orientation);

  /// Update the current position of the robot by specifying the x, y, yaw
  /// position of the robot and one or more lanes that the robot is occupying.
  ///
  /// \warning At least one lane must be specified. If no lane information is
  /// available, then use the update_position(std::string, Eigen::Vector3d)
  /// signature of this function.
  void update_position(
      const Eigen::Vector3d& position,
      const std::vector<std::size_t>& lanes);

  /// Update the current position of the robot by specifying the x, y, yaw
  /// position of the robot and what map the robot is on.
  ///
  /// \warning This function should only be used if the robot has diverged from
  /// the navigation graph somehow.
  void update_position(
      const std::string& map_name,
      const Eigen::Vector3d& position);

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// Each fleet will be given an instance of this class to update RoMi-H about
/// its available robots.
class FleetUpdateHandle
{
public:

  /// Add a new robot, specifying what waypoint it is starting on.
  std::shared_ptr<RobotUpdateHandle> add_robot(
      std::shared_ptr<RobotCommandHandle> handle,
      const std::string& name,
      const rmf_traffic::Profile& profile,
      std::size_t initial_waypoint,
      double orientation);

  /// Add a new robot, specifying what its current position is. This should only
  /// be used if it is not starting on any waypoint. Use the other signature of
  /// this function if the robot starts on a waypoint.
  std::shared_ptr<RobotUpdateHandle> add_robot(
      std::shared_ptr<RobotCommandHandle> handle,
      const std::string& name,
      const rmf_traffic::Profile& profile,
      const Eigen::Vector3d& initial_position,
      const std::vector<std::size_t>& initial_lanes);

  class Implementation;
private:
  FleetUpdateHandle();
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

//==============================================================================
/// This class mocks up a scenario for testing purposes.
class TestScenario
{
public:

  /// Add a fleet to the test scenario
  ///
  /// \param[in] fleet_name
  ///   The name of the fleet that is being adapted
  ///
  /// \param[in] nav_graph
  ///   The AGV navigation graph information for this fleet
  FleetUpdateHandle add_fleet(
      std::string fleet_name,
      rmf_traffic::agv::Graph nav_graph,
      rmf_traffic::agv::VehicleTraits traits);

  /// Specify a condition for the test
  struct Condition
  {
    std::shared_ptr<RobotUpdateHandle> robot;
    rmf_traffic::agv::Plan::Goal goal;
  };

  /// Initiate a test with the given conditions.
  void test(std::vector<Condition> conditions);

  bool finished() const;

  /// Constructor
  TestScenario();

  class Implementation;
private:
  rmf_utils::unique_impl_ptr<Implementation> _pimpl;
};

} // namespace rmf_mock_adapter

#endif // RMF_MOCK_ADAPTER__ADAPTER_HPP
