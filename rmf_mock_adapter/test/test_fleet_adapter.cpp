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

#include <rmf_mock_adapter/adapter.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <random>

class MockRobotCommandHandle : public rmf_mock_adapter::RobotCommandHandle
{
public:

  void follow_new_path(
      const std::vector<rmf_traffic::agv::Plan::Waypoint>& waypoints,
      std::function<void()> path_finished_callback) final
  {
    _current_path_index = 0;
    _waypoints = waypoints;
    _path_finished_callback = path_finished_callback;
  }

  void dock(const std::string&, std::function<void()>)
  {
    // Not supported
  }

  void step()
  {
    if (_waypoints.empty())
      return;

    if (_current_path_index < _waypoints.size())
      ++_current_path_index;

    if (_current_path_index < _waypoints.size())
    {
      const auto& wp = _waypoints[_current_path_index];
      if (wp.graph_index())
      {
        updater->update_position(*wp.graph_index(), 0.0);
      }
      else
      {
        updater->update_position(wp.position(), {});
      }

      if (interruped())
        updater->interrupted();
    }

    if (_current_path_index == _waypoints.size() && _path_finished_callback)
    {
      _path_finished_callback();
      _path_finished_callback = nullptr;
    }
  }

  bool interruped()
  {
    // This gives a 40% probability of an interruption occurring at each step.
    return std::uniform_real_distribution<double>(0.0, 1.0)(rng) < 0.4;
  }

  std::shared_ptr<rmf_mock_adapter::RobotUpdateHandle> updater;

  std::size_t _current_path_index;
  std::vector<rmf_traffic::agv::Plan::Waypoint> _waypoints;
  std::function<void()> _path_finished_callback;

  MockRobotCommandHandle()
    : rng(std::random_device()())
  {
    // Do nothing
  }

  std::mt19937 rng;

};

int main()
{
  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph;
  graph.add_waypoint(test_map_name, {0.0, -10.0}); // 0
  graph.add_waypoint(test_map_name, {0.0, -5.0});  // 1
  graph.add_waypoint(test_map_name, {5.0, -5.0});  // 2
  graph.add_waypoint(test_map_name, {-10.0, 0.0}); // 3
  graph.add_waypoint(test_map_name, {-5.0, 0.0}); // 4
  graph.add_waypoint(test_map_name, {0.0, 0.0}); // 5
  graph.add_waypoint(test_map_name, {5.0, 0.0}); // 6
  graph.add_waypoint(test_map_name, {10.0, 0.0}); // 7
  graph.add_waypoint(test_map_name, {0.0, 5.0}); // 8
  graph.add_waypoint(test_map_name, {5.0, 5.0}); // 9
  graph.add_waypoint(test_map_name, {0.0, 10.0}); // 10

  /*
   *                  10
   *                   |
   *                   |
   *                   8------9
   *                   |      |
   *                   |      |
   *     3------4------5------6------7
   *                   |      |
   *                   |      |
   *                   1------2
   *                   |
   *                   |
   *                   0
   **/

  auto add_bidir_lane = [&](const std::size_t w0, const std::size_t w1)
    {
      graph.add_lane(w0, w1);
      graph.add_lane(w1, w0);
    };

  add_bidir_lane(0, 1);
  add_bidir_lane(1, 2);
  add_bidir_lane(1, 5);
  add_bidir_lane(2, 6);
  add_bidir_lane(3, 4);
  add_bidir_lane(4, 5);
  add_bidir_lane(5, 6);
  add_bidir_lane(6, 7);
  add_bidir_lane(5, 8);
  add_bidir_lane(6, 9);
  add_bidir_lane(8, 9);
  add_bidir_lane(8, 10);

  rmf_traffic::Profile profile{
    rmf_traffic::geometry::make_final_convex<
      rmf_traffic::geometry::Circle>(1.0)
  };

  const rmf_traffic::agv::VehicleTraits traits{
    {0.7, 0.3},
    {1.0, 0.45},
    profile
  };

  rmf_mock_adapter::TestScenario scenario;
  auto fleet_update_handle = scenario.add_fleet(
        "mock_fleet", graph, traits);

  std::vector<std::shared_ptr<MockRobotCommandHandle>> robot_handles;
  for (const auto wp : {3, 7})
  {
    robot_handles.push_back(std::make_shared<MockRobotCommandHandle>());
    robot_handles.back()->updater = fleet_update_handle.add_robot(
          robot_handles.back(),
          "robot_" + std::to_string(wp), profile, wp, 0.0);
  }

  scenario.test(
  {
      {robot_handles.front()->updater, 10},
      {robot_handles.back()->updater, 0}
  });

  while (!scenario.finished())
  {
    for (const auto& handle : robot_handles)
      handle->step();
  }
}
