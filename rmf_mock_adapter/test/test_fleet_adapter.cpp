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

#include "MockRobotCommandHandle.hpp"
#include "TestMap.hpp"

#include <rmf_traffic/geometry/Circle.hpp>

int main()
{
  const std::string test_map_name = "test_map";
  rmf_traffic::agv::Graph graph = make_graph();

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
