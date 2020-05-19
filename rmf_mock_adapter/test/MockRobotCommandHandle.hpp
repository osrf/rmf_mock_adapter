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

#ifndef RMF_MOCK_ADAPTER__MOCKROBOTCOMMANDHANDLE_HPP
#define RMF_MOCK_ADAPTER__MOCKROBOTCOMMANDHANDLE_HPP

#include <rmf_mock_adapter/adapter.hpp>
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

#endif //RMF_MOCK_ADAPTER__MOCKROBOTCOMMANDHANDLE_HPP
