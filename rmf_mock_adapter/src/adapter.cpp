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

#include <rmf_traffic/schedule/Database.hpp>

#include <iostream>

namespace rmf_mock_adapter {

//==============================================================================
class RobotUpdateHandle::Implementation
{
public:

  rmf_traffic::schedule::Participant participant;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;
  rmf_traffic::agv::Planner::Options options;
  std::weak_ptr<RobotCommandHandle> weak_command;
  std::vector<rmf_traffic::agv::Plan::Start> position;

  rmf_utils::optional<rmf_traffic::agv::Plan::Goal> goal = rmf_utils::nullopt;
  bool finished = false;

  void send_path_command()
  {
    if (!goal)
    {
      std::cerr << "ERROR: Trying to issue a path command to ["
                << participant.description().name() << "] before a goal is set!"
                << std::endl;
      return;
    }

    const auto command = weak_command.lock();
    if (!command)
    {
      std::cerr << "ERROR: Trying to issue a path command to ["
                << participant.description().name() << "] after its command "
                << "handle has expired" << std::endl;
      return;
    }

    auto result = planner->plan(position, *goal);
    if (!result)
    {
      std::cerr << "Unable to find a feasible plan.\nStart conditions:";
      for (const auto& s : position)
      {
        std::cout << " -- initial waypoint: " << s.waypoint();
        if (s.location())
        {
          std::cout << " | initial location: (" << s.location()->transpose()
                    << ")";
        }

        if (s.lane())
        {
          std::cout << " | initial lane: " << *s.lane() << std::endl;
        }
      }
      return;
    }

    finished = false;
    participant.set(result->get_itinerary());
    std::cout << "Issuing new path command to ["
              << participant.description().name() << "]" << std::endl;
    command->follow_new_path(
          result->get_waypoints(),
          [&]()
    {
      std::cout << "[" << participant.description().name() << "] has reported "
                << "finishing!" << std::endl;
      finished = true;
    });
  }

  static std::shared_ptr<RobotUpdateHandle> make_shared(
      const rmf_traffic::schedule::Viewer& viewer,
      rmf_traffic::schedule::Participant participant,
      std::shared_ptr<rmf_traffic::agv::Planner> planner,
      std::shared_ptr<RobotCommandHandle> command,
      std::vector<rmf_traffic::agv::Plan::Start> position)
  {
    auto handle = std::shared_ptr<RobotUpdateHandle>(new RobotUpdateHandle);
    auto options = rmf_traffic::agv::Planner::Options(
          rmf_utils::make_clone<rmf_traffic::agv::ScheduleRouteValidator>(
            viewer, participant.id(), participant.description().profile()));

    handle->_pimpl = rmf_utils::make_unique_impl<Implementation>(
          Implementation{
            std::move(participant),
            std::move(planner),
            std::move(options),
            std::move(command),
            std::move(position)
          });

    return handle;
  }

  static void set_goal(
      RobotUpdateHandle& handle,
      rmf_traffic::agv::Plan::Goal goal)
  {
    handle._pimpl->goal = std::move(goal);
    handle._pimpl->send_path_command();
  }

  static bool is_finished(const RobotUpdateHandle& handle)
  {
    return handle._pimpl->finished;
  }
};

//==============================================================================
std::vector<rmf_traffic::agv::Plan::Start> make_starts(
    std::size_t initial_waypoint,
    double orientation)
{
  const auto now = std::chrono::steady_clock::now();
  return {rmf_traffic::agv::Plan::Start(now, initial_waypoint, orientation)};
}

//==============================================================================
std::vector<rmf_traffic::agv::Plan::Start> make_starts(
    const rmf_traffic::agv::Graph& nav_graph,
    const Eigen::Vector3d& initial_position,
    const std::vector<std::size_t>& initial_lanes)
{
  assert(!initial_lanes.empty());
  const auto now = std::chrono::steady_clock::now();
  std::vector<rmf_traffic::agv::Plan::Start> starts;
  for (const std::size_t l : initial_lanes)
  {
    const auto wp = nav_graph.get_lane(l).exit().waypoint_index();

    starts.emplace_back(
        now, wp, initial_position[2],
        Eigen::Vector2d(initial_position.block<2,1>(0,0)), l);
  }

  return starts;
}

//==============================================================================
void RobotUpdateHandle::add_delay(rmf_traffic::Duration duration)
{
  _pimpl->participant.delay(std::chrono::steady_clock::now(), duration);
}

//==============================================================================
void RobotUpdateHandle::interrupted()
{
  std::cout << "[" << _pimpl->participant.description().name()
            << "] was interrupted! We will send a new plan." << std::endl;

  _pimpl->send_path_command();
}

//==============================================================================
void RobotUpdateHandle::update_position(
    std::size_t waypoint,
    double orientation)
{
  _pimpl->position = make_starts(waypoint, orientation);
}

//==============================================================================
void RobotUpdateHandle::update_position(
    const Eigen::Vector3d& position,
    const std::vector<std::size_t>& lanes)
{
  _pimpl->position = make_starts(
        _pimpl->planner->get_configuration().graph(), position, lanes);
}

//==============================================================================
void RobotUpdateHandle::update_position(
    const std::string& map_name,
    const Eigen::Vector3d& position)
{
  const auto now = std::chrono::steady_clock::now();
  _pimpl->position = rmf_traffic::agv::compute_plan_starts(
        _pimpl->planner->get_configuration().graph(),
        map_name, position, now);
}

//==============================================================================
class FleetUpdateHandle::Implementation
{
public:

  std::shared_ptr<rmf_traffic::schedule::Database> database;
  std::string fleet_name;
  std::shared_ptr<rmf_traffic::agv::Planner> planner;

  static FleetUpdateHandle make(
      std::shared_ptr<rmf_traffic::schedule::Database> database,
      std::string fleet_name,
      rmf_traffic::agv::Graph nav_graph,
      rmf_traffic::agv::VehicleTraits traits)
  {
    auto planner = std::make_shared<rmf_traffic::agv::Planner>(
          rmf_traffic::agv::Planner::Configuration(
            nav_graph, traits),
          rmf_traffic::agv::Planner::Options(nullptr));

    FleetUpdateHandle handle;
    handle._pimpl = rmf_utils::make_unique_impl<Implementation>(
          Implementation{
            std::move(database),
            std::move(fleet_name),
            std::move(planner)
          });

    return handle;
  }

  std::shared_ptr<RobotUpdateHandle> add_robot(
      std::shared_ptr<RobotCommandHandle> command,
      const std::string& name,
      const rmf_traffic::Profile& profile,
      std::vector<rmf_traffic::agv::Plan::Start> position)
  {
    auto description = rmf_traffic::schedule::ParticipantDescription(
          name, fleet_name,
          rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          profile);

    auto participant = rmf_traffic::schedule::make_participant(
          std::move(description), database);

    return RobotUpdateHandle::Implementation::make_shared(
          *database,
          std::move(participant),
          planner,
          std::move(command),
          std::move(position));
  }
};

//==============================================================================
std::shared_ptr<RobotUpdateHandle> FleetUpdateHandle::add_robot(
    std::shared_ptr<RobotCommandHandle> handle,
    const std::string& name,
    const rmf_traffic::Profile& profile,
    std::size_t initial_waypoint,
    double orientation)
{
  return _pimpl->add_robot(
        handle, name, profile,
        make_starts(initial_waypoint, orientation));
}

//==============================================================================
std::shared_ptr<RobotUpdateHandle> FleetUpdateHandle::add_robot(
    std::shared_ptr<RobotCommandHandle> handle,
    const std::string& name,
    const rmf_traffic::Profile& profile,
    const Eigen::Vector3d& initial_position,
    const std::vector<std::size_t>& initial_lanes)
{
  return _pimpl->add_robot(
        handle, name, profile,
        make_starts(
          _pimpl->planner->get_configuration().graph(),
          initial_position, initial_lanes));
}

//==============================================================================
FleetUpdateHandle::FleetUpdateHandle()
{
  // Do nothing
}

//==============================================================================
class TestScenario::Implementation
{
public:

  std::shared_ptr<rmf_traffic::schedule::Database> database;
  std::vector<std::shared_ptr<RobotUpdateHandle>> handles;

  Implementation()
    : database(std::make_shared<rmf_traffic::schedule::Database>())
  {
    // Do nothing
  }

};

//==============================================================================
FleetUpdateHandle TestScenario::add_fleet(
    std::string fleet_name,
    rmf_traffic::agv::Graph nav_graph,
    rmf_traffic::agv::VehicleTraits traits)
{
  return FleetUpdateHandle::Implementation::make(
        _pimpl->database,
        std::move(fleet_name),
        std::move(nav_graph),
        std::move(traits));
}

//==============================================================================
void TestScenario::test(std::vector<Condition> conditions)
{
  for (const auto& condition : conditions)
  {
    RobotUpdateHandle::Implementation::set_goal(
          *condition.robot, condition.goal);

    _pimpl->handles.push_back(condition.robot);
  }
}

//==============================================================================
bool TestScenario::finished() const
{
  for (const auto& handle : _pimpl->handles)
    if (!RobotUpdateHandle::Implementation::is_finished(*handle))
      return false;

  return true;
}

//==============================================================================
TestScenario::TestScenario()
  : _pimpl(rmf_utils::make_unique_impl<Implementation>())
{
  // Do nothing
}

} // namespace rmf_mock_adapter
