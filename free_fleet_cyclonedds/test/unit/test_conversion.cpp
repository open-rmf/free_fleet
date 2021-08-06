/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <chrono>
#include <memory>
#include <iostream>

#include <rmf_utils/catch.hpp>

#include "src/messages/convert.hpp"

SCENARIO("Testing conversion between free fleet messags and DDS messages")
{
  using namespace free_fleet::cyclonedds;
  using namespace free_fleet::messages;

  GIVEN("Time")
  {
    rmf_traffic::Time t = std::chrono::steady_clock::now();
    
    std::unique_ptr<MiddlewareMessages_Time> t_dds_ptr;
    REQUIRE_NOTHROW(
      t_dds_ptr = std::make_unique<MiddlewareMessages_Time>(convert(t)));
    REQUIRE_FALSE(t_dds_ptr == nullptr);
    
    rmf_traffic::Time converted_t;
    REQUIRE_NOTHROW(converted_t = convert(*t_dds_ptr));
    CHECK(t == converted_t);
  }

  GIVEN("Location without yaw")
  {
    Location loc("test_map", {1.2, 3.4});

    std::unique_ptr<MiddlewareMessages_Location> loc_dds_ptr;
    REQUIRE_NOTHROW(
      loc_dds_ptr =
        std::make_unique<MiddlewareMessages_Location>(convert(loc)));
    REQUIRE_FALSE(loc_dds_ptr == nullptr);
    CHECK(std::string(loc_dds_ptr->map_name) == "test_map");
    CHECK(loc_dds_ptr->x == Approx(1.2));
    CHECK(loc_dds_ptr->y == Approx(3.4));
    CHECK_FALSE(loc_dds_ptr->yaw_available);

    std::unique_ptr<Location> converted_loc_ptr;
    REQUIRE_NOTHROW(
      converted_loc_ptr = std::make_unique<Location>(convert(*loc_dds_ptr)));
    REQUIRE_FALSE(converted_loc_ptr == nullptr);
    CHECK(loc == *converted_loc_ptr);
  }

  GIVEN("Location with yaw")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);

    std::unique_ptr<MiddlewareMessages_Location> loc_dds_ptr;
    REQUIRE_NOTHROW(
      loc_dds_ptr =
        std::make_unique<MiddlewareMessages_Location>(convert(loc)));
    REQUIRE_FALSE(loc_dds_ptr == nullptr);
    CHECK(std::string(loc_dds_ptr->map_name) == "test_map");
    CHECK(loc_dds_ptr->x == Approx(1.2));
    CHECK(loc_dds_ptr->y == Approx(3.4));
    REQUIRE(loc_dds_ptr->yaw_available);
    CHECK(loc_dds_ptr->yaw == Approx(5.6));

    std::unique_ptr<Location> converted_loc_ptr;
    REQUIRE_NOTHROW(
      converted_loc_ptr = std::make_unique<Location>(convert(*loc_dds_ptr)));
    REQUIRE_FALSE(converted_loc_ptr == nullptr);
    CHECK(loc == *converted_loc_ptr);
  }

  GIVEN("Waypoint without wait_until")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);
    Waypoint wp(123, loc);

    std::unique_ptr<MiddlewareMessages_Waypoint> wp_dds_ptr;
    REQUIRE_NOTHROW(
      wp_dds_ptr = std::make_unique<MiddlewareMessages_Waypoint>(convert(wp)));
    REQUIRE_FALSE(wp_dds_ptr == nullptr);
    CHECK(wp_dds_ptr->index == 123);
    
    std::unique_ptr<Location> converted_wp_loc_ptr;
    REQUIRE_NOTHROW(
      converted_wp_loc_ptr =
        std::make_unique<Location>(convert(wp_dds_ptr->location)));
    REQUIRE_FALSE(converted_wp_loc_ptr == nullptr);
    CHECK(loc == *converted_wp_loc_ptr);

    CHECK_FALSE(wp_dds_ptr->wait_until_available);

    std::unique_ptr<Waypoint> converted_wp_ptr;
    REQUIRE_NOTHROW(
      converted_wp_ptr = std::make_unique<Waypoint>(convert(*wp_dds_ptr)));
    REQUIRE_FALSE(converted_wp_ptr == nullptr);
    CHECK(wp == *converted_wp_ptr);
  }

  GIVEN("Waypoint with wait_until")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);
    rmf_traffic::Time t = std::chrono::steady_clock::now();
    Waypoint wp(123, loc, t);

    std::unique_ptr<MiddlewareMessages_Waypoint> wp_dds_ptr;
    REQUIRE_NOTHROW(
      wp_dds_ptr = std::make_unique<MiddlewareMessages_Waypoint>(convert(wp)));
    REQUIRE_FALSE(wp_dds_ptr == nullptr);
    CHECK(wp_dds_ptr->index == 123);
    
    std::unique_ptr<Location> converted_wp_loc_ptr;
    REQUIRE_NOTHROW(
      converted_wp_loc_ptr =
        std::make_unique<Location>(convert(wp_dds_ptr->location)));
    REQUIRE_FALSE(converted_wp_loc_ptr == nullptr);
    CHECK(loc == *converted_wp_loc_ptr);

    REQUIRE(wp_dds_ptr->wait_until_available);
    rmf_traffic::Time converted_t = convert(wp_dds_ptr->wait_until);
    CHECK(t == converted_t);

    std::unique_ptr<Waypoint> converted_wp_ptr;
    REQUIRE_NOTHROW(
      converted_wp_ptr = std::make_unique<Waypoint>(convert(*wp_dds_ptr)));
    REQUIRE_FALSE(converted_wp_ptr == nullptr);
    CHECK(wp == *converted_wp_ptr);
  }

  GIVEN("RobotMode without info")
  {
    RobotMode m(RobotMode::Mode::Charging);

    std::unique_ptr<MiddlewareMessages_RobotMode> m_dds_ptr;
    REQUIRE_NOTHROW(
      m_dds_ptr = std::make_unique<MiddlewareMessages_RobotMode>(convert(m)));
    REQUIRE_FALSE(m_dds_ptr == nullptr);
    CHECK(m_dds_ptr->mode == MiddlewareMessages_RobotMode_Constants_Charging);
    CHECK(std::string(m_dds_ptr->info).empty());

    std::unique_ptr<RobotMode> converted_m_ptr;
    REQUIRE_NOTHROW(
      converted_m_ptr = std::make_unique<RobotMode>(convert(*m_dds_ptr)));
    REQUIRE_FALSE(converted_m_ptr == nullptr);
    CHECK(m == *converted_m_ptr);
  }

  GIVEN("RobotMode with info")
  {
    RobotMode m(RobotMode::Mode::Charging, "test_info");

    std::unique_ptr<MiddlewareMessages_RobotMode> m_dds_ptr;
    REQUIRE_NOTHROW(
      m_dds_ptr = std::make_unique<MiddlewareMessages_RobotMode>(convert(m)));
    REQUIRE_FALSE(m_dds_ptr == nullptr);
    CHECK(m_dds_ptr->mode == MiddlewareMessages_RobotMode_Constants_Charging);
    CHECK(std::string(m_dds_ptr->info) == "test_info");

    std::unique_ptr<RobotMode> converted_m_ptr;
    REQUIRE_NOTHROW(
      converted_m_ptr = std::make_unique<RobotMode>(convert(*m_dds_ptr)));
    REQUIRE_FALSE(converted_m_ptr == nullptr);
    CHECK(m == *converted_m_ptr);
  }

  GIVEN("PauseRequest")
  {
    PauseRequest p("test_robot", 123);

    std::unique_ptr<MiddlewareMessages_PauseRequest> p_dds_ptr;
    REQUIRE_NOTHROW(
      p_dds_ptr =
        std::make_unique<MiddlewareMessages_PauseRequest>(convert(p)));
    REQUIRE_FALSE(p_dds_ptr == nullptr);
    CHECK(std::string(p_dds_ptr->robot_name) == "test_robot");
    CHECK(p_dds_ptr->task_id == 123);

    std::unique_ptr<PauseRequest> converted_p_ptr;
    REQUIRE_NOTHROW(
      converted_p_ptr = std::make_unique<PauseRequest>(convert(*p_dds_ptr)));
    REQUIRE_FALSE(converted_p_ptr == nullptr);
  }

  GIVEN("ResumeRequest")
  {
    ResumeRequest r("test_robot", 123);

    std::unique_ptr<MiddlewareMessages_ResumeRequest> r_dds_ptr;
    REQUIRE_NOTHROW(
      r_dds_ptr =
        std::make_unique<MiddlewareMessages_ResumeRequest>(convert(r)));
    REQUIRE_FALSE(r_dds_ptr == nullptr);
    CHECK(std::string(r_dds_ptr->robot_name) == "test_robot");
    CHECK(r_dds_ptr->task_id == 123);

    std::unique_ptr<ResumeRequest> converted_r_ptr;
    REQUIRE_NOTHROW(
      converted_r_ptr = std::make_unique<ResumeRequest>(convert(*r_dds_ptr)));
    REQUIRE_FALSE(converted_r_ptr == nullptr);
  }

  GIVEN("DockRequest")
  {
    DockRequest d("test_robot", 123, "test_dock");

    std::unique_ptr<MiddlewareMessages_DockRequest> d_dds_ptr;
    REQUIRE_NOTHROW(
      d_dds_ptr =
        std::make_unique<MiddlewareMessages_DockRequest>(convert(d)));
    REQUIRE_FALSE(d_dds_ptr == nullptr);
    CHECK(std::string(d_dds_ptr->robot_name) == "test_robot");
    CHECK(d_dds_ptr->task_id == 123);
    CHECK(std::string(d_dds_ptr->dock_name) == "test_dock");

    std::unique_ptr<DockRequest> converted_d_ptr;
    REQUIRE_NOTHROW(
      converted_d_ptr = std::make_unique<DockRequest>(convert(*d_dds_ptr)));
    REQUIRE_FALSE(converted_d_ptr == nullptr);
  }

  GIVEN("NavigationRequest")
  {
    Location loc1("test_map", {1.2, 3.4}, 5.6);
    rmf_traffic::Time t1 = std::chrono::steady_clock::now();
    Waypoint wp1(123, loc1, t1);

    Location loc2("test_map", {3.4, 5.6}, 7.8);
    rmf_traffic::Time t2 = std::chrono::steady_clock::now();
    Waypoint wp2(124, loc2, t2);
    
    NavigationRequest n("test_robot", 123, {wp1, wp2});

    std::unique_ptr<MiddlewareMessages_NavigationRequest> n_dds_ptr;
    REQUIRE_NOTHROW(
      n_dds_ptr =
        std::make_unique<MiddlewareMessages_NavigationRequest>(convert(n)));
    REQUIRE_FALSE(n_dds_ptr == nullptr);
    CHECK(std::string(n_dds_ptr->robot_name) == "test_robot");
    CHECK(n_dds_ptr->task_id == 123);
    REQUIRE(n_dds_ptr->path._length == 2);
    
    std::unique_ptr<Waypoint> converted_wp1_ptr;
    REQUIRE_NOTHROW(
      converted_wp1_ptr =
        std::make_unique<Waypoint>(convert(n_dds_ptr->path._buffer[0])));
    REQUIRE_FALSE(converted_wp1_ptr == nullptr);
    CHECK(wp1 == *converted_wp1_ptr);

    std::unique_ptr<Waypoint> converted_wp2_ptr;
    REQUIRE_NOTHROW(
      converted_wp2_ptr =
        std::make_unique<Waypoint>(convert(n_dds_ptr->path._buffer[1])));
    REQUIRE_FALSE(converted_wp2_ptr == nullptr);
    CHECK(wp2 == *converted_wp2_ptr);

    std::unique_ptr<NavigationRequest> converted_n_ptr;
    REQUIRE_NOTHROW(
      converted_n_ptr =
        std::make_unique<NavigationRequest>(convert(*n_dds_ptr)));
    REQUIRE_FALSE(converted_n_ptr == nullptr);
    CHECK(n == *converted_n_ptr);
  }

  GIVEN("RelocalizationRequest")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);
    RelocalizationRequest reloc("test_robot", 123, loc, 321);

    std::unique_ptr<MiddlewareMessages_RelocalizationRequest> reloc_dds_ptr;
    REQUIRE_NOTHROW(
      reloc_dds_ptr =
        std::make_unique<MiddlewareMessages_RelocalizationRequest>(
          convert(reloc)));
    REQUIRE_FALSE(reloc_dds_ptr == nullptr);
    CHECK(std::string(reloc_dds_ptr->robot_name) == "test_robot");
    CHECK(reloc_dds_ptr->task_id == 123);

    std::unique_ptr<Location> converted_loc_ptr;
    REQUIRE_NOTHROW(
      converted_loc_ptr =
        std::make_unique<Location>(convert(reloc_dds_ptr->location)));
    REQUIRE_FALSE(converted_loc_ptr == nullptr);
    CHECK(loc == *converted_loc_ptr);

    CHECK(reloc_dds_ptr->last_visited_waypoint_index == 321);

    std::unique_ptr<RelocalizationRequest> converted_reloc_ptr;
    REQUIRE_NOTHROW(
      converted_reloc_ptr =
        std::make_unique<RelocalizationRequest>(convert(*reloc_dds_ptr)));
    REQUIRE_FALSE(converted_reloc_ptr == nullptr);
    CHECK(reloc == *converted_reloc_ptr);
  }

  GIVEN("RobotState without task_id")
  {
    rmf_traffic::Time t = std::chrono::steady_clock::now();
    RobotMode m(RobotMode::Mode::Charging, "test_info");
    Location loc("test_map", {1.2, 3.4}, 5.6);
    
    RobotState s(t, "test_robot", "test_model", std::nullopt, m, 0.9, loc, 321);

    std::unique_ptr<MiddlewareMessages_RobotState> s_dds_ptr;
    REQUIRE_NOTHROW(
      s_dds_ptr = std::make_unique<MiddlewareMessages_RobotState>(convert(s)));
    REQUIRE_FALSE(s_dds_ptr == nullptr);
    
    rmf_traffic::Time converted_t;
    REQUIRE_NOTHROW(converted_t = convert(s_dds_ptr->time));
    CHECK(t == converted_t);
    
    CHECK(std::string(s_dds_ptr->name) == "test_robot");
    CHECK(std::string(s_dds_ptr->model) == "test_model");
    CHECK_FALSE(s_dds_ptr->task_id_available);

    std::unique_ptr<RobotMode> converted_m_ptr;
    REQUIRE_NOTHROW(
      converted_m_ptr = std::make_unique<RobotMode>(convert(s_dds_ptr->mode)));
    REQUIRE_FALSE(converted_m_ptr == nullptr);
    CHECK(m == *converted_m_ptr);
    
    CHECK(s_dds_ptr->battery_percent == Approx(0.9));

    std::unique_ptr<Location> converted_loc_ptr;
    REQUIRE_NOTHROW(
      converted_loc_ptr =
        std::make_unique<Location>(convert(s_dds_ptr->location)));
    REQUIRE_FALSE(converted_loc_ptr == nullptr);
    CHECK(loc == *converted_loc_ptr);
    
    CHECK(s_dds_ptr->target_path_index == 321);

    std::unique_ptr<RobotState> converted_s_ptr;
    REQUIRE_NOTHROW(
      converted_s_ptr = std::make_unique<RobotState>(convert(*s_dds_ptr)));
    REQUIRE_FALSE(converted_s_ptr == nullptr);
    CHECK(s == *converted_s_ptr);
  }

  GIVEN("RobotState with task_id")
  {
    rmf_traffic::Time t = std::chrono::steady_clock::now();
    RobotMode m(RobotMode::Mode::Charging, "test_info");
    Location loc("test_map", {1.2, 3.4}, 5.6);
    
    RobotState s(t, "test_robot", "test_model", 123, m, 0.9, loc, 321);

    std::unique_ptr<MiddlewareMessages_RobotState> s_dds_ptr;
    REQUIRE_NOTHROW(
      s_dds_ptr = std::make_unique<MiddlewareMessages_RobotState>(convert(s)));
    REQUIRE_FALSE(s_dds_ptr == nullptr);
    
    rmf_traffic::Time converted_t;
    REQUIRE_NOTHROW(converted_t = convert(s_dds_ptr->time));
    CHECK(t == converted_t);
    
    CHECK(std::string(s_dds_ptr->name) == "test_robot");
    CHECK(std::string(s_dds_ptr->model) == "test_model");
    REQUIRE(s_dds_ptr->task_id_available);
    CHECK(s_dds_ptr->task_id == 123);

    std::unique_ptr<RobotMode> converted_m_ptr;
    REQUIRE_NOTHROW(
      converted_m_ptr = std::make_unique<RobotMode>(convert(s_dds_ptr->mode)));
    REQUIRE_FALSE(converted_m_ptr == nullptr);
    CHECK(m == *converted_m_ptr);
    
    CHECK(s_dds_ptr->battery_percent == Approx(0.9));

    std::unique_ptr<Location> converted_loc_ptr;
    REQUIRE_NOTHROW(
      converted_loc_ptr =
        std::make_unique<Location>(convert(s_dds_ptr->location)));
    REQUIRE_FALSE(converted_loc_ptr == nullptr);
    CHECK(loc == *converted_loc_ptr);
    
    CHECK(s_dds_ptr->target_path_index == 321);

    std::unique_ptr<RobotState> converted_s_ptr;
    REQUIRE_NOTHROW(
      converted_s_ptr = std::make_unique<RobotState>(convert(*s_dds_ptr)));
    REQUIRE_FALSE(converted_s_ptr == nullptr);
    CHECK(s == *converted_s_ptr);
  }
}
