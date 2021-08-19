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
   
    auto t_dds = convert(t);
    REQUIRE(t_dds.has_value());
    
    auto converted_t = convert(t_dds.value());
    REQUIRE(converted_t.has_value());
    CHECK(converted_t.value() == t);
  }

  GIVEN("Location without yaw")
  {
    Location loc("test_map", {1.2, 3.4});

    auto loc_dds = convert(loc);
    REQUIRE(loc_dds.has_value());
    CHECK(std::string(loc_dds->map_name) == "test_map");
    CHECK(loc_dds->x == Approx(1.2));
    CHECK(loc_dds->y == Approx(3.4));
    CHECK_FALSE(loc_dds->yaw_available);

    auto converted_loc = convert(loc_dds.value());
    REQUIRE(converted_loc.has_value());
    CHECK(converted_loc.value() == loc);
  }

  GIVEN("Location with yaw")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);

    auto loc_dds = convert(loc);
    REQUIRE(loc_dds.has_value());
    CHECK(std::string(loc_dds->map_name) == "test_map");
    CHECK(loc_dds->x == Approx(1.2));
    CHECK(loc_dds->y == Approx(3.4));
    REQUIRE(loc_dds->yaw_available);
    CHECK(loc_dds->yaw == Approx(5.6));

    auto converted_loc = convert(loc_dds.value());
    REQUIRE(converted_loc.has_value());
    CHECK(converted_loc.value() == loc);
  }

  GIVEN("Waypoint without wait_until")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);
    Waypoint wp(123, loc);

    auto wp_dds = convert(wp);
    REQUIRE(wp_dds.has_value());
    CHECK(wp_dds->index == 123);
    
    auto converted_wp_loc = convert(wp_dds->location);
    REQUIRE(converted_wp_loc.has_value());
    CHECK(converted_wp_loc.value() == loc);

    CHECK_FALSE(wp_dds->wait_until_available);

    auto converted_wp = convert(wp_dds.value());
    REQUIRE(converted_wp.has_value());
    CHECK(converted_wp.value() == wp);
  }

  GIVEN("Waypoint with wait_until")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);
    rmf_traffic::Time t = std::chrono::steady_clock::now();
    Waypoint wp(123, loc, t);

    auto wp_dds = convert(wp);
    REQUIRE(wp_dds.has_value());
    CHECK(wp_dds->index == 123);
    
    auto converted_wp_loc = convert(wp_dds->location);
    REQUIRE(converted_wp_loc.has_value());
    CHECK(converted_wp_loc.value() == loc);

    CHECK(wp_dds->wait_until_available);
    auto converted_t = convert(wp_dds->wait_until);
    REQUIRE(converted_t.has_value());
    CHECK(converted_t.value() == t);

    auto converted_wp = convert(wp_dds.value());
    REQUIRE(converted_wp.has_value());
    CHECK(converted_wp.value() == wp);
  }

  GIVEN("RobotMode without info")
  {
    RobotMode m(RobotMode::Mode::Charging);

    auto m_dds = convert(m);
    REQUIRE(m_dds.has_value());
    CHECK(m_dds->mode == MiddlewareMessages_RobotMode_Constants_Charging);
    REQUIRE_FALSE(m_dds->info == nullptr);
    CHECK(std::string(m_dds->info).empty());

    auto converted_m = convert(m_dds.value());
    REQUIRE(converted_m.has_value());
    CHECK(converted_m.value() == m);
  }

  GIVEN("RobotMode with info")
  {
    RobotMode m(RobotMode::Mode::Charging, "test_info");

    auto m_dds = convert(m);
    REQUIRE(m_dds.has_value());
    CHECK(m_dds->mode == MiddlewareMessages_RobotMode_Constants_Charging);
    REQUIRE_FALSE(m_dds->info == nullptr);
    CHECK(std::string(m_dds->info) == "test_info");

    auto converted_m = convert(m_dds.value());
    REQUIRE(converted_m.has_value());
    CHECK(converted_m.value() == m);
  }

  GIVEN("PauseRequest")
  {
    PauseRequest p("test_robot", 123);

    auto p_dds = convert(p);
    REQUIRE(p_dds.has_value());
    CHECK(std::string(p_dds->robot_name) == "test_robot");
    CHECK(p_dds->task_id == 123);

    auto converted_p = convert(p_dds.value());
    REQUIRE(converted_p.has_value());
    CHECK(converted_p.value() == p);
  }

  GIVEN("ResumeRequest")
  {
    ResumeRequest r("test_robot", 123);

    auto r_dds = convert(r);
    REQUIRE(r_dds.has_value());
    CHECK(std::string(r_dds->robot_name) == "test_robot");
    CHECK(r_dds->task_id == 123);

    auto converted_r = convert(r_dds.value());
    REQUIRE(converted_r.has_value());
    CHECK(converted_r.value() == r);
  }

  GIVEN("DockRequest")
  {
    DockRequest d("test_robot", 123, "test_dock");

    auto d_dds = convert(d);
    REQUIRE(d_dds.has_value());
    CHECK(std::string(d_dds->robot_name) == "test_robot");
    CHECK(d_dds->task_id == 123);
    CHECK(std::string(d_dds->dock_name) == "test_dock");

    auto converted_d = convert(d_dds.value());
    REQUIRE(converted_d.has_value());
    REQUIRE(converted_d.value() == d);
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

    auto n_dds = convert(n);
    REQUIRE(n_dds.has_value());
    CHECK(std::string(n_dds->robot_name) == "test_robot");
    CHECK(n_dds->task_id == 123);
    REQUIRE(n_dds->path._length == 2);

    auto converted_wp1 = convert(n_dds->path._buffer[0]);
    REQUIRE(converted_wp1.has_value());
    CHECK(converted_wp1.value() == wp1);

    auto converted_wp2 = convert(n_dds->path._buffer[1]);
    REQUIRE(converted_wp2.has_value());
    CHECK(converted_wp2.value() == wp2);

    auto converted_n = convert(n_dds.value());
    REQUIRE(converted_n.has_value());
    CHECK(converted_n.value() == n);
  }

  GIVEN("RelocalizationRequest")
  {
    Location loc("test_map", {1.2, 3.4}, 5.6);
    RelocalizationRequest reloc("test_robot", 123, loc, 321);

    auto reloc_dds = convert(reloc);
    REQUIRE(reloc_dds.has_value());
    CHECK(std::string(reloc_dds->robot_name) == "test_robot");
    CHECK(reloc_dds->task_id == 123);
    CHECK(reloc_dds->last_visited_waypoint_index == 321);

    auto converted_loc = convert(reloc_dds->location);
    REQUIRE(converted_loc.has_value());
    CHECK(converted_loc.value() == loc);

    auto converted_reloc = convert(reloc_dds.value());
    REQUIRE(converted_reloc.has_value());
    CHECK(converted_reloc.value() == reloc);
  }

  GIVEN("RobotState without task_id")
  {
    rmf_traffic::Time t = std::chrono::steady_clock::now();
    RobotMode m(RobotMode::Mode::Charging, "test_info");
    Location loc("test_map", {1.2, 3.4}, 5.6);

    RobotState s(t, "test_robot", "test_model", std::nullopt, m, 0.9, loc, 321);

    auto s_dds = convert(s);
    REQUIRE(s_dds.has_value());

    CHECK(std::string(s_dds->name) == "test_robot");
    CHECK(std::string(s_dds->model) == "test_model");
    CHECK_FALSE(s_dds->task_id_available);
    CHECK(s_dds->battery_percent == Approx(0.9));
    CHECK(s_dds->target_path_index == 321);

    auto converted_t = convert(s_dds->time);
    REQUIRE(converted_t.has_value());
    CHECK(converted_t.value() == t);

    auto converted_m = convert(s_dds->mode);
    REQUIRE(converted_m.has_value());
    CHECK(converted_m.value() == m);

    auto converted_loc = convert(s_dds->location);
    REQUIRE(converted_loc.has_value());
    CHECK(converted_loc.value() == loc);

    auto converted_s = convert(s_dds.value());
    REQUIRE(converted_s.has_value());
    CHECK(converted_s.value() == s); 
  }

  GIVEN("RobotState with task_id")
  {
    rmf_traffic::Time t = std::chrono::steady_clock::now();
    RobotMode m(RobotMode::Mode::Charging, "test_info");
    Location loc("test_map", {1.2, 3.4}, 5.6);

    RobotState s(t, "test_robot", "test_model", 123, m, 0.9, loc, 321);

    auto s_dds = convert(s);
    REQUIRE(s_dds.has_value());

    CHECK(std::string(s_dds->name) == "test_robot");
    CHECK(std::string(s_dds->model) == "test_model");
    REQUIRE(s_dds->task_id_available);
    CHECK(s_dds->task_id == 123);
    CHECK(s_dds->battery_percent == Approx(0.9));
    CHECK(s_dds->target_path_index == 321);

    auto converted_t = convert(s_dds->time);
    REQUIRE(converted_t.has_value());
    CHECK(converted_t.value() == t);

    auto converted_m = convert(s_dds->mode);
    REQUIRE(converted_m.has_value());
    CHECK(converted_m.value() == m);

    auto converted_loc = convert(s_dds->location);
    REQUIRE(converted_loc.has_value());
    CHECK(converted_loc.value() == loc);

    auto converted_s = convert(s_dds.value());
    REQUIRE(converted_s.has_value());
    CHECK(converted_s.value() == s); 
  }
}
