/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "ros/ros.h"
#include "dds/dds.h"

// include the message definitions
#include "FreeFleet.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "free_fleet_client");
  ros::NodeHandle ros_node;
  ROS_INFO("greetings from free_fleet_client");

  // set up a DDS writer
  dds_return_t rc;

  // todo: pull dds domain from a config file
  dds_entity_t participant = dds_create_participant(0, NULL, NULL);
  if (participant < 0)
  {
    ROS_FATAL("couldn't create DDS participant: %s",
        dds_strretcode(-participant));
    return 1;
  }

  dds_entity_t topic = dds_create_topic(
      participant,
      &FreeFleetData_RobotState_desc,
      "rt/free_fleet_robot_states",
      NULL,
      NULL);

  if (topic < 0)
  {
    ROS_FATAL("couldn't create DDS topic: %s", dds_strretcode(-topic));
    return 1;
  }

  dds_entity_t publisher = dds_create_publisher(participant, NULL, NULL);
  if (publisher < 0)
  {
    ROS_FATAL("couldn't create DDS publisher: %s",
        dds_strretcode(-publisher));
    return 1;
  }

  dds_qos_t *qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
  dds_entity_t writer = dds_create_writer(
      participant,
      topic,
      qos,
      NULL);
  dds_delete_qos(qos);

  if (writer < 0)
  {
    ROS_FATAL("couldn't create DDS writer: %s", dds_strretcode(-writer));
    return 1;
  }

  ROS_INFO("created DDS stuff.");

  FreeFleetData_RobotState msg;
  msg.name = "robot_name";
  msg.model = "robot_model";
  msg.mode.mode = FreeFleetData_RobotMode_Constants_MODE_IDLE;
  msg.battery_percent = 100.0;
  msg.location.sec = 0;
  msg.location.nanosec = 0;
  msg.location.x = 1.0;
  msg.location.y = 2.0;
  msg.location.yaw = 3.0;
  msg.location.level_name = "L1";
  msg.path._maximum = 0;
  msg.path._length = 0;
  msg.path._buffer = NULL;
  msg.path._release = true;  // not sure what this means

  ros::Time t_prev_send(ros::Time::now());
  uint32_t status = 0;
  while (ros::ok())
  {
    ros::spinOnce();
 
    dds_get_status_changes(writer, &status);

    dds_sleepfor(DDS_MSECS(20));
    
    ros::Time t(ros::Time::now());
    if ((t - t_prev_send).toSec() > 1.0)
    {
      t_prev_send = t;
      ROS_INFO("send");
      msg.location.sec = t.sec;
      msg.location.nanosec = t.nsec;
      rc = dds_write(writer, &msg);
      if (rc != DDS_RETCODE_OK)
      {
        ROS_ERROR("dds write failed: %s", dds_strretcode(-rc));
        break;
      }
    }
  }

  rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK)
  {
    ROS_FATAL("dds_delete: %s", dds_strretcode(-rc));
  }

  return 0;
}
