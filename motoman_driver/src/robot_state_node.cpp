/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Southwest Research Institute
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Southwest Research Institute, nor the names
 *      of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "motoman_driver/industrial_robot_client/robot_state_interface.h"

// Parallel-Linkage
#include "motoman_driver/gp400/gp400_joint_relay_handler.h"
#include "motoman_driver/gp400/gp400_joint_feedback_relay_handler.h"

using industrial_robot_client::robot_state_interface::RobotStateInterface;

int main(int argc, char** argv)
{
  const int FS100_state_port = 50241;  // FS100 uses a "non-standard" port to comply with MotoPlus guidelines

  // initialize node
  ros::init(argc, argv, "state_interface");


  ROS_ERROR("ROBOT STATE NODE INITIALIZED");

  // launch the default Robot State Interface connection/handlers
  RobotStateInterface rsi;

  rsi.init("", FS100_state_port, false);
  sleep(5);

  while(rsi.get_connection()->isConnected()==false){
    rsi.init("", FS100_state_port, false);
    sleep(5);
  }

  // Parallel-Linkage
  // -------------------------------
  // If the Robot is configured with a coupled-linkage between Joint 2 and Joint 3
  // also known as a Parallel-Linkage.
  // A manipulation is required of the received (Motor->ROS) and sent (ROS->Motor) Joint-Info
  // This is governed by the J23_coupled parameter from the parameter-server
  
  // Local variable for active Parallel-Linkage
  bool j23_coupled = false; // Initialize as false

  // Declare Parallel-Linkage Joint-Handler(s) for GP400-version
  motoman::GP400::JointRelayHandler jointHandler;                   // for joint-linkage correction
  motoman::GP400::JointFeedbackRelayHandler jointFeedbackHandler;   // for joint-linkage correction

  // Get Parallel-Linkage value from Parameter-Server
  // (assign "false" as default value, if parameter doesn't exist)
  ros::param::param("J23_coupled", j23_coupled, false);

  // Check for enabled Parallel-Linkage
  if (j23_coupled)
  {
    // Report to Terminal
    ROS_INFO("RobotStateNode: Intializing with Parallel-Linkage");

    // Obtain Robot-Group from RobotStateInterface
    RobotGroup robot_group;
    std::map<int, RobotGroup> robot_group_map;
    std::vector<std::string> joint_names;

    // Get Robot Group data
    robot_group_map = rsi.get_robot_groups();

    // Loop-through map of Robot Groups
    for(auto it = robot_group_map.begin(); it != robot_group_map.end(); it++)
    {
      // Get Robot-Group object from the map
      robot_group = it->second; // (first: key of the map, second: element of the map )

      // Obtain Joint-Names from the Robot-Group
      joint_names = robot_group.get_joint_names();

      // Debug to confirm Joint-Names are correctly obtained 
      ROS_INFO("Joint Names:");
      ROS_INFO("Size Names: (%zd)", joint_names.size());
      for (int i = 0; i < joint_names.size(); i++)
      {
        ROS_INFO("Joint (%i): (%s)", i, joint_names.at(i).c_str());
      }
    }

    // Initialize Joint-Handler(s)
    jointHandler.init(rsi.get_connection(), robot_group_map);
    jointFeedbackHandler.init(rsi.get_connection(), robot_group_map);

    // Add Joint-Handler(s) to RobotStateInterface
    rsi.add_handler(&jointHandler);
    rsi.add_handler(&jointFeedbackHandler);
  }
  
  rsi.run();

  return 0;
}

