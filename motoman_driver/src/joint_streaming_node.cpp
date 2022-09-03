/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Southwest Research Institute
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

#include "motoman_driver/joint_trajectory_streamer.h"
#include "industrial_utils/param_utils.h"

// Parallel-Linkage
#include "motoman_driver/gp400/gp400_joint_trajectory_streamer.h"

using motoman::joint_trajectory_streamer::MotomanJointTrajectoryStreamer;

int main(int argc, char** argv)
{
  const int FS100_motion_port = 50240;  // FS100 uses a "non-standard" port to comply with MotoPlus guidelines

  // initialize node
  ros::init(argc, argv, "motion_interface");

  // Parallel-Linkage
  // -------------------------------
  // If the Robot is configured with a coupled-linkage between Joint 2 and Joint 3
  // also known as a Parallel-Linkage.
  // A manipulation is required of the received (Motor->ROS) and sent (ROS->Motor) Joint-Info
  // This is governed by the J23_coupled parameter from the parameter-server

  // Local variable for active Parallel-Linkage
  bool j23_coupled = false; // Initialize as false;

  // Get Parallel-Linkage value from Parameter-Server
  // (assign "false" as default value, if parameter doesn't exist)
  ros::param::param("J23_coupled", j23_coupled, false);

  // Check for enabled Parallel-Linkage
  if (j23_coupled)
  {
    // Report to Terminal
    ROS_INFO("JointStreamingNode: Intializing with Parallel-Linkage");

    // Launch the GP400 JointTrajectoryStreamer connection/handlers
    // (Enabling Parallel-Linkage)
    motoman::GP400::JointTrajectoryStreamer motionInterface;

    motionInterface.init("", FS100_motion_port, false);

    if(motionInterface.is_connected()==false)
      throw "Communication not established";

    motionInterface.run();
  }

  // No enabled Parallel-Linkage parameter
  else
  {
    // launch the FS100 JointTrajectoryStreamer connection/handlers
    MotomanJointTrajectoryStreamer motionInterface;

    motionInterface.init("", FS100_motion_port, false);

    if(motionInterface.is_connected()==false)
      throw "Communication not established";

    motionInterface.run();
  }

  return 0;
}
