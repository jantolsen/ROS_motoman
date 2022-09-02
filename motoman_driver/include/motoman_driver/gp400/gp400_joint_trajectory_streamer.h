/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, NOV
 * All rights reserved.
 *
 */

// Include guard:
// -------------------------------
// Prevents double declaration of identifiers (e.g. types, enums, static variables)
//  #ifndef: 
//      Check whether header-file with the unique value "xxx_H" is already included
//  #define: 
//      If header-file not earlier included, it continues and defines the rest of the file 
//  #endif: 
//      End of include guard
#ifndef MOTOMAN_GP400_JOINT_TRAJECTORY_STREAMER_H
#define MOTOMAN_GP400_JOINT_TRAJECTORY_STREAMER_H

// Include Header-files:
// -------------------------------
#include "motoman_driver/joint_trajectory_streamer.h"
#include "motoman_driver/gp400/gp400_utils.h"

// Namespace: Motoman
// -------------------------------
namespace motoman
{
// Namespace: GP400
// -------------------------------
namespace GP400
{

/**
 * \brief Motman GP400 Specific Message handler that relays joint positions (converts simple message
 * types to ROS message types and publishes them)
 * This class inherits from the "joint_trajectory_streamer" of the "industrial_robot_client" to accomodate 
 * Parallel-Linkage implementation  
 */
class JointTrajectoryStreamer : public motoman::joint_trajectory_streamer::MotomanJointTrajectoryStreamer
{
  
  bool J23_coupled_;
  using motoman::joint_trajectory_streamer::MotomanJointTrajectoryStreamer::init;

  // Public Class members
  // -------------------------------
  // Accessible for everyone
  public:

    

    // Class constructor
    // -------------------------------
    JointTrajectoryStreamer();

    
    // Class destructor
    // -------------------------------
    ~JointTrajectoryStreamer();

    // Class methods
    // -----------------------

      // Initialization
      // -------------------------------
      // Initialize robot connection using default method
      bool init(std::string default_ip = "", int default_port = industrial::simple_socket::StandardSocketPorts::MOTION, bool version_0 = false);

      // Transform
      // -------------------------------
      // Correct for Parellel-Linkage effects if desired 
      // Function Overloading:
      //  Multiple definitions, allows for different ways of calling the function

        // Transform
        // (Joint Trajectory Points)
        bool transform(const trajectory_msgs::JointTrajectoryPoint& pos_in, trajectory_msgs::JointTrajectoryPoint* pos_out);

        // Transform
        // (Dynamic Joints Group)
        bool transform(const motoman_msgs::DynamicJointsGroup& pos_in, motoman_msgs::DynamicJointsGroup* pos_out);

        // Transform
        // (Joint-Vector)
        bool transform(const std::vector<double>& pos_in, std::vector<double>* pos_out);
};  

} // Namespace GP400
} // Namespace Motoman
#endif  // MOTOMAN_GP400_JOINT_TRAJECTORY_STREAMER_H
