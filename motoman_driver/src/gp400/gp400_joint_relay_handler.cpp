/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, NOV
 * All rights reserved.
 *
 */

// Include Header-files:
// -------------------------------

    // Class Header-File
    #include "motoman_driver/gp400/gp400_joint_relay_handler.h"

// Namespace: Motoman
// -------------------------------
namespace motoman
{
// Namespace: GP400
// -------------------------------
namespace GP400
{

// Motoman GP400 - Joint Relay Handler Class
// -------------------------------

// Class Constructor
// -------------------------------
// JointRelayHandler::JointRelayHandler() : industrial_robot_client::joint_info_handler::JointInfoHandler()
JointRelayHandler::JointRelayHandler() : industrial_robot_client::joint_relay_handler::JointRelayHandler()
{
  // Check for Parallel-Linkage
  checkParalellLinkage();
  
  // // Check Parameter Server for enabled Parallel-Linkage
  // if (ros::param::has("J23_coupled"))
  // {
  //   // Enable J23-Coupled parameter
  //   ros::param::get("J23_coupled", this->J23_coupled_);

  // }
  
  // // No enabled Parallel-Linkage parameter
  // else
  // {
  //   // Disable J23-Coupled parameter
  //   J23_coupled_ = false;
  // }
}

// Class Destructor
// -------------------------------
JointRelayHandler::~JointRelayHandler()
{
  
}

// Parallel-Linkage
// -------------------------------
// Check for Parameter-Server for Parallel-Linkage
bool JointRelayHandler::checkParalellLinkage()
{
  // Check Parameter Server for enabled Parallel-Linkage
  if (ros::param::has("J23_coupled"))
  {
    // Enable J23-Coupled parameter
    ros::param::get("J23_coupled", this->J23_coupled_);

    // Report to terminal
    ROS_INFO("Joint Relay Handler - Using Parallel-Linkage (J23-Coupled)");
  }
  
  // No enabled Parallel-Linkage parameter
  else
  {
    // Disable J23-Coupled parameter
    J23_coupled_ = false;
  }

  return J23_coupled_;
}

// Transform
// -------------------------------
// Correct for Parellel-Linkage effects if desired 
// (Function Overloading)

  // Transform
  // (Joint Trajectory Points)
  bool JointRelayHandler::transform(const trajectory_msgs::JointTrajectoryPoint& pos_in, trajectory_msgs::JointTrajectoryPoint* pos_out)
  {
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );

    return true;
  }

  // Transform
  // (Dynamic Joints Group)
  bool JointRelayHandler::transform(const motoman_msgs::DynamicJointsGroup& pos_in, motoman_msgs::DynamicJointsGroup* pos_out)
  {
    /// Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );

    return true;
  }

  // Transform
  // (Joint-Vector)
  bool JointRelayHandler::transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );

    return true;
  }

} // Namespace GP400
} // Namespace Motoman
