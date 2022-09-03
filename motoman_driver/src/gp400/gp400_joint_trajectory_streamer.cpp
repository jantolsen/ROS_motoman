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
    #include "motoman_driver/gp400/gp400_joint_trajectory_streamer.h"

// Namespace: Motoman
// -------------------------------
namespace motoman
{
// Namespace: GP400
// -------------------------------
namespace GP400
{

// Motoman - Joint Trajectory Streamer Class
// -------------------------------

// Class Constructor
// -------------------------------
JointTrajectoryStreamer::JointTrajectoryStreamer() : motoman::joint_trajectory_streamer::MotomanJointTrajectoryStreamer()
{
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
JointTrajectoryStreamer::~JointTrajectoryStreamer()
{

}
// Initialization
// -------------------------------
// Initialize robot connection using default method
// bool JointTrajectoryStreamer::init(std::string default_ip = "", int default_port = 50240, bool version_0 = false)
bool JointTrajectoryStreamer::init(std::string default_ip, int default_port, bool version_0)
{
  // Call Base-Class Initialization
  if (!motoman::joint_trajectory_streamer::MotomanJointTrajectoryStreamer::init(default_ip, default_port, version_0))
  {
    return false;
  }
    
  // Check Parameter Server for enabled Parallel-Linkage
  if (ros::param::has("J23_coupled"))
  {
    // Enable J23-Coupled parameter
    ros::param::get("J23_coupled", this->J23_coupled_);

    // Report to terminal
    ROS_INFO("Joint Trajectory Streamer - Using Parallel-Linkage (J23-Coupled)");
  }
  
  // No enabled Parallel-Linkage parameter
  else
  {
    // Disable J23-Coupled parameter
    J23_coupled_ = false;
  }

  return true;
}

// Transform
// -------------------------------
// Correct for Parellel-Linkage effects if desired 
// (Function Overloading)

  // Transform
  // (Joint Trajectory Points)
  bool JointTrajectoryStreamer::transform(const trajectory_msgs::JointTrajectoryPoint& pos_in, trajectory_msgs::JointTrajectoryPoint* pos_out)
  {
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );

    return true;
  }

  // Transform
  // (Dynamic Joints Group)
  bool JointTrajectoryStreamer::transform(const motoman_msgs::DynamicJointsGroup& pos_in, motoman_msgs::DynamicJointsGroup* pos_out)
  {
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );

    return true;
  }

  // Transform
  // (Joint-Vector)
  bool JointTrajectoryStreamer::transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );

    return true;
  }

} // Namespace GP400
} // Namespace Motoman
