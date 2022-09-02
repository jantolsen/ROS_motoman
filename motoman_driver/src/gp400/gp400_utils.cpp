/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022, NOV
 * All rights reserved.
 *
 */

// Include Header-files:
// -------------------------------
#include "motoman_driver/gp400/gp400_utils.h"
#include "ros/ros.h"

// Namespace: Motoman
// -------------------------------
namespace motoman
{
// Namespace: GP400
// -------------------------------
namespace GP400
{
// Namespace: GP400
// -------------------------------
namespace utils
{

// Linkage Transform
// -------------------------------
// Correct for Parellel-Linkage effects if desired 
// (Function Overloading)

  // Linkage Transform
  // (Joint Trajectory Points)
  void linkage_transform(const trajectory_msgs::JointTrajectoryPoint& point_in, trajectory_msgs::JointTrajectoryPoint* point_out, double J23_factor)
  {
    // Copy the original points
    *point_out = point_in;

    // Transform for Parallel-Linkage
    linkage_transform(point_in.positions, &(point_out->positions), J23_factor);
    
  }

  // Transform
  // (Dynamic Joints Group)
  void linkage_transform(const motoman_msgs::DynamicJointsGroup& point_in, motoman_msgs::DynamicJointsGroup* point_out, double J23_factor)
  {
    // Copy the original points
    *point_out = point_in;

    // Transform for Parallel-Linkage
    linkage_transform(point_in.positions, &(point_out->positions), J23_factor);
  }

  // Transform
  // (Joint-Vector)
  void linkage_transform(const std::vector<double>& joints_in, std::vector<double>* joints_out, double J23_factor)
  {

    // Ensure incomming Joint-Vector size is large enough for Paralell-Linkage transform
    // ROS_ERROR("Linkage Transform - Size of Input-Points: (%zd)", joints_in.size());
    ROS_ASSERT(joints_in.size() > 3);

    // Copy original Points
    *joints_out = joints_in;

    // Transform and accomodate for Parallel-Linkage
    // -------------------------------
    // (Ensure incomming Joint-Vector size is large enough for Paralell-Linkage transform)
    if (joints_in.size() > 3)
    {
      // ROS_INFO("Linkage Transform - Before Transform: Joint 3: (%f)", joints_out->at(2));

      joints_out->at(2) += J23_factor * joints_out->at(1);

      // ROS_INFO("Linkage Transform - After Transform: Joint 3: (%f)", joints_out->at(2));
    }
    else
    {
      ROS_ERROR("Linkage Transform - Size of Input-Points is below 3");
    }
  }

} // Namespace Utilities
} // Namespace GP400
} // Namespace Motoman
