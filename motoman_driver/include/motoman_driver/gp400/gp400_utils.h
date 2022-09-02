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
#ifndef MOTOMAN_GP400_UTILS_H_
#define MOTOMAN_GP400_UTILS_H_

// Include Header-files:
// -------------------------------
#include <vector>
#include "trajectory_msgs/JointTrajectory.h"
#include "motoman_msgs/DynamicJointsGroup.h"

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

/**
 * \brief Corrects for parallel linkage coupling between joints.
 *
 * \param[in] pt_in input joint trajectory point
 * \param[out] pt_out output joint trajectory point
 * \param[in] J23_factor  Linkage factor for J2-J3
 *   J3_out = J3_in + j23_factor * J2_in
 */
void linkage_transform(const trajectory_msgs::JointTrajectoryPoint& point_in, trajectory_msgs::JointTrajectoryPoint* point_out, double J23_factor=0);

/**
 * \brief Corrects for parallel linkage coupling between joints.
 *
 * \param[in] pt_in input dynamic joint group
 * \param[out] pt_out output dynamic joint group
 * \param[in] J23_factor  Linkage factor for J2-J3
 *   J3_out = J3_in + j23_factor * J2_in
 */
void linkage_transform(const motoman_msgs::DynamicJointsGroup& point_in, motoman_msgs::DynamicJointsGroup* point_out, double J23_factor=0);

/**
 * \brief Corrects for parallel linkage coupling between joints.
 *
 * \param[in] joints_in input joint angles
 * \param[out] joints_out output joint angles
 * \param[in] J23_factor  Linkage factor for J2-J3
 *   J3_out = J3_in + j23_factor * J2_in
 */
void linkage_transform(const std::vector<double>& joint_in, std::vector<double>* joint_out, double J23_factor=0);

} // Namespace Utilities
} // Namespace GP400
} // Namespace Motoman

#endif // MOTOMAN_GP400_UTILS_H_
