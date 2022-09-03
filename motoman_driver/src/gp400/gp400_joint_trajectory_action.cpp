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
    #include "motoman_driver/gp400/gp400_joint_trajectory_action.h"

    // #include <motoman_driver/industrial_robot_client/joint_trajectory_action.h>
    #include "motoman_driver/industrial_robot_client/motoman_utils.h"
    #include <industrial_robot_client/utils.h>
    #include <industrial_utils/param_utils.h>
    #include <industrial_utils/utils.h>
    #include <map>
    #include <string>
    #include <vector>

using industrial_robot_client::motoman_utils::getJointGroups;
using motoman::simple_message::motion_reply::MotionReply;

// Namespace: Industrial Robot Client
// -------------------------------
namespace industrial_robot_client
{

// Namespace: Joint Trajectory Action
// -------------------------------
namespace joint_trajectory_action
{

// // Namespace: Motoman
// // -------------------------------
// namespace motoman
// {
  
// Namespace: GP400
// -------------------------------
namespace GP400
{

const double JointTrajectoryAction::WATCHD0G_PERIOD_ = 1.0;
const double JointTrajectoryAction::DEFAULT_GOAL_THRESHOLD_ = 0.01;

// Motoman - Joint Trajectory Action Class
// -------------------------------

// Class Constructor
// -------------------------------
JointTrajectoryAction::JointTrajectoryAction() :
  action_server_(node_, "joint_trajectory_action",
                 boost::bind(&JointTrajectoryAction::goalCB, this, _1),
                 boost::bind(&JointTrajectoryAction::cancelCB, this, _1), false)
{
  // Check Parallel-Linkage
  checkParalellLinkage();

  has_active_goal_ = false;

  ros::NodeHandle pn("~");

  pn.param("constraints/goal_threshold", goal_threshold_, DEFAULT_GOAL_THRESHOLD_);

  std::map<int, RobotGroup> robot_groups;
  getJointGroups("topic_list", robot_groups);

  for (int i = 0; i < robot_groups.size(); i++)
  {

    std::string joint_path_action_name = robot_groups[i].get_ns() + "/" + robot_groups[i].get_name();
    std::vector<std::string> rg_joint_names = robot_groups[i].get_joint_names();
    int group_number_int = robot_groups[i].get_group_id();

    all_joint_names_.insert(all_joint_names_.end(), rg_joint_names.begin(), rg_joint_names.end());

    actionServer_ = new JointTractoryActionServer(
      node_, joint_path_action_name + "/joint_trajectory_action" , false);
    actionServer_->registerGoalCallback(
      boost::bind(&JointTrajectoryAction::goalCB,
                  this, _1, group_number_int));
    actionServer_->registerCancelCallback(
      boost::bind(&JointTrajectoryAction::cancelCB,
                  this, _1, group_number_int));

    pub_trajectory_command_ = node_.advertise<motoman_msgs::DynamicJointTrajectory>(
                                joint_path_action_name + "/joint_path_command", 1);
    sub_trajectory_state_  = this->node_.subscribe<control_msgs::FollowJointTrajectoryFeedback>(
                               joint_path_action_name + "/feedback_states", 1,
                               boost::bind(&JointTrajectoryAction::controllerStateCB,
                                           this, _1, group_number_int));
    sub_robot_status_ = node_.subscribe(
                          "robot_status", 1, &JointTrajectoryAction::robotStatusCB, this);

    pub_trajectories_[group_number_int] = pub_trajectory_command_;
    sub_trajectories_[group_number_int] = (sub_trajectory_state_);
    sub_status_[group_number_int] = (sub_robot_status_);


    this->act_servers_[group_number_int] = actionServer_;

    this->act_servers_[group_number_int]->start();

    this->watchdog_timer_map_[group_number_int] = node_.createTimer(
          ros::Duration(WATCHD0G_PERIOD_), boost::bind(
            &JointTrajectoryAction::watchdog, this, _1, group_number_int));
  }

  feedback_states_ = node_.subscribe<control_msgs::FollowJointTrajectoryFeedback>(
        "feedback_states_dgm", 1,
        boost::bind(&JointTrajectoryAction::controllerStateCBDGM,
                    this, _1));

  pub_trajectory_command_ = node_.advertise<motoman_msgs::DynamicJointTrajectory>(
                              "joint_path_command", 1);

  sub_motoman_motion_reply_codes_ = this->node_.subscribe(
                                    "motoman_motion_reply_codes", 1, &JointTrajectoryAction::motomanMotionReplyCodesCB, this);


  this->robot_groups_ = robot_groups;

  action_server_.start();
}

// Class Destructor
// -------------------------------
JointTrajectoryAction::~JointTrajectoryAction()
{

}

// Parallel-Linkage
// -------------------------------
// Check for Parameter-Server for Parallel-Linkage
bool JointTrajectoryAction::checkParalellLinkage()
{
  // Check Parameter Server for enabled Parallel-Linkage
  if (ros::param::has("J23_coupled"))
  {
    // Enable J23-Coupled parameter
    ros::param::get("J23_coupled", this->J23_coupled_);

    // Report to terminal
    ROS_INFO("Joint Trajectory Action - Using Parallel-Linkage (J23-Coupled)");
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
  bool JointTrajectoryAction::transform(const trajectory_msgs::JointTrajectoryPoint& pos_in, trajectory_msgs::JointTrajectoryPoint* pos_out)
  {
    // ROS_ERROR("GP400 - Joint Trajectory Action: TRANSFORM No. 1");
    
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );

    return true;
  }

  // Transform
  // (Dynamic Joints Group)
  bool JointTrajectoryAction::transform(const motoman_msgs::DynamicJointsGroup& pos_in, motoman_msgs::DynamicJointsGroup* pos_out)
  {
    // ROS_ERROR("GP400 - Joint Trajectory Action: TRANSFORM No. 2");
    
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );

    return true;
  }

  // Transform
  // (Joint-Vector)
  bool JointTrajectoryAction::transform(const std::vector<double>& pos_in, std::vector<double>* pos_out)
  {
    // Correct for Parallel-Linkage effects
    //   - use POSITIVE factor for motor->joint correction
    //   - use NEGATIVE factor for joint->motor correction
    motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? -1:0 );
    // motoman::GP400::utils::linkage_transform(pos_in, pos_out, J23_coupled_ ? 1:0 );

    return true;
  }

// Goal Callback
// -------------------------------
// Controller state callback 
// (executed when feedback message is received)
// (Function Overloading)

  // Goal Callback
  // (Action-Goal Handle with Group-Number)
  void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh)
  {
    gh.setAccepted();
    int group_number;

    // Parallel-Linkage
    // -------------------------------
    trajectory_msgs::JointTrajectory trajectory_;
    
    // Copy the original Trajectory
    trajectory_ = gh.getGoal()->trajectory;

    // Transform for Parallel-Linkage
    for (int i = 0; i < gh.getGoal()->trajectory.points.size(); i++)
    {
      if (!transform(gh.getGoal()->trajectory.points[i], &trajectory_.points[i]))
      {
        ROS_ERROR("Failed to transform joint state");
        return;
      }
    }

  // TODO(thiagodefreitas): change for getting the id from the group instead of a sequential checking on the map

    ros::Duration last_time_from_start(0.0);

    motoman_msgs::DynamicJointTrajectory dyn_traj;

    active_goal_ = gh;
    current_traj_ = trajectory_;

    for (int i = 0; i < trajectory_.points.size(); i++)
    {
      motoman_msgs::DynamicJointPoint dpoint;

      for (int rbt_idx = 0; rbt_idx < robot_groups_.size(); rbt_idx++)
      {
        size_t ros_idx = std::find(
                          trajectory_.joint_names.begin(),
                          trajectory_.joint_names.end(),
                          robot_groups_[rbt_idx].get_joint_names()[0])
                        - trajectory_.joint_names.begin();

        bool is_found = ros_idx < trajectory_.joint_names.size();

        group_number = rbt_idx;
        motoman_msgs::DynamicJointsGroup dyn_group;

        int num_joints = robot_groups_[group_number].get_joint_names().size();

        if (is_found)
        {
          if (trajectory_.points[i].positions.empty())
          {
            std::vector<double> positions(num_joints, 0.0);
            dyn_group.positions = positions;
          }
          else
            dyn_group.positions.insert(
              dyn_group.positions.begin(),
              trajectory_.points[i].positions.begin() + ros_idx,
              trajectory_.points[i].positions.begin() + ros_idx
              + robot_groups_[rbt_idx].get_joint_names().size());

          if (trajectory_.points[i].velocities.empty())
          {
            std::vector<double> velocities(num_joints, 0.0);
            dyn_group.velocities = velocities;
          }
          else
            dyn_group.velocities.insert(
              dyn_group.velocities.begin(),
              trajectory_.points[i].velocities.begin()
              + ros_idx, trajectory_.points[i].velocities.begin()
              + ros_idx + robot_groups_[rbt_idx].get_joint_names().size());

          if (trajectory_.points[i].accelerations.empty())
          {
            std::vector<double> accelerations(num_joints, 0.0);
            dyn_group.accelerations = accelerations;
          }
          else
            dyn_group.accelerations.insert(
              dyn_group.accelerations.begin(),
              trajectory_.points[i].accelerations.begin()
              + ros_idx, trajectory_.points[i].accelerations.begin()
              + ros_idx + robot_groups_[rbt_idx].get_joint_names().size());
          if (trajectory_.points[i].effort.empty())
          {
            std::vector<double> effort(num_joints, 0.0);
            dyn_group.effort = effort;
          }
          else
            dyn_group.effort.insert(
              dyn_group.effort.begin(),
              trajectory_.points[i].effort.begin()
              + ros_idx, trajectory_.points[i].effort.begin()
              + ros_idx + robot_groups_[rbt_idx].get_joint_names().size());
          dyn_group.time_from_start = trajectory_.points[i].time_from_start;
          dyn_group.group_number = group_number;
          dyn_group.num_joints = dyn_group.positions.size();
        }

        // Generating message for groups that were not present in the trajectory message
        else
        {
          std::vector<double> positions(num_joints, 0.0);
          std::vector<double> velocities(num_joints, 0.0);
          std::vector<double> accelerations(num_joints, 0.0);
          std::vector<double> effort(num_joints, 0.0);

          dyn_group.positions = positions;
          dyn_group.velocities = velocities;
          dyn_group.accelerations = accelerations;
          dyn_group.effort = effort;

          dyn_group.time_from_start = trajectory_.points[i].time_from_start;
          dyn_group.group_number = group_number;
          dyn_group.num_joints = num_joints;
        }

        dpoint.groups.push_back(dyn_group);
      }
      dpoint.num_groups = dpoint.groups.size();
      dyn_traj.points.push_back(dpoint);
    }
    dyn_traj.header = trajectory_.header;
    dyn_traj.header.stamp = ros::Time::now();
    // Publishing the joint names for the 4 groups
    dyn_traj.joint_names = all_joint_names_;

    this->pub_trajectory_command_.publish(dyn_traj);

    has_active_goal_ = true;
    have_points_streamed_ = true;
  }

  // Goal Callback
  // (Action-Goal Handle with Group-Number)
  void JointTrajectoryAction::goalCB(JointTractoryActionServer::GoalHandle gh, int group_number)
  {
    if (!gh.getGoal()->trajectory.points.empty())
    {
      if (industrial_utils::isSimilar(
            this->robot_groups_[group_number].get_joint_names(),
            gh.getGoal()->trajectory.joint_names))
      {
        // Cancels the currently active goal.
        if (has_active_goal_map_[group_number])
        {
          ROS_WARN("Received new goal, canceling current goal");
          abortGoal(group_number);
        }
        // Sends the trajectory along to the controller
        if (withinGoalConstraints(last_trajectory_state_map_[group_number],
                                  gh.getGoal()->trajectory, group_number))
        {
          ROS_INFO_STREAM("Already within goal constraints, setting goal succeeded");
          gh.setAccepted();
          gh.setSucceeded();
          has_active_goal_map_[group_number] = false;
        }
        else
        {
          gh.setAccepted();
          active_goal_map_[group_number] = gh;
          has_active_goal_map_[group_number]  = true;

          ROS_INFO("Publishing trajectory");

          current_traj_map_[group_number] = active_goal_map_[group_number].getGoal()->trajectory;

          int num_joints = robot_groups_[group_number].get_joint_names().size();

          motoman_msgs::DynamicJointTrajectory dyn_traj;

          // Parallel-Linkage
          // -------------------------------
          trajectory_msgs::JointTrajectory trajectory_;
          
          // Copy the original Trajectory
          trajectory_ = gh.getGoal()->trajectory;

          // Transform for Parallel-Linkage
          for (int i = 0; i < gh.getGoal()->trajectory.points.size(); i++)
          {
            if (!transform(gh.getGoal()->trajectory.points[i], &trajectory_.points[i]))
            {
              ROS_ERROR("Failed to transform joint state");
              return;
            }
          }

          for (int i = 0; i < current_traj_map_[group_number].points.size(); ++i)
          {
            motoman_msgs::DynamicJointsGroup dyn_group;
            motoman_msgs::DynamicJointPoint dyn_point;

            if (trajectory_.points[i].positions.empty())
            {
              std::vector<double> positions(num_joints, 0.0);
              dyn_group.positions = positions;
            }
            else
              dyn_group.positions = trajectory_.points[i].positions;

            if (trajectory_.points[i].velocities.empty())
            {
              std::vector<double> velocities(num_joints, 0.0);
              dyn_group.velocities = velocities;
            }
            else
              dyn_group.velocities = trajectory_.points[i].velocities;
            if (trajectory_.points[i].accelerations.empty())
            {
              std::vector<double> accelerations(num_joints, 0.0);
              dyn_group.accelerations = accelerations;
            }
            else
              dyn_group.accelerations = trajectory_.points[i].accelerations;
            if (trajectory_.points[i].effort.empty())
            {
              std::vector<double> effort(num_joints, 0.0);
              dyn_group.effort = effort;
            }
            else
              dyn_group.effort = trajectory_.points[i].effort;
            dyn_group.time_from_start = trajectory_.points[i].time_from_start;
            dyn_group.group_number = group_number;
            dyn_group.num_joints = robot_groups_[group_number].get_joint_names().size();
            dyn_point.groups.push_back(dyn_group);

            dyn_point.num_groups = 1;
            dyn_traj.points.push_back(dyn_point);
          }
          dyn_traj.header = trajectory_.header;
          dyn_traj.joint_names = trajectory_.joint_names;
          this->pub_trajectories_[group_number].publish(dyn_traj);
        }
      }
      else
      {
        ROS_ERROR("Joint trajectory action failing on invalid joints");
        control_msgs::FollowJointTrajectoryResult rslt;
        rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_JOINTS;
        gh.setRejected(rslt, "Joint names do not match");
      }
    }
    else
    {
      ROS_ERROR("Joint trajectory action failed on empty trajectory");
      control_msgs::FollowJointTrajectoryResult rslt;
      rslt.error_code = control_msgs::FollowJointTrajectoryResult::INVALID_GOAL;
      gh.setRejected(rslt, "Empty trajectory");
    }

    // Adding some informational log messages to indicate unsupported goal constraints
    if (gh.getGoal()->goal_time_tolerance.toSec() > 0.0)
    {
      ROS_WARN_STREAM("Ignoring goal time tolerance in action goal, may be supported in the future");
    }
    if (!gh.getGoal()->goal_tolerance.empty())
    {
      ROS_WARN_STREAM(
        "Ignoring goal tolerance in action, using paramater tolerance of " << goal_threshold_ << " instead");
    }
    if (!gh.getGoal()->path_tolerance.empty())
    {
      ROS_WARN_STREAM("Ignoring goal path tolerance, option not supported by ROS-Industrial drivers");
    }
  }

  void JointTrajectoryAction::robotStatusCB(
  const industrial_msgs::RobotStatusConstPtr &msg)
  {
    last_robot_status_ = msg;  // caching robot status for later use.
  }

  void JointTrajectoryAction::createTimers()
  {
    timer1 = node_.createTimer(ros::Duration(0.25), boost::bind(&JointTrajectoryAction::novTriggers, this, _1));
    timer2 = node_.createTimer(ros::Duration(WATCHD0G_PERIOD_), boost::bind(&JointTrajectoryAction::watchdog, this, _1));
    feedback_states_ = node_.subscribe<control_msgs::FollowJointTrajectoryFeedback>(
          "feedback_states_dgm", 1,
          boost::bind(&JointTrajectoryAction::controllerStateCBDGM,
                      this, _1));
  }

  void JointTrajectoryAction::novTriggers(const ros::TimerEvent &e)
  {
    if (has_active_goal_)
    {
      if(!have_points_streamed_)
      {
        ROS_DEBUG("Points haven't streamed yet, waiting till next trigger.");
        last_nonstream_time_ = ros::Time::now();
        return;
      }
      ROS_DEBUG("Looking at Triggers");

      // kills goal if the robot can't move
      if(last_robot_status_.get()->in_error.val == 1)
      {
        ROS_ERROR_STREAM("Robot Status indicates the robot is in an error state. Code: " << last_robot_status_.get()->error_code);
        abortGoal();
        return;
      }else if (last_robot_status_.get()->e_stopped.val == 1)
      {
        ROS_ERROR("Well this isn't going to work...the robot is showing E-Stopped.");
        abortGoal();
        return;
      }

      if(robot_motion_since_stream_ == false)
      {
        if(last_robot_status_.get()->in_motion.val == 1)
        {
          robot_motion_since_stream_ = true;
          ROS_INFO("Robot in motion");
          return;
        }
        else
        {
          // kills goal if robot doesnt start soon after stream
          if (ros::Time::now() - last_nonstream_time_ > ros::Duration(5.0))
          {
            ROS_ERROR("Its been over 5 seconds since the points have streamed. Motion Streaming Interface might have triggered an error.");
            abortGoal();
            return;
          }
          ROS_INFO("Waiting for robot to start moving");
          return;
        }
      }
      else
      {
        if(last_robot_status_.get()->in_motion.val == 0)
        {
          if(motion_stopped_check_count_ > 4) // novTriggers loops every .25s, so 4 here = 1 sec.
          {
            motion_stopped_check_count_ = 0;
            ROS_ERROR_STREAM("Something happened and the robot stopped.");
            novAbortGoalTraj();
            return;
          }
          else
          {
            motion_stopped_check_count_++;
            return;
          }
        }
        else
        {
          return;
        }
      }
    }
    motion_stopped_check_count_ = 0;
    robot_motion_since_stream_ = false;
    last_nonstream_time_ = ros::Time::now();
    return;
  }

  void JointTrajectoryAction::watchdog(const ros::TimerEvent &e)
  {
    // Some debug logging
    if (!last_trajectory_state_)
    {
      ROS_DEBUG("Waiting for subscription to joint trajectory state");
    }
    if (!trajectory_state_recvd_)
    {
      ROS_DEBUG("Trajectory state not received since last watchdog");
    }

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_)
    {
      if (!trajectory_state_recvd_)
      {
        // last_trajectory_state_ is null if the subscriber never makes a connection
        if (!last_trajectory_state_)
        {
          ROS_WARN("Aborting goal because we have never heard a controller state message.");
        }
        else
        {
          ROS_WARN_STREAM(
            "Aborting goal because we haven't heard from the controller in " << WATCHD0G_PERIOD_ << " seconds");
        }
        abortGoal();
      }
    }

    // Reset the trajectory state received flag
    trajectory_state_recvd_ = false;
  }

  void JointTrajectoryAction::watchdog(const ros::TimerEvent &e, int group_number)
  {
    // Some debug logging
    if (!last_trajectory_state_map_[group_number])
    {
      ROS_DEBUG("Waiting for subscription to joint trajectory state");
    }
    if (!trajectory_state_recvd_map_[group_number])
    {
      ROS_DEBUG("Trajectory state not received since last watchdog");
    }

    // Aborts the active goal if the controller does not appear to be active.
    if (has_active_goal_map_[group_number])
    {
      if (!trajectory_state_recvd_map_[group_number])
      {
        // last_trajectory_state_ is null if the subscriber never makes a connection
        if (!last_trajectory_state_map_[group_number])
        {
          ROS_WARN("Aborting goal because we have never heard a controller state message.");
        }
        else
        {
          ROS_WARN_STREAM(
            "Aborting goal because we haven't heard from the controller in " << WATCHD0G_PERIOD_ << " seconds");
        }
        abortGoal(group_number);
      }
    }
    // Reset the trajectory state received flag
    trajectory_state_recvd_ = false;
  }

  void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle gh)
  {
    // The interface is provided, but it is recommended to use
    //  void JointTrajectoryAction::cancelCB(JointTractoryActionServer::GoalHandle & gh, int group_number)

    ROS_DEBUG("Received action cancel request");

    // Stops the controller.
  //  trajectory_msgs::JointTrajectory empty;
  //  this->pub_trajectory_command_.publish(empty);

    // Marks the current goal as aborted.
    active_goal_.setCanceled();
    has_active_goal_ = false;
  }

  void JointTrajectoryAction::cancelCB(
    JointTractoryActionServer::GoalHandle gh, int group_number)
  {
    ROS_DEBUG("Received action cancel request");
    if (active_goal_map_[group_number] == gh)
    {
      // Stops the controller.
      motoman_msgs::DynamicJointTrajectory empty;
      empty.joint_names = robot_groups_[group_number].get_joint_names();
      this->pub_trajectories_[group_number].publish(empty);

      // Marks the current goal as canceled.
      active_goal_map_[group_number].setCanceled();
      has_active_goal_map_[group_number] = false;
    }
    else
    {
      ROS_WARN("Active goal and goal cancel do not match, ignoring cancel request");
    }
  }

  void JointTrajectoryAction::controllerStateCB(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg, int robot_id)
  {
    ROS_DEBUG("Checking controller state feedback");
    last_trajectory_state_map_[robot_id] = msg;
    trajectory_state_recvd_map_[robot_id] = true;

    if (!has_active_goal_map_[robot_id])
    {
      ROS_DEBUG("No active goal, ignoring feedback");
      return;
    }

    if (current_traj_map_[robot_id].points.empty())
    {
      ROS_DEBUG("Current trajectory is empty, ignoring feedback");
      return;
    }

    if (!industrial_utils::isSimilar(robot_groups_[robot_id].get_joint_names(), msg->joint_names))
    {
      ROS_ERROR("Joint names from the controller don't match our joint names.");
      return;
    }

    // Checking for goal constraints
    // Checks that we have ended inside the goal constraints and has motion stopped

    ROS_DEBUG("Checking goal constraints");
    if (withinGoalConstraints(last_trajectory_state_map_[robot_id], current_traj_map_[robot_id], robot_id))
    {
      if (last_robot_status_)
      {
        // Additional check for motion stoppage since the controller goal may still
        // be moving.  The current robot driver calls a motion stop if it receives
        // a new trajectory while it is still moving.  If the driver is not publishing
        // the motion state (i.e. old driver), this will still work, but it warns you.
        if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
        {
          ROS_INFO("Inside goal constraints, stopped moving, return success for action");
          active_goal_map_[robot_id].setSucceeded();
          has_active_goal_map_[robot_id] = false;
        }
        else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
        {
          ROS_INFO("Inside goal constraints, return success for action");
          ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
          active_goal_map_[robot_id].setSucceeded();
          has_active_goal_map_[robot_id] = false;
        }
        else
        {
          ROS_DEBUG("Within goal constraints but robot is still moving");
        }
      }
      else
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
        active_goal_map_[robot_id].setSucceeded();
        has_active_goal_map_[robot_id] = false;
      }
    }
  }

  void JointTrajectoryAction::controllerStateCB(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
  {
    ROS_INFO("Checking controller state feedback");
    last_trajectory_state_ = msg;
    trajectory_state_recvd_ = true;

    if (!has_active_goal_)
    {
      ROS_DEBUG("No active goal, ignoring feedback");
      return;
    }
    if (current_traj_.points.empty())
    {
      ROS_DEBUG("Current trajectory is empty, ignoring feedback");
      return;
    }

    if (!industrial_utils::isSimilar(all_joint_names_, msg->joint_names))
    {
      ROS_ERROR("Joint names from the controller don't match our joint names.");
      return;
    }

    // Checking for goal constraints
    // Checks that we have ended inside the goal constraints and has motion stopped

    ROS_DEBUG("Checking goal constraints");
    if (withinGoalConstraints(last_trajectory_state_, current_traj_))
    {
      if (last_robot_status_)
      {
        // Additional check for motion stoppage since the controller goal may still
        // be moving.  The current robot driver calls a motion stop if it receives
        // a new trajectory while it is still moving.  If the driver is not publishing
        // the motion state (i.e. old driver), this will still work, but it warns you.
        if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
        {
          ROS_INFO("Inside goal constraints, stopped moving, return success for action");
          active_goal_.setSucceeded();
          has_active_goal_ = false;
        }
        else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
        {
          ROS_INFO("Inside goal constraints, return success for action");
          ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
          active_goal_.setSucceeded();
          has_active_goal_ = false;
        }
        else
        {
          ROS_DEBUG("Within goal constraints but robot is still moving");
        }
      }
      else
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
    }
  }

  void JointTrajectoryAction::novAbortGoalTraj()
  {
    // Stops the controller.
    //trajectory_msgs::JointTrajectory empty;
    //pub_trajectory_command_.publish(empty);

    // Marks the current goal as aborted.

    control_msgs::FollowJointTrajectoryResult errorCodeTaggedResult;
    errorCodeTaggedResult.error_code = static_cast<int>(motoman_motion_reply_codes_.subCode);
    ROS_ERROR_STREAM("Joint Trajectory Action is aborting the current goal with error code: " << errorCodeTaggedResult.error_code);
    active_goal_.setAborted(errorCodeTaggedResult);
    has_active_goal_ = false;
    robot_motion_since_stream_ = false;
  }

  void JointTrajectoryAction::abortGoal()
  {
    // Stops the controller.
    //trajectory_msgs::JointTrajectory empty;
    //pub_trajectory_command_.publish(empty);

    // Marks the current goal as aborted.
    ROS_ERROR("Joint Trajectory Action is aborting the current goal");
    active_goal_.setAborted();
    has_active_goal_ = false;
    robot_motion_since_stream_ = false;
  }

  void JointTrajectoryAction::abortGoal(int robot_id)
  {
    // Stops the controller.
    motoman_msgs::DynamicJointTrajectory empty;
    pub_trajectories_[robot_id].publish(empty);

    // Marks the current goal as aborted.
    active_goal_map_[robot_id].setAborted();
    has_active_goal_map_[robot_id] = false;
  }

  bool JointTrajectoryAction::withinGoalConstraints(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
    const trajectory_msgs::JointTrajectory & traj)
  {
    bool rtn = false;
    if (traj.points.empty())
    {
      ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
      rtn = false;
    }
    else
    {
      int last_point = traj.points.size() - 1;

      if (industrial_robot_client::utils::isWithinRange(
            last_trajectory_state_->joint_names,
            last_trajectory_state_->actual.positions, traj.joint_names,
            traj.points[last_point].positions, goal_threshold_))
      {
        rtn = true;
      }
      else
      {
        rtn = false;
      }
    }
    return rtn;
  }

  bool JointTrajectoryAction::withinGoalConstraints(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
    const trajectory_msgs::JointTrajectory & traj, int robot_id)
  {
    bool rtn = false;
    if (traj.points.empty())
    {
      ROS_WARN("Empty joint trajectory passed to check goal constraints, return false");
      rtn = false;
    }
    else
    {
      int last_point = traj.points.size() - 1;

      int group_number = robot_id;

      if (industrial_robot_client::utils::isWithinRange(
            robot_groups_[group_number].get_joint_names(),
            last_trajectory_state_map_[group_number]->actual.positions, traj.joint_names,
            traj.points[last_point].positions, goal_threshold_))
      {
        rtn = true;
      }
      else
      {
        rtn = false;
      }
    }
    return rtn;
  }
  void JointTrajectoryAction::controllerStateCBDGM(
    const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg)
  {
    ROS_DEBUG("Checking controller state feedback");
    last_trajectory_state_ = msg;
    trajectory_state_recvd_ = true;

    if (!has_active_goal_)
    {
      ROS_DEBUG("No active goal, ignoring feedback");
      return;
    }
    if (current_traj_.points.empty())
    {
      ROS_INFO("Current trajectory is empty, ignoring feedback");
      return;
    }

    if (!industrial_utils::isSimilar(all_joint_names_, msg->joint_names))
    {
      ROS_ERROR("Joint names from the controller don't match our joint names.");
      return;
    }

    // Checking for goal constraints
    // Checks that we have ended inside the goal constraints and has motion stopped

    ROS_DEBUG("Checking goal constraints");
    if (withinGoalConstraints(last_trajectory_state_, current_traj_))
    {
      if (last_robot_status_)
      {
        // Additional check for motion stoppage since the controller goal may still
        // be moving.  The current robot driver calls a motion stop if it receives
        // a new trajectory while it is still moving.  If the driver is not publishing
        // the motion state (i.e. old driver), this will still work, but it warns you.
        if (last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
        {
          ROS_INFO("Inside goal constraints, stopped moving, return success for action");
          active_goal_.setSucceeded();
          has_active_goal_ = false;
        }
        else if (last_robot_status_->in_motion.val == industrial_msgs::TriState::UNKNOWN)
        {
          ROS_INFO("Inside goal constraints, return success for action");
          ROS_WARN("Robot status in motion unknown, the robot driver node and controller code should be updated");
          active_goal_.setSucceeded();
          has_active_goal_ = false;
        }
        else
        {
          ROS_DEBUG("Within goal constraints but robot is still moving");
        }
      }
      else
      {
        ROS_INFO("Inside goal constraints, return success for action");
        ROS_WARN("Robot status is not being published the robot driver node and controller code should be updated");
        active_goal_.setSucceeded();
        has_active_goal_ = false;
      }
    }
  }

  void JointTrajectoryAction::motomanMotionReplyCodesCB(const motoman_msgs::MotomanMotionReplyCodes &input_msg)
  {
    motoman_motion_reply_codes_ = input_msg;
    ROS_INFO_STREAM("motomanMotionReplyCodesCB: received message! result code: " << motoman_motion_reply_codes_.resultCode << ", subcode: " << motoman_motion_reply_codes_.subCode);
  }

} // Namespace GP400
// } // Namespace Motoman
} // Namespace Joint Trajectory Action
} // Namespace Industrial Robot Client
