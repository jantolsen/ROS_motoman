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
#ifndef MOTOMAN_GP400_JOINT_TRAJECTORY_ACTION_H
#define MOTOMAN_GP400_JOINT_TRAJECTORY_ACTION_H

// Include Header-files:
// -------------------------------
// #include "motoman_driver/industrial_robot_client/joint_trajectory_action.h"
#include <map>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <industrial_msgs/RobotStatus.h>
#include <motoman_driver/industrial_robot_client/robot_group.h>
#include <motoman_msgs/DynamicJointTrajectory.h>
#include <motoman_driver/simple_message/motoman_motion_reply_message.h>
#include <motoman_msgs/MotomanMotionReplyCodes.h>

#include "motoman_driver/gp400/gp400_utils.h"

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

/**
 * \brief Motman GP400 Specific action that handles trajectory actions 
 * This class inherits from the "joint_trajectory_action" of the "industrial_robot_client" to accomodate 
 * Parallel-Linkage implementation  
 */
class JointTrajectoryAction
{
  
  bool J23_coupled_;
  // using industrial_robot_client::joint_trajectory_action::JointTrajectoryAction::init;

  // Public Class members
  // -------------------------------
  // Accessible for everyone
  public:

    // Class constructor
    // -------------------------------
    JointTrajectoryAction();

    // Class destructor
    // -------------------------------
    ~JointTrajectoryAction();

    // Class methods
    // -----------------------

      // Begin processing messages and publishing topics.
      bool init();
      void run()
      {
        ros::spin();
      }

      void createTimers();
      ros::Timer timer1;
      ros::Timer timer2;

      void novTriggers(const ros::TimerEvent &e);


      // Parallel-Linkage
      // -------------------------------
      // Check for Parameter-Server for Parallel-Linkage
      bool checkParalellLinkage();

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

  // Protected Class members
  // -------------------------------
  // Accessible within the class which defines them, 
  // and classes which inherits from the parent class
  protected:

  // Private Class members
  // -------------------------------
  // Accessible only for the class which defines them
  private:
    typedef actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction> JointTractoryActionServer;

    /**
     * \brief Have all the points been sent to motion streaming interface?
     */
    bool have_points_streamed_;

    /**
     * \brief Has the robot moved since the points have been streamed?
     */
    bool robot_motion_since_stream_;

    /**
     * \brief Last time novTriggers didn't see points streamed.
     */
    ros::Time last_nonstream_time_;
    int motion_stopped_check_count_;

    /**
     * \brief Internal ROS node handle
     */
    ros::NodeHandle node_;
    JointTractoryActionServer* actionServer_;

    /**
     * \brief Internal action server
     */
    JointTractoryActionServer action_server_;

    /**
     * \brief Publishes desired trajectory (typically to the robot driver)
     */
    ros::Publisher pub_trajectory_command_;

    std::map<int, ros::Publisher> pub_trajectories_;

    std::map<int, RobotGroup> robot_groups_;

    /**
     * \brief Subscribes to trajectory feedback (typically published by the
     * robot driver).
     */
    ros::Subscriber sub_trajectory_state_;

    std::map<int, ros::Subscriber> sub_trajectories_;

    ros::Subscriber feedback_states_;

    /**
     * \brief Subscribes to the robot status (typically published by the
     * robot driver).
     */
    ros::Subscriber sub_robot_status_;

    std::map<int, ros::Subscriber> sub_status_;

    std::map<int, JointTractoryActionServer*> act_servers_;
    /**
     * \brief Watchdog time used to fail the action request if the robot
     * driver is not responding.
     */
    ros::Timer watchdog_timer_;

    std::map<int, ros::Timer>watchdog_timer_map_;

    /**
     * \brief Indicates action has an active goal
     */
    bool has_active_goal_;

    std::map<int, bool> has_active_goal_map_;

    /**
     * \brief Cache of the current active goal
     */
    JointTractoryActionServer::GoalHandle active_goal_;

    std::map<int, JointTractoryActionServer::GoalHandle> active_goal_map_;
    /**
     * \brief Cache of the current active trajectory
     */
    trajectory_msgs::JointTrajectory current_traj_;

    std::map<int, trajectory_msgs::JointTrajectory> current_traj_map_;

    std::vector<std::string> all_joint_names_;

    /**
     * \brief The default goal joint threshold see(goal_threshold). Unit
     * are joint specific (i.e. radians or meters).
     */
    static const double DEFAULT_GOAL_THRESHOLD_;// = 0.01;

    /**
     * \brief The goal joint threshold used for determining if a robot
     * is near it final destination.  A single value is used for all joints
     *
     * NOTE: This value is used in conjunction with the robot inMotion
     * status (see industrial_msgs::RobotStatus) if it exists.
     */
    double goal_threshold_;

    /**
     * \brief The joint names associated with the robot the action is
     * interfacing with.  The joint names must be the same as expected
     * by the robot driver.
     */
    std::vector<std::string> joint_names_;

    /**
     * \brief Cache of the last subscribed feedback message
     */
    control_msgs::FollowJointTrajectoryFeedbackConstPtr last_trajectory_state_;

    std::map<int, control_msgs::FollowJointTrajectoryFeedbackConstPtr> last_trajectory_state_map_;

    /**
     * \brief Indicates trajectory state has been received.  Used by
     * watchdog to determine if the robot driver is responding.
     */
    bool trajectory_state_recvd_;

    std::map<int, bool> trajectory_state_recvd_map_;

    /**
     * \brief Cache of the last subscribed status message
     */
    industrial_msgs::RobotStatusConstPtr last_robot_status_;

    ros::Subscriber sub_motoman_motion_reply_codes_;
    motoman_msgs::MotomanMotionReplyCodes motoman_motion_reply_codes_;

    /**
     * \brief The watchdog period (seconds)
     */
    static const double WATCHD0G_PERIOD_;// = 1.0;

    /**
     * \brief Watch dog callback, used to detect robot driver failures
     *
     * \param e time event information
     *
     */
    void watchdog(const ros::TimerEvent &e);

    void watchdog(const ros::TimerEvent &e, int group_number);

    /**
     * \brief Action server goal callback method
     *
     * \param gh goal handle
     *
     */

    void goalCB(JointTractoryActionServer::GoalHandle gh);

    /**
     * \brief Action server cancel callback method
     *
     * \param gh goal handle
     *
     */

    void cancelCB(JointTractoryActionServer::GoalHandle gh);
    /**
     * \brief Controller state callback (executed when feedback message
     * received)
     *
     * \param msg joint trajectory feedback message
     *
     */

    void goalCB(JointTractoryActionServer::GoalHandle gh, int group_number);

    /**
     * \brief Action server cancel callback method
     *
     * \param gh goal handle
     *
     */

    void cancelCB(JointTractoryActionServer::GoalHandle gh, int group_number);
    /**
     * \brief Controller state callback (executed when feedback message
     * received)
     *
     * \param msg joint trajectory feedback message
     *
     */
    void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);

    void controllerStateCB(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg, int robot_id);

    void controllerStateCBDGM(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg);


    /**
     * \brief Controller status callback (executed when robot status
     *  message received)
     *
     * \param msg robot status message
     *
     */
    void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg);

    /**
     * \brief Aborts the current action goal and sends a stop command
     * (empty message) to the robot driver.
     *
     *
     */
    void abortGoal();

    void novAbortGoalTraj();

    void abortGoal(int robot_id);

    /**
     * \brief Controller status callback (executed when robot status
     *  message received)
     *
     * \param msg trajectory feedback message
     * \param traj trajectory to test against feedback
     *
     * \return true if all joints are within goal contraints
     *
     */
    bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                              const trajectory_msgs::JointTrajectory & traj);

    bool withinGoalConstraints(const control_msgs::FollowJointTrajectoryFeedbackConstPtr &msg,
                              const trajectory_msgs::JointTrajectory & traj, int robot_id);

    void motomanMotionReplyCodesCB(const motoman_msgs::MotomanMotionReplyCodes& input_msg);
};  

} // Namespace GP400
// } // Namespace Motoman
} // Namespace Joint Trajectory Action
} // Namespace Industrial Robot Client
#endif  // MOTOMAN_GP400_JOINT_TRAJECTORY_ACTION_H
