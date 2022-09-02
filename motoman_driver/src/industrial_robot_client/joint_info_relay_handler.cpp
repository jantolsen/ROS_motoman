/*
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Southwest Research Institute
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   * Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*   * Neither the name of the Southwest Research Institute, nor the names
* of its contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
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

#include "motoman_driver/industrial_robot_client/joint_info_relay_handler.h"
#include "simple_message/log_wrapper.h"
#include <map>
#include <string>
#include <vector>

using industrial::joint_data::JointData;
using industrial::shared_types::shared_real;
namespace StandardMsgTypes = industrial::simple_message::StandardMsgTypes;

namespace industrial_robot_client
{
namespace joint_info_relay_handler
{

bool JointInfoRelayHandler::init(SmplMsgConnection* connection,
                                     std::map<int, RobotGroup> &robot_groups)
{
  this->version_0_ = false;
  bool rtn = JointInfoHandler::init(connection, static_cast<int>(StandardMsgTypes::JOINT_INFO), robot_groups);
  // try to read robot_id parameter, if none specified
  if ((robot_id_ < 0))
    node_.param("robot_id", robot_id_, 0);
  //This executes
  return rtn;
}

bool JointInfoRelayHandler::init(SmplMsgConnection* connection,
                                     std::vector<std::string> &joint_names)
{
  this->version_0_ = true;
  bool rtn = JointInfoHandler::init(connection, static_cast<int>(StandardMsgTypes::JOINT_INFO), joint_names);

  // try to read robot_id parameter, if none specified
  if ((robot_id_ < 0))
    node_.param("robot_id", robot_id_, 0);
  return rtn;
}


bool JointInfoRelayHandler::create_messages(SimpleMessage& msg_in,
                                            industrial_msgs::JointInfo* joint_info)
{
  // inspect robot_id field first, to avoid "Failed to Convert" message
  JointInfoMessage tmp_msg;

  tmp_msg.init(msg_in);

  if (this->version_0_)
    return JointInfoHandler::create_messages(msg_in, joint_info);
  else
    return JointInfoRelayHandler::create_messages(msg_in, joint_info, tmp_msg.getRobotID());

}

bool JointInfoRelayHandler::create_messages(SimpleMessage& msg_in,
                                                industrial_msgs::JointInfo* joint_info, int robot_id)
{
  industrial_msgs::JointInfo all_joint_info;
  if (!convert_message(msg_in, &all_joint_info, robot_id))
  {
    LOG_ERROR("Failed to convert SimpleMessage");
    return false;
  }
  // apply transform, if required
  industrial_msgs::JointInfo xform_joint_info;
  if (!transform(all_joint_info, &xform_joint_info))
  {
    LOG_ERROR("Failed to transform joint state");
    return false;
  }

  // select specific joints for publishing
  industrial_msgs::JointInfo pub_joint_info;
  std::vector<std::string> pub_joint_names;
  if (!select(xform_joint_info, robot_groups_[robot_id].get_joint_names(), &pub_joint_info, &pub_joint_names))
  {
    LOG_ERROR("Failed to select joints for publishing");
    return false;
  }
  // assign values to messages

  *joint_info = industrial_msgs::JointInfo();  // always start with a "clean" message
  joint_info->header.stamp = ros::Time::now();
  joint_info->name = pub_joint_names;
  joint_info->torque = pub_joint_info.torque;
  joint_info->temp = pub_joint_info.temp;

  this->pub_info_[robot_id].publish(*joint_info);

  return true;
}

bool JointInfoRelayHandler::convert_message(SimpleMessage& msg_in, industrial_msgs::JointInfo* joint_info, int robot_id)
{
  JointInfoMessage joint_info_msg;
  if (!joint_info_msg.init(msg_in))
  {
    LOG_ERROR("Failed to initialize joint feedback message");
    return false;
  }

  return convert_message(joint_info_msg, joint_info, robot_id);
}

bool JointInfoRelayHandler::convert_message(SimpleMessage& msg_in, industrial_msgs::JointInfo* joint_info)
{
  JointInfoMessage joint_feedback_msg;
  if (!joint_feedback_msg.init(msg_in))
  {
    LOG_ERROR("Failed to initialize joint feedback message");
    return false;
  }

  return convert_message(joint_feedback_msg, joint_info);
}

bool JointInfoRelayHandler::JointDataToVector(const JointData &joints,
    std::vector<double> &vec,
    int len)
{
  if ((len < 0) || (len > joints.getMaxNumJoints()))
  {
    LOG_ERROR("Failed to copy JointData.  Len (%d) out of range (0 to %d)",
              len, joints.getMaxNumJoints());
    return false;
  }

  vec.resize(len);
  for (int i = 0; i < len; ++i)
    vec[i] = joints.getJoint(i);

  return true;
}

bool JointInfoRelayHandler::convert_message(JointInfoMessage& msg_in, industrial_msgs::JointInfo* joint_info, int robot_id)
{
  JointData values;
  int num_jnts = robot_groups_[robot_id].get_joint_names().size();

  if (msg_in.getTorques(values))
  {
    if (!JointDataToVector(values, joint_info->torque, num_jnts))
    {
      LOG_ERROR("Failed to parse torque data from JointInfoMessage");
      return false;
    }
  }
  else
    joint_info->torque.clear();

  if (msg_in.getTemps(values))
  {
    if (!JointDataToVector(values, joint_info->temp, num_jnts))
    {
      LOG_ERROR("Failed to parse temp data from JointInfoMessage");
      return false;
    }
  }
  else
    joint_info->temp.clear();

  return true;
}

bool JointInfoRelayHandler::convert_message(JointInfoMessage& msg_in, industrial_msgs::JointInfo* joint_info)
{
  JointData values;
  int num_jnts = all_joint_names_.size();

  // copy position data
  if (msg_in.getTorques(values))
  {
    if (!JointDataToVector(values, joint_info->torque, num_jnts))
    {
      LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
      return false;
    }
  }
  else
    joint_info->torque.clear();
  // copy position data
  if (msg_in.getTemps(values))
  {
    if (!JointDataToVector(values, joint_info->temp, num_jnts))
    {
      LOG_ERROR("Failed to parse position data from JointFeedbackMessage");
      return false;
    }
  }
  else
    joint_info->temp.clear();

  return true;
}

bool JointInfoRelayHandler::select(const industrial_msgs::JointInfo& all_joint_state, const std::vector<std::string>& all_joint_names,
                                       industrial_msgs::JointInfo* pub_joint_info, std::vector<std::string>* pub_joint_names)
{

  //ROS_ASSERT(all_joint_state.torque.size() == all_joint_names.size());

  *pub_joint_info = industrial_msgs::JointInfo();  // start with a "clean" message
  pub_joint_names->clear();
  std::string a;
  // skip over "blank" joint names
  for (int i = 0; i < all_joint_names.size(); ++i)
  {
    if (all_joint_names[i].empty())
      continue;
    pub_joint_names->push_back(all_joint_names[i]);
    a=all_joint_names[i];
    if (!all_joint_state.torque.empty())
      pub_joint_info->torque.push_back(all_joint_state.torque[i]);
    if (!all_joint_state.temp.empty())
      pub_joint_info->temp.push_back(all_joint_state.temp[i]);
  }

  return true;
}

}  // namespace joint_feedback_relay_handler
}  // namespace industrial_robot_client




