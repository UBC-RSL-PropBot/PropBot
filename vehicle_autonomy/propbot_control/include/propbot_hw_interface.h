/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the RRBot
           For a more detailed simulation example, see sim_hw_interface.h
*/

#pragma once

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
// ros_control
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>
// ostringstream
#include <sstream>

const unsigned int NUM_JOINTS = 2;

/// \brief Hardware interface for a robot
class PropbotHWInterface : public hardware_interface::RobotHW
{
public:
  PropbotHWInterface();

  /*
   *
   */
  void write() {
    double diff_ang_speed_left = cmd[0];
    double diff_ang_speed_right = cmd[1];

	// Publish results																																																									
	std_msgs::Float32 left_wheel_vel_msg;
	std_msgs::Float32 right_wheel_vel_msg;
	left_wheel_vel_msg.data = diff_ang_speed_left;
	right_wheel_vel_msg.data = diff_ang_speed_right;
	left_wheel_vel_pub_.publish(left_wheel_vel_msg);
	right_wheel_vel_pub_.publish(right_wheel_vel_msg);
  }

  /**
   * Reading encoder values and setting position and velocity of enconders 
   * (not implemented because there are currently no encoders on PropBot)
   * 
   */
  void read(const ros::Duration &period) {
  }

  void loadURDF(const ros::NodeHandle &nh, std::string param_name)
  {
        std::string urdf_string;
        urdf_model_ = new urdf::Model();

        // search and wait for robot_description on param server
        while (urdf_string.empty() && ros::ok())
        {
            std::string search_param_name;
            if (nh.searchParam(param_name, search_param_name))
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << search_param_name);
                nh.getParam(search_param_name, urdf_string);
            }
            else
            {
                ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<
                                        nh.getNamespace() << param_name);
                nh.getParam(param_name, urdf_string);
            }

            usleep(100000);
        }

        if (!urdf_model_->initString(urdf_string))
            ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
        else
            ROS_DEBUG_STREAM_NAMED(name_, "Received URDF from param server");
  }

  ros::NodeHandle nh;

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::VelocityJointInterface jnt_vel_interface;
  double cmd[NUM_JOINTS];
  double pos[NUM_JOINTS];
  double vel[NUM_JOINTS];
  double eff[NUM_JOINTS];

  // Short name of this class
  std::string name_;

  double _wheel_diameter;
  double _max_speed;
  double _wheel_angle[NUM_JOINTS];
  urdf::Model *urdf_model_;


  ros::Publisher left_wheel_vel_pub_;
  ros::Publisher right_wheel_vel_pub_;

};  // class

PropbotHWInterface::PropbotHWInterface()
{
    name_ = "PropbotHWInterface";

    // load robot model 
    loadURDF(&nh, "robot_description");

    // Intialize raw data
    std::fill_n(pos, NUM_JOINTS, 0.0);
    std::fill_n(vel, NUM_JOINTS, 0.0);
    std::fill_n(eff, NUM_JOINTS, 0.0);
    std::fill_n(cmd, NUM_JOINTS, 0.0);

    // connect and register the joint state and velocity interfaces
    for (unsigned int i = 0; i < NUM_JOINTS; ++i)
    {
        std::ostringstream os;
        os << "wheel_" << i << "_joint";

        hardware_interface::JointStateHandle state_handle(os.str(), &pos[i], &vel[i], &eff[i]);
        jnt_state_interface.registerHandle(state_handle);

        hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(os.str()), &cmd[i]);
        jnt_vel_interface.registerHandle(vel_handle);
    }
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_vel_interface);

    // Initialize publishers and subscribers
    left_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("propbot_velocity_controller/left_wheel_vel", 1);
    right_wheel_vel_pub_ = nh.advertise<std_msgs::Float32>("propbot_velocity_controller/right_wheel_vel", 1);

}