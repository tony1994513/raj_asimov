/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using the RobotModel class is very easy. In
  // general, you will find that most higher-level components will
  // return a shared pointer to the RobotModel. You should always use
  // that when possible. In this example, we will start with such a
  // shared pointer and discuss only the basic API. You can have a
  // look at the actual code API for these classes to get more
  // information about how to use more features provided by these
  // classes.
  //
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.
  //
  // .. _RobotModelLoader: http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "right_arm" of the PR2
  // robot.
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");

  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  //Default position of link6
  //^^^^^^^^^^^^^^^^^^^^^^^^^
  //now lets see where the link6 exists for default joint link values
 
const Eigen::Affine3d &initial_end_effector_state = kinematic_state->getGlobalLinkTransform("link6");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: " << initial_end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << initial_end_effector_state.rotation());
  
  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "r_wrist_roll_link" which is the most distal link in the
  // "right_arm" of the robot.
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d &final_end_effector_state = kinematic_state->getGlobalLinkTransform("link6");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: " << final_end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << final_end_effector_state.rotation());

//Printing the new Joint values corresponding to the random pose we assigned to the robot in the above step

std::vector<double> new_joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, new_joint_values);
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), new_joint_values[i]);
  }
	//Inverse Kinematics
	//^^^^^^^^^^^^^^^^^^
	//We try to figure out the inverse kinematics of the arm

	joint_values[0] = 1.170510;
	joint_values[1]=-0.877894;
	joint_values[2]=2.120417;
	joint_values[3]=-1.028201;
	joint_values[4]=-1.863308;
	joint_values[5]=0.575414;
	
	/*we will use this to store the joint values retured by our IK solver*/
	std::vector<double> IK_joint_values;

	kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
	const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("link6");

	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

	if (found_ik)
	{
  		kinematic_state->copyJointGroupPositions(joint_model_group, IK_joint_values);
  		for(std::size_t i=0; i < joint_names.size(); ++i)
  		{
    			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), IK_joint_values[i]);
  		}		
	}

	else
	{
  		ROS_INFO("Did not find IK solution");
	}




/*check if the joint values returned by the IK is same as the values we set*/
/*There is a good possibility that the values might not be the same as IK solutions are not unique*/


  ros::shutdown();
  return 0;
}
