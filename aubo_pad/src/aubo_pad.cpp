/*
 * our_pad
 * Copyright (c) 2016, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *
 * \author Robotnik Automation SLL
 * \brief Allows to use a PS3 or other pad with our node
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

#define DEFAULT_NUM_OF_BUTTONS		20
#define DEFAULT_SCALE_ANGULAR		0.5
#define DEFAULT_AXIS_ANGULAR_X		1
#define MAX_JOINTS					6

class OurPad
{
	public:
	OurPad();

	private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

	ros::NodeHandle nh_;
	ros::NodeHandle pnh_; // Private node handle

	//! It will publish into command joint_states /joint_command (for the robot)
	ros::Publisher arm_ref_joint_pub_;
	
	//! Joint states published by the arm controller (also all devices that control joints)
	ros::Subscriber joint_state_sub_;
	
	//! Contains the arm joint_states_ (once they have been published the 1st time)
	sensor_msgs::JointState joint_state_;
	
	//! Flag that is true when the arm joint_states_ have been read the 1st time
	bool read_state_;
	
	//! It will be suscribed to the joystick
	ros::Subscriber joy_sub_;

	//! Number of the button actions
	int button_up_, button_down_;
    
	//! Number of buttons of the joystick
	int num_of_buttons_;
	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];
	//! buttons to the arm
	int dead_man_arm_;
	
	//! angular axis that gives vel ref to the joint
	int angular_x_;
	
	double a_scale_;
	std::string topic_joy;
				
	//! selected joint
	int iSelectedJoint_;	
};


OurPad::OurPad() : pnh_("~")
 {
	// ARM CONF
	pnh_.param("num_of_buttons_arm", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);	
	pnh_.param("axis_angular_arm", angular_x_, DEFAULT_AXIS_ANGULAR_X);
	pnh_.param("scale_angular_arm", a_scale_, DEFAULT_SCALE_ANGULAR);
	pnh_.param("button_speed_up_arm", button_up_, button_up_);  				// Triangle PS3
	pnh_.param("button_speed_down_arm", button_down_, button_down_); 	    	// Cross PS3	
	pnh_.param("button_dead_man_arm", dead_man_arm_, dead_man_arm_);			// R2 PS3
	
	//bRegisteredButtonEvent = new bool(num_of_buttons_);
	for(int i = 0; i < DEFAULT_NUM_OF_BUTTONS; i++){
		bRegisteredButtonEvent[i] = false;
	}


 	 // Listen through the node handle sensor_msgs::Joy messages from joystick
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &OurPad::joyCallback, this);

	// Subscribe to joint states topic
	joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>("/joint_states", 1, &OurPad::jointStateCallback, this);

	// Publishes into the arm controller
	arm_ref_joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_command", 1);
	  		        
    // Joint selected by default
    iSelectedJoint_ = 6;   // 4,5,6 are less dangerous    	
    
    // Still no joint state read 
    read_state_ = false;

}

// Topic command
void OurPad::jointStateCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  sensor_msgs::JointState all_kind_of_msg;
  
  all_kind_of_msg = *msg;
  
  if (all_kind_of_msg.name[0]=="our_joint1") {
	joint_state_ = *msg;
	read_state_ = true;
	// ROS_INFO("Received a our joint_state");
	}
}

void OurPad::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	// ARM MOVEMENTS
	// Actions dependant on arm dead-man button
	if (joy->buttons[dead_man_arm_] == 1){
		
		if (joy->buttons[button_up_] == 1){
			if(!bRegisteredButtonEvent[button_up_]){
				bRegisteredButtonEvent[button_up_] = true;
				iSelectedJoint_ = iSelectedJoint_ + 1; 
				if (iSelectedJoint_ > MAX_JOINTS) iSelectedJoint_ = MAX_JOINTS;			
				ROS_INFO("Selected Joint = %d", iSelectedJoint_);
				}
		}else if (joy->buttons[button_down_] == 1){
			if(!bRegisteredButtonEvent[button_down_]){
				bRegisteredButtonEvent[button_down_] = true;
				iSelectedJoint_ = iSelectedJoint_ - 1; 
				if (iSelectedJoint_ < 1) iSelectedJoint_ = 1;
				ROS_INFO("Selected Joint = %d", iSelectedJoint_);
				}
		}else{		
								
				if (read_state_) {				
					sensor_msgs::JointState joint_state_command;	
					//Header header
					//string[] name
					//float64[] position
					//float64[] velocity
					//float64[] effort
					// name: ['our_joint1', 'our_joint2', 'our_joint3', 'our_joint4', 'our_joint5', 'our_joint6']
					// position: [0.0012392282951623201, -9.508145922154654e-06, 0.0007067721453495324, 0.0011770643759518862, -0.0004214649961795658, 2.657887489476707e-05]
					// velocity: [6.90615797218257e-310, 1.7e-322, 1.7e-322, 6.9532666309199e-310, 0.0, 6.95326663091e-310]
					// effort: [1.03525053e-316, 6.90615935282056e-310, 0.0, 6.9532666309179e-310, 6.95314360825596e-310, 3.854474065e-315]				
					
					joint_state_command.name.resize(MAX_JOINTS);
					joint_state_command.position.resize(MAX_JOINTS);
					joint_state_command.velocity.resize(MAX_JOINTS);
					joint_state_command.effort.resize(MAX_JOINTS);
		
					joint_state_command.name[0] = "our_joint1";
					joint_state_command.name[1] = "our_joint2";
					joint_state_command.name[2] = "our_joint3";
					joint_state_command.name[3] = "our_joint4";
					joint_state_command.name[4] = "our_joint5";
					joint_state_command.name[5] = "our_joint6";
		
					joint_state_command.position = joint_state_.position;  // update only if the joint_state measured is the arm state !
						
					joint_state_command.velocity[0] = 0.0;
					joint_state_command.velocity[1] = 0.0;
					joint_state_command.velocity[2] = 0.0;
					joint_state_command.velocity[3] = 0.0;
					joint_state_command.velocity[4] = 0.0;
					joint_state_command.velocity[5] = 0.0;
					 
					joint_state_command.effort[0] = 0.0;
					joint_state_command.effort[1] = 0.0;
					joint_state_command.effort[2] = 0.0;
					joint_state_command.effort[3] = 0.0;
					joint_state_command.effort[4] = 0.0;
					joint_state_command.effort[5] = 0.0;
																																	
					// Values in rad / second					
					double ref = a_scale_ * joy->axes[angular_x_]; 
					double hz = 10.0;  // Joystick frequency
					double scaled_ref = ref / hz; // convert to increments per period
					// saturation
					if (scaled_ref > 0.025) scaled_ref = 0.025; // *f = [rad/s]
					if (scaled_ref < -0.025) scaled_ref = -0.025; 					
	
					if (iSelectedJoint_==1) joint_state_command.position[0] = joint_state_command.position[0] + scaled_ref;
					if (iSelectedJoint_==2) joint_state_command.position[1] = joint_state_command.position[1] + scaled_ref;
					if (iSelectedJoint_==3) joint_state_command.position[2] = joint_state_command.position[2] + scaled_ref;
					if (iSelectedJoint_==4) joint_state_command.position[3] = joint_state_command.position[3] + scaled_ref;
					if (iSelectedJoint_==5) joint_state_command.position[4] = joint_state_command.position[4] + scaled_ref;
					if (iSelectedJoint_==6) joint_state_command.position[5] = joint_state_command.position[5] + scaled_ref;
				
					// Publish jbj ref message				
					arm_ref_joint_pub_.publish(joint_state_command);
					
					/*	            		
					ROS_INFO("js [ %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f ]",  
						joint_state_command.position[0],
						joint_state_command.position[1],
						joint_state_command.position[2],
						joint_state_command.position[3],
						joint_state_command.position[4],
						joint_state_command.position[5]);
					*/
					
					} // 				
				bRegisteredButtonEvent[button_up_] = false;
				bRegisteredButtonEvent[button_down_] = false;			
                        }
		}
}	

int main(int argc, char** argv)
{
	ros::init(argc, argv, "our_pad");
	OurPad our_pad;
	ros::spin();
}

