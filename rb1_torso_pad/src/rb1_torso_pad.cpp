/*
 * rb1_torso_pad
 * Copyright (c) 2012, Robotnik Automation, SLL
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
 * \author Robotnik Automation, SLL
 * \brief Allows to use a pad with the robot controller, sending the messages received from the joystick device
 */

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <robotnik_msgs/ptz.h>
// Not yet catkinized 9/2013
// #include <sound_play/sound_play.h>
#include <unistd.h>
#include <robotnik_msgs/set_mode.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <sensor_msgs/JointState.h>

#define DEFAULT_NUM_OF_BUTTONS		16
#define DEFAULT_AXIS_LINEAR_Z       3	
#define DEFAULT_AXIS_PAN            0
#define DEFAULT_AXIS_TILT		    1
#define DEFAULT_SCALE_LINEAR_Z      0.1 
#define DEFAULT_SCALE_PAN		    2.0
#define DEFAULT_SCALE_TILT          2.0

#define ITERATIONS_AFTER_DEADMAN    3.0

class RB1TorsoPad
{
	public:
	RB1TorsoPad();
	void Update();

	private:
	void padCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	int axis_linear_z_;
	double scale_linear_z_;
	
	int axis_pan_;
	double scale_pan_;

	int axis_tilt_;
	double scale_tilt_;
	 
	//! It will publish into command velocity (for the robot) and the ptz_state (for the pantilt)
	ros::Publisher joint_command_pub_;
	//! It will be suscribed to the joystick
	ros::Subscriber pad_sub_;

	double current_vel;

	//! Number of the DEADMAN button
	int dead_man_button_;

	//! Number of the button for increase or decrease the speed max of the joystick	
	int speed_up_button_, speed_down_button_;

	//! Number of buttons of the joystick
	int num_of_buttons_;

	//! Pointer to a vector for controlling the event when pushing the buttons
	bool bRegisteredButtonEvent[DEFAULT_NUM_OF_BUTTONS];

	// DIAGNOSTICS
	//! Diagnostic to control the frequency of the published command velocity topic
	diagnostic_updater::HeaderlessTopicDiagnostic *pub_command_freq; 
	//! Diagnostic to control the reception frequency of the subscribed joy topic 
	diagnostic_updater::HeaderlessTopicDiagnostic *sus_joy_freq; 
	//! General status diagnostic updater
	diagnostic_updater::Updater updater_pad;	
	//! Diagnostics min freq
	double min_freq_command, min_freq_joy; // 
	//! Diagnostics max freq
	double max_freq_command, max_freq_joy; // 	
    //! Client of the sound play service
    //  sound_play::SoundClient sc;
    std::string cmd_topic_;
};


//RB1TorsoPad::RB1TorsoPad():
//  axis_pan_(0),
//  axis_tilt_(1),
//  linear_z_(3)
RB1TorsoPad::RB1TorsoPad()
{
	current_vel = 0.1;
	// 
	nh_.param("num_of_buttons", num_of_buttons_, DEFAULT_NUM_OF_BUTTONS);
	
    // MOTION CONF
	nh_.param("axis_linear_z", axis_linear_z_, DEFAULT_AXIS_LINEAR_Z);
	nh_.param("scale_linear_z", scale_linear_z_, DEFAULT_SCALE_LINEAR_Z);

	nh_.param("axis_pan", axis_pan_, DEFAULT_AXIS_PAN);	
	nh_.param("scale_pan", scale_pan_, DEFAULT_SCALE_PAN);
	nh_.param("axis_tilt", axis_tilt_, DEFAULT_AXIS_TILT);	
	nh_.param("scale_tilt", scale_tilt_, DEFAULT_SCALE_TILT);	
			
	nh_.param("button_dead_man_torso", dead_man_button_, dead_man_button_);
	nh_.param("button_speed_up", speed_up_button_, speed_up_button_);  
	nh_.param("button_speed_down", speed_down_button_, speed_down_button_); 
	
	
	ROS_INFO("RB1TorsoPad num_of_buttons_ = %d", num_of_buttons_);	
	for(int i = 0; i < num_of_buttons_; i++){
		bRegisteredButtonEvent[i] = false;
		ROS_INFO("bREG %d", i);
		}

	/*ROS_INFO("Service I/O = [%s]", cmd_service_io_.c_str());
	ROS_INFO("Topic PTZ = [%s]", cmd_topic_ptz_.c_str());
	ROS_INFO("Service I/O = [%s]", cmd_topic_vel_.c_str());
	ROS_INFO("Axis linear = %d", linear_);
	ROS_INFO("Axis angular = %d", angular_);
	ROS_INFO("Scale angular = %d", a_scale_);
	ROS_INFO("Deadman button = %d", dead_man_button_);
	ROS_INFO("OUTPUT1 button %d", button_output_1_);
	ROS_INFO("OUTPUT2 button %d", button_output_2_);
	ROS_INFO("OUTPUT1 button %d", button_output_1_);
	ROS_INFO("OUTPUT2 button %d", button_output_2_);*/	

  	// Publish through the node handle Twist type messages to the guardian_controller/command topic
  	nh_.param<std::string>("cmd_topic", cmd_topic_, "/joint_commands");
	joint_command_pub_ = nh_.advertise<sensor_msgs::JointState>(cmd_topic_, 1);
	
 	// Listen through the node handle sensor_msgs::Joy messages from joystick 
    // (these are the references that we will sent to cmd_vel)
	pad_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &RB1TorsoPad::padCallback, this);
	
	// Diagnostics
	updater_pad.setHardwareID("None");
	// Topics freq control 
	min_freq_command = min_freq_joy = 5.0;
	max_freq_command = max_freq_joy = 50.0;
	sus_joy_freq = new diagnostic_updater::HeaderlessTopicDiagnostic("/joy", updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_joy, &max_freq_joy, 0.1, 10));

	pub_command_freq = new diagnostic_updater::HeaderlessTopicDiagnostic(cmd_topic_.c_str(), updater_pad,
	                    diagnostic_updater::FrequencyStatusParam(&min_freq_command, &max_freq_command, 0.1, 10));
}

/*
 *	\brief Updates the diagnostic component. Diagnostics
 *
 */
void RB1TorsoPad::Update(){
	updater_pad.update();
}

void RB1TorsoPad::padCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
   static int send_iterations_after_dead_man;

	sensor_msgs::JointState cmd;
		
	// Joints: j1_torso (linear in m/s) j1_head (pan in rad/s), j2_head (tilt in rad/s)
	cmd.name.resize(3);
    //cmd.name[0] = "j1_torso"; cmd.name[1] = "j1_head"; cmd.name[2] = "j2_head";
    cmd.name[0] = "torso_slider_joint"; cmd.name[1] = "head_pan_joint"; cmd.name[2] = "head_tilt_joint";
    cmd.velocity.resize(3);
    cmd.velocity[0] = 0.0; cmd.velocity[1] = 0.0; cmd.velocity[2] = 0.0;

	
    // Actions dependant on dead-man button
    if (joy->buttons[dead_man_button_] == 1) {
		// Set the current velocity level
		if ( joy->buttons[speed_down_button_] == 1 ){
			if(!bRegisteredButtonEvent[speed_down_button_]) 
				if(current_vel > 0.1){
		  			current_vel = current_vel - 0.1;
					bRegisteredButtonEvent[speed_down_button_] = true;
					ROS_INFO("Velocity: %f%%", current_vel*100.0);	
					char buf[50]="\0";
 					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}	 	
		}else{
			bRegisteredButtonEvent[speed_down_button_] = false;
		 }
		 
		if (joy->buttons[speed_up_button_] == 1){
			if(!bRegisteredButtonEvent[speed_up_button_])
				if(current_vel < 0.9){
					current_vel = current_vel + 0.1;
					bRegisteredButtonEvent[speed_up_button_] = true;
			 	 	ROS_INFO("Velocity: %f%%", current_vel*100.0);
  					char buf[50]="\0";
					int percent = (int) (current_vel*100.0);
					sprintf(buf," %d percent", percent);
                    // sc.say(buf);
				}
		  
		}else{
			bRegisteredButtonEvent[speed_up_button_] = false;
		}
		 
		cmd.velocity[0] = current_vel*scale_linear_z_*joy->axes[axis_linear_z_];
		cmd.velocity[1] = current_vel*scale_pan_ * joy->axes[axis_pan_];
		cmd.velocity[2] = current_vel*scale_tilt_ * joy->axes[axis_tilt_];
		

	}
   	else {
        cmd.velocity[0] = 0.0; cmd.velocity[1] = 0.0; cmd.velocity[2] = 0.0;
		}

	sus_joy_freq->tick();	// Ticks the reception of joy events

    // Publish only with deadman button pushed
    if (joy->buttons[dead_man_button_] == 1) {                
                send_iterations_after_dead_man = ITERATIONS_AFTER_DEADMAN;
		cmd.header.stamp = ros::Time::now();
		joint_command_pub_.publish(cmd);
		pub_command_freq->tick();
		}
    else { // send some 0 if deadman is released
          if (send_iterations_after_dead_man >0) {
                send_iterations_after_dead_man--;
                cmd.header.stamp = ros::Time::now();
                joint_command_pub_.publish(cmd);
                pub_command_freq->tick();
                }
             }

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "rb1_torso_pad");
	RB1TorsoPad rb1_torso_pad;

	ros::Rate r(50.0);

	while( ros::ok() ){
		// UPDATING DIAGNOSTICS
		rb1_torso_pad.Update();
		ros::spinOnce();
		r.sleep();
		}
}

