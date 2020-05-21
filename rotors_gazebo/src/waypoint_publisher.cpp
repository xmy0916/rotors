/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <stdio.h>
#include <termios.h>
static struct termios initial_settings, new_settings;
static int peek_character = -1;
void init_keyboard(void);
void close_keyboard(void);
int kbhit(void);
int readch(void); 
void init_keyboard()
{
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag |= ICANON;
    new_settings.c_lflag |= ECHO;
    new_settings.c_lflag |= ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}
 
void close_keyboard()
{
    tcsetattr(0, TCSANOW, &initial_settings);
}
 
int kbhit()
{
    unsigned char ch;
    int nread;
 
    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0,&ch,1);
    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1) 
    {
        peek_character = ch;
        return 1;
    }
    return 0;
}
 
int readch()
{
    char ch;
 
    if(peek_character != -1) 
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);
    return ch;
}


int main(int argc, char** argv) {
  init_keyboard();
  ros::init(argc, argv, "waypoint_publisher");
  ros::NodeHandle nh("//firefly");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);

  ROS_INFO("Started waypoint_publisher.");

  const float DEG_2_RAD = M_PI / 180.0;
  float pos_x = 0,pos_y = 0,pos_z = 1;
  float yaw_deg = 0;
  int isChange = 1;
  while(ros::ok())
  {
    if(kbhit())
    {
      int key = readch();
      switch(key)
      {
	case 56:
		pos_z += 0.5;
		isChange = 1;
		printf("%d\n",key);
		break;
	case 50:
		pos_z -= 0.5;
		isChange = 1;
		printf("%d\n",key);
		break;
	case 97:
		pos_y += 0.5;
		isChange = 1;
		printf("%d\n",key);
		break;
	case 100:
		pos_y -= 0.5;
		isChange = 1;
		printf("%d\n",key);
		break;
	case 119:
		pos_x += 0.5;
		isChange = 1;
		printf("%d\n",key);
		break;
	case 120:
		pos_x -= 0.5;
		isChange = 1;
		printf("%d\n",key);
		break;
	default:
		printf("%d\n",key);
		break;
      }
    }
    if(isChange == 1)
    {
      trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
      trajectory_msg.header.stamp = ros::Time::now();

      Eigen::Vector3d desired_position(pos_x, pos_y,pos_z);

      double desired_yaw = yaw_deg * DEG_2_RAD;

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
        desired_yaw, &trajectory_msg);


      ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
               nh.getNamespace().c_str(),
               desired_position.x(),
               desired_position.y(),
               desired_position.z());

      trajectory_pub.publish(trajectory_msg);   
      isChange = 0;
    }


    ros::spinOnce();
  }

  ros::shutdown();

  return 0;
}
