# -*- coding: UTF-8 -*-
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

import sys, select, os
import mav_msgs.msg as mav
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

FETCH_MAX_LIN_VEL = 1
FETCH_MAX_ANG_VEL = 1

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.01

msg = """
Control Your uav!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (FETCH : ~ 0.22)
a/d : increase/decrease angular velocity (FETCH : ~ 2.84)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -FETCH_MAX_LIN_VEL, FETCH_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -FETCH_MAX_ANG_VEL, FETCH_MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('rotors_teleop')
    pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10)

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    pos_x = 0
    pos_y = 0
    pos_z = 1

    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            # 创建MultiDOFJointTrajectory消息
            trajectory_msg = MultiDOFJointTrajectory()
            trajectory_msg.joint_names.append("base_link")
            trajectory_points = MultiDOFJointTrajectoryPoint()

            #创建transform消息
            tran = Transform()
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0)) # 限速
            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0)) # 限速
            pos_x += control_linear_vel
            pos_y += control_angular_vel
            tran.translation.z = 1
            tran.translation.x = pos_x
            tran.translation.y = pos_y

            # 创建velocities消息
            twist1 = Twist()
            # 创建accelerations消息
            twist2 = Twist()

            # 合成trajectory_points消息
            trajectory_points.transforms.append(tran)
            trajectory_points.accelerations.append(twist1)
            trajectory_points.velocities.append(twist2)

            # 合成trajectory_msg消息
            trajectory_msg.points.append(trajectory_points)

            pub.publish(trajectory_msg)

    except:
        print(e)

    finally:
        # control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        # twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        #
        # control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        # twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        trajectory_msg = MultiDOFJointTrajectory()
        trajectory_msg.joint_names.append("base_link")

        trajectory_points = MultiDOFJointTrajectoryPoint()

        tran = Transform()

        twist = Twist()

        twist2 = Twist()
        trajectory_points.transforms.append(tran)
        trajectory_points.accelerations.append(twist)
        trajectory_points.velocities.append(twist2)
        # control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
        # twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0
        #
        # control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
        # twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

        trajectory_msg.points.append(trajectory_points)
        pub.publish(trajectory_msg)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
