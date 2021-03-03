#! /usr/bin/env python3

from math import pi
from rospy import ROSInterruptException
import geometry_msgs.msg
from moveit_pkg.moveit_interface import Moveit_Interface


def main():
    try:
        input("--- Pose Control Test Ready ---")
        print("--- Inbitiating moveit controls ---")
        nbv_control = Moveit_Interface()
        
        pose0 = nbv_control.get_pose()
        print(pose0)

        print("\n--- Move via joint state goals out of singularity ---")
        joint_goal1 = [pi/8, pi/10, -pi/4, pi/6, pi/12]
        nbv_control.go_to_joint_state(joint_goal1)
        pose1 = nbv_control.get_pose()
        print(pose1)

        print("\n--- Rotate end manip for second joint goal ---")
        joint_goal2 = joint_goal1.copy()
        joint_goal2[-1] = pi/9
        nbv_control.go_to_joint_state(joint_goal2)
        pose2 = nbv_control.get_pose()
        print(pose2)

        print("\n--- Move via pose goal back to first position ---")
        pose3 = nbv_control.go_to_pose(pose1)
        print(pose3)

    except ROSInterruptException:
        print("Interrupted by ROS")
        return
    except KeyboardInterrupt:
        return



if __name__ == '__main__':
    main()
