#!/usr/bin/env python3
import rospy
from math import pi
from sensor_msgs.msg import JointState

'''
Currently moves joints through all ranges and publishes cycle_joints"
'''
def cycle_joints_nbv():
    # Init publisher for joint messages
    joint_pub = rospy.Publisher('CycleJoints', JointState, queue_size=10)

    # Start node
    rospy.init_node('cycle_joints_nbv')
    loop_rate = rospy.Rate(10) #10 hz

    degree  = pi / 180

    # Robot state
    # joint1 <-> base-seg1
    joint1 = 0
    modify_joint1 = degree * 1.75
    limit_joint1 = [-3.14, 3.14]
    # joint2 <-> seg1
    joint2 = 0
    modify_joint2 = degree * 1.75
    limit_joint2 = [-0.78, 3.92]
    # joint3 <-> seg1-seg2
    joint3 = 0
    modify_joint3 = degree * 1.75
    limit_joint3 = [-1.57, 1.57]
    # joint4 <-> seg2
    joint4 = 0
    modify_joint4 = degree * 1.75
    limit_joint4 = [-3.14, 3.14]
    # joint5 <-> end
    joint5 = 0
    modify_joint5 = degree * 1.75
    limit_joint5 = [-1.75, 1.75]
    # joint6 <-> manip
    joint6 = 0
    modify_joint6 = degree * 1.75
    limit_join6 = [-1*float('inf'), float('inf')]


    joint_state = JointState()


    while not rospy.is_shutdown():
        # Update joint2-"seg1"  state
        now = rospy.Time.now()
        joint_state.header.stamp = now
        joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        joint_state.position = [joint1, joint2, joint3, joint4, joint5]
        joint_state.velocity = [1, 1, 1, 1, 1]
        joint_state.effort = [1, 1, 1, 1, 1]



        # Send joint state
        joint_pub.publish(joint_state)

        # Make new robot state

        if joint1 < limit_joint1[0] or joint1 > limit_joint1[1]:
            modify_joint1 *= -1
        joint1 += modify_joint1
        if joint2 < limit_joint2[0] or joint2 > limit_joint2[1]:
            modify_joint2 *= -1
        joint2 += modify_joint2
        if joint3 < limit_joint3[0] or joint3 > limit_joint3[1]:
            modify_joint3 *= -1
        joint3 += modify_joint3
        if joint4 < limit_joint4[0] or joint4 > limit_joint4[1]:
            modify_joint4 *= -1
        joint4 += modify_joint4
        if joint5 < limit_joint5[0] or joint5 > limit_joint5[1]:
            modify_joint5 *= -1
        joint5 += modify_joint5

        # Necessary sleep time per iteration
        loop_rate.sleep()
            

if __name__ == '__main__':
    try:
        cycle_joints_nbv()
    except rospy.ROSInterruptException:
        pass
