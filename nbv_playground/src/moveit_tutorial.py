#!/usr/bin/env python3

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class Moveit_Tutorial(object):

    def __init__(self):

        # Initialize robot commander and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_tutorial', anonymous=True)

        # Robot commander gives info about udrf and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Planning Scene Interface is robots internal understanding of environment
        self.scene = moveit_commander.PlanningSceneInterface()

        # Interface to planning group of joints
        self.group_name = "nbv_arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.eef_link = self.move_group.get_end_effector_link()
        self.planning_frame = self.move_group.get_planning_frame()
        self.group_names = self.robot.get_group_names()

        # Publisher to display trajectories in Rviz
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory, queue_size = 20)

        # Display info in moveit interfaces
        print("============ Planning Frame: %s" % self.planning_frame)
        print("\n")
        print("============ End Effector Link: %s" % self.eef_link)
        print("\n")
        print("============ Available Planning Groups:", self.group_names)
        print("\n")
        print("============ Printing Robot State ")
        print("\n")
        print(self.robot.get_current_state())
        print("\n")
        print("============ Print Joint State")
        print(self.move_group.get_current_joint_values())
        print("\n")



    def go_to_joint_state(self):
        # Move the joints a bit to get out of possible singularity
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = pi/10 
        joint_goal[1] = pi/6
        joint_goal[2] = -pi/5
        joint_goal[3] = pi/8
        joint_goal[4] = -pi/6

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        # For testing:
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)


    def go_to_pose_goal(self):
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4

        self.move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        # For testing:
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


    def plan_cartesian_path(self, scale=1):
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through. If executing  interactively in a
        # Python shell, set scale = 1.0.
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z += scale * 0.2  # First move up (z)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x -= scale * 0.3  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient here
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction


    def display_trajectory(self, plan):
        # You can ask RViz to visualize a trajectory for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        #
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # Publish
        self.display_trajectory_publisher.publish(display_trajectory);

    def execute_plan(self, plan):
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

        # *Note: The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def add_box(self, timeout=4):
        # First, we will create a box in the planning scene at the location of the effector:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "manip"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = -0.07 # slightly above the end effector
        self.box_name = "box"
        self.scene.add_box(self.box_name, box_pose, size=(0.1, 0.1, 0.1))

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def attach_box(self, timeout=4):
        # Attach the box to the roboy. Manipulating objects requires the
        # robot be able to touch them without the planning scene reporting the contact as a
        # collision. By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. 
        # Change graping_group to the name of your end effector group name.
        grasping_group = 'nbv_manip'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(self.eef_link, self.box_name, touch_links=touch_links)

        # Wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


    def detach_box(self, timeout=4):
        # We can also detach and remove the object from the planning scene:
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


    def remove_box(self, timeout=4):
        ## We can remove the box from the world.
        self.scene.remove_world_object(self.box_name)

        # Note: The object must be detached before we can remove it from the world

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
    try:
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
        input()
        tutorial = Moveit_Tutorial()

        print("============ Press `Enter` to execute a movement using a joint state goal ...")
        input()
        tutorial.go_to_joint_state()

        print("============ Press `Enter` to execute a movement using a pose goal ...")
        input()
        tutorial.go_to_pose_goal()

        print("============ Press `Enter` to plan and display a Cartesian path ...")
        input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path()

        print("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
        input()
        tutorial.display_trajectory(cartesian_plan)

        print("============ Press `Enter` to execute a saved path ...")
        input()
        tutorial.execute_plan(cartesian_plan)

        print("============ Press `Enter` to add a box to the planning scene ...")
        input()
        tutorial.add_box()

        print("============ Press `Enter` to attach a Box to the Panda robot ...")
        input()
        tutorial.attach_box()

        print("============ Press `Enter` to plan and execute a path with an attached collision object ...")
        input()
        cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        tutorial.execute_plan(cartesian_plan)

        print("============ Press `Enter` to detach the box from the Panda robot ...")
        input()
        tutorial.detach_box()

        print("============ Press `Enter` to remove the box from the planning scene ...")
        input()
        tutorial.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
    main()

# Docs
# .. _moveit_commander:
#    http://docs.ros.org/melodic/api/moveit_commander/html/namespacemoveit__commander.html
#
# .. _MoveGroupCommander:
#    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
#
# .. _RobotCommander:
#    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
#
# .. _PlanningSceneInterface:
#    http://docs.ros.org/melodic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
#
# .. _DisplayTrajectory:
#    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/DisplayTrajectory.html
#
# .. _RobotTrajectory:
#    http://docs.ros.org/melodic/api/moveit_msgs/html/msg/RobotTrajectory.html
#
# .. _rospy:
#    http://docs.ros.org/melodic/api/rospy/html/
