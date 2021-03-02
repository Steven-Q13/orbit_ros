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


# Call methods within a try: ... except rospy.ROSInterruptException: ...
#  except KeyboardInterrupt: ...
class Moveit_Interface:

    def __init__(self, publish_planned_path=False, group_name="nbv_arm"):
        # Initialize robot commander and node
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('moveit_interface', anonymous=True)

        # Robot commander gives info about udrf and current joint state
        self.robot = moveit_commander.RobotCommander()

        # Planning Scene Interface is robots internal understanding of environment
        self.scene = moveit_commander.PlanningSceneInterface()

        # Interface to planning group of joints
        self.group_name = group_name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        self.eef_link = self.move_group.get_end_effector_link()
        self.planning_frame = self.move_group.get_planning_frame()
        self.group_names = self.robot.get_group_names()

        # Publisher to display trajectories in Rviz
        if publish_planned_path:
            self.display_trajectory_publisher = rospy.Publisher(
                '/move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory, queue_size = 20)


    def __str__(self):
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


    # Pass in 5 element array angles for:
    # [base rotation, 1st joint, 2nd joint, 3rd joint, end rotation]
    def go_to_joint_state(self, joint_goal):
        # The go command can be called with joint values, poses, or without any
        # parameters if you've already set the pose/joint target for the group

        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        return self.move_group.get_current_joint_values()


    # pose_goal of geometry_msgs.msg.Pose():
    # .oreintation.w, .position.x, .position.y, .position.z
    def go_to_pose(self, pose_goal):
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        self.move_group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)

        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        self.move_group.clear_pose_targets()

        return self.move_group.get_current_pose().pose

    def get_pose(self):
        return self.move_group.get_current_pose().pose


    def get_joint_state(self):
        return self.move_group.get_current_joint_values()


    # Provide list of pose goals for robot to try and meet
    # Only plans paths, return value is the plan
    def plan_cartesian_path(self, pose_goals, resolution=0.01):
        waypoints = []
        waypoint.append(copy.deepcopy(self.get_joint_state()))

        for goal in pose_goals:
            waypoints.append(copy.deepcopy(goal))

        # Resolution determines the how fine the cartesian path is interpreted
        # We disable jump threshold, ignoring check for infeasible jumps in
        # joint space
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            resolution,        # eef_step
            0.0)         # jump_threshold

        # Return plan, fraction details amount of given points succesfully
        # calculated/added to plan
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


    # Pass in plan from plan_cartesian_path
    def execute_plan(self, plan):
        # Robots current joint state must be within some tolerance of the
        # first waypoint or execute fails
        # ie Dont move robot between making and executing plan
        self.move_group.execute(plan, wait=True)


    # Call after adding, removing, attaching or detaching object in planning
    #  scene. Waits until updates or ''timeout'' passes
    # Returns true on success
    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # If node dies before publishing collision object update message,
        #  the message could get lost/box not appear.
        # We wait until see changes reflected in ``get_attached_objects()`` 
        # and ``get_known_object_names()'' lists.
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


    # Pass in pose of a box, a geometry_msgs.msg.PoseStamped(), and set:
    #       .header.frame_id()  <- Frame position is relative to, for
    #                               for stationary do "base_link", for 
    #                               relative to end do "manip"
    #       .pose.position.x/y/z  <- for a point in 3D space
    #       .pose.orientation.w/x/y/z  <- For a quaternion orientation
    # Give name identifier of box
    # Give 3 tuple of size of box in meters (x, y, z)
    # Returns true on success
    def add_box(self, name, size, position=False, orientation=False, timeout=4):
        self.box_name = name
        msg = position if position else orientation
        if not orientation:
            msg = geometry_msgs.msg.PoseStamped()
            msg.header.frame_id = "manip"
            msg.pose.orientation.w = 1.0
            # The orientation x,y,z values are 0
            
        # Create box im planning scene
        self.scene.add_box(name, msg, size=size)

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    # Attach the box to the robot true on success
    def attach_box(self, grasping_group="nbv_manip", timeout=4):
        # By adding link names to the ``touch_links`` array, we are telling the
        # planning scene to ignore collisions between those links and the box. 
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box(
            self.eef_link, self.box_name, touch_links=touch_links)

        # Wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout)


    # Deattach the box to the robot true on success
    def detach_box(self, timeout=4):
        self.scene.remove_attached_object(self.eef_link, name=self.box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout)


    # Remove box from world, object must be dettached before removal
    def remove_box(self, timeout=4):
        self.scene.remove_world_object(self.box_name)

        # We wait for the planning scene to update.
        return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


    def all_close(goal, actual, tolerance):
        """
        Method for testing if the values in two lists are 
         within a tolerance of each other.
        For Pose and PoseStamped inputs, the angle between 
         the two quaternions is compared (the angle between
         the identical orientations q and -q is calculated correctly).
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

