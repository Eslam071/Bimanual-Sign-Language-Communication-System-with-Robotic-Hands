#!/usr/bin/env python
# coding=utf-8

import sys
from std_msgs.msg import String
import rospy
import moveit_commander
import moveit_msgs.msg
from sensor_msgs.msg import JointState
import actionlib

class MyRobot:
    def __init__(self, group_name):
        moveit_commander.roscpp_initialize(sys.argv)                  #movit_commander initialization
        rospy.init_node('node_set_redefined_pose', anonymous=True)
        self.voice_sub = rospy.Subscriber('voice', String, self.arm_callback)
        self.position = ""

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._planning_group = group_name
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        # Create action client for the Execute Trajectory action server
        self._execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._execute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        
        self.joint_pub = rospy.Publisher("/moving", JointState, queue_size=10)          #new joint state /moving

        rospy.loginfo('\033[95m' + "Planning Frame: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        rospy.loginfo('\033[95m' + " >>> MyRobot initialization is done." + '\033[0m')

    def set_pose(self, arg_pose_name):
        rospy.loginfo('\033[32m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')

        # Publish the joint angles
        self.publish_joint_angles(arg_pose_name)

        self._group.set_named_target(arg_pose_name)
        plan_result = self._group.plan()
        if isinstance(plan_result, tuple):
            if len(plan_result) == 2:
                success, plan = plan_result
            elif len(plan_result) == 3:
                success, plan, _ = plan_result
            elif len(plan_result) == 4:
                success, plan, val3, val4 = plan_result
            else:
                rospy.logerr(f"Unexpected number of values in plan_result: {len(plan_result)}")
                return
            
            if not success:
                rospy.logerr("Planning failed")
                return
        else:
            plan = plan_result
            if not hasattr(plan, 'joint_trajectory') or not plan.joint_trajectory.points:
                rospy.logerr("Planning did not return a valid trajectory")
                return

        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()

        goal.trajectory.joint_trajectory = plan.joint_trajectory
        
        self._execute_trajectory_client.send_goal(goal)
        self._execute_trajectory_client.wait_for_result()
        
        rospy.loginfo('\033[32m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')

    def publish_joint_angles(self, pose_name):
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self._group.get_active_joints()

        # Get the joint values from the named target
        joint_values = self._group.get_named_target_values(pose_name)
        joint_state.position = [joint_values[joint] for joint in joint_state.name]

        rospy.loginfo("Joint State: %s", joint_state)
        self.joint_pub.publish(joint_state)
        rospy.loginfo("Published joint angles for the pose: %s", pose_name)
        #rospy.sleep(3)
    
    
    def __del__(self):
        # When the actions are finished, shut down the moveit commander
        moveit_commander.roscpp_shutdown()
        rospy.loginfo('\033[95m' + "Object of class MyRobot Deleted." + '\033[0m')

    def arm_callback(self, voice):
        self.position = voice.data.strip()
        rospy.loginfo("Received data: %s", self.position)

    def is_valid_position(self, position):
        valid_positions = [u"الف", "close", u"باء", u"تاء", u"جيم", u"حاء", u"خاء", u"دال", u"راء", u"زين", u"سين", u"شين", u"صاد", u"ص", u"ضاد", u"ض", u"طاء", u"عين", u"غين", u"فاء", u"ف", u"ق", u"قاف", u"كاف", u"ك", u"لام", u"ل", u"ميم", u"نون", u"ن", u"هاء", u"واو", u"ياء"]
        return position in valid_positions

    def execution(self):
        while not rospy.is_shutdown():
            if self.position:  # Only process if position is not empty
                if self.is_valid_position(self.position):
                    self.set_pose(self.position)
                    rospy.sleep(3)
                else:
                    rospy.logwarn("Invalid position received: %s Waiting for a valid position...", self.position)
                    rospy.sleep(3)
            else:
                rospy.sleep(1)

if __name__ == '__main__':
    try:
        controller = MyRobot("main")
        controller.execution()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node was interrupted")
