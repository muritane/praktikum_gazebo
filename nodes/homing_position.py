#!/usr/bin/env python

import rospy
import rospkg
import actionlib
from gazebo_msgs.msg import LinkState
from gazebo_msgs.srv import SetLinkState, GetLinkState
from std_srvs.srv import Trigger
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def get_ball_home_link_state():
    state_msg = LinkState()
    state_msg.link_name = 'ball_link'
    state_msg.reference_frame = 'wrist_3_link'
    state_msg.pose.position.x = 0
    state_msg.pose.position.y = 0.2
    state_msg.pose.position.z = 0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0
    return state_msg

class HomingPositionBallUR:
    def __init__(self):
        rospy.wait_for_service('/gazebo/set_link_state')
        rospy.wait_for_service('/gazebo/get_link_state')
        try:
            self.set_link_state_srv = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
            self.get_link_state_srv = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        except rospy.ServiceException, e:
            print 'Service call failed: %s' % e

        self.ball_home_link_state = get_ball_home_link_state()
        self.ball_model_name = rospy.get_param('~ball_model_name', 'ur5e_plate_ball::ball_link')
        self.auto_ball_homing = rospy.get_param('~auto_ball_homing', False)
        self.ball_homing_srv = rospy.Service('/ball_homing_position', Trigger, self.ball_homing_position_cb)

        self.joint_names = rospy.get_param('~joint_names')
        self.move_to_standby_joint_positions = rospy.get_param('~move_to_standby_joint_positions')
        follow_joint_trajectory_param = rospy.get_param('~follow_joint_trajectory_param')
        self.arm_traj_client = actionlib.SimpleActionClient(follow_joint_trajectory_param, FollowJointTrajectoryAction)
        self.move_to_standby_srv = rospy.Service('/ur5e/move_to_standby', Trigger, self._move_to_standby)

    def _move_to_standby(self, req):
        self.move_to_standby()
        self.arm_traj_client.wait_for_result()
        return {'success': True, 'message': 'Moved to standby'}

    def move_to_standby(self, duration=0.5):
        self.arm_traj_client.cancel_all_goals()
        positions_to_send = self.move_to_standby_joint_positions
        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory.joint_names = self.joint_names
        waypoint = JointTrajectoryPoint()
        waypoint.positions = positions_to_send
        waypoint.time_from_start = rospy.Duration.from_sec(duration)
        arm_goal.trajectory.points.append(waypoint)
        self.arm_traj_client.send_goal(arm_goal)

    def ball_homing_position_cb(self, req):
        self.set_link_state(self.ball_home_link_state)
        return {'success': True, 'message': 'Ball moved to home position'}

    def check_ball_link_state(self, min_height_for_homing=0.1):
        if not self.auto_ball_homing:
            return
        response = self.get_link_state_srv(self.ball_model_name , 'world')
        if response.link_state.pose.position.z < min_height_for_homing:
            self.set_link_state(self.ball_home_link_state)

    def set_link_state(self, link_state):
        response = self.set_link_state_srv(link_state)
        rospy.loginfo(response.status_message)


def main(homing_rate=0.4):
    rospy.init_node('HomingPositionBallUR')
    homing = HomingPositionBallUR()
    rospy.loginfo('HomingPositionBallUR initialized')
    rate = rospy.Rate(homing_rate)
    while not rospy.is_shutdown():
        homing.check_ball_link_state()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
