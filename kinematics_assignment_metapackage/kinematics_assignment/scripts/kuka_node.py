#! /usr/bin/env python
import xlsxwriter
import numpy as np

"""
    This node publishes the joint states to make a given trajectory with the KUKA's end-effector

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
from square_trajectory import SquareTrajectory
import IK_functions
from sensor_msgs.msg import JointState
from std_srvs.srv import EmptyResponse, EmptyRequest, Empty

filename = '/home/p/a/patidar/catkin_ws/src/ResearchMethodologyKTH/Data/kuka_line.xlsx'


def output(filename, sheet, points_list, orientation_list, joint_list, curr_pos_list, curr_or_list, error_list):
    workbook = xlsxwriter.Workbook(filename)
    sh = workbook.add_worksheet(sheet)

    position = ['Tx', 'Ty', 'Tz']

    orientation = ['Tr11', 'Tr12', 'Tr13',
                   'Tr21', 'Tr22', 'Tr23', 'Tr31', 'Tr32', 'Tr33']

    joint_angles = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']

    actual_position = ['Ax', 'Ay', 'Az']

    actual_orientation = ['Ar11', 'Ar12', 'Ar13',
                          'Ar21', 'Ar22', 'Ar23', 'Ar31', 'Ar32', 'Ar33']
    error = ['ex', 'ey', 'ez', 'etheta1', 'etheta2', 'etheta3']

    for n, v in enumerate(position):
        sh.write(0, n, v)
    for n, v in enumerate(orientation):
        sh.write(0, n+3, v)
    for n, v in enumerate(joint_angles):
        sh.write(0, n+12, v)
    for n, v in enumerate(actual_position):
        sh.write(0, n+19, v)
    for n, v in enumerate(actual_orientation):
        sh.write(0, n+22, v)
    for n, v in enumerate(error):
        sh.write(0, n+25, v)

    col = 0
    for ind, val in enumerate(points_list, 1):
        sh.write(ind, col, val[0])
        col += 1
        sh.write(ind, col, val[1])
        col += 1
        sh.write(ind, col, val[2])
        col += 1

    for ind, val in enumerate(orientation_list, 1):
        sh.write(ind, col, val[0][0])
        col += 1
        sh.write(ind, col, val[0][1])
        col += 1
        sh.write(ind, col, val[0][2])
        col += 1
        sh.write(ind, col, val[1][0])
        col += 1
        sh.write(ind, col, val[1][1])
        col += 1
        sh.write(ind, col, val[1][2])
        col += 1
        sh.write(ind, col, val[2][0])
        col += 1
        sh.write(ind, col, val[2][1])
        col += 1
        sh.write(ind, col, val[2][2])
        col += 1

    for ind, val in enumerate(joint_list, 1):
        sh.write(ind, col, val[0])
        col += 1
        sh.write(ind, col, val[1])
        col += 1
        sh.write(ind, col, val[2])
        col += 1
        sh.write(ind, col, val[3])
        col += 1
        sh.write(ind, col, val[4])
        col += 1
        sh.write(ind, col, val[5])
        col += 1
        sh.write(ind, col, val[6])
        col += 1

    for ind, val in enumerate(curr_pos_list, 1):
        sh.write(ind, col, val[0])
        col += 1
        sh.write(ind, col, val[1])
        col += 1
        sh.write(ind, col, val[2])
        col += 1

    for ind, val in enumerate(curr_or_list, 1):
        sh.write(ind, col, val[0][0])
        col += 1
        sh.write(ind, col, val[0][1])
        col += 1
        sh.write(ind, col, val[0][2])
        col += 1
        sh.write(ind, col, val[1][0])
        col += 1
        sh.write(ind, col, val[1][1])
        col += 1
        sh.write(ind, col, val[1][2])
        col += 1
        sh.write(ind, col, val[2][0])
        col += 1
        sh.write(ind, col, val[2][1])
        col += 1
        sh.write(ind, col, val[2][2])
        col += 1

    for ind, val in enumerate(error_list, 1):
        sh.write(ind, col, val[0])
        col += 1
        sh.write(ind, col, val[1])
        col += 1
        sh.write(ind, col, val[2])
        col += 1
        sh.write(ind, col, val[3])
        col += 1
        sh.write(ind, col, val[4])
        col += 1
        sh.write(ind, col, val[5])
        col += 1

    workbook.close()
    return 0


def main():
    rospy.init_node('kuka_node')
    rate = rospy.Rate(10)

    points_list = []
    q_list = []
    r_list = []
    curr_pos_list = []
    curr_or_list = []
    error_list = []

    # the vertices of the square trajectory (in this case it will be a line)
    vertices = [[-0.217, 0, 0.84], [-0.2, 0, 0.65],
                [-0.2, 0, 0.65], [-0.217, 0, 0.84]]
    # vertices = [[-0.217, -0.217, 0.84], [-0.217, -0.217, 0.42],
    #             [-0.217, 0.217, 0.42], [-0.217, 0.217, 0.84]]

    # the name of the robot's base frame
    base_frame = 'lwr_base_link'

    trajectory_publisher = SquareTrajectory(vertices, base_frame)
    desired_orientation = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]

    current_q = [0, 1.12, 0, 1.71, 0, 1.84, 0]

    # the joint names of kuka:
    joint_names = ['lwr_a1_joint', 'lwr_a1_joint_stiffness', 'lwr_a2_joint', 'lwr_a2_joint_stiffness', 'lwr_e1_joint',
                   'lwr_e1_joint_stiffness', 'lwr_a3_joint', 'lwr_a3_joint_stiffness', 'lwr_a4_joint', 'lwr_a4_joint_stiffness',
                   'lwr_a5_joint', 'lwr_a5_joint_stiffness', 'lwr_a6_joint', 'lwr_a6_joint_stiffness']
    # define the ros message for publishing the joint positions
    joint_msg = JointState()
    joint_msg.name = joint_names

    # define the ros topic where to publish the joints values
    topic_name = rospy.get_param('~topic_name', 'controller/joint_states')
    publisher = rospy.Publisher(topic_name, JointState, queue_size=10)

    # A service is used to restart the trajectory execution and reload a new solution to test
    def restart(req):
        reload(IK_functions)
        current_q[0:7] = [0, 1.12, 0, 1.71, 0, 1.84, 0]
        q_msg = [current_q[0], 0, current_q[1], 0, current_q[2], 0,
                 current_q[3], 0, current_q[4], 0, current_q[5], 0, current_q[6], 0]
        joint_msg.position = q_msg
        publisher.publish(joint_msg)
        trajectory_publisher.restart()
        trajectory_publisher.publish_path()
        rate.sleep()
        return EmptyResponse()

    s = rospy.Service('restart', Empty, restart)

    restart(EmptyRequest())

    rate.sleep()

    while not rospy.is_shutdown():
        # get the current point in the trajectory
        point = trajectory_publisher.get_point()
        if point is not None:
            # get the IK solution for this point
            q, cur_pos, cur_or, err = IK_functions.kuka_IK(
                point, desired_orientation, current_q)

            current_q = q
            points_list.append(point)
            r_list.append(desired_orientation)
            q_list.append(q)
            curr_pos_list.append(cur_pos)
            curr_or_list.append(cur_or)
            error_list.append(err)

            q_msg = [q[0], 0, q[1], 0, q[2], 0,
                     q[3], 0, q[4], 0, q[5], 0, q[6], 0]
            # publish this solution
            joint_msg.position = q_msg
            publisher.publish(joint_msg)

            # publish the path to be visualized in rviz
            trajectory_publisher.publish_path()
            rate.sleep()
        else:
            # write to excel file
            output(filename, 'square_pinv', points_list, r_list,
                   q_list, curr_pos_list, curr_or_list, error_list)
            print('Done')
            rospy.signal_shutdown('Done')


if __name__ == '__main__':
    main()
