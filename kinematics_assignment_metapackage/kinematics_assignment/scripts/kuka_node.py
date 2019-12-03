#! /usr/bin/env python
import xlsxwriter
import numpy as np
import time
from IK_functions import method

"""
    This node publishes the joint states to make a given trajectory with the KUKA's end-effector

    @author: Silvia Cruciani (cruciani@kth.se)
"""

import rospy
from square_trajectory import SquareTrajectory
from circle_trajectory import CircularTrajectory
import IK_functions
from sensor_msgs.msg import JointState
from std_srvs.srv import EmptyResponse, EmptyRequest, Empty


def output(points_list, orientation_list, joint_list, curr_pos_list, curr_or_list, error_list, threshold):
    filename = '/home/p/a/patidar/catkin_ws/src/ResearchMethodologyKTH/Data/' + \
        path_type + '_' + method + '_' + str(threshold) + '.xlsx'
    workbook = xlsxwriter.Workbook(filename)
    sh_tp = workbook.add_worksheet("target_pose")
    cell_format_pos = workbook.add_format(
        {'bold': True, 'font_color': 'green'})
    cell_format_or = workbook.add_format(
        {'bold': True, 'font_color': 'red'})
    cell_format_q = workbook.add_format(
        {'bold': True, 'font_color': 'black'})

    position = ['Tx', 'Ty', 'Tz']

    orientation = ['r11', 'r12', 'r13',
                   'r21', 'r22', 'r23', 'r31', 'r32', 'r33']

    joint_angles = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']

    actual_position = ['Tx', 'Ty', 'Tz']

    actual_orientation = ['r11', 'r12', 'r13',
                          'r21', 'r22', 'r23', 'r31', 'r32', 'r33']
    error = ['ex', 'ey', 'ez', 'etheta1', 'etheta2', 'etheta3']

    for n, v in enumerate(position):
        sh_tp.write(0, n, v, cell_format_pos)
    for n, v in enumerate(orientation):
        sh_tp.write(0, n+3, v, cell_format_or)

    for ind, val in enumerate(points_list, 1):
        sh_tp.write(ind, 0, val[0])

        sh_tp.write(ind, 1, val[1])

        sh_tp.write(ind, 2, val[2])

    for ind, val in enumerate(orientation_list, 1):
        sh_tp.write(ind, 3, val[0][0])

        sh_tp.write(ind, 4, val[0][1])

        sh_tp.write(ind, 5, val[0][2])

        sh_tp.write(ind, 6, val[1][0])

        sh_tp.write(ind, 7, val[1][1])

        sh_tp.write(ind, 8, val[1][2])

        sh_tp.write(ind, 9, val[2][0])

        sh_tp.write(ind, 10, val[2][1])

        sh_tp.write(ind, 11, val[2][2])

    sh_q = workbook.add_worksheet("joint_angles")
    for n, v in enumerate(joint_angles):
        sh_q.write(0, n, v, cell_format_q)

    for ind, val in enumerate(joint_list, 1):
        sh_q.write(ind, 0, val[0])

        sh_q.write(ind, 1, val[1])

        sh_q.write(ind, 2, val[2])

        sh_q.write(ind, 3, val[3])

        sh_q.write(ind, 4, val[4])

        sh_q.write(ind, 5, val[5])

        sh_q.write(ind, 6, val[6])

    sh_cp = workbook.add_worksheet("current_pose")

    for n, v in enumerate(actual_position):
        sh_cp.write(0, n, v, cell_format_pos)
    for n, v in enumerate(actual_orientation):
        sh_cp.write(0, n+3, v, cell_format_or)

    for ind, val in enumerate(curr_pos_list, 1):
        sh_cp.write(ind, 0, val[0])

        sh_cp.write(ind, 1, val[1])

        sh_cp.write(ind, 2, val[2])

    for ind, val in enumerate(curr_or_list, 1):
        sh_cp.write(ind, 3, val[0][0])
        sh_cp.write(ind, 4, val[0][1])
        sh_cp.write(ind, 5, val[0][2])
        sh_cp.write(ind, 6, val[1][0])
        sh_cp.write(ind, 7, val[1][1])
        sh_cp.write(ind, 8, val[1][2])
        sh_cp.write(ind, 9, val[2][0])
        sh_cp.write(ind, 10, val[2][1])
        sh_cp.write(ind, 11, val[2][2])

    sh_er = workbook.add_worksheet("error")
    for n, v in enumerate(error):
        sh_er.write(0, n, v, cell_format_or)

    for ind, val in enumerate(error_list, 1):
        sh_er.write(ind, 0, val[0])

        sh_er.write(ind, 1, val[1])

        sh_er.write(ind, 2, val[2])

        sh_er.write(ind, 3, val[3])

        sh_er.write(ind, 4, val[4])

        sh_er.write(ind, 5, val[5])

    workbook.close()
    return 0


def output_tolerance(method, tolerance_list, time_list):
    filename = '/home/p/a/patidar/catkin_ws/src/ResearchMethodologyKTH/Data/' + \
        path_type + '_' + method + '_ToleranceVsTime.xlsx'
    workbook = xlsxwriter.Workbook(filename)
    sh = workbook.add_worksheet(method)
    sh.write(0, 0, "tolerance")
    sh.write(0, 1, "time")

    for n, v in enumerate(tolerance_list):
        sh.write(1+n, 0, v)
    for n, v in enumerate(time_list):
        sh.write(1+n, 1, v)

    workbook.close()
    return 0


def main(path):
    rospy.init_node('kuka_node')
    rate = rospy.Rate(10)

    points_list = []
    q_list = []
    r_list = []
    curr_pos_list = []
    curr_or_list = []
    error_list = []

    trajectory_publisher = None
    # the name of the robot's base frame
    base_frame = 'lwr_base_link'

    if path == 'line':
        vertices = [[-0.217, 0, 0.84], [-0.2, 0, 0.65],
                    [-0.2, 0, 0.65], [-0.217, 0, 0.84]]
        trajectory_publisher = SquareTrajectory(vertices, base_frame)
    elif path == 'square':

        vertices = [[-0.217, -0.217, 0.84], [-0.217, -0.217, 0.42],
                    [-0.217, 0.217, 0.42], [-0.217, 0.217, 0.84]]
        trajectory_publisher = SquareTrajectory(vertices, base_frame)
    elif path == 'circle':
        vertices = [[-0.45, -0.3, 0.74], 0.15]   # Circular parameters
        trajectory_publisher = CircularTrajectory(vertices, base_frame)

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

    threshold_list = [1e0, 1e-1, 1e-2, 1e-3, 1e-4, 1e-5, 1e-6]
    time_list = []
    for i in range(len(threshold_list)):
        restart(EmptyRequest())
        rate.sleep()
        error_threshold = threshold_list[i]
        done = False
        start_time = time.time()
        while not done:

            # get the current point in the trajectory
            point = trajectory_publisher.get_point()
            if point is not None:
                # get the IK solution for this point
                q, cur_pos, cur_or, err = IK_functions.kuka_IK(
                    point, desired_orientation, current_q, error_threshold)

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
                end_time = time.time()
                running_time = (end_time - start_time)
                print("Running time: ", running_time)
                time_list.append(running_time)
                # write to excel file
                output(points_list, r_list,
                       q_list, curr_pos_list, curr_or_list, error_list, error_threshold)

                print('Done')
                done = True
                # rospy.signal_shutdown('Done')
    output_tolerance(method, threshold_list, time_list)


if __name__ == '__main__':
    global path_type
    path_type = "circle"

    main(path_type)
