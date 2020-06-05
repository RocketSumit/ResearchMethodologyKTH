#! /usr/bin/env python3
import numpy as np
import random
import time
import math
import os


"""
    # {student full name} Sumit Patidar
    # {student email} patidar@kth.se
"""

dir = 'kuka_data1_5m'
os.mkdir(dir)

inputfile = dir + '/network_input'
outputfile = dir + '/network_output'
statsfile = dir + '/data_stats'


def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)


def euler_angles_from_rotation_matrix(R):
    '''
    From a paper by Gregory G. Slabaugh (undated),
    "Computing Euler angles from a rotation matrix
    '''
    phi = 0.0
    if isclose(R[2, 0], -1.0):
        theta = math.pi/2.0
        psi = math.atan2(R[0, 1], R[0, 2])
    elif isclose(R[2, 0], 1.0):
        theta = -math.pi/2.0
        psi = math.atan2(-R[0, 1], -R[0, 2])
    else:
        theta = -math.asin(R[2, 0])
        cos_theta = math.cos(theta)
        psi = math.atan2(R[2, 1]/cos_theta, R[2, 2]/cos_theta)
        phi = math.atan2(R[1, 0]/cos_theta, R[0, 0]/cos_theta)
    return np.array([psi, theta, phi]).reshape(3, 1)


def forwardKinemtaticsKuka(joint_angles):

     # define constants
    L = 0.4
    M = 0.39
    # define DH parameters - do they need to be modified ?
    alpha = np.array([np.pi/2, -np.pi/2, -np.pi/2,
                      np.pi/2, np.pi/2, -np.pi/2, 0.0])
    d = np.array([0.311, 0.0, L, 0.0, M, 0.0, 0.078])
    a = np.zeros(7)
    links = 7

    # carry out transformation
    T = []
    links = len(joint_angles)
    T_trans = np.identity(4)
    for i in range(links):
        T_temp = np.array([[np.cos(joint_angles[i]), -np.sin(joint_angles[i])*np.cos(alpha[i]), np.sin(joint_angles[i])*np.sin(alpha[i]), a[i]*np.cos(joint_angles[i])], [np.sin(joint_angles[i]), np.cos(joint_angles[i])*np.cos(alpha[i]), -np.cos(joint_angles[i])*np.sin(alpha[i]), a[i]*np.sin(joint_angles[i])],
                           [0.0, np.sin(alpha[i]), np.cos(alpha[i]), d[i]], [0.0, 0.0, 0.0, 1]])
        # print('t', T_temp)
        T_trans = np.matmul(T_trans, T_temp)
        # print('T trans',T_trans)
        T.append(T_trans)
        # print('T',T[i])

    # pose = np.zeros((12, 1))
    # pose[0:9] = orientation_matrix.reshape(9, 1)
    pose = np.zeros((6, 1))
    # get angular elements
    orientation_matrix = T[6][:3, :3]
    pose[0:3] = euler_angles_from_rotation_matrix(orientation_matrix)
    # get position elements
    pose[3:] = np.reshape(T[links-1][:3, 3], (3, 1))

    return pose


def main():

    # parameters
    random.seed(5)
    iterations = 5000000
    resolution = 1  # in degrees
    links = 7
    tried_angles = []
    reached_pose = []
    start_time = time.time()
    # total steps of angles
    steps = int(np.radians(340)/np.radians(resolution))
    joints_range = np.array([[np.radians(-170), np.radians(170)], [np.radians(-120), np.radians(120)], [np.radians(-170), np.radians(170)],
                             [np.radians(-120), np.radians(120)], [np.radians(-170), np.radians(170)], [np.radians(-120), np.radians(120)], [np.radians(-175), np.radians(175)]], dtype=np.float32)

    # get range of values for each joint
    joints_values = np.array([])
    for i in range(links):
        joints_values = np.append(joints_values, np.linspace(
            joints_range[i][0], joints_range[i][1], num=steps), axis=0)

    # break the 1d array into each joint angle array
    joints_values = np.reshape(joints_values, (links, steps))

    # pick random values for each joint and loop for given iterations
    print("running for ", iterations, " iterations")
    for i in range(1, iterations+1):

        cur_angles = ([random.choice(joints_values[0]),
                       random.choice(joints_values[1]), random.choice(joints_values[2]), random.choice(joints_values[3]), random.choice(joints_values[4]), random.choice(joints_values[5]), random.choice(joints_values[6])])
        cur_pose = forwardKinemtaticsKuka(
            np.array(cur_angles, dtype=np.float32))

        # save current angles and pose
        tried_angles.append(cur_angles)
        reached_pose.append(cur_pose)

        if (i % 100000 == 0):
            elapsed_time = time.time() - start_time
            print("status: ", i, "time: ", elapsed_time)
            start_time = time.time()

    tried_angles = np.array(tried_angles, dtype=np.float32)
    reached_pose = np.array(reached_pose, dtype=np.float32)
    tried_angles = np.reshape(tried_angles, (iterations, links))
    reached_pose = np.reshape(reached_pose, (iterations, 6))

    # check for duplicates and remove them
    uniq_reached_pose, uniq_index = np.unique(
        reached_pose, axis=0, return_index=True)
    uniq_tried_angles = tried_angles[uniq_index]
    print("unique datapoints: ", uniq_reached_pose.shape[0])

    # save to file
    np.save(outputfile, uniq_tried_angles)
    np.save(inputfile, uniq_reached_pose)

    # check for max, min values
    stats = []

    # network input stats
    input_min_stats = np.amin(reached_pose, axis=0)
    stats.append(input_min_stats)
    input_max_stats = np.amax(reached_pose, axis=0)
    stats.append(input_max_stats)

    # network output stats
    output_min_stats = np.amin(tried_angles, axis=0)
    stats.append(output_min_stats)
    output_max_stats = np.amax(tried_angles, axis=0)
    stats.append(output_max_stats)

    # save the stats to file
    np.save(statsfile, np.array(stats))

    print("Success!")


if __name__ == '__main__':
    st = time.time()
    main()
    print('Total time taken: ', time.time()-st)
    # print("input: \n", np.load(inputfile + ".npy"))
    # print("output: \n", np.load(outputfile + ".npy"))
    print("stats: \n", np.load(statsfile + ".npy", allow_pickle=True))
    #forwardKinemtaticsKuka([0, 1.12, 0, 1.71, 0, 1.84, 0])
