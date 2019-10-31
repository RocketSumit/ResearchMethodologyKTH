#! /usr/bin/env python

"""
    This contains a circular trajectory in 3D given the centre and radius, and publishes in rviz
    @author: Utkarsh Kunwar (utkarshk@kth.se)
"""

import numpy as np
from geometry_msgs.msg import Pose, PoseArray
import rospy
import rospkg
rospack = rospkg.RosPack()


class CircularTrajectory():
    def __init__(self, parameters=[[-0.217, -0.217, 0.42], 0.21], base_frame='base'):
        self._path_publisher = rospy.Publisher(
            'desired_path', PoseArray, queue_size=10)
        self._path = PoseArray()
        self._path.header.frame_id = base_frame
        self._dt = 0.1
        self._v = 0.05
        self._resolution = 181    # Number of sides of polygon + 1

        # The 2 points of the circle
        self._parameters = parameters
        self._vertices = self.get_vertices()

        self._current_segment = 0
        self._current_idx = 0
        self._waypoints = None
        self.compute_waypoints()

    def next_segment(self):
        """ This function returns the next segment of the circle. -1 if the path ended """
        if self._current_segment is (self._resolution - 1):
            self._current_segment = -1
        else:
            self._current_segment += 1

        self._current_idx = 0

    def return_list_of_waypoints(self, p1, p2, dp, it):
        """ This function returns a list of waypoints between a start and a goal point """
        waypoints = list()
        current_p = p1
        waypoints.append(current_p)
        p = Pose()
        p.position.x = current_p[0]
        p.position.y = current_p[1]
        p.position.z = current_p[2]
        p.orientation.w = 0.707
        p.orientation.y = 0.707
        self._path.poses.append(p)
        for i in range(1, int(abs(it))):
            current_p = current_p + dp
            waypoints.append(current_p)
            p = Pose()
            p.position.x = current_p[0]
            p.position.y = current_p[1]
            p.position.z = current_p[2]
            p.orientation.w = 0.707
            p.orientation.y = 0.707
            self._path.poses.append(p)

        waypoints.append(p2)

        return waypoints

    def get_vertices(self):
        c = self._parameters[0]
        r = self._parameters[1]

        points = []
        for angle in np.linspace(0.0, 2 * np.pi, self._resolution):
            point = [c[0], c[1] + r * np.cos(angle), c[2] + r * np.sin(angle)]
            points.append(point)

        return points

    def compute_waypoints(self):
        """ This function computes all the segments of the circle, given the initial vertices """
        ds = self._v*self._dt

        waypoints = []
        for i in range(self._resolution):
            v1 = np.array(self._vertices[i])

            if i != (self._resolution - 1):
                v2 = np.array(self._vertices[i + 1])
            else:
                v2 = np.array(self._vertices[0])

            v = v2 - v1
            v_ds = v / ds
            it = abs(v_ds[(np.absolute(v_ds)).argmax()])

            if abs(float(it) < 1e-6):
                v1v2 = []
            else:
                dv = v / float(it)
                v1v2 = self.return_list_of_waypoints(v1, v2, dv, it)

            waypoints.append(v1v2)
            self._waypoints = waypoints

    def publish_path(self):
        """ This function publishes the path in rviz"""
        self._path_publisher.publish(self._path)

    def get_point(self):
        """ This function returns the next point in the path. None if the path ended """
        if self._current_segment is -1:
            return None
        l = len(self._waypoints[self._current_segment])
        if(self._current_idx >= l):
            self.next_segment()

        if self._current_segment is -1:
            return None

        if len(self._waypoints[self._current_segment]) < 1:
            self.next_segment()

        if self._current_segment is -1:
            return None

        desired_point = (self._waypoints[self._current_segment])[
            self._current_idx]
        self._current_idx += 1

        return desired_point

    def restart(self):
        """ This function resets the current point to go through the path once again """
        self._current_segment = 0
        self._current_idx = 0
