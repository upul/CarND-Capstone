#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from tf.transformations import euler_from_quaternion

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200  # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.pose = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose
        if self.pose is not None and self.base_waypoints is not None:
            self.loop()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints

    def loop(self):
        # get vehicle pose
        car_x = self.pose.position.x
        car_y = self.pose.position.y
        car_z = self.pose.position.z
        car_o = self.pose.orientation
        rospy.loginfo('car x, y: {}, {}'.format(car_x, car_y))
        car_q = (car_o.x, car_o.y, car_o.z, car_o.w)
        car_roll, car_pitch, car_yaw = euler_from_quaternion(car_q)

        # compute list of waypoints in front of the car
        waypoints_ahead = []

        # for each waypoint w
        for w in range(len(self.base_waypoints)):
            wp = self.base_waypoints[w]
            wp_x = wp.pose.pose.position.x
            wp_y = wp.pose.pose.position.y
            wp_z = wp.pose.pose.position.z
            # check if waypoint is in front of the car
            wp_ahead = ((wp_x - car_x) * math.cos(car_yaw) +
                        (wp_y - car_y) * math.sin(car_yaw)) > 0.0
            # ignore if waypoint is not
            if not wp_ahead:
                continue

            # calculate distance between waypoint and car and store (waypoint, distance) tuple
            wp_dist = math.sqrt((car_x - wp_x) ** 2 + (car_y - wp_y) ** 2 +(car_z - wp_z)**2)
            waypoints_ahead.append((wp, wp_dist))

        # sort waypoints by distance
        waypoints_ahead = sorted(waypoints_ahead, key=lambda x: x[1])[:LOOKAHEAD_WPS]

        # get list of waypoints (from sorted list of (waypoint, distance) tuples)
        wps_ahead = [wp[0] for wp in waypoints_ahead]

        # create Lane message with list of waypoints ahead
        lane = Lane()
        lane.waypoints = wps_ahead
        #rospy.loginfo('next waypoint: {}'.format(wps_ahead[0].pose.pose.position.x))
        self._print_selected_waypoints(wps_ahead)

        # publish lane to final waypoints
        self.final_waypoints_pub.publish(lane)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        for i in range(wp1, wp2 + 1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def _print_selected_waypoints(self, waypoints):
        for w in range(len(waypoints)):
            wp = waypoints[w]
            wp_x = wp.pose.pose.position.x
            wp_y = wp.pose.pose.position.y
            wp_z = wp.pose.pose.position.z
            rospy.loginfo('point: {} {} {}'.format(wp_x, wp_y, wp_z))
        rospy.loginfo('=========================================')


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')