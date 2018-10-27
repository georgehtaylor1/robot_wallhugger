#!/usr/bin/env python
"""
A simple 'Wall Hugger' node which demonstrates the basics of subscribing and
publishing to channels specific to the Pioneer robot.

I have tried to excessively comment the code and use best practices, including
using Google's documentation style for writing docstrings (python's version of
JavaDoc or Doxygen).
Several TODO's have been left as exercises for the reader :)

Author: Matthew Broadway
"""
import math
import numpy as np
import subprocess
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def nth_smallest(values, n):
    """ get the n'th smallest value from the list.
    This helps avoid noise where some values are incorrectly very small.

    Args:
        values (list): the values to select from
        n (int): the element number to pick
    """
    return np.sort(values)[n]


def clean_lasers(readings, limits):
    """ discard bad values from the laser scan and clip the values between the limits.
    Args:
        readings (list of floats): a list of laser readings
        limits (tuple): the reported limits of distance readings (min_value, max_value)
    """
    r_min, r_max = limits
    # TODO: instead of filtering out the bad values, you may wish to replace them
    #      with a constant value instead (e.g. r_min) if you are relying on a
    #      particular element always corresponding to the same angle.
    valid_readings = [np.clip(r, a_min=r_min, a_max=r_max) for r in readings
                      if not math.isnan(r) and not math.isinf(r)]
    assert valid_readings, 'none of the readings were valid!'
    return valid_readings


class WallHugger:
    """ A behaviour which instructs the robot to follow the wall to its right

    Attributes:
        move_pub: a publisher to send movement commands to the robot
        noisy (bool): whether to print debugging information
        speed (Float): the linear speed to use when able to move forwards
        rotate_speed (Float): the angular speed to use when turning
        min_wall_distance (Float): the threshold at which the robot should turn
            away from the wall if it is closer to it.
        max_wall_distance (Float): the threshold at which the robot should turn
            away from the wall if it is further from it.
    """

    def __init__(self, noisy=False):
        """
        Args:
            noisy (bool): whether to print debugging information
        """
        # listen to the laser messages. The callback will be repeatedly called
        # with `LaserScan` objects containing the latest sensor readings.
        rospy.Subscriber('base_scan', LaserScan, self.laser_callback)
        # create a publisher for sending commands to the wheels (in the form of
        # `Twist` objects). The queue size is used for buffering messages if too
        # many messages are published to process at once.
        self.move_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
        # whether to print debugging messages
        self.noisy = noisy

        # TODO: these values are just chosen to work reasonably well in the
        #      simulator, different parameters are probably required when
        #      running on the real robot.
        self.speed = 0.3
        # TODO: setting this too high causes the robot to spin in a tight circle
        #      without ever 'attaching' to a wall if placed in a large enough
        #      open space. Setting too low makes the turning circle very large
        #      once it reaches a corner. Perhaps a large value is best, with
        #      some fallback if the robot completes 360 degrees without finding
        #      a wall.
        self.rotate_speed = 0.5
        self.rotate_speed_slow = 0.2
        self.prev_dist = 5
        desired_wall_distance = 1.2
        threshold = 0.2
        self.min_wall_distance = desired_wall_distance - threshold
        self.max_wall_distance = desired_wall_distance + threshold
        self.angle_thresh = 1
        self.side_range = 0.3
        self.front_range = 0.8

    def is_obstructed(self, front, left, right, ranges):
        """ whether the robot is able to safely move forwards.
        i.e. If there is space in front and there is no obstacle too close anywhere.

        Args:
            front (Float): the distance reading representing the section
                directly in front of the robot.
            ranges (list): all the laser distance readings (with bad values
                filtered out)

        Returns:
            whether the robot is 'obstructed'
        """
        return front < self.front_range, left < self.side_range, right < self.side_range

    def laser_callback(self, laser_msg):
        """ callback for receiving laser sensor messages

        Documentation for LaserScan message:
        http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
        The LaserScan messages contain metadata such as the min and max values,
        and an array of distance values. The first value corresponds to 90
        degrees to the right of the robot and the lasers sweep ~180 degrees
        anti-clockwise.

        Args:
            laser_msg (LaserScan): the lidar distance readings message
        """
        limits = (laser_msg.range_min, laser_msg.range_max)
        ranges = laser_msg.ranges  # clean_lasers(laser_msg.ranges, limits)

        mid = lambda x: x  # x[len(x)/3:len(x)*2/3]

        # split the readings spanning 180 degrees into 3 sections
        right, front, left = np.array_split(ranges, 3)

        # right, left = np.array_split(ranges,2)

        # choose representatives for each section
        right = nth_smallest(mid(right), 60)
        front = nth_smallest(mid(front), 60)
        left = nth_smallest(mid(left), 60)

        d_dist = self.prev_dist - right

        if self.noisy:
            rospy.loginfo('L: {}, F: {}, R: {}'.format(left, front, right))

        new_speed = Twist()
        # angular.z positive is anti-clockwise

        # TODO: instead of using constants for speed and rotation speed, could
        #      use something like a PID or PD controller to aim for a set-point
        #      distance from the wall in a more smooth manner to avoid jerky
        #      motion and overshooting. Could speed up when the wall is flat but
        #      slow down to navigate tricky obstacles.
        action = "None"

        a = laser_msg.ranges[190]
        b = laser_msg.ranges[250]

        ma = -1
        dir = None

        # > 0 = left
        # < 0 = right
        dir = (a - b) * -1
        f, l, r = self.is_obstructed(front, left, right, ranges)
        if f or r:
            new_speed.angular.z = self.rotate_speed  # ac
            action = "Left"
        elif l:
            new_speed.angular.z = -self.rotate_speed  # ac
            action = "Right"
        else:
            new_speed.linear.x = self.speed
            action = "Forward"


        if self.noisy:
            rospy.loginfo(
                'lin: {}, ang: {}, act: {}, ma: {}, dir: {}'.format(new_speed.linear.x, new_speed.angular.z, action, ma,
                                                                    dir))

        # send the movement command to the robot
        self.move_pub.publish(new_speed)

if __name__ == '__main__':
    # register this process as a ROS node
    rospy.init_node(name='wall_hugger')
    # this is a useful trick if you keep forgetting to release the brakes for the wheels.
    raw_input('turn on the motors, then press enter to start')
    wh = WallHugger(noisy=True)
    # process messages until rospy.is_shutdown()
    rospy.spin()
