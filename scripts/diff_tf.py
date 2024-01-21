#!/usr/bin/env python


'''
fr - Front Right
fl - Front Left
br - Back Right
bl - Back Left


global variables/used for all wheels
    ticks_meter
    encoder_min
    encoder_max
    encoder_low_wrap
    encoder_high_wrap

variables per wheel
    enc_left - contains previous CORRECTED encoder reading
    left - contains latest CORRECTED encoder reading
    lmult - number of times the encoder has wrapped around
    prev_lencoder - used to correct encoder readings - remove wrap around/overflow

use prev_lencoder and left to remove wrap around/overflow
use left and lmult to add number of times the encoder has wrapped around
use left and enc_left to get ticks diff from previous message and use this difference to calculate distance traveled and velocity

subscribed topics
    fr_wheel
    fl_wheel
    br_wheel
    bl_wheel

published topics
    odom

'''


from std_msgs.msg import Int16
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from math import sin, cos, pi
import rospy
import roslib
roslib.load_manifest('differential_drive')


#############################################################################

class DiffTf:
    #############################################################################

    #############################################################################
    def __init__(self):
        #############################################################################
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        #### parameters #######
        # the rate at which to publish the transform
        self.rate = rospy.get_param('~rate', 10.0)
        # The number of wheel encoder ticks per meter of travel
        self.ticks_meter = float(rospy.get_param('ticks_meter', 50))
        # The wheel base width in meters
        self.base_width = float(rospy.get_param('~base_width', 0.245))

        # the name of the base frame of the robot
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        # the name of the odometry reference frame
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')

        self.encoder_min = rospy.get_param('encoder_min', -32768)
        self.encoder_max = rospy.get_param('encoder_max', 32768)
        self.encoder_low_wrap = rospy.get_param(
            'wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param(
            'wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)

        self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        #### variables ######
        self.wheels = ['fr', 'fl', 'br', 'bl']

        # old CORRECTED wheel encoder readings #self.enc_left
        self.encs_old = {wheel: None for wheel in self.wheels}
        # new CORRECTED wheel encoder readings #self.left
        self.encs_new = {wheel: 0 for wheel in self.wheels}
        # number of times the encoder has wrapped around #self.lmult
        self.mults = {wheel: 0 for wheel in self.wheels}
        # used to correct encoder readings - remove wrap around/overflow #self.prev_lencoder
        self.prev_encs = {wheel: 0 for wheel in self.wheels}

        # distance traveled by each wheel #self.d
        self.dists = {wheel: 0 for wheel in self.wheels}
        # angle of each wheel #self.th
        self.thetas = {wheel: 0 for wheel in self.wheels}

        # internal data

        # # wheel encoder readings
        # self.enc_left = None
        # self.enc_right = None

        # # actual values coming back from robot
        # self.left = 0
        # self.right = 0
        # self.lmult = 0
        # self.rmult = 0
        # self.prev_lencoder = 0
        # self.prev_rencoder = 0

        # position in xy plane
        self.x = 0
        self.y = 0
        self.th = 0

        # speeds in x/rotation
        self.dx = 0
        self.dy = 0
        self.dz = 0  # angular velocity about z axis
        # self.dr = 0

        self.then = rospy.Time.now()

        # subscriptions
        for wheel in self.wheels:
            rospy.Subscriber(wheel + "_wheel", Int16,
                             self.wheel_callback, wheel)
        # rospy.Subscriber("lwheel", Int16, self.lwheelCallback)
        # rospy.Subscriber("rwheel", Int16, self.rwheelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    #############################################################################
    def spin(self):
        #############################################################################
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    #############################################################################

    def update(self):
        #############################################################################
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            for wheel in self.wheels:
                # calculate odometry
                if self.encs_old[wheel] == None:
                    self.dists[wheel] = 0
                    # d_left = 0
                    # d_right = 0
                else:
                    self.dists[wheel] = (
                        self.encs_new[wheel] - self.encs_old[wheel]) / self.ticks_meter
                    # d_left = (self.left - self.enc_left) / self.ticks_meter
                    # d_right = (self.right - self.enc_right) / self.ticks_meter
                self.encs_old[wheel] = self.encs_new[wheel]
                # self.enc_left = self.left
                # self.enc_right = self.right

            self.distance_travelled_x = {wheel: cos(
                self.thetas[wheel]) * self.dists[wheel] for wheel in self.wheels}
            self.distance_travelled_y = {wheel: sin(
                self.thetas[wheel]) * self.dists[wheel] for wheel in self.wheels}

            x = sum(self.distance_travelled_x.values()) / len(self.wheels)
            y = sum(self.distance_travelled_y.values()) / len(self.wheels)
            # should i wrap the value back to 0-2pi? is front 0 or 90 deg
            th = sum(self.thetas.values()) / len(self.wheels)
            # distance traveled is the average of the two wheels
            # d = ( d_left + d_right ) / 2
            # this approximation works (in radians) for small angles
            # th = ( d_right - d_left ) / self.base_width
            # calculate velocities
            self.dx = x/elapsed
            self.dy = y/elapsed
            self.dz = y/elapsed
            # self.dx = d / elapsed
            # self.dr = th / elapsed

            # calculate distance traveled in x and y
            if (x != 0):
                self.x = self.x + x

            if (y != 0):
                self.y = self.y + y

            if (th != 0):
                self.th = self.th + th

            # publish the odom information
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = self.dy
            odom.twist.twist.angular.z = self.dz
            self.odomPub.publish(odom)

    def wheel_callback(self, msg, wheel):
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_encs[wheel] > self.encoder_high_wrap):
            self.mults[wheel] = self.mults[wheel] + 1

        if (enc > self.encoder_high_wrap and self.prev_encs[wheel] < self.encoder_low_wrap):
            self.mults[wheel] = self.mults[wheel] - 1

        self.encs_new[wheel] = 1.0 * \
            (enc + self.mults[wheel] * (self.encoder_max - self.encoder_min))
        self.prev_encs[wheel] = enc

    # #############################################################################
    # def lwheelCallback(self, msg):
    #     #############################################################################
    #     enc = msg.data
    #     if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
    #         self.lmult = self.lmult + 1

    #     if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
    #         self.lmult = self.lmult - 1

    #     self.left = 1.0 * (enc + self.lmult *
    #                        (self.encoder_max - self.encoder_min))
    #     self.prev_lencoder = enc

    # #############################################################################
    # def rwheelCallback(self, msg):
    #     #############################################################################
    #     enc = msg.data
    #     if (enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
    #         self.rmult = self.rmult + 1

    #     if (enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
    #         self.rmult = self.rmult - 1

    #     self.right = 1.0 * (enc + self.rmult *
    #                         (self.encoder_max - self.encoder_min))
    #     self.prev_rencoder = enc


#############################################################################
#############################################################################
if __name__ == '__main__':
    """ main """
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
