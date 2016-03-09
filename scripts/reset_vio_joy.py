#!/usr/bin/python

#   Copyright (c) 2016 AIT, ETH Zurich. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name AIT nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# File: reset_vio_joy.py
# Created on: 09.03.16
# Author: Nicolas de Palezieux

from __future__ import print_function, division
import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
import datetime

__author__ = 'nicolas'


def joy_cb(data):
    global reset_pub, last_time, timeout
    if any(data.buttons) and last_time + datetime.timedelta(seconds=timeout) < datetime.datetime.now():
        reset_pub.publish(Empty())
        last_time = datetime.datetime.now()

if __name__ == "__main__":
    rospy.init_node("vio_reset")

    last_time = datetime.datetime.now()
    timeout = 0.5  # min seconds between two reset sends

    reset_pub = rospy.Publisher("vio_ros/reset", Empty, queue_size=1)

    rospy.Subscriber("/joy", Joy, joy_cb, queue_size=1)

    rospy.spin()
