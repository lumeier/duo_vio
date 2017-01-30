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
# File: IMUCalibrator.py
## Created on: 26.01.17
# Author: Lukas Meier (Adapted from DUOCalibrator.py by Nicolas de Palezieux)

from __future__ import print_function, division
import yaml
import sys
import os
import rospy
from ait_ros_messages.msg import VioSensorMsg
from std_msgs.msg import String
import rospkg
from sensor_msgs.msg import Imu


def imu_cb(imu_msg):
    global gyr, acc, cnt, active, user_ready

    if not user_ready:
        return
    num_samples = 1000  # number of IMU samples to capture


    gyr['x'].append(imu_msg.angular_velocity.x)
    gyr['y'].append(imu_msg.angular_velocity.y)
    gyr['z'].append(imu_msg.angular_velocity.z)

    acc['x'].append(imu_msg.linear_acceleration.x)
    acc['y'].append(imu_msg.linear_acceleration.y)
    acc['z'].append(imu_msg.linear_acceleration.z + 9.81)

    cnt += 1

    if cnt >= num_samples:
        gyr['x'] = sum(gyr['x'])/len(gyr['x'])
        gyr['y'] = sum(gyr['y'])/len(gyr['y'])
        gyr['z'] = sum(gyr['z'])/len(gyr['z'])

        acc['x'] = sum(acc['x'])/len(acc['x'])
        acc['y'] = sum(acc['y'])/len(acc['y'])
        acc['z'] = sum(acc['z'])/len(acc['z'])
        print('\nFinished capturing data.')
        active = 0

    if cnt >= num_samples:
        cnt = 0
        user_ready = 0

if __name__ == "__main__":
    rospy.init_node('calibrate_IMU')

    gyr = {'x': [], 'y': [], 'z': []}
    acc = {'x': [], 'y': [], 'z': []}

    cnt = 0
    active = 1
    user_ready = 0
    device_serial_nr = None

    rospy.Subscriber("/imu", Imu, imu_cb, queue_size=1)

    print('Setting device serial nr to slamdunk...')
    device_serial_nr = 'slamdunk'
    print('... done')

    print('Calibrating IMU. Place the device with zero roll and pitch (i.e. facing horizontally forward). Do not move the device')
    rate = rospy.Rate(100)
    while active and not rospy.is_shutdown():
        if not user_ready:
            raw_input("Press Enter when ready...")
            user_ready = 1
            print('Recording data...')
        rate.sleep()

    if not rospy.is_shutdown():

        rospack = rospkg.RosPack()
        duo_path = rospack.get_path('duo_vio')

        # select lens
        lenses = os.listdir(os.path.join(duo_path, 'calib', device_serial_nr))
        lenses = [lens for lens in lenses if os.path.isdir(os.path.join(duo_path, 'calib', device_serial_nr, lens))]

        if len(lenses) == 1:
            print('Found one lens: {}. Using that one.'.format(lenses[0]))
            lens = lenses[0]
        else:
            print('Found several lenses:')
            for i, lens in enumerate(lenses):
                print('{}: {}'.format(i+1, lens))
            selection = int(raw_input('Select the lens you want by providing the appropriate number: '))
            if selection < 1 or selection > len(lenses):
                raise Exception('The provided number {} is not in the valid range [{}:{}]'.format(selection, 1, len(lenses)))
            lens = lenses[selection-1]

        # select resolution
        resolutions = os.listdir(os.path.join(duo_path, 'calib', device_serial_nr, lens))
        resolutions = [resolution for resolution in resolutions if os.path.isdir(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution))]

        if len(resolutions) == 1:
            print('Found one resolution: {}. Using that one.'.format(resolutions[0]))
            resolution = resolutions[0]
        else:
            print('Found several resolutions:')
            for i, resolution in enumerate(resolutions):
                print('{}: {}'.format(i+1, resolution))
            selection = int(raw_input('Select the resolution you want by providing the appropriate number: '))
            if selection < 1 or selection > len(resolutions):
                raise Exception('The provided number {} is not in the valid range [{}:{}]'.format(selection, 1, len(resolutions)))
            resolution = resolutions[selection-1]

        # load the yaml file
        with open(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution, 'cameraParams.yaml'), 'r') as infile:
            cameraParams = yaml.load(infile)

        print('For each axis of the accelerometer and gyroscope you can decide to use the new estimate (answer y), keep the old one (answer n).')

        print('Accelerometer biases:')
        print('Old       \tNew')
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['acc_bias'][0][0], acc['x']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['acc_bias'][1][0], acc['y']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['acc_bias'][2][0], acc['z']))

        axes = 'xyz'
        for i in range(3):
            try:
                selection = raw_input('Do you want to use the new accelerometer {} axis estimate? [Y/n]: '.format(axes[i]))
            except EOFError:
                print('')
                sys.exit(-1)

            if not selection or selection == 'y' or selection == 'Y':
                cameraParams['acc_bias'][i][0] = acc[axes[i]]

        print('Gyroscope biases:')
        print('Old       \tNew')
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][0][0], gyr['x']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][1][0], gyr['y']))
        print('{0: 6.6f}\t{1: 6.6f}'.format(cameraParams['gyro_bias'][2][0], gyr['z']))

        for i in range(3):
            try:
                selection = raw_input('Do you want to use the new gyroscope {} axis estimate? [Y/n]: '.format(axes[i]))
            except EOFError:
                print('')
                sys.exit(-1)

            if not selection or selection == 'y' or selection == 'Y':
                cameraParams['gyro_bias'][i][0] = gyr[axes[i]]

        with open(os.path.join(duo_path, 'calib', device_serial_nr, lens, resolution, 'cameraParams.yaml'), 'w') as outfile:
            outfile.write(yaml.dump(cameraParams, default_flow_style=None))
