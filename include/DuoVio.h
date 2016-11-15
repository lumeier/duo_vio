/****************************************************************************
 *
 *   Copyright (c) 2015-2016 AIT, ETH Zurich. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name AIT nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/*
 * DuoVio.h
 *
 *  Created on: Mar 9, 2016
 *      Author: nicolas
 */
#ifndef _DuoVio_H_
#define _DuoVio_H_

#include <ros/ros.h>
#include <ros/package.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Path.h>

#include <opencv2/opencv.hpp>
#include <boost/circular_buffer.hpp>

#include <vector>
#include <string>
#include <cstdio>
#include <fstream>

#include "sensor_msgs/Joy.h"

#include "VIO.h"
#include "dynamic_reconfigure/server.h"
#include "duo_vio/duo_vioConfig.h"

#include "ait_ros_messages/vio_vis.h"
#include "ait_ros_messages/VioSensorMsg.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/String.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/UInt32.h"

#include "InterfaceStructs.h"
#include "IMULowpass.h"

#include "Precision.h"

#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class DuoVio {
 public:
    DuoVio();
    ~DuoVio();

 private:
    VIO vio;


    sensor_msgs::Image lastImgLeft;
    sensor_msgs::Image lastImgRight;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    void ImageSetCb(const sensor_msgs::ImageConstPtr& msg_img_l, const sensor_msgs::ImageConstPtr& msg_img_r);
    bool new_left=0;
    bool new_right=0;

    // Visualization topics
    ros::Publisher vio_vis_pub;
    ros::Publisher vio_vis_reset_pub;

    int vis_publish_delay;
    bool SLAM_reset_flag;
    int display_tracks_cnt;
    int max_clicks_;
    int clear_queue_counter;
    double fps;
    int vio_cnt;
    int vision_subsample;

    DUOParameters cameraParams;
    NoiseParameters noiseParams;
    VIOParameters vioParams;
    ros::NodeHandle nh_;
    IMULowpass imulp_;

    cv::Mat darkCurrentL, darkCurrentR;
    bool use_dark_current;

    ros::Subscriber vio_sensor_sub;
    ros::Subscriber left_image_sub;
    ros::Subscriber right_image_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber device_serial_nr_sub;
    std::string device_serial_nr;
    bool got_device_serial_nr;
    bool auto_subsample;  // if true, predict with messages without image data, otherwise update

    ros::Publisher vio_sensor_processed_pub;
    dynamic_reconfigure::Server<duo_vio::duo_vioConfig> dynamic_reconfigure_server;

    ros::Publisher pose_pub;
    ros::Publisher vel_pub;
    ros::Publisher timing_SLAM_pub;
    ros::Publisher timing_feature_tracking_pub;
    ros::Publisher timing_total_pub;
    ros::Publisher vis_pub_;
    ros::Publisher smoothed_imu_pub;  // debug

    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform camera_tf;
    tf::Transform body_tf;
    tf::Quaternion cam2body;

    ros::Time prev_time_;
    std::vector<int> update_vec_;
    geometry_msgs::Pose pose;

    unsigned int num_points_;
    bool show_camera_image_;
    int image_visualization_delay;
    RobotState robot_state;
    double dist;
    double last_pos[3];

    ros::Subscriber reset_sub;
    void resetCb(const std_msgs::Empty &msg);

    std::vector<FloatType> h_u_apo;
    std::vector<FloatType> map;
    std::vector<AnchorPose> anchor_poses;

    void vioSensorMsgCb(const ait_ros_messages::VioSensorMsg &msg);
    void rightImageMsgCb(const sensor_msgs::Image &msg);
    void leftImageMsgCb(const sensor_msgs::Image &msg);
    void imuCb(const sensor_msgs::Imu &msg);
    void deviceSerialNrCb(const std_msgs::String &msg);
    void loadCustomCameraCalibration(const std::string calib_path);
    void update(double dt, const ait_ros_messages::VioSensorMsg &msg, bool debug_publish, bool show_image, bool reset);

    void getIMUData(const sensor_msgs::Imu& imu, VIOMeasurements& meas);

    void updateVis(RobotState &robot_state, std::vector<AnchorPose> &anchor_poses, std::vector<FloatType> &map, std::vector<int> &updateVect,
            const ait_ros_messages::VioSensorMsg &msg, std::vector<FloatType> &z_l, bool show_image);

    tf::Quaternion camera2world;  // the rotation that transforms a vector in the camera frame to one in the world frame

    void dynamicReconfigureCb(duo_vio::duo_vioConfig &config, uint32_t level);
};

#endif /* _DuoVio_H_ */
