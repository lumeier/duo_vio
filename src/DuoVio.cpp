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
 * DuoVio.cpp
 *
 *  Created on: Mar 9, 2016
 *      Author: nicolas
 */
#include "DuoVio.h"

#include <math.h>
#include <stdio.h>
#include <time.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <trackFeatures.h>


static const int VIO_SENSOR_QUEUE_SIZE = 30;

DuoVio::DuoVio() :
                nh_("~"),
                SLAM_reset_flag(1),
                cam2body(-0.5, 0.5, -0.5, -0.5),
                max_clicks_(0),
                clear_queue_counter(0),
                vio_cnt(0),
                image_visualization_delay(0),
                auto_subsample(false),
                dist(0.0),
                got_device_serial_nr(false),
                use_dark_current(false) {
    // initialize structs
    cameraParams = { {}, {}};
    noiseParams = {};
    vioParams = {};

    left_image_sub = nh_.subscribe("/left_grayscale/image", VIO_SENSOR_QUEUE_SIZE, &DuoVio::leftImageMsgCb, this);
    right_image_sub = nh_.subscribe("/right_grayscale/image", VIO_SENSOR_QUEUE_SIZE, &DuoVio::rightImageMsgCb, this);

    messge_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(left_image_sub, right_image_sub, 10);
    sync.registerCallback(boost::bind(&ImageSetCb(), _1, _2));

    imu_sub = nh_.subscribe("/imu", VIO_SENSOR_QUEUE_SIZE, &DuoVio::imuCb, this);

    vio_sensor_sub = nh_.subscribe("/vio_sensor", VIO_SENSOR_QUEUE_SIZE, &DuoVio::vioSensorMsgCb, this);
    device_serial_nr_sub = nh_.subscribe("/vio_sensor/device_serial_nr", 1, &DuoVio::deviceSerialNrCb, this);
    reset_sub = nh_.subscribe("reset", 1, &DuoVio::resetCb, this);

    pose_pub = nh_.advertise<geometry_msgs::Pose>("pose", 1);
    vel_pub = nh_.advertise<geometry_msgs::Vector3>("vel", 1);

    smoothed_imu_pub = nh_.advertise<sensor_msgs::Imu>("imu_smoothed", 1);  // debug

    // visualization topics
    vio_vis_pub = nh_.advertise<ait_ros_messages::vio_vis>("/vio_vis/vio_vis", 1);
    vio_vis_reset_pub = nh_.advertise<std_msgs::Empty>("/vio_vis/reset", 1);
    ros::Duration(0.5).sleep();  // otherwise the following message might not be received
    vio_vis_reset_pub.publish(std_msgs::Empty());

    vio_sensor_processed_pub = nh_.advertise<std_msgs::UInt64>("/vio_sensor/msg_processed", 1);

    // Load parameters from launch file
    double tmp_scalar;
    std::vector<double> tmp_vector;

    if (!nh_.getParam("noise_acc", tmp_scalar))
        ROS_WARN("Failed to load parameter noise_acc");
    else
        noiseParams.process_noise.qv = tmp_scalar;
    if (!nh_.getParam("noise_gyro", tmp_scalar))
        ROS_WARN("Failed to load parameter noise_gyro");
    else
        noiseParams.process_noise.qw = tmp_scalar;
    if (!nh_.getParam("noise_gyro_bias", tmp_scalar))
        ROS_WARN("Failed to load parameter noise_gyro_bias");
    else
        noiseParams.process_noise.qwo = tmp_scalar;
    if (!nh_.getParam("noise_acc_bias", tmp_scalar))
        ROS_WARN("Failed to load parameter noise_acc_bias");
    else
        noiseParams.process_noise.qao = tmp_scalar;
    if (!nh_.getParam("noise_R_ci", tmp_scalar))
        ROS_WARN("Failed to load parameter noise_R_ci");
    else
        noiseParams.process_noise.qR_ci = tmp_scalar;
    if (!nh_.getParam("noise_inv_depth_initial_unc", tmp_scalar))
        ROS_WARN("Failed to load parameter noise_inv_depth_initial_unc");
    else
        noiseParams.inv_depth_initial_unc = tmp_scalar;
    if (!nh_.getParam("noise_image", tmp_scalar))
        ROS_WARN("Failed to load parameter noise_image");
    else
        noiseParams.image_noise = tmp_scalar;

    if (nh_.getParam("noise_gyro_bias_initial_unc", tmp_vector)) {
        for (int i = 0; i < tmp_vector.size(); i++)
            noiseParams.gyro_bias_initial_unc[i] = tmp_vector[i];
    } else {
        ROS_WARN("Failed to load parameter noise_gyro_bias_initial_unc");
    }
    if (nh_.getParam("noise_acc_bias_initial_unc", tmp_vector)) {
        for (int i = 0; i < tmp_vector.size(); i++)
            noiseParams.acc_bias_initial_unc[i] = tmp_vector[i];
    } else {
        ROS_WARN("Failed to load parameter noise_acc_bias_initial_unc");
    }

    if (!nh_.getParam("vio_max_ekf_iterations", vioParams.max_ekf_iterations))
        ROS_WARN("Failed to load parameter vio_max_ekf_iterations");

    bool tmp_bool;
    if (!nh_.getParam("vio_delayed_initiazation", tmp_bool)) {
        ROS_WARN("Failed to load parameter vio_delayed_initiazation");
        vioParams.delayed_initialization = false;
    } else {
        vioParams.delayed_initialization = tmp_bool;
    }
    if (!nh_.getParam("vio_mono", tmp_bool)) {
        ROS_WARN("Failed to load parameter vio_mono");
        vioParams.mono = false;
    } else {
        vioParams.mono = tmp_bool;
    }
    if (!nh_.getParam("vio_fixed_feature", tmp_bool)) {
        ROS_WARN("Failed to load parameter vio_fixed_feature");
        vioParams.fixed_feature = false;
    } else {
        vioParams.fixed_feature = tmp_bool;
    }
    if (!nh_.getParam("vio_RANSAC", tmp_bool)) {
        ROS_WARN("Failed to load parameter vio_RANSAC");
        vioParams.RANSAC = false;
    } else {
        vioParams.RANSAC = tmp_bool;
    }
    if (!nh_.getParam("vio_full_stereo", tmp_bool)) {
        ROS_WARN("Failed to load parameter vio_full_stereo");
        vioParams.full_stereo = false;
    } else {
        vioParams.full_stereo = tmp_bool;
    }

    if (!nh_.getParam("cam_FPS", fps))
        ROS_WARN("Failed to load parameter cam_FPS");
    if (!nh_.getParam("cam_vision_subsample", vision_subsample))
        ROS_WARN("Failed to load parameter cam_vision_subsample");
    if (vision_subsample < 1) {
        auto_subsample = true;
        ROS_INFO("Auto subsamlple: Using every VIO message with images to update, others to predict");
    }

    double visualization_freq;
    if (!nh_.getParam("visualization_freq", visualization_freq))
        ROS_WARN("Failed to load parameter visualization_freq");
    vis_publish_delay = fps / vision_subsample / visualization_freq;
    vis_publish_delay = !vis_publish_delay ? 1 : vis_publish_delay;
    if (!nh_.getParam("show_camera_image", show_camera_image_))
        ROS_WARN("Failed to load parameter show_camera_image");
    if (!nh_.getParam("image_visualization_delay", image_visualization_delay))
        ROS_WARN("Failed to load parameter image_visualization_delay");
    image_visualization_delay = !image_visualization_delay ? 1 : image_visualization_delay;

    // try to load a custom camera calibration file if the launch parameter was set
    std::string calibration_path;
    if (nh_.getParam("camera_calibration", calibration_path)) {
        ROS_INFO("Got custom camera calibration path %s", calibration_path.c_str());
        loadCustomCameraCalibration(calibration_path);
    }

    double imu_smoothing_factor = 1.0;  // default to no smoothing
    nh_.getParam("imu_smoothing_factor", imu_smoothing_factor);
    imulp_.setSmoothingFactor(imu_smoothing_factor);

    dynamic_reconfigure::Server<duo_vio::duo_vioConfig>::CallbackType f = boost::bind(&DuoVio::dynamicReconfigureCb, this, _1, _2);
    dynamic_reconfigure_server.setCallback(f);

    update_vec_.assign(matlab_consts::numTrackFeatures, 0);
    map.resize(matlab_consts::numTrackFeatures * 3);
    anchor_poses.resize(matlab_consts::numAnchors);

    // publishers to check timings
    timing_SLAM_pub = nh_.advertise<std_msgs::Float32>("timing_SLAM", 10);
    timing_feature_tracking_pub = nh_.advertise<std_msgs::Float32>("timing_feature_tracking", 10);
    timing_total_pub = nh_.advertise<std_msgs::Float32>("timing_total", 10);

    body_tf.setOrigin(tf::Vector3(0.0, 0.0, 0.0));

}

DuoVio::~DuoVio() {

    printf("Longest update duration: %.3f msec, %.3f Hz\n", float(max_clicks_) / CLOCKS_PER_SEC * 1000, CLOCKS_PER_SEC / float(max_clicks_));

    printf("Last position: %f %f %f\n", robot_state.pos[0], robot_state.pos[1], robot_state.pos[2]);
    printf("Trajectory length: %f\n", dist);

    // write the estimated biases to file
    if (got_device_serial_nr && !device_serial_nr.empty()) {
        std::string file_path = ros::package::getPath("duo3d_ros") + "/calib/" + device_serial_nr + "/last_bias_estimate.yaml";
        YAML::Node node;  // starts out as null
        node["acc_bias"].push_back(robot_state.IMU.acc_bias[0]);
        node["acc_bias"].push_back(robot_state.IMU.acc_bias[1]);
        node["acc_bias"].push_back(robot_state.IMU.acc_bias[2]);

        node["gyro_bias"].push_back(robot_state.IMU.gyro_bias[0]);
        node["gyro_bias"].push_back(robot_state.IMU.gyro_bias[1]);
        node["gyro_bias"].push_back(robot_state.IMU.gyro_bias[2]);
        std::ofstream fout(file_path.c_str());
        fout << node;  // dump it into the file
        printf("Writing estimated IMU biases to %s\n", file_path.c_str());
    }
}


void DuoVio::leftImageMsgCb(const sensor_msgs::Image &msg) {
//Receive left camera image
new_left=1;
}

void DuoVio::rightImageMsgCb(const sensor_msgs::Image &msg) {
//Receive right camera image
new_right=1;
}

void DuoVio::ImageSetCb(const ImageConstPtr& msg_img_l, const ImageConstPtr& msg_img_r){
    printf("Synchronized image!!\n", );
}

void DuoVio::imuCb(const sensor_msgs::Imu &msg){
//Receive IMU messages do Prediction and do Update if images are available
if (new_left==1 && new_right==1)
  {
    //Check for timestamps
    printf("New left and right image\n");
    new_right=0;
    new_left=0;
  }
}


void DuoVio::vioSensorMsgCb(const ait_ros_messages::VioSensorMsg& msg) {
    if (!got_device_serial_nr)
        return;
    ros::Time tic_total = ros::Time::now();

    bool reset = false;
    // upon reset, catch up with the sensor messages before resetting SLAM
    if (SLAM_reset_flag) {
        if (clear_queue_counter < VIO_SENSOR_QUEUE_SIZE) {
            clear_queue_counter++;
            std_msgs::UInt32 id_msg;
            id_msg.data = msg.header.seq;
            vio_sensor_processed_pub.publish(msg.seq);
            return;
        } else {
            clear_queue_counter = 0;
            SLAM_reset_flag = false;
            std_msgs::UInt32 id_msg;
            id_msg.data = msg.header.seq;
            vio_sensor_processed_pub.publish(msg.seq);
            reset = true;
            vio_vis_reset_pub.publish(std_msgs::Empty());
        }
    }

    clock_t tic_total_clock = clock();
    double dt;
    // Init time on first call
    if (prev_time_.isZero()) {
        prev_time_ = msg.header.stamp;
        dt = vision_subsample / fps;
    } else {
        dt = (msg.header.stamp - prev_time_).toSec();
        if (dt < 0) {
            ROS_ERROR("Negative time difference: %f", dt);
//          dt = 1/fps;
            prev_time_ = msg.header.stamp;
            return;
        }
        if (std::abs(dt - 1 / fps) > 10 / fps)
            ROS_WARN("Jitter! dt: %f", dt);
        if (dt > 100 / fps)
            dt = 1 / fps;  // sometimes dt is huge, probably a camera driver issue
        prev_time_ = msg.header.stamp;
    }

    bool vis_publish = (vio_cnt % vis_publish_delay) == 0;

    update(dt, msg, vis_publish, show_camera_image_, reset);

    clock_t toc_total_clock = clock();

    if (toc_total_clock - tic_total_clock > max_clicks_)
        max_clicks_ = toc_total_clock - tic_total_clock;

    vio_sensor_processed_pub.publish(msg.seq);

    double duration_total = (ros::Time::now() - tic_total).toSec();
    std_msgs::Float32 duration_total_msg;
    duration_total_msg.data = duration_total;
    timing_total_pub.publish(duration_total_msg);

    if (duration_total > vision_subsample / fps)
        ROS_WARN_THROTTLE(1.0, "Duration: %.3f ms. Theoretical max frequency: %.3f Hz", duration_total * 1000, 1 / duration_total);
}

void DuoVio::deviceSerialNrCb(const std_msgs::String &msg) {
    if (got_device_serial_nr) {
        ROS_WARN("Got device serial nr but already have one. Ignoring.");
        return;
    }
    device_serial_nr = msg.data;
    got_device_serial_nr = true;

    ROS_INFO("Got device serial nr %s", device_serial_nr.c_str());
    std::string lense_type;
    nh_.param<std::string>("cam_lense_type", lense_type, "NoType");
    int resolution_width;
    nh_.param<int>("cam_resolution_width", resolution_width, 0);
    int resolution_height;
    nh_.param<int>("cam_resolution_height", resolution_height, 0);

    std::stringstream res;
    res << resolution_height << "x" << resolution_width;
    std::string calib_path = ros::package::getPath("duo3d_ros") + "/calib/" + device_serial_nr + "/" + lense_type + "/" + res.str() + "/cameraParams.yaml";

    ROS_INFO("Reading camera calibration from %s", calib_path.c_str());

    try {
        YAML::Node YamlNode = YAML::LoadFile(calib_path);
        if (YamlNode.IsNull()) {
            ROS_FATAL("Failed to open camera calibration %s", calib_path.c_str());
            exit(-1);
        }
        cameraParams = parseYaml(YamlNode);
        vio.setParams(cameraParams, noiseParams, vioParams);
    } catch (YAML::BadFile &e) {
        ROS_FATAL("Failed to open camera calibration %s\nException: %s", calib_path.c_str(), e.what());
        exit(-1);
    }

    std::string dark_current_l_path = ros::package::getPath("duo3d_ros") + "/calib/" + device_serial_nr + "/" + lense_type + "/" + res.str()
            + "/dark_current_r.bmp";
    darkCurrentL = cv::imread(dark_current_l_path, CV_LOAD_IMAGE_GRAYSCALE);

    if (!darkCurrentL.data) {
        ROS_WARN("Failed to open left dark current image %s!", dark_current_l_path.c_str());
        use_dark_current = false;
    } else if (darkCurrentL.rows != resolution_height || darkCurrentL.cols != resolution_width) {
        ROS_WARN("Left dark current image has the wrong dimensions %s!", dark_current_l_path.c_str());
        use_dark_current = false;
    }

    if (use_dark_current) {
        std::string dark_current_r_path = ros::package::getPath("duo3d_ros") + "/calib/" + device_serial_nr + "/" + lense_type + "/" + res.str()
                + "/dark_current_r.bmp";
        darkCurrentR = cv::imread(dark_current_r_path, CV_LOAD_IMAGE_GRAYSCALE);
        if (!darkCurrentR.data) {
            ROS_WARN("Failed to open right dark current image %s!", dark_current_r_path.c_str());
            use_dark_current = false;
        } else if (darkCurrentR.rows != resolution_height || darkCurrentR.cols != resolution_width) {
            ROS_WARN("Right dark current image has the wrong dimensions %s!", dark_current_r_path.c_str());
            use_dark_current = false;
        }
    }
}

void DuoVio::loadCustomCameraCalibration(const std::string calib_path) {
    // load a camera calibration defined in the launch script
    got_device_serial_nr = true;
    try {
        YAML::Node YamlNode = YAML::LoadFile(calib_path);
        if (YamlNode.IsNull()) {
            ROS_FATAL("Failed to open camera calibration %s", calib_path.c_str());
            exit(-1);
        }
        cameraParams = parseYaml(YamlNode);
        vio.setParams(cameraParams, noiseParams, vioParams);
    } catch (YAML::BadFile &e) {
        ROS_FATAL("Failed to open camera calibration %s\nException: %s", calib_path.c_str(), e.what());
        exit(-1);
    }
}

void DuoVio::dynamicReconfigureCb(duo_vio::duo_vioConfig &config, uint32_t level) {
    noiseParams.image_noise = config.noise_image;
    noiseParams.process_noise.qv = config.noise_acc;
    noiseParams.process_noise.qw = config.noise_gyro;
    noiseParams.process_noise.qwo = config.noise_gyro_bias;
    noiseParams.process_noise.qao = config.noise_acc_bias;
    noiseParams.inv_depth_initial_unc = config.noise_inv_depth_initial_unc;

    vioParams.fixed_feature = config.vio_fixed_feature;
    vioParams.max_ekf_iterations = config.vio_max_ekf_iterations;
    vioParams.delayed_initialization = config.vio_delayed_initialization;
    vioParams.mono = config.vio_mono;
    vioParams.RANSAC = config.vio_RANSAC;
}

void DuoVio::resetCb(const std_msgs::Empty &msg) {
    ROS_WARN("Got reset command");
    SLAM_reset_flag = true;

    ROS_INFO("Longest update duration: %.3f msec, %.3f Hz\n", float(max_clicks_)/CLOCKS_PER_SEC, CLOCKS_PER_SEC/float(max_clicks_));

    ROS_INFO("Last position: %f %f %f\n", robot_state.pos[0], robot_state.pos[1], robot_state.pos[2]);
    ROS_INFO("Trajectory length: %f\n", dist);
    dist = 0;
}

void DuoVio::update(double dt, const ait_ros_messages::VioSensorMsg &msg, bool update_vis, bool show_image, bool reset) {
    std::vector<FloatType> z_all_l(matlab_consts::numTrackFeatures * 2, 0.0);
    std::vector<FloatType> z_all_r(matlab_consts::numTrackFeatures * 2, 0.0);
    std::vector<cv::Point2f> features_l(matlab_consts::numTrackFeatures);
    std::vector<cv::Point2f> features_r(matlab_consts::numTrackFeatures);
    std::vector<FloatType> delayedStatus(matlab_consts::numTrackFeatures);

    //*********************************************************************
    // SLAM prediction
    //*********************************************************************
    ros::Time tic_SLAM = ros::Time::now();

    VIOMeasurements meas;

    if (reset)
        vio.reset();

    for (int i = 0; i < msg.imu.size(); i++) {
        getIMUData(msg.imu[i], meas);  // write the IMU data into the appropriate struct
        imulp_.put(meas);  // filter the IMU data
        imulp_.get(meas);
        vio.predict(meas, dt / msg.imu.size());
    }

    sensor_msgs::Imu smoothed;
    smoothed.header = msg.header;
    smoothed.linear_acceleration.x = meas.acc[0];
    smoothed.linear_acceleration.y = meas.acc[1];
    smoothed.linear_acceleration.z = meas.acc[2];
    smoothed.angular_velocity.x = meas.gyr[0];
    smoothed.angular_velocity.y = meas.gyr[1];
    smoothed.angular_velocity.z = meas.gyr[2];

    smoothed_imu_pub.publish(smoothed);

    if ((auto_subsample || vio_cnt % vision_subsample == 0) && !msg.left_image.data.empty() && !msg.right_image.data.empty()) {
        cv_bridge::CvImagePtr left_image;
        cv_bridge::CvImagePtr right_image;
        try {
            left_image = cv_bridge::toCvCopy(msg.left_image, "mono8");
            right_image = cv_bridge::toCvCopy(msg.right_image, "mono8");
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Error while converting ROS image to OpenCV: %s", e.what());
            return;
        }

        //*********************************************************************
        // Point tracking
        //*********************************************************************

        ros::Time tic_feature_tracking = ros::Time::now();

        cv::Mat left, right;
        if (use_dark_current) {
            left = left_image->image - darkCurrentL;
            right = right_image->image - darkCurrentR;
        } else {
            left = left_image->image;
            right = right_image->image;
        }

        trackFeatures(left, right, features_l, features_r, update_vec_, 1+vioParams.full_stereo);

        for (int i = 0; i < features_l.size(); i++) {
            z_all_l[2*i + 0] = features_l[i].x;
            z_all_l[2*i + 1] = features_l[i].y;

            z_all_r[2*i + 0] = features_r[i].x;
            z_all_r[2*i + 1] = features_r[i].y;
        }

        double duration_feature_tracking = (ros::Time::now() - tic_feature_tracking).toSec();
        std_msgs::Float32 duration_feature_tracking_msg;
        duration_feature_tracking_msg.data = duration_feature_tracking;
        timing_feature_tracking_pub.publish(duration_feature_tracking_msg);

        //*********************************************************************
        // SLAM update
        //*********************************************************************
        vio.update(update_vec_, z_all_l, z_all_r, robot_state, map, anchor_poses, delayedStatus);

        camera_tf.setOrigin(tf::Vector3(robot_state.pos[0], robot_state.pos[1], robot_state.pos[2]));
        camera_tf.setRotation(tf::Quaternion(robot_state.att[0], robot_state.att[1], robot_state.att[2], robot_state.att[3]));
        tf_broadcaster.sendTransform(tf::StampedTransform(camera_tf, ros::Time::now(), "world", "camera"));

        body_tf.setRotation(cam2body);
        tf_broadcaster.sendTransform(tf::StampedTransform(body_tf, ros::Time::now(), "camera", "body"));

        geometry_msgs::Pose pose;
        pose.position.x = robot_state.pos[0];
        pose.position.y = robot_state.pos[1];
        pose.position.z = robot_state.pos[2];
        pose.orientation.x = robot_state.att[0];
        pose.orientation.y = robot_state.att[1];
        pose.orientation.z = robot_state.att[2];
        pose.orientation.w = robot_state.att[3];
        pose_pub.publish(pose);

        geometry_msgs::Vector3 vel;
        vel.x = robot_state.vel[0];
        vel.y = robot_state.vel[1];
        vel.z = robot_state.vel[2];
        vel_pub.publish(vel);

        double duration_SLAM = (ros::Time::now() - tic_SLAM).toSec() - duration_feature_tracking;
        std_msgs::Float32 duration_SLAM_msg;
        duration_SLAM_msg.data = duration_SLAM;
        timing_SLAM_pub.publish(duration_SLAM_msg);

        dist += sqrt(
                (robot_state.pos[0] - last_pos[0]) * (robot_state.pos[0] - last_pos[0])
                        + (robot_state.pos[1] - last_pos[1]) * (robot_state.pos[1] - last_pos[1])
                        + (robot_state.pos[2] - last_pos[2]) * (robot_state.pos[2] - last_pos[2]));

        last_pos[0] = robot_state.pos[0];
        last_pos[1] = robot_state.pos[1];
        last_pos[2] = robot_state.pos[2];

        if (update_vis) {
            show_image = show_image && (display_tracks_cnt % image_visualization_delay == 0);
            display_tracks_cnt++;

            updateVis(robot_state, anchor_poses, map, update_vec_, msg, z_all_l, show_image);
        }
    } else {
        double duration_SLAM = (ros::Time::now() - tic_SLAM).toSec();
        std_msgs::Float32 duration_SLAM_msg;
        duration_SLAM_msg.data = duration_SLAM;
        timing_SLAM_pub.publish(duration_SLAM_msg);
    }
    vio_cnt++;
}

void DuoVio::getIMUData(const sensor_msgs::Imu& imu, VIOMeasurements& meas) {
    meas.acc[0] = imu.linear_acceleration.x;
    meas.acc[1] = imu.linear_acceleration.y;
    meas.acc[2] = imu.linear_acceleration.z;

    meas.gyr[0] = imu.angular_velocity.x;
    meas.gyr[1] = imu.angular_velocity.y;
    meas.gyr[2] = imu.angular_velocity.z;
}

void DuoVio::updateVis(RobotState &robot_state, std::vector<AnchorPose> &anchor_poses, std::vector<FloatType> &map, std::vector<int> &updateVect,
        const ait_ros_messages::VioSensorMsg &sensor_msg, std::vector<FloatType> &z_l, bool show_image) {
    ait_ros_messages::vio_vis msg;

    msg.robot_pose.position.x = robot_state.pos[0];
    msg.robot_pose.position.y = robot_state.pos[1];
    msg.robot_pose.position.z = robot_state.pos[2];

    msg.robot_pose.orientation.x = robot_state.att[0];
    msg.robot_pose.orientation.y = robot_state.att[1];
    msg.robot_pose.orientation.z = robot_state.att[2];
    msg.robot_pose.orientation.w = robot_state.att[3];

    for (int i = 0; i < matlab_consts::numAnchors; i++) {
        geometry_msgs::Pose pose;
        pose.position.x = anchor_poses[i].pos[0];
        pose.position.y = anchor_poses[i].pos[1];
        pose.position.z = anchor_poses[i].pos[2];

        pose.orientation.x = anchor_poses[i].att[0];
        pose.orientation.y = anchor_poses[i].att[1];
        pose.orientation.z = anchor_poses[i].att[2];
        pose.orientation.w = anchor_poses[i].att[3];

        msg.anchor_poses.poses.push_back(pose);
    }

    for (int i = 0; i < matlab_consts::numTrackFeatures; i++) {
        msg.map.data.push_back(map[i * 3 + 0]);
        msg.map.data.push_back(map[i * 3 + 1]);
        msg.map.data.push_back(map[i * 3 + 2]);

        msg.status_vect.data.push_back(updateVect[i]);
        if (updateVect[i] == 1) {
            msg.feature_tracks.data.push_back(z_l[i * 2 + 0]);
            msg.feature_tracks.data.push_back(z_l[i * 2 + 1]);
        } else {
            msg.feature_tracks.data.push_back(-100);
            msg.feature_tracks.data.push_back(-100);
        }
    }

    if (show_image)
        msg.image = sensor_msg.left_image;

    msg.gyro_bias.data.push_back(robot_state.IMU.gyro_bias[0]);
    msg.gyro_bias.data.push_back(robot_state.IMU.gyro_bias[1]);
    msg.gyro_bias.data.push_back(robot_state.IMU.gyro_bias[2]);

    msg.acc_bias.data.push_back(robot_state.IMU.acc_bias[0]);
    msg.acc_bias.data.push_back(robot_state.IMU.acc_bias[1]);
    msg.acc_bias.data.push_back(robot_state.IMU.acc_bias[2]);

    vio_vis_pub.publish(msg);
}
