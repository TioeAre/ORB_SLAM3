/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of
 * Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <chrono>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "../../include/System.h"

using namespace std;

class ImageGrabber {
public:
    vector<unsigned long> key_frame_ids;
    uint32_t pub_id = 0;
    ros::NodeHandle nh1;
    ros::Publisher pub_rgb, pub_depth, pub_tcw, pub_camerapath, pub_odom, pub_isLoop;
    nav_msgs::Path camerapath;

    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM), nh1("~") {
        pub_rgb = nh1.advertise<sensor_msgs::Image>("RGB/Image", 1);
        pub_depth = nh1.advertise<sensor_msgs::Image>("Depth/Image", 1);
        pub_tcw = nh1.advertise<geometry_msgs::PoseStamped>("CameraPose", 1);
        pub_odom = nh1.advertise<nav_msgs::Odometry>("Odometry", 1);
        pub_camerapath = nh1.advertise<nav_msgs::Path>("Path", 1);
        pub_isLoop = nh1.advertise<std_msgs::Bool>("isLoop", 1);
    }

    // ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD);

    ORB_SLAM3::System *mpSLAM;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "RGBD");
    ros::start();

    if (argc != 3) {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, true);
    ros::NodeHandle nh;
    ImageGrabber igb(&SLAM);

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub, depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD, &igb, _1, _2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

tf::Pose trans_pose(Sophus::SE3f se3, bool &if_empty) {
    cv::Mat Tcw =
        (cv::Mat_<float>(4, 4) << se3.matrix()(0, 0), se3.matrix()(0, 1), se3.matrix()(0, 2), se3.matrix()(0, 3),
         se3.matrix()(1, 0), se3.matrix()(1, 1), se3.matrix()(1, 2), se3.matrix()(1, 3), se3.matrix()(2, 0),
         se3.matrix()(2, 1), se3.matrix()(2, 2), se3.matrix()(2, 3), 0.0f, 0.0f, 0.0f, 1.0f);
    if_empty = Tcw.empty();
    cv::Mat RWC = Tcw.rowRange(0, 3).colRange(0, 3);
    cv::Mat tWC = Tcw.rowRange(0, 3).col(3);
    tf::Matrix3x3 M(RWC.at<float>(0, 0), RWC.at<float>(0, 1), RWC.at<float>(0, 2), RWC.at<float>(1, 0),
                    RWC.at<float>(1, 1), RWC.at<float>(1, 2), RWC.at<float>(2, 0), RWC.at<float>(2, 1),
                    RWC.at<float>(2, 2));
    tf::Vector3 V(tWC.at<float>(0) / 25, tWC.at<float>(1) / 25, tWC.at<float>(2) / 25);
    tf::Quaternion q;
    M.getRotation(q);
    tf::Pose tf_pose(q, V);
    double roll, pitch, yaw;
    M.getRPY(roll, pitch, yaw);
    if (roll == 0 || pitch == 0 || yaw == 0) if_empty = true;
    return tf_pose;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &msgRGB, const sensor_msgs::ImageConstPtr &msgD) {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    if (cv_ptrRGB->image.empty() || cv_ptrD->image.empty()) return;
    Sophus::SE3f se3 = mpSLAM->TrackRGBD(cv_ptrRGB->image, cv_ptrD->image, cv_ptrRGB->header.stamp.toSec());
    bool if_empty;
    tf::Pose tf_pose = trans_pose(se3, if_empty);
    if (!if_empty) {
        std_msgs::Header header;
        header.stamp = msgRGB->header.stamp;
        header.seq = msgRGB->header.seq;
        header.frame_id = "camera";
        sensor_msgs::Image::ConstPtr rgb_msg = msgRGB;
        sensor_msgs::Image::ConstPtr depth_msg = msgD;

        geometry_msgs::PoseStamped tcw_msg;
        tcw_msg.header = header;
        tf::poseTFToMsg(tf_pose, tcw_msg.pose);

        camerapath.header = header;
        std_msgs::Bool isLoop_msg;
        isLoop_msg.data = mpSLAM->is_loop;
        if (mpSLAM->is_loop) {
            vector<geometry_msgs::PoseStamped> after_loop_poses;
            vector<unsigned long> after_loop_ids;
            for (long i = 0; i < mpSLAM->current_all_KF.size(); i++) {
                for (long j = 0; j < key_frame_ids.size(); j++) {
                    if (key_frame_ids[j] == mpSLAM->current_all_KF[i]->mnId) {
                        tf_pose = trans_pose(mpSLAM->current_all_KF[i]->GetPose(), if_empty);
                        geometry_msgs::PoseStamped tcw_msg1;
                        tf::poseTFToMsg(tf_pose, tcw_msg1.pose);
                        tcw_msg1.header = header;
                        tcw_msg1.header.seq = i;
                        after_loop_poses.push_back(tcw_msg1);
                        after_loop_ids.push_back(key_frame_ids[j]);
                        break;
                    }
                }
            }
            camerapath.poses.swap(after_loop_poses);
            key_frame_ids.swap(after_loop_ids);
        }
        if (bool(isLoop_msg.data)) {
            pub_isLoop.publish(isLoop_msg);
        }
        if (mpSLAM->is_key_frame) {
            key_frame_ids.push_back(mpSLAM->current_KF_mnId);
            pub_camerapath.publish(camerapath);
            pub_tcw.publish(tcw_msg);
            pub_rgb.publish(rgb_msg);
            pub_depth.publish(depth_msg);
        }
    }
}
