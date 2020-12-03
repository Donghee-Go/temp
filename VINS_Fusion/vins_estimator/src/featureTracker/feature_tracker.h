/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "../estimator/parameters.h"
#include "../utility/tic_toc.h"

typedef struct FeaturePoints {
public:
	std::vector<int> ids;
	std::vector<int> track_cnt;

	std::vector<cv::Point2f> prev_features;
	std::vector<cv::Point2f> curr_features;

	std::vector<cv::Point2f> undistorted_curr_features;

	std::vector<cv::Point2f> velocity;
}FeaturePoints;

class FeatureTracker {
public:
	FeatureTracker();

	std::map<int, Eigen::Matrix<double, 7, 1>> trackFeatures(double &time, const cv::Mat &img);
	cv::Mat getTrackImage();

private:
	void tracking();
	void detecting();
	void setMask();
	void addNewFeatures();
	void undistortPoints();
	void calcFeatureVelocity();
	void drawTrackImage();
	bool inBorder(const cv::Point2f& pt);

	Eigen::Vector3d liftProjective(cv::Point2f& p);
	void distortion(Eigen::Vector2d p_u, Eigen::Vector2d& d_u);

	cv::Mat mask_;

	cv::Mat prev_img_;
	cv::Mat curr_img_;

	cv::Mat track_img;

	FeaturePoints feature_points_;
	std::vector<cv::Point2f> new_features_;

	std::map<int, cv::Point2f> undistorted_prev_features_map_;
	std::map<int, cv::Point2f> prev_left_features_map_;

	double prev_time_;
	double curr_time_;
};

// using namespace std;
// using namespace camodocal;
// using namespace Eigen;

// bool inBorder(const cv::Point2f &pt);
// void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
// void reduceVector(vector<int> &v, vector<uchar> status);

// class FeatureTracker
// {
// public:
//     FeatureTracker();
//     map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());
//     void setMask();
//     void readIntrinsicParameter(const vector<string> &calib_file);
//     void showUndistortion(const string &name);
//     void rejectWithF();
//     void undistortedPoints();
//     vector<cv::Point2f> undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam);
//     vector<cv::Point2f> ptsVelocity(vector<int> &ids, vector<cv::Point2f> &pts, 
//                                     map<int, cv::Point2f> &cur_id_pts, map<int, cv::Point2f> &prev_id_pts);
//     void showTwoImage(const cv::Mat &img1, const cv::Mat &img2, 
//                       vector<cv::Point2f> pts1, vector<cv::Point2f> pts2);
//     void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight, 
//                                    vector<int> &curLeftIds,
//                                    vector<cv::Point2f> &curLeftPts, 
//                                    vector<cv::Point2f> &curRightPts,
//                                    map<int, cv::Point2f> &prevLeftPtsMap);
//     void setPrediction(map<int, Eigen::Vector3d> &predictPts);
//     double distance(cv::Point2f &pt1, cv::Point2f &pt2);
//     void removeOutliers(set<int> &removePtsIds);
//     cv::Mat getTrackImage();
//     bool inBorder(const cv::Point2f &pt);

//     int row, col;
//     cv::Mat imTrack;
//     cv::Mat mask;
//     cv::Mat fisheye_mask;
//     cv::Mat prev_img, cur_img;
//     vector<cv::Point2f> n_pts;
//     vector<cv::Point2f> predict_pts;
//     vector<cv::Point2f> predict_pts_debug;
//     vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts;
//     vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;
//     vector<cv::Point2f> pts_velocity, right_pts_velocity;
//     vector<int> ids, ids_right;
//     vector<int> track_cnt;
//     map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map;
//     map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;
//     map<int, cv::Point2f> prevLeftPtsMap;
//     vector<camodocal::CameraPtr> m_camera;
//     double cur_time;
//     double prev_time;
//     bool stereo_cam;
//     int n_id;
//     bool hasPrediction;
// };
