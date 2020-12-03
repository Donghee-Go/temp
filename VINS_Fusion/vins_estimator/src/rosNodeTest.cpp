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

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;


void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

std::queue<std::pair<double, cv::Mat>> img_buffer;
std::queue<std::pair<double, cv::Mat>> image_buffer;
std::queue<std::pair<double, Eigen::Vector3d>> acc_buffer;
std::queue<std::pair<double, Eigen::Vector3d>> gyro_buffer;
std::mutex img_m;

std::vector<std::string> csv_read_row(std::istream &file)
{
    std::stringstream ss;
    std::vector<std::string> row;//relying on RVO
 
    while(file.good())
    {
        char c = file.get();
        if (c==',') 
        {
            row.push_back( ss.str() );
            ss.str("");
        }
        else if ((c=='\r' || c=='\n') )
        {
            if(file.peek()=='\n') { file.get(); }
            row.push_back( ss.str() );
            return row;
        }
        else
        {
            ss << c;
        }
    }
	
	return row;
}

void putImage() {
	std::cout << "feature tracker thread start" << std::endl;
	while(1) {
		img_m.lock();
		if (!image_buffer.empty()) {
			estimator.inputImage(image_buffer.front().first, image_buffer.front().second);
			image_buffer.pop();
		}
		img_m.unlock();	
		
		std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
	}
}

void imgPub() {
	std::cout << "image pulisher thread start" << std::endl;
	while (!img_buffer.empty()) {
		img_m.lock();
		image_buffer.push(img_buffer.front());
		img_buffer.pop();
		img_m.unlock();
		std::chrono::milliseconds dura(54);
        std::this_thread::sleep_for(dura);
	}
}

void readImage() {
	std::ifstream file("/home/ubuntu/Downloads/mav0/cam0/data.csv");

	std::vector<std::string> row;
	double time;
	cv::Mat img;
	
	row = csv_read_row(file);
	while (file.good()) {
		row = csv_read_row(file);
		if (!row.empty()) {
			time = std::stod(row[0]) / 1e+9;
			img = cv::imread("/home/ubuntu/Downloads/mav0/cam0/data/" + row[1], cv::IMREAD_GRAYSCALE);
			img_buffer.push(std::make_pair(time, img));
		}
	}
}

void readImu() {
	std::ifstream file("/home/ubuntu/Downloads/mav0/imu0/data.csv");
	if (file.is_open()) {
		double time;
		Eigen::Vector3d acc;
		Eigen::Vector3d gyro;

		std::vector<std::string> row;

		row = csv_read_row(file);
		while (file.good()) {
			row = csv_read_row(file);

			if (!row.empty()) {
				time = stod(row[0]) / 1e+9;

				for (int i = 0; i < 3; ++i) {
					gyro(i) = stod(row[1 + i]);
					acc(i) = stod(row[4 + i]);
				}

				acc_buffer.push(std::make_pair(time, acc));
				gyro_buffer.push(std::make_pair(time, gyro));
			}
		}
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);
    readImage();
	readImu();
    std::cout << "read done" << std::endl;
    

    std::thread image_publisher{imgPub};
	std::thread image_thread{putImage};

    while (!acc_buffer.empty()) {
		estimator.inputIMU(acc_buffer.front().first, acc_buffer.front().second, gyro_buffer.front().second);
		acc_buffer.pop();
		gyro_buffer.pop();
		std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
	}

    image_publisher.join();
	image_thread.join();

    return 0;
}

int main2() {
	Eigen::Matrix3d ric[10];
	FeatureManager f_m{ric};

	std::ifstream file("/home/ubuntu/tracker_test/features.bin", std::ios::binary);

	using Feature = Eigen::Matrix<double, 7, 1>;

   

	if (file.is_open()) {
		for (int k = 0; k < 10; k++) {
             map<int, vector<pair<int, Feature>>> featureFrame;
			size_t size;
			file.read((char*)&size, sizeof(size_t));
			for (size_t j = 0; j < size; j++) {
				int id;
				double value;
				Feature temp_f;
				file.read((char*)&id, sizeof(int));

				for (int i = 0; i < 7; i++) {
					file.read((char*)&value, sizeof(double));
					temp_f(i, 0) = value;
				}
				
                vector<pair<int, Feature>> f;
                f.push_back(make_pair(0, temp_f));
                featureFrame.insert(std::make_pair(id, f));
			}
            f_m.addFeatureCheckParallax(k, featureFrame, 0);
			std::cout << featureFrame.size() << std::endl;
		}
	}

	std::cout << f_m.feature.size() << std::endl;
	
	file.close();



	return 0;
}



int main3(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}
