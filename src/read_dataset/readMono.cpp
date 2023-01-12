#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include <vector>
#include <deque>
#include <sstream>
#include <fstream>
#include "vlp_msgs/VLP.h"

using namespace std;
using namespace Eigen;

#define SEQ_END 9999999999
#define SENSOR_SLEEP_TIME 15 //7 //used to control the sleep time between two loop
#define NS2S 0.000000001

std::string SEASON_START_TIME;

struct IMUData {
    string timestamp;
    double ax;
    double ay;
    double az;
    double wx;
    double wy;
    double wz;
};

struct VLPData {
    string timestamp;
    double x;
    double y;
    double z;
};

deque<IMUData> imuDataDeq;
deque<VLPData> vlpDataDeq;

enum eSensorType{CAM, IMU, VLP_GPS};
int main(int argc, char** argv) {
    // start-time
    auto start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&start_time), "%FT%T");
    SEASON_START_TIME = ss.str();
    cout << "Start Time: " << SEASON_START_TIME << "\n\n";

    // node name: read_mono
    ros::init(argc, argv, "read_mono");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // Publisher
    ros::Publisher pubImage = nh.advertise<sensor_msgs::Image>("/camera/image_raw",1000);
    ros::Publisher pubImu = nh.advertise<sensor_msgs::Imu>("/imu",1000);
    ros::Publisher pubVLP = nh.advertise<vlp_msgs::VLP>("/vlp",1000);

    if(argc != 3) {
        cout << "Wrong Usage!\n";
        cout << "rosrun ai_robot_lcsfl read_mono /home/linux/KITTI/Blender_1600 noise";
        return 1;
    }
    
    string sequence = argv[1];
    string dataPath = sequence + "/";
    cout << "read sequence: "<< dataPath << endl;

    string oxtsType = argv[2];
    printf("oxts type: %s\n", argv[2]);

    /*
    string sequence = "/home/linux/KITTI/Blender_1600";
    string dataPath = sequence + "/";
    string oxtsType = "noise";
    cout << dataPath << endl;
    */

    vector<pair<double, eSensorType>> pairTimeTypeVec;
    {
        // load image and store timestamp in imageTimeVec
        vector<double> imageTimeVec;
        {
            string pathImg = dataPath + "image/timestamps.txt";
            ifstream fileImg(pathImg, ios::in);
            if(fileImg.is_open()) {
                string line;

                while(getline(fileImg, line)) {
                    istringstream str(line);

                    string str_timestamp;
                    while(str >> str_timestamp) {
                        imageTimeVec.emplace_back(stold(str_timestamp.c_str()));
                    }
                }
            } else {
                cout << "cannot find file "  << pathImg << endl;
                ROS_BREAK();
                return 0;
            }
            
            fileImg.close();
        }
        
        // load imu and store timestamp in imuTimeVec
        vector<double> imuTimeVec;
        {
            string pathImu = dataPath + "oxts-" + oxtsType + "/imu/IMU_Acc_Gyro.txt";
            ifstream fileImu(pathImu, ios::in);
            if(fileImu.is_open()) {
                string line;

                while(getline(fileImu, line)) {
                    istringstream str(line);

                    string str_timestamp;
                    string str_ax;
                    string str_ay;
                    string str_az;
                    string str_wx;
                    string str_wy;
                    string str_wz;

                    while(str >> str_timestamp >> str_ax >> str_ay >> str_az >> str_wx >> str_wy >> str_wz) {
                        double t = stold(str_timestamp.c_str()) * NS2S;
                        imuTimeVec.emplace_back(t);

                        IMUData imu_data;
                        imu_data.timestamp = to_string(stold(str_timestamp.c_str()) * NS2S);
                        imu_data.ax = stold(str_ax.c_str());
                        imu_data.ay = stold(str_ay.c_str());
                        imu_data.az = stold(str_az.c_str());
                        imu_data.wx = stold(str_wx.c_str());
                        imu_data.wy = stold(str_wy.c_str());
                        imu_data.wz = stold(str_wz.c_str());
                        imuDataDeq.push_back(imu_data);
                    }
                }
            } else {
                cout << "cannot find file "  << pathImu << endl;
                ROS_BREAK();
                return 0;
            }
            
            fileImu.close();
        }

        // load vlp and store timestamp in vlpTimeVec
        vector<double> vlpTimeVec;
        {
            string pathVLP = dataPath + "oxts-" + oxtsType + "/vlp/position-VLP.txt";
            ifstream fileVLP(pathVLP, ios::in);
            if(fileVLP.is_open()) {
                string line;

                while(getline(fileVLP, line)) {
                    istringstream str(line);

                    string str_timestamp;
                    string str_x;
                    string str_y;
                    string str_z;
                    string str_qx;
                    string str_qy;
                    string str_qz;
                    string str_qw;

                    while(str >> str_timestamp >> str_x >> str_y >> str_z >> str_qx >> str_qy >> str_qz >> str_qw) {
                        vlpTimeVec.emplace_back(stold(str_timestamp.c_str()));

                        VLPData vlp_data;
                        vlp_data.timestamp = str_timestamp;
                        vlp_data.x = stold(str_x.c_str());
                        vlp_data.y = stold(str_y.c_str());
                        vlp_data.z = stold(str_z.c_str());
                        vlpDataDeq.push_back(vlp_data);
                    }
                }
            } else {
                cout << "cannot find file "  << pathVLP << endl;
                ROS_BREAK();
                return 0;
            }
            
            fileVLP.close();
        }

        // timestamp fusion
        auto imgItr(imageTimeVec.begin());
        auto imuItr(imuTimeVec.begin());
        auto vlpItr(vlpTimeVec.begin());
        while((imgItr != imageTimeVec.end() || vlpItr != vlpTimeVec.end()) && imuItr != imuTimeVec.end()) {
            if(imgItr != imageTimeVec.end() && *imgItr < *imuItr) {
                pairTimeTypeVec.emplace_back(make_pair(*imgItr++, CAM));
            }
            else if(vlpItr != vlpTimeVec.end() && *vlpItr < *imuItr) {
                pairTimeTypeVec.emplace_back(make_pair(*vlpItr++, VLP_GPS));
            }
            else if(imuItr != imuTimeVec.end()) {
                pairTimeTypeVec.emplace_back(make_pair(*imuItr++, IMU));
            }
        }
    }

    // check timelist
    string pathTL = "/home/linux/MSF_developed/src/MSF_developed/LOG/TimeList.txt";
    ofstream fileTL(pathTL, ios::out);
    if(fileTL.is_open()) {
        for(auto elem : pairTimeTypeVec) {
            fileTL << to_string(elem.first) << ' ' << elem.second << endl;
        }
    }

    string ImagePath;
    cv::Mat img;

    const int imu_begin_idx(0);
    const int imu_end_idx(SEQ_END);

    const int img_begin_idx(0);
    const int img_end_idx(SEQ_END);

    const int vlp_begin_idx(0);

    int imageIdx = 1;
    int imuIdx = 0;
    int vlpIdx = 0;
    int first_img_idx = -1;

    for(auto itPairTimeType = pairTimeTypeVec.begin(); itPairTimeType != pairTimeTypeVec.end(); ++itPairTimeType) {
        if(ros::ok()) {
            if(itPairTimeType->second == CAM ) {
                // load img
                stringstream ss;
                ss << setfill('0') << setw(5) << imageIdx;
                if(imageIdx > img_begin_idx
                    && (next(itPairTimeType) == pairTimeTypeVec.end() || next(itPairTimeType)->second != CAM)) {
                    if (first_img_idx == -1) first_img_idx = imageIdx - 1;
                    
                    ImagePath = dataPath + "image/data/" + ss.str() + ".png";
                    img = cv::imread(ImagePath, CV_LOAD_IMAGE_GRAYSCALE);
                    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
                    imgMsg->header.stamp = ros::Time(itPairTimeType->first);

                    pubImage.publish(imgMsg);

                    ROS_INFO("Input Image, time:%f, seq:%d", itPairTimeType->first, imageIdx);
                    ++imageIdx;
                }
            }
            else if(itPairTimeType->second == VLP_GPS) {
                // use vlpDataDeq
                if(vlpIdx >= vlp_begin_idx
                    && (next(itPairTimeType) == pairTimeTypeVec.end() || next(itPairTimeType)->second != VLP_GPS)) {
                        vlp_msgs::VLP vlp_position;

                        VLPData vlp_elem = vlpDataDeq.front();
                        vlp_position.timestamp = vlp_elem.timestamp;
                        vlp_position.position_x = vlp_elem.x;
                        vlp_position.position_y = vlp_elem.y;
                        vlp_position.position_z = vlp_elem.z;
                        vlpDataDeq.pop_front();

                        pubVLP.publish(vlp_position);
                        ROS_INFO("Input VLP, time:%f, seq:%d", itPairTimeType->first, vlpIdx++);
                        cout << "VLP details: " << vlp_elem.timestamp << ' ' << vlp_elem.x << ' ' << vlp_elem.y << ' ' << vlp_elem.z << endl;
                    }
            }
            else if(itPairTimeType->second == IMU) {
                if(imuIdx > imu_end_idx || imageIdx > img_end_idx) break;

                if(first_img_idx >= -1) {
                    // use imuDataDeq
                    double ax, ay, az;
                    double wx, wy, wz;
                    string ts;
                    IMUData imu_elem = imuDataDeq.front();
                    imuDataDeq.pop_front();
                    ts = imu_elem.timestamp;
                    ax = imu_elem.ax;
                    ay = imu_elem.ay;
                    az = imu_elem.az;
                    wx = imu_elem.wx;
                    wy = imu_elem.wy;
                    wz = imu_elem.wz;

                    Vector3d linearAcceleration(ax, ay, az);
                    Vector3d angularVelocity(wx, wy, wz);
                    sensor_msgs::Imu msgImu;
                    msgImu.header.frame_id = "body";
                    msgImu.header.stamp = ros::Time(stold(ts.c_str()));
                    msgImu.linear_acceleration.x = ax;
                    msgImu.linear_acceleration.y = ay;
                    msgImu.linear_acceleration.z = az;
                    msgImu.angular_velocity.x = wx;
                    msgImu.angular_velocity.y = wy;
                    msgImu.angular_velocity.z = wz;

                    pubImu.publish(msgImu);
                    ROS_INFO("Input IMU, time:%f, seq:%d", itPairTimeType->first, imuIdx++);
                    cout << "ax ay az: " << ax << ' ' << ay << ' ' << az << endl;
                    cout << "wx wy wz: " << wx << ' ' << wy << ' ' << wz << endl;
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(SENSOR_SLEEP_TIME));
            }
        }
        else break;
    }

    ros::spin();

    return 0;
}