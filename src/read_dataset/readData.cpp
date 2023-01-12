#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
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

using namespace std;
using namespace Eigen;

#define NS2S 0.000000001
#define SEQ_END 9999999999
#define SENSOR_SLEEP_TIME 15

std::string SEASON_START_TIME;

struct SLAMData {
    string timestamp;
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;
};

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

deque<SLAMData> slamDeq;
deque<IMUData> imuDataDeq;
deque<VLPData> vlpDataDeq;

enum eSensorType{CAM, IMU, VLP};
int main(int argc, char** argv) {
    // start-time
    auto start_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&start_time), "%FT%T");
    SEASON_START_TIME = ss.str();
    cout << "Start Time: " << SEASON_START_TIME << "\n\n";

    // node name: readSLAM
    ros::init(argc, argv, "readData");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    // Publisher
    ros::Publisher pubSLAM = nh.advertise<geometry_msgs::TransformStamped>("/slam/tf",1000);
    ros::Publisher pubImu = nh.advertise<sensor_msgs::Imu>("/imu",1000);
    // ros::Publisher pubVLP = nh.advertise<vlp_msgs::VLP>("/vlp",1000);
    ros::Publisher pubVLP = nh.advertise<geometry_msgs::TransformStamped>("/vlp",1000);

    if(argc != 3) {
        cout << "Wrong Usage!\n";
        cout << "rosrun ai_robot_lcsfl readData /home/linux/KITTI/Blender_1600 noise";
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
        // load slam-pose and store timestamp in slamTimeVec
        vector<double> slamTimeVec;
        {
            string pathSLAM = dataPath + "slam/slamPose.txt";
            // string pathSLAM = dataPath + "slam/NeuroSLAM.txt";
            ifstream fileSLAM(pathSLAM, ios::in);
            if(fileSLAM.is_open()) {
                string line;

                while(getline(fileSLAM, line)) {
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
                        double t = stold(str_timestamp.c_str());
                        slamTimeVec.emplace_back(t);

                        SLAMData slam_data;
                        slam_data.timestamp = to_string(t);
                        slam_data.x = stold(str_x.c_str());
                        slam_data.y = stold(str_y.c_str());
                        slam_data.z = stold(str_z.c_str());
                        slam_data.qx = stold(str_qx.c_str());
                        slam_data.qy = stold(str_qy.c_str());
                        slam_data.qz = stold(str_qz.c_str());
                        slam_data.qw = stold(str_qw.c_str());
                        slamDeq.push_back(slam_data);
                    }
                }
            } else {
                cout << "cannot find file "  << pathSLAM << endl;
                ROS_BREAK();
                return 0;
            }
            
            fileSLAM.close();
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
                        imu_data.timestamp = to_string(t);
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
        auto slamItr(slamTimeVec.begin());
        auto imuItr(imuTimeVec.begin());
        auto vlpItr(vlpTimeVec.begin());
        while((slamItr != slamTimeVec.end() || vlpItr != vlpTimeVec.end()) && imuItr != imuTimeVec.end()) {
            if(vlpItr != vlpTimeVec.end() && *vlpItr < *imuItr) {
                pairTimeTypeVec.emplace_back(make_pair(*vlpItr++, VLP));
            }
            else if(slamItr != slamTimeVec.end() && *slamItr < *imuItr) {
                pairTimeTypeVec.emplace_back(make_pair(*slamItr++, CAM));
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

    const int imu_begin_idx(100);
    const int imu_end_idx(SEQ_END);

    const int slam_begin_idx(0);
    const int slam_end_idx(SEQ_END);

    const int vlp_begin_idx(0);
    const int vlp_end_idx(SEQ_END);

    int slamIdx = 0;
    int imuIdx = 0;
    int vlpIdx = 0;

    for(auto itPairTimeType = pairTimeTypeVec.begin(); itPairTimeType != pairTimeTypeVec.end(); ++itPairTimeType) {
        if(ros::ok()) {
            if(itPairTimeType->second == CAM ) {
                slamIdx++;
                if(imuIdx > imu_begin_idx &&
                    slamIdx > slam_begin_idx &&
                    (next(itPairTimeType) == pairTimeTypeVec.end() || next(itPairTimeType)->second != CAM)) {
                    // use slamDeq
                    SLAMData slam_elem = slamDeq.front();
                    slamDeq.pop_front();

                    geometry_msgs::TransformStamped msgSLAM;
                    msgSLAM.header.stamp = ros::Time(stold(slam_elem.timestamp.c_str()));
                    msgSLAM.transform.translation.x = slam_elem.x;
                    msgSLAM.transform.translation.y = -slam_elem.y;
                    msgSLAM.transform.translation.z = slam_elem.z;
                    // msgSLAM.transform.rotation.x = slam_elem.qx;
                    // msgSLAM.transform.rotation.y = slam_elem.qy;
                    msgSLAM.transform.rotation.z = slam_elem.qz;
                    msgSLAM.transform.rotation.w = slam_elem.qw;

                    pubSLAM.publish(msgSLAM);
                    ROS_INFO("Input SLAM-Pose, time:%f, seq:%d", itPairTimeType->first, slamIdx);
                    cout << "x y z: " << slam_elem.x << ' ' << slam_elem.y << ' ' << slam_elem.z << endl;
                    // cout << "qx qy qz qw: " << slam_elem.qx << ' ' << slam_elem.qy << ' ' << slam_elem.qz << ' ' << slam_elem.qw << endl;
                    cout << "qz qw: " << slam_elem.qz << ' ' << slam_elem.qw << endl;
                } else {
                    slamDeq.pop_front();
                }
            }
            else if(itPairTimeType->second == VLP) {
                vlpIdx++;
                // use vlpDataDeq
                if(vlpIdx > vlp_begin_idx && vlpIdx < vlp_end_idx) {
                        // vlp_msgs::VLP vlp_position;
                        geometry_msgs::TransformStamped vlp_position;

                        VLPData vlp_elem = vlpDataDeq.front();
                        /*
                        vlp_position.timestamp = vlp_elem.timestamp;
                        vlp_position.position_x = vlp_elem.x;
                        vlp_position.position_y = vlp_elem.y;
                        vlp_position.position_z = vlp_elem.z;
                        */
                        vlp_position.header.stamp = ros::Time(stold(vlp_elem.timestamp.c_str()));
                        vlp_position.transform.translation.x = vlp_elem.x;
                        vlp_position.transform.translation.y = vlp_elem.y;
                        vlpDataDeq.pop_front();

                        pubVLP.publish(vlp_position);
                        ROS_INFO("Input VLP, time:%f, seq:%d", itPairTimeType->first, vlpIdx);
                        // cout << "VLP details: " << vlp_elem.timestamp << ' ' << vlp_elem.x << ' ' << vlp_elem.y << ' ' << vlp_elem.z << endl;
                        cout << "VLP details: " << vlp_elem.timestamp << ' ' << vlp_elem.x << ' ' << vlp_elem.y << endl;
                } else {
                    vlpDataDeq.pop_front();
                }
            }
            else if(itPairTimeType->second == IMU) {
                imuIdx++;
                if(imuIdx > imu_end_idx || slamIdx > slam_end_idx || vlpIdx > vlp_end_idx) break;
                
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
                ROS_INFO("Input IMU, time:%f, seq:%d", itPairTimeType->first, imuIdx);
                cout << "ax ay az: " << ax << ' ' << ay << ' ' << az << endl;
                cout << "wx wy wz: " << wx << ' ' << wy << ' ' << wz << endl;

                std::this_thread::sleep_for(std::chrono::milliseconds(SENSOR_SLEEP_TIME));
            }
        }
        else break;
    }

    ros::spin();

    return 0;
}