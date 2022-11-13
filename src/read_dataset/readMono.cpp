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

using namespace std;
using namespace Eigen;

#define SEQ_END 9999999999
#define GPS_DIV_FREQ  100 //used to control the GPS and IMU freq
#define SENSOR_SLEEP_TIME 15 //7 //used to control the sleep time between two loop

Eigen::Matrix3d c1Rc0, c0Rc1;
Eigen::Vector3d c1Tc0, c0Tc1;

std::string SEASON_START_TIME;

enum eSensorType{CAM, IMU, GNSS, IMU_GNSS};
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
    ros::Publisher pubImage = nh.advertise<sensor_msgs::Image>("/camera/rgb/image_raw",1000);
    ros::Publisher pubImu = nh.advertise<sensor_msgs::Imu>("/imu",1000);
    ros::Publisher pubOrignGnss = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix",1000);

    if(argc != 3) {
        cout << "Wrong Usage!\n";
        cout << "rosrun ai_robot_lcsfl read_mono /home/linux/KITTI/RawData/2011_10_03/2011_10_03_drive_0027_sync noised-04";
        return 1;
    }
    
    // dataPath = ".../KITTI_Data/RawData/2011_10_03/2011_10_03_drive_0027_sync"
    string sequence = argv[1];
    string dataPath = sequence + "/";
    cout << "read sequence: "<< dataPath << endl;

    string oxtsType = argv[2];
    printf("oxts type: %s\n", argv[2]);

    list<pair<double, eSensorType>> pairTimeTypeList;
    {
        // load image and store timestamp in imageTimeList
        list<double> imageTimeList;
        {
            FILE* file;
            file = fopen((dataPath + "image_00/timestamps.txt").c_str(), "r");
            if (file == NULL) {
                printf("cannot find file: %simage_00/timestamps.txt \n", dataPath.c_str());
                ROS_BREAK();
                return 0;
            }
            
            int year, month, day;
            int hour, minute;
            double second;
            while (fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF) {
                imageTimeList.emplace_back(hour * 60 * 60 + minute * 60 + second );
            }
            std::fclose(file);
        }

        // load gps-imu and store timestamp in imu_gpsTimeList
        list<double> imu_gpsTimeList;
        {
            FILE* file;
            file = fopen((dataPath + "oxts-" + oxtsType + "/timestamps.txt").c_str(), "r");
            if (file == NULL) {
                printf("cannot find file: %soxts-%s/timestamps.txt \n", dataPath.c_str(),oxtsType.c_str());
                ROS_BREAK();
                return 0;
            }

            int year, month, day;
            int hour, minute;
            double second;
            while(fscanf(file, "%d-%d-%d %d:%d:%lf", &year, &month, &day, &hour, &minute, &second) != EOF) {
                imu_gpsTimeList.emplace_back(hour * 60 * 60 + minute * 60 + second);
            }
            std::fclose(file);
        }

        // timestamp fusion
        auto imgItr(imageTimeList.begin());
        auto imu_gpsItr(imu_gpsTimeList.begin());
        while(imgItr != imageTimeList.end() && imu_gpsItr != imu_gpsTimeList.end()) {
            if(imgItr != imageTimeList.end() && *imgItr < *imu_gpsItr) {
                pairTimeTypeList.emplace_back(make_pair(*imgItr++, CAM));
            }
            else if(imu_gpsItr != imu_gpsTimeList.end()) {
                pairTimeTypeList.emplace_back(make_pair(*imu_gpsItr++, IMU_GNSS));
            }
        }
    }

    // check timelist
    string pathTL = "/home/linux/MSF_developed/src/MSF_developed/LOG/TimeList.txt";
    ofstream fileTL(pathTL, ios::out);
    if(fileTL.is_open()) {
        for(auto elem : pairTimeTypeList) {
            fileTL << to_string(elem.first) << ' ' << elem.second << endl;
        }
    }

    string ImagePath;
    cv::Mat img;

    const int imu_begin_idx(0); // 300
    const int imu_end_idx(SEQ_END);

    const int img_begin_idx(0);
    const int img_end_idx(SEQ_END);

    int imageIdx = 0;
    int imu_gnssIdx = 0;
    int first_img_idx = -1;
    const int gnss_ex_div(GPS_DIV_FREQ);

    for(auto itPairTimeType = pairTimeTypeList.begin(); itPairTimeType != pairTimeTypeList.end(); ++itPairTimeType) {
        if(ros::ok()) {
            if(itPairTimeType->second == CAM ) {
                // load img
                stringstream ss;
                ss << setfill('0') << setw(10) << imageIdx++;
                if(imu_gnssIdx > imu_begin_idx && imageIdx > img_begin_idx
                    && (next(itPairTimeType) == pairTimeTypeList.end() || next(itPairTimeType)->second != CAM)) {
                    cout << "imu_gnssIdx: " << imu_gnssIdx << " imageIdx: " << imageIdx << endl;
                    cout << "first_img_idx: " << first_img_idx << endl;

                    if (first_img_idx == -1) first_img_idx = imageIdx-1;    // the first image idx should be 0 in process
                    
                    ImagePath = dataPath + "image_00/data/" + ss.str() + ".png";
                    cout << "image path: " << ImagePath << endl;
                    img = cv::imread(ImagePath, CV_LOAD_IMAGE_GRAYSCALE);
                    sensor_msgs::ImagePtr imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
                    imgMsg->header.stamp = ros::Time(itPairTimeType->first);

                    pubImage.publish(imgMsg);

                    ROS_INFO("Input Image, time:%f, seq:%d", itPairTimeType->first, imageIdx);
                }
            }
            else if(itPairTimeType->second == IMU_GNSS) {
                stringstream ss;
                ss << setfill('0') << setw(10) << imu_gnssIdx++;

                if(imu_gnssIdx > imu_end_idx || imageIdx > img_end_idx) break;

                if(first_img_idx > -1) {
                    FILE* GPSFile;
                    string GPSFilePath = dataPath + "oxts-"+ oxtsType + "/data/" + ss.str() + ".txt";
                    GPSFile = fopen(GPSFilePath.c_str(), "r");
                    if(GPSFile == NULL) {
                        printf("cannot find file: %s\n", GPSFilePath.c_str());
                        ROS_BREAK();
                        return 0;
                    }

                    double lat, lon, alt, roll, pitch, yaw;
                    double vn, ve, vf, vl, vu;
                    double ax, ay, az, af, al, au;
                    double wx, wy, wz, wf, wl, wu;
                    double pos_accuracy, vel_accuracy;
                    double navstat, numsats;
                    double velmode, orimode;

                    fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &lat, &lon, &alt, &roll, &pitch, &yaw);
                    fscanf(GPSFile, "%lf %lf %lf %lf %lf ", &vn, &ve, &vf, &vl, &vu);
                    fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &ax, &ay, &az, &af, &al, &au);
                    fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &wx, &wy, &wz, &wf, &wl, &wu);
                    fscanf(GPSFile, "%lf %lf %lf %lf %lf %lf ", &pos_accuracy, &vel_accuracy, &navstat, &numsats, &velmode,
                    &orimode);

                    fclose(GPSFile);

                    Vector3d linearAcceleration(ax, ay, az);
                    Vector3d angularVelocity(wx, wy, wz);
                    sensor_msgs::Imu msgImu;
                    msgImu.header.frame_id = "body";
                    msgImu.header.stamp = ros::Time(itPairTimeType->first);
                    msgImu.linear_acceleration.x = ax;
                    msgImu.linear_acceleration.y = ay;
                    msgImu.linear_acceleration.z = az;
                    msgImu.angular_velocity.x = wx;
                    msgImu.angular_velocity.y = wy;
                    msgImu.angular_velocity.z = wz;

                    pubImu.publish(msgImu);
                    ROS_INFO("Input IMU, time:%f, seq:%d", itPairTimeType->first, imu_gnssIdx);
                    printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);
                    printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);
                    printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);
                    printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);
                    printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n",
                            pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);

                    if((imu_gnssIdx % gnss_ex_div == 0) && lat > 0) {
                        sensor_msgs::NavSatFix gps_position;
                        gps_position.header.frame_id = "gnss";
                        gps_position.header.stamp = ros::Time(itPairTimeType->first);
                        gps_position.status.status = navstat;
                        gps_position.status.service = numsats;
                        gps_position.latitude = lat;
                        gps_position.longitude = lon;
                        gps_position.altitude = alt;
                        gps_position.position_covariance[0] = pos_accuracy;
                        gps_position.position_covariance[4] = pos_accuracy;
                        gps_position.position_covariance[8] = pos_accuracy;

                        pubOrignGnss.publish(gps_position);
                        ROS_INFO("Input GNSS, time:%f, seq:%d", itPairTimeType->first, imu_gnssIdx);
                        printf("lat:%lf lon:%lf alt:%lf roll:%lf pitch:%lf yaw:%lf \n",  lat, lon, alt, roll, pitch, yaw);
                        printf("vn:%lf ve:%lf vf:%lf vl:%lf vu:%lf \n",  vn, ve, vf, vl, vu);
                        printf("ax:%lf ay:%lf az:%lf af:%lf al:%lf au:%lf\n",  ax, ay, az, af, al, au);
                        printf("wx:%lf wy:%lf wz:%lf wf:%lf wl:%lf wu:%lf\n",  wx, wy, wz, wf, wl, wu);
                        printf("pos_accuracy:%lf vel_accuracy:%lf navstat:%lf numsats:%lf velmode:%lf orimode:%lf\n",
                                pos_accuracy, vel_accuracy, navstat, numsats, velmode, orimode);

                    }
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(SENSOR_SLEEP_TIME));
            }
        }
        else break;
    }

    ros::spin();

    return 0;
}