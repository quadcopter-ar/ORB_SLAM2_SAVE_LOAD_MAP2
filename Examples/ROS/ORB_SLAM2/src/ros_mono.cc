/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
//--QuadcpterAR--
#include <stdio.h>
#include <stdlib.h>
//--QuadcpterAR--

#include<ros/ros.h>
#include<nav_msgs/Odometry.h> //
#include<geometry_msgs/Point.h> //
#include<pangolin/pangolin.h>
#include<std_msgs/UInt8.h> //
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);
    //--QuadcpterAR--
    bool GetStdoutFromCommand(string cmd);
    //--QuadcpterAR--
    ORB_SLAM2::System* mpSLAM;
};
//--QuadcpterAR--
ros::Publisher *g_pub;
//--QuadcpterAR--
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);
    ros::NodeHandle nodeHandler;
    //--QuadcpterAR--
    // std::string topicName;
    // std::string subName;
    // if(igb.GetStdoutFromCommand("rosnode list"))
    // {
    //     std::cout << "First instance found. This is 2nd instance" << std::endl;
    //     topicName = "MonoPose2";
    //     subName = "/camera2/image_raw";
    // }

    // else
    // {
    //     std::cout << "This is the first instance." << std::endl;
    //     topicName = "MonoPose1";
    //     subName = "/camera1/image_raw";
    // }
    // ros::Publisher pub = nodeHandler.advertise<nav_msgs::Odometry>(topicName, 1);
    // g_pub = &pub;
    // ros::Subscriber sub = nodeHandler.subscribe(subName, 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher pub = nodeHandler.advertise<nav_msgs::Odometry>("MonoPose", 1);
    g_pub = &pub;
    ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    //--QuadcpterAR--

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

//--QuadcpterAR--
void rotationMatrixToQuaternion(pangolin::OpenGlMatrix &M, float &qw, float &qx, float &qy, float &qz) 
{
    float 
        m00 = M.m[0],
        m10 = M.m[1],
        m20 = M.m[2],

        m01 = M.m[4],
        m11 = M.m[5],
        m21 = M.m[6],

        m02 = M.m[8],
        m12 = M.m[9],
        m22 = M.m[10];

    qw = sqrt(1 + m00 + m11 + m22) /2;
    qx = (m21 - m12)/( 4 * qw);
    qy = (m02 - m20)/( 4 * qw);
    qz = (m10 - m01)/( 4 * qw);
}
//--QuadcpterAR--

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
    //--QuadcpterAR--
    //  Publish camera position as a ROS msg.
    pangolin::OpenGlMatrix M;
    mpSLAM->mpMapDrawer->GetCurrentOpenGLCameraMatrix(M);

    if(g_pub != NULL) 
    {
        nav_msgs::Odometry msg;
        msg.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = M.m[12]; //tx
        msg.pose.pose.position.y = M.m[13]; //ty
        msg.pose.pose.position.z = M.m[14]; //tz
        // msg.pose.pose.position.x = M.m[12] * mpSLAM->real_world_scale; //tx
        // msg.pose.pose.position.y = M.m[13] * mpSLAM->real_world_scale; //ty
        // msg.pose.pose.position.z = M.m[14] * mpSLAM->real_world_scale; //tz
        // Orientation in quaternion.
        float qw, qx, qy, qz;
        rotationMatrixToQuaternion(M, qw, qx, qy, qz);
        
        msg.pose.pose.orientation.w = qw;
        msg.pose.pose.orientation.x = qx;
        msg.pose.pose.orientation.y = qy;
        msg.pose.pose.orientation.z = qz;

        g_pub->publish(msg);
        //--QuadcpterAR--
    }
}

//--QuadcpterAR--
bool ImageGrabber::GetStdoutFromCommand(string cmd) 
{
    std::cout << "Inside getSTDOutput: " << std::endl;
    // string data;
    bool instance_running = false;
    FILE * stream;
    const int max_buffer = 256;
    char buffer[max_buffer];
    std::string instance = "/ORB_SLAM_2\n";
    // cmd.append(" 2>&1");

    stream = popen(cmd.c_str(), "r");
    if (stream) 
    {
        while (!feof(stream))
        {
            if (fgets(buffer, max_buffer, stream) != NULL)
            {
                std::cout << std::string(buffer) << std::endl;
                // data.append(buffer);
                if(std::string(buffer) == instance)
                {
                    std::cout << "Found instance 1 running" << std::endl;
                    instance_running = true;
                    break;
                }
            }
        }       
        pclose(stream);
    }
    std::cout << "getSTDOutput Over: " << std::endl;
    // return data;
    return instance_running;
}
//--QuadcpterAR--
