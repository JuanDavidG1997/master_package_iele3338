/*
 * //======================================================================//
 * //  This software is free: you can redistribute it and/or modify        //
 * //  it under the terms of the GNU General Public License Version 3,     //
 * //  as published by the Free Software Foundation.                       //
 * //  This software is distributed in the hope that it will be useful,    //
 * //  but WITHOUT ANY WARRANTY; without even the implied warranty of      //
 * //  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE..  See the      //
 * //  GNU General Public License for more details.                        //
 * //  You should have received a copy of the GNU General Public License   //
 * //  Version 3 in the file COPYING that came with this distribution.     //
 * //  If not, see <http://www.gnu.org/licenses/>                          //
 * //======================================================================//
 * //                                                                      //
 * //      Copyright (c) 2019 Carlos Quintero		                   //             
 * //      Universidad de los Andes                                        //
 * //                                                                      //
 * //======================================================================//
 */

#include "master_package_iele3338/ros_thread.h"

ros_thread::ros_thread()
{
    service_ack = n.advertiseService("ack_service", &ros_thread::AckService_callback, this);
    service_end = n.advertiseService("end_service", &ros_thread::EndService_callback, this);
    service_start = n.advertiseService("start_service", &ros_thread::StartService_callback, this);    
}

ros_thread::~ros_thread()
{
}

void ros_thread::run()
{
    ros::spin();
}


bool ros_thread::AckService_callback(master_msgs_iele3338::AckService::Request  &req,
				     master_msgs_iele3338::AckService::Response &res)
{
    res.state = 1;
    ROS_INFO("Request: Group number = %d, IP = %s", (int)req.group_number, ((std::string)req.ip_address).c_str());
    ROS_INFO("Response: State = %d", (int)res.state);
    return true;
}

bool ros_thread::EndService_callback(master_msgs_iele3338::EndService::Request  &req,
				     master_msgs_iele3338::EndService::Response &res)
{
    if ((int)req.password == 1234) 
    {
      res.correct = 1;
    }
    else
    {
      res.correct = 0;
    }
    
    ROS_INFO("Request: Password = %d", (int)req.password);
    ROS_INFO("Response: Correct = %d", (int)res.correct);
    return true;
}

bool ros_thread::StartService_callback(master_msgs_iele3338::StartService::Request  &req,
				     master_msgs_iele3338::StartService::Response &res)
{
    geometry_msgs::Pose startPoint, goalPoint;
    int numberObstacles = 3;
    master_msgs_iele3338::Obstacle obsArray[3];
    startPoint.position.x = 0;
    startPoint.position.y = 0;
    startPoint.orientation.w = 0;
    goalPoint.position.x = 1;
    goalPoint.position.y = 1;
    goalPoint.orientation.w = 1;
    obsArray[0].position.position.x = 1;
    obsArray[0].position.position.y = 1;
    obsArray[0].radius = 5;
    obsArray[1].position.position.x = 1;
    obsArray[1].position.position.y = 1;
    obsArray[1].radius = 5;
    obsArray[2].position.position.x = 3;
    obsArray[2].position.position.y = 3;
    obsArray[2].radius = 5;
    
    res.start = startPoint;
    res.goal = goalPoint;
    res.n_obstacles = numberObstacles;
    res.obstacles.resize(numberObstacles);
    res.obstacles[0] = obsArray[0];
    res.obstacles[1] = obsArray[1];
    res.obstacles[2] = obsArray[2];
    
    ROS_INFO("Response: Start Point x = %f, y = %f, theta = %f", (double)res.start.position.x, (double)res.start.position.y, (double)res.start.orientation.w);
    ROS_INFO("Response: Goal Point x = %f, y = %f, theta = %f", (double)res.goal.position.x, (double)res.goal.position.y, (double)res.goal.orientation.w);
    ROS_INFO("Response: Number of Obstacles = %d", (int)res.n_obstacles);
    return true;
}
