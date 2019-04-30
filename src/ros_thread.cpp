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
    start_client = n.serviceClient<master_msgs_iele3338::StartService>("start_service");
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
      res.correct = 1;
    else
      res.correct = 0;
    
    ROS_INFO("Request: Password = %d", (int)req.password);
    ROS_INFO("Response: Correct = %d", (int)res.correct);
    return true;
}

void ros_thread::startServiceSlot(geometry_msgs::Pose startPoint, geometry_msgs::Pose goalPoint, int numberObstacles, QVector<master_msgs_iele3338::Obstacle> *obstacles)
{
    srv.request.start = startPoint;
    srv.request.goal = goalPoint;
    srv.request.n_obstacles = numberObstacles;
    srv.request.obstacles.resize(numberObstacles);
    srv.request.obstacles[0] = obstacles->at(0);
    srv.request.obstacles[1] = obstacles->at(1);
    srv.request.obstacles[2] = obstacles->at(2);
    
    if (start_client.call(srv))
      ROS_INFO("Start service client called");
    else
      ROS_ERROR("Failed to call start_service");
}