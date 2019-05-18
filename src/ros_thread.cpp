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
    groupNumber = -1;
    posSub = n.subscribe("robot_position", 10, &ros_thread::robotPositionCallback, this); 
    covSub = n.subscribe("robot_uncertainty", 100, &ros_thread::robotUncertaintyCallback, this); 
}

ros_thread::~ros_thread()
{
  
}

void ros_thread::run()
{
    ros::spin();
}


void ros_thread::setGroupNumber(int group)
{
    groupNumber = group;
}


bool ros_thread::AckService_callback(master_msgs_iele3338::AckService::Request  &req,
				     master_msgs_iele3338::AckService::Response &res)
{
    ROS_INFO("Request: Group number = %d, IP = %s", (int)req.group_number, ((std::string)req.ip_address).c_str());
    if((int)req.group_number == groupNumber)
	res.state = 1;
    else
        res.state = 0;
    ROS_INFO("Response: State = %d", (int)res.state);
    emit ipAddressSignal(QString::fromStdString(req.ip_address));
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

void ros_thread::robotPositionCallback(const geometry_msgs::Pose& msg)
{
    emit robotPositionSignal(msg.position.x, msg.position.y, msg.orientation.w); 
}

void ros_thread::robotUncertaintyCallback(const master_msgs_iele3338::Covariance& msg)
{
    emit robotUncertaintySignal(msg.sigma11, msg.sigma12, msg.sigma13, msg.sigma21, msg.sigma22, msg.sigma23, msg.sigma31, msg.sigma32, msg.sigma33); 
}


void ros_thread::startServiceSlot(geometry_msgs::Pose startPoint, geometry_msgs::Pose goalPoint, int numberObstacles, QVector<master_msgs_iele3338::Obstacle> *obstacles)
{
    srv.request.start = startPoint;
    srv.request.goal = goalPoint;
    srv.request.n_obstacles = numberObstacles;
    srv.request.obstacles.resize(numberObstacles);
    for (int i = 0;i < numberObstacles;i++)
      srv.request.obstacles[i] = obstacles->at(i);
    
    if (start_client.call(srv) || (numberObstacles == 0))
      ROS_INFO("Start service client called");
    else
      ROS_ERROR("Failed to call start_service");
}
