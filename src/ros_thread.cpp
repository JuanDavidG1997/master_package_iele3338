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
}

ros_thread::~ros_thread()
{
}

void ros_thread::run()
{
    ros::NodeHandle n;
    ros::ServiceServer service_ack = n.advertiseService("ack_service", &ros_thread::AckService_callback, this);
    ros::ServiceServer service_end = n.advertiseService("end_service", &ros_thread::EndService_callback, this);
    ros::spin();
}


bool ros_thread::AckService_callback(master_package_iele3338::AckService::Request  &req,
				     master_package_iele3338::AckService::Response &res)
{
    res.state = 1;
    ROS_INFO("Request: Group number = %d, IP = %s", (int)req.group_number, ((std::string)req.ip_address).c_str());
    ROS_INFO("Response: State = %d", (int)res.state);
    return true;
}

bool ros_thread::EndService_callback(master_package_iele3338::EndService::Request  &req,
				     master_package_iele3338::EndService::Response &res)
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
