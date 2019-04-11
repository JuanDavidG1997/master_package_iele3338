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
    ros::ServiceServer service = n.advertiseService("ack_service", &ros_thread::AckService_callback, this);
    ros::spin();
}


bool ros_thread::AckService_callback(master_package_iele3338::AckService::Request  &req,
				     master_package_iele3338::AckService::Response &res)
{
    res.state = 1;
    ROS_INFO("Request: Group number = %d, IP = %s", (int)req.group_number, ((std::string)req.ip_address).c_str());
    ROS_INFO("Response: State = %ld", (long int)res.state);
    return true;
}
