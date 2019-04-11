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

#ifndef ROS_THREAD_H
#define ROS_THREAD_H

#include <QObject>
#include <QThread>
#include "ros/ros.h"
#include "master_package_iele3338/AckService.h"

class ros_thread : public QThread
{
    Q_OBJECT

public:
  ros_thread();
  ~ros_thread();
  void run();

private:
  bool AckService_callback(master_package_iele3338::AckService::Request  &req,
			   master_package_iele3338::AckService::Response &res);
};

#endif // ROS_THREAD_H
