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
#include <QVector>
#include <QString>
#include "ros/ros.h"
#include "master_msgs_iele3338/AckService.h"
#include "master_msgs_iele3338/EndService.h"
#include "master_msgs_iele3338/StartService.h"
#include "geometry_msgs/Pose.h"

class ros_thread : public QThread
{
    Q_OBJECT

public:
  ros_thread();
  ~ros_thread();
  void run();
  void setGroupNumber(int groupNumber);

private:
  bool AckService_callback(master_msgs_iele3338::AckService::Request  &req,
			   master_msgs_iele3338::AckService::Response &res);
  bool EndService_callback(master_msgs_iele3338::EndService::Request  &req,
			   master_msgs_iele3338::EndService::Response &res);
  
  ros::NodeHandle n;
  ros::ServiceServer service_ack;
  ros::ServiceServer service_end;
  ros::ServiceClient start_client;
  master_msgs_iele3338::StartService srv;
  int groupNumber;
  
signals:
  void ipAddressSignal(QString address);
  
private slots:
  void startServiceSlot(geometry_msgs::Pose startPoint, geometry_msgs::Pose goalPoint, int numberObstacles, QVector<master_msgs_iele3338::Obstacle> *obstacles);
};

#endif // ROS_THREAD_H
