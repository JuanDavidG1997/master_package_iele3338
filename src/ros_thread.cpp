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
    passwordTimer = new QTimer();
    service_ack = n.advertiseService("ack_service", &ros_thread::AckService_callback, this);
    service_end = n.advertiseService("end_service", &ros_thread::EndService_callback, this);
    start_client = n.serviceClient<master_msgs_iele3338::StartService>("start_service");
    posSub = n.subscribe("robot_position", 10, &ros_thread::robotPositionCallback, this); 
    covSub = n.subscribe("robot_uncertainty", 100, &ros_thread::robotUncertaintyCallback, this); 
    groupNumber = -1;
    readyReceivePassword = true;
    passwordTimer->setSingleShot(true);
    connect(passwordTimer, SIGNAL(timeout()), this, SLOT(passwordTimerSlot()));
    connect(this, SIGNAL(startTimerSignal()), this, SLOT(startTimerSlot()));
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

void ros_thread::setPassword(QString newPassword)
{
    password = newPassword.toInt();
}

bool ros_thread::AckService_callback(master_msgs_iele3338::AckService::Request  &req,
				     master_msgs_iele3338::AckService::Response &res)
{
    if((int)req.group_number == groupNumber)
	res.state = 1;
    else
        res.state = 0;
    
    emit ackServiceSignal((int)req.group_number, QString::fromStdString(req.ip_address));
    return true;
}

bool ros_thread::EndService_callback(master_msgs_iele3338::EndService::Request  &req,
				     master_msgs_iele3338::EndService::Response &res)
{
  if (readyReceivePassword)
  {
    if ((int)req.password == password)
    {
      res.correct = 1;
    }
    else if ((int)req.password != password)
    {
      res.correct = 0;
    }
    readyReceivePassword = false;
    emit startTimerSignal();
    emit endServiceSignal((int)req.password, (int)res.correct);
    return true;
  }
  emit endServiceErrorSignal();
  return false;
    
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
    
    bool serviceCalled = false;
    if (start_client.call(srv) || (numberObstacles == 0))
      serviceCalled = true;
    emit startServiceSignal(serviceCalled);
}

void ros_thread::passwordTimerSlot()
{
    readyReceivePassword = true;
}

void ros_thread::startTimerSlot()
{
    passwordTimer->start(2000);
}