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

#ifndef master_window_H
#define master_window_H

#include <QWidget>
#include <QLabel>
#include "ros/package.h"
#include "ros/ros.h"
#include "master_package_iele3338/ros_thread.h"
#include "master_package_iele3338/Obstacle.h"
#include "master_package_iele3338/Covariance.h"


class MainWindow;
class MasterWindow : public QWidget
{
    Q_OBJECT

public:
    MasterWindow();
    virtual ~MasterWindow();
    
private:
    ros_thread *rosSpinThread;
    master_package_iele3338::Obstacle obstacleExample;
    master_package_iele3338::Covariance covarianceExample;
};

#endif
