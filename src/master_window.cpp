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

#include "master_package_iele3338/master_window.h"

MasterWindow::MasterWindow()
{
    char **argv;
    int argc = 0;  
    ros::init(argc, argv, "master_node");
    QLabel* label = new QLabel( this );
    label->setText( "Hello World!" );
    this->setWindowTitle(tr("Master Node"));
    rosSpinThread = new ros_thread();
    rosSpinThread->start();
    
    obstacleExample.position.position.x = 10;
    obstacleExample.position.position.y = 10;
    obstacleExample.radius = 2;
    
    covarianceExample.sigma11 = 1.0;
    covarianceExample.sigma12 = 1.0;
    covarianceExample.sigma13 = 1.0;
    covarianceExample.sigma21 = 1.0;
    covarianceExample.sigma22 = 1.0;
    covarianceExample.sigma23 = 1.0;
    covarianceExample.sigma31 = 1.0;
    covarianceExample.sigma32 = 1.0;
    covarianceExample.sigma33 = 1.0;
}

MasterWindow::~MasterWindow()
{
  
  
}
