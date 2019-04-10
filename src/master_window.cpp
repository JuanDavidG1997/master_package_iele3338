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

MasterWindow::MasterWindow(int xw, int yw)
{
    char **argv;
    int argc = 0;  
    ros::init(argc, argv, "master_node");
    
    //Objects allocation
    QWidget *window = new QWidget();
    mainLayout = new QGridLayout();
    titleLayout = new QGridLayout();
    configurationLayout = new QGridLayout();
    readyLayout = new QGridLayout();
    infoLayout = new QGridLayout();
    testLayout = new QGridLayout();
    rosSpinThread = new ros_thread();
    startTestButton = new QPushButton("Start");
    loadConfigFileButton = new QPushButton("Load Configuration File");
    appNameLabel = new QLabel("Master Node (IELE3338)");
    groupNumberLabel = new QLabel("Group Number:");
    startPointLabel = new QLabel("Start Point:");
    goalPointLabel = new QLabel("Goal Point:");
    obstaclesLabel = new QLabel("Obstacles:");
    groupNumberComboBox = new QComboBox();
    startPointComboBox = new QComboBox();
    goalPointComboBox = new QComboBox();
    obstacleList = new QListWidget();
    //console = new QPlainTextEdit();
    
    //Add widgets to Main Layout 
    mainLayout->addLayout(titleLayout, 0, 0);
    mainLayout->addLayout(configurationLayout, 1, 0);
    mainLayout->addLayout(readyLayout, 2, 0);
    mainLayout->addLayout(infoLayout, 3, 0);
    mainLayout->addLayout(testLayout, 4, 0);
    
    //Add widgets to Title Layout
    titleLayout->addWidget(appNameLabel, 0, 0);
    
    //Add widgets to Configuration Layout
    configurationLayout->addWidget(groupNumberLabel, 0, 0);
    configurationLayout->addWidget(groupNumberComboBox, 0, 1);
    configurationLayout->addWidget(startPointLabel, 1, 0);
    configurationLayout->addWidget(startPointComboBox, 1, 1);
    configurationLayout->addWidget(goalPointLabel, 2, 0);
    configurationLayout->addWidget(goalPointComboBox, 2, 1);
    configurationLayout->addWidget(obstaclesLabel, 3, 0);
    configurationLayout->addWidget(obstacleList, 4, 0);
    
    //Add widgets to Ready Layout
    
    //Add widgets to Info Layout
    
    //Add widgets to Test Layout
    
    //Objects initialization
    window->setLayout(mainLayout);
    setCentralWidget(window);
    window->setFixedSize(xw,yw);
    setWindowTitle("Basic Robot Motion Application");
    
    startTestButton->setEnabled(false);
    loadConfigFileButton->setEnabled(true);
    rosSpinThread->start();
    
    //Obstacle Example
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
