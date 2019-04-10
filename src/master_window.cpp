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
    configurationLayout = new QGridLayout();
    readyLayout = new QGridLayout();
    infoLayout = new QGridLayout();
    testLayout = new QGridLayout();
    rosSpinThread = new ros_thread();
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
    readyCheckBox = new QCheckBox("Master Ready");
    startTestButton = new QPushButton("Start");
    masterIpAddressLabel = new QLabel("Master IP Address");
    robotIpAddressLabel = new QLabel("Robot IP Adress");
    console = new QPlainTextEdit();
    
    //Central Widget
    setCentralWidget(window);
    window->setLayout(mainLayout);
    window->setFixedSize(QSize(xw, yw));
    setWindowTitle("Basic Robot Motion Application");
    
    
    //Add widgets to Main Layout 
    mainLayout->addWidget(appNameLabel, 0, 0);
    mainLayout->addLayout(configurationLayout, 1, 0);
    mainLayout->addLayout(readyLayout, 2, 0);
    mainLayout->addLayout(infoLayout, 3, 0);
    mainLayout->addWidget(console, 4, 0);
    mainLayout->addLayout(testLayout, 5, 0);
    
    //Add widgets to Configuration Layout
    configurationLayout->addWidget(groupNumberLabel, 0, 0);
    configurationLayout->addWidget(groupNumberComboBox, 0, 1);
    configurationLayout->addWidget(startPointLabel, 1, 0);
    configurationLayout->addWidget(startPointComboBox, 1, 1);
    configurationLayout->addWidget(goalPointLabel, 2, 0);
    configurationLayout->addWidget(goalPointComboBox, 2, 1);
    configurationLayout->addWidget(obstaclesLabel, 3, 0);
    configurationLayout->addWidget(obstacleList, 3, 1);
    obstacleList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    
    
    obstacleList->setFixedSize(QSize(0.75*xw, 0.3*yw));
    obstacleList->addItem("Test1");
    obstacleList->addItem("Test2");
    
    //Add widgets to Ready Layout
    readyLayout->addWidget(readyCheckBox, 0, 0);
    readyLayout->addWidget(startTestButton, 0, 1);
    
    //Add widgets to Info Layout
    infoLayout->addWidget(masterIpAddressLabel, 0, 0);
    infoLayout->addWidget(robotIpAddressLabel, 0, 1);

    //Add widgets to Test Layout
    
    //Objects initialization
    startTestButton->setEnabled(false);
    startTestButton->setFixedSize(QSize(0.75*xw, 30));
    loadConfigFileButton->setEnabled(true);
    QPalette p = console->palette();
    p.setColor(QPalette::Base, QColor(0, 0, 0));
    p.setColor(QPalette::Text, Qt::white);
    console->setPalette(p);
    console->setEnabled(false);
    console->setMaximumBlockCount(10);
    appNameLabel->setAlignment(Qt::AlignCenter);
    QFont f("Arial",16);
    QFontMetrics fm(f);
    appNameLabel->setFont(f); 
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
