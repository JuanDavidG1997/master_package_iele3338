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
    configurationFile = new QFile(QString::fromStdString(ros::package::getPath("master_package_iele3338")) + "/.test_configuration_file.conf");
    
    //Central Widget
    setCentralWidget(window);
    window->setLayout(mainLayout);
    //window->setFixedSize(QSize(xw, yw));
    setWindowTitle("Master Package IELE3338");
    
    
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
    readyCheckBox->setChecked(false);
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
    
    //Signals and slots connection
    connect(startTestButton, SIGNAL(clicked()), this, SLOT(startTestButtonSlot()));
    connect(this, SIGNAL(startServiceSignal(geometry_msgs::Pose, geometry_msgs::Pose, int, QVector<master_msgs_iele3338::Obstacle>*)), rosSpinThread, SLOT(startServiceSlot(geometry_msgs::Pose, geometry_msgs::Pose, int, QVector<master_msgs_iele3338::Obstacle>*)));
    connect(readyCheckBox, SIGNAL(stateChanged(int)), this, SLOT(readyCheckBoxSlot(int)));
    
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
    
    //Start thread
    if (!configurationFile->open(QIODevice::ReadOnly | QIODevice::Text))
      ROS_ERROR("Configuration file not found");
    else
    {
      QTextStream in(configurationFile);
      int fileCount = 1;
      while (!in.atEnd())
      {
	QString line = in.readLine();
	if (fileCount < 5)
	{
	  QStringList pieces = line.split(": ");	   
	  if (pieces.at(0) == "Number of Groups")
	    numberOfGroups = pieces.at(1).toInt();
	  else if (pieces.at(0) == "Number of Obstacles")
	    numberOfObstacles = pieces.at(1).toInt();
	  else if (pieces.at(0) == "Number of Start Points")
	    numberOfStartPoints = pieces.at(1).toInt();
	  else if (pieces.at(0) == "Number of Goal Points")
	    numberOfGoalPoints = pieces.at(1).toInt();
	  cout << pieces.at(1).toStdString() << endl;
	}
	else if ((fileCount > 5) && (fileCount < (numberOfGroups+6)))
	  groupNames.append(line);
	else if ((fileCount > (numberOfGroups+6)) && fileCount < (numberOfGroups+7+numberOfObstacles))
	{
	  
	}
	fileCount++;
      }
      ROS_INFO("Configuration file succesfully loaded");
    }
      
    rosSpinThread->start();
}

MasterWindow::~MasterWindow()
{
  
}

void MasterWindow::startTestButtonSlot()
{
    int numberObstacles = 3;
    geometry_msgs::Pose start, goal;
    master_msgs_iele3338::Obstacle obsArray[numberObstacles];
    start.position.x = 0;
    start.position.y = 0;
    start.orientation.w = 0;
    goal.position.x = 1;
    goal.position.y = 1;
    goal.orientation.w = 1;
    obsArray[0].position.position.x = 2;
    obsArray[0].position.position.y = 2;
    obsArray[0].radius = 5;
    obsArray[1].position.position.x = 10;
    obsArray[1].position.position.y = 10;
    obsArray[1].radius = 8;
    obsArray[2].position.position.x = 15;
    obsArray[2].position.position.y = 15;
    obsArray[2].radius = 10;
    
    
    numberOfObstacles = numberObstacles;
    startPoint = start;
    goalPoint = goal;
    obstaclesVector = new QVector<master_msgs_iele3338::Obstacle>;
    for (int i = 0;i < numberOfObstacles;i++)
      obstaclesVector->append(obsArray[i]);
        
    emit startServiceSignal(startPoint, goalPoint, numberOfObstacles, obstaclesVector);
}

void MasterWindow::readyCheckBoxSlot(int checkBoxState)
{
    if (checkBoxState == Qt::Checked)
    {
      startTestButton->setEnabled(true);
      groupNumberComboBox->setEnabled(false);
      startPointComboBox->setEnabled(false);
      goalPointComboBox->setEnabled(false);
      obstacleList->setEnabled(false);
    }
    else
    {
      startTestButton->setEnabled(false);
      groupNumberComboBox->setEnabled(true);
      startPointComboBox->setEnabled(true);
      goalPointComboBox->setEnabled(true);
      obstacleList->setEnabled(true);
    }
}