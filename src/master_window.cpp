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
    headerLayout = new QGridLayout();
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
    masterIpAddressLabel = new QLabel("Master IP Address: ");
    robotIpAddressLabel = new QLabel("Robot IP Adress: ");
    console = new QPlainTextEdit();
    configurationFile = new QFile();
    obstaclesVector = new QVector<master_msgs_iele3338::Obstacle>();
    startPointsVector = new QVector<geometry_msgs::Pose>();
    goalPointsVector = new QVector<geometry_msgs::Pose>();
    testRemainingTimerLCD = new QLCDNumber(4);
    testRemainingTime = new QTime(0, initialTestTimeMins, initialTestTimeSecs);
    testRemainingTimer = new QTimer();
    
    //Central Widget
    setCentralWidget(window);
    window->setLayout(mainLayout);
    //this->setWindowState(Qt::WindowMaximized);
    window->setFixedSize(QSize(xw, yw));
    setWindowTitle("Master Package IELE3338");
    
    //Add widgets to Main Layout 
    mainLayout->addLayout(headerLayout, 0, 0);
    mainLayout->addLayout(configurationLayout, 1, 0);
    mainLayout->addLayout(readyLayout, 2, 0);
    mainLayout->addLayout(infoLayout, 3, 0);
    mainLayout->addWidget(console, 4, 0);
    mainLayout->addLayout(testLayout, 5, 0);
    
    //Add widgets to header Layout
    headerLayout->addWidget(appNameLabel, 0, 0);
    headerLayout->addWidget(testRemainingTimerLCD, 0, 1);
    
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
    
    //Add widgets to Ready Layout
    readyLayout->addWidget(readyCheckBox, 0, 0);
    readyLayout->addWidget(startTestButton, 0, 1);
    
    //Add widgets to Info Layout
    infoLayout->addWidget(masterIpAddressLabel, 0, 0);
    infoLayout->addWidget(robotIpAddressLabel, 1, 0);

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
    QFont f("Arial",24);
    QFontMetrics fm(f);
    appNameLabel->setFont(f);
    testRemainingTimerLCD->setSegmentStyle(QLCDNumber::Filled);
    mainLayout->setRowStretch(0, 10);
    QString time = testRemainingTime->toString();
    testRemainingTimerLCD->display(time);
    configurationFileName = QString::fromStdString(ros::package::getPath("master_package_iele3338")) + "/.test_configuration_file.conf";
    
    const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
    for (const QHostAddress &address: QNetworkInterface::allAddresses())
    {
      if (address.protocol() == QAbstractSocket::IPv4Protocol && address != localhost)
	ipAddress = address.toString();
    }
    
    masterIpAddressLabel->setText(masterIpAddressLabel->text() + ipAddress);
    QFont f1("Arial",16);
    QFontMetrics fm1(f1);
    masterIpAddressLabel->setFont(f1);
    robotIpAddressLabel->setFont(f1);
    
    //Signals and slots connection
    connect(startTestButton, SIGNAL(clicked()), this, SLOT(startTestButtonSlot()));
    connect(this, SIGNAL(startServiceSignal(geometry_msgs::Pose, geometry_msgs::Pose, int, QVector<master_msgs_iele3338::Obstacle>*)), rosSpinThread, SLOT(startServiceSlot(geometry_msgs::Pose, geometry_msgs::Pose, int, QVector<master_msgs_iele3338::Obstacle>*)));
    connect(readyCheckBox, SIGNAL(stateChanged(int)), this, SLOT(readyCheckBoxSlot(int)));
    connect(groupNumberComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(groupNumberChangedSlot(int)));
    connect(testRemainingTimer, SIGNAL(timeout()), this, SLOT(initializeCounterTimerSlot()));
    connect(rosSpinThread, SIGNAL(ipAddressSignal(QString)), this, SLOT(ipAddressSlot(QString)));
    
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
    
    loadConfigurationFile();
    rosSpinThread->start();
}

MasterWindow::~MasterWindow()
{
  
}

void MasterWindow::loadConfigurationFile()
{
    configurationFile->setFileName(configurationFileName);
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
	}
	else if ((fileCount > 5) && (fileCount < (numberOfGroups+6)))
	  groupNames.append(line);
	else if ((fileCount > (numberOfGroups+6)) && fileCount < (numberOfGroups+7+numberOfObstacles))
	{
	  QStringList pieces = line.split(",");
	  master_msgs_iele3338::Obstacle obstacle;
	  obstacle.position.position.x = pieces.at(0).toDouble();
	  obstacle.position.position.y = pieces.at(1).toDouble();
	  obstacle.radius = pieces.at(2).toInt();
	  obstaclesVector->append(obstacle);
	  obstaclesNames.append(line);
	}
	else if ((fileCount > (numberOfGroups+7+numberOfObstacles)) && fileCount < (numberOfGroups+8+numberOfObstacles+numberOfStartPoints))
	{
	  QStringList pieces = line.split(",");
	  geometry_msgs::Pose point;
	  point.position.x = pieces.at(0).toDouble();
	  point.position.y = pieces.at(1).toDouble();
	  point.orientation.w = pieces.at(2).toDouble();
	  startPointsVector->append(point);
	  startPointNames.append(line);
	}
	else if ((fileCount > (numberOfGroups+8+numberOfObstacles+numberOfStartPoints)) && fileCount < (numberOfGroups+9+numberOfObstacles+numberOfStartPoints+numberOfGoalPoints))
	{
	  QStringList pieces = line.split(",");
	  geometry_msgs::Pose point;
	  point.position.x = pieces.at(0).toDouble();
	  point.position.y = pieces.at(1).toDouble();
	  point.orientation.w = pieces.at(2).toDouble();
	  goalPointsVector->append(point);
	  goalPointNames.append(line);
	}
	fileCount++;
      }
      for (int i = 0;i < numberOfGroups;i++)
	groupNumberComboBox->addItem(groupNames.at(i));
      for (int i = 0;i < numberOfObstacles;i++)
	obstacleList->addItem(QString::number(i+1) + ": " + obstaclesNames.at(i));
      for (int i = 0;i < numberOfStartPoints;i++)
	startPointComboBox->addItem(QString::number(i+1) + ": " + startPointNames.at(i));
      for (int i = 0;i < numberOfGoalPoints;i++)
	goalPointComboBox->addItem(QString::number(i+1) + ": " + goalPointNames.at(i));
      
      ROS_INFO("Configuration file succesfully loaded");
    }
}

void MasterWindow::startTestButtonSlot()
{    
  if (startTestButton->text() == "Start")
  {
    startPoint = startPointsVector->at(startPointComboBox->currentIndex());
    goalPoint = goalPointsVector->at(goalPointComboBox->currentIndex());
    QVector<master_msgs_iele3338::Obstacle> *selectedObstaclesVector;
    selectedObstaclesVector = new QVector<master_msgs_iele3338::Obstacle>();
    
    QModelIndexList indexes = obstacleList->selectionModel()->selectedIndexes();
    foreach(QModelIndex index, indexes)
      selectedObstaclesVector->append(obstaclesVector->at(index.row()));
        
    emit startServiceSignal(startPoint, goalPoint, (int)(selectedObstaclesVector->size()), selectedObstaclesVector);
    testRemainingTime->setHMS(0, initialTestTimeMins, initialTestTimeSecs);
    testRemainingTimerLCD->display(testRemainingTime->toString());
    testRemainingTimer->start(oneSecondTimeMilisecs);
    readyCheckBox->setEnabled(false);
    startTestButton->setText("Stop");
  }
  else if (startTestButton->text() == "Stop")
  {
     startTestButton->setText("Start");
     testRemainingTimer->stop();
     readyCheckBox->setEnabled(true);
  } 
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

void MasterWindow::groupNumberChangedSlot(int index)
{
    int groupNumber = index + 1;
    rosSpinThread->setGroupNumber(groupNumber);
}

void MasterWindow::initializeCounterTimerSlot()
{
    *testRemainingTime = testRemainingTime->addMSecs(-oneSecondTimeMilisecs);
    QString time = testRemainingTime->toString();
    testRemainingTimerLCD->display(time);
    
    if (*testRemainingTime == QTime(0,0,0))
    {
      testRemainingTimer->stop();
      startTestButton->setText("Start");
      readyCheckBox->setEnabled(true);
    }
}

void MasterWindow::ipAddressSlot(QString address)
{
  robotIpAddressLabel->setText(robotIpAddressLabel->text() + address);
}