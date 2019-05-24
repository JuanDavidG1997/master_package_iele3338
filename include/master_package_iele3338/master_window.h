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

#include <QMainWindow>
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QGridLayout>
#include <QDesktopWidget>
#include <QPlainTextEdit>
#include <QTextEdit>
#include <QTimer>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QListWidget>
#include <QVector>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QListWidgetItem>
#include <QModelIndexList>
#include <QLCDNumber>
#include <QTime>
#include <QTimer>
#include <QChar>
#include <iostream>
#include "ros/package.h"
#include "ros/ros.h"
#include "master_package_iele3338/ros_thread.h"
#include "master_package_iele3338/plot_node_thread.h"
#include "master_msgs_iele3338/Obstacle.h"
#include "master_msgs_iele3338/Covariance.h"
#include <QHostAddress>
#include <QNetworkInterface>

using namespace std;

class MainWindow;
class MasterWindow : public QMainWindow
{
    Q_OBJECT

public:
    MasterWindow(int xw, int yw);
    virtual ~MasterWindow();
    void loadConfigurationFile();
    
private:
    const int initialTestTimeMins = 1;
    const int initialTestTimeSecs = 0;
    const int oneSecondTimeMilisecs = 1000;
    QGridLayout *mainLayout, *configurationLayout, *infoLayout, *headerLayout, *infoRobotLayout, *optionsLayout;
    QPushButton *startTestButton, *loadConfigFileButton, *graphStartButton;
    QLabel *appNameLabel, *groupNumberLabel, *startPointLabel, *goalPointLabel, *obstaclesLabel, *masterIpAddressLabel,
    *robotIpAddressLabel, *robotPosLabel, *robotPosXLabel, *robotPosYLabel, *robotPosThetaLabel, *covarianceLabel,
    *covariance_1_1Label, *covariance_1_2Label, *covariance_1_3Label, *covariance_2_1Label, *covariance_2_2Label,
    *covariance_2_3Label, *covariance_3_1Label, *covariance_3_2Label, *covariance_3_3Label, *xLabel, *yLabel, *thetaLabel, 
    *passwordLabel;
    QComboBox *groupNumberComboBox, *startPointComboBox, *goalPointComboBox;
    QListWidget *obstacleList;
    QPlainTextEdit *console;
    QTextEdit *passwordTextEdit;
    QCheckBox *readyCheckBox;
    ros_thread *rosSpinThread;
    plot_node_thread *plotNodeThread;
    QFile *configurationFile;
    int numberOfGroups, numberObstacles, numberOfStartPoints, numberOfGoalPoints;
    QStringList groupNames, obstaclesNames, startPointNames, goalPointNames;
    QString configurationFileName;
    QLCDNumber *testRemainingTimerLCD;
    QTime *testRemainingTime;
    QTimer *testRemainingTimer;
    QString ipAddress;
    
    //Ejemplos
    master_msgs_iele3338::Obstacle obstacleExample;
    master_msgs_iele3338::Covariance covarianceExample;
    geometry_msgs::Pose startPoint, goalPoint;
    int numberOfObstacles;
    QVector<master_msgs_iele3338::Obstacle> *obstaclesVector;
    QVector<geometry_msgs::Pose> *startPointsVector, *goalPointsVector;
    
private slots:
  void startTestButtonSlot();
  void graphStartButtonSlot();
  void readyCheckBoxSlot(int checkBoxState);
  void groupNumberChangedSlot(int index);
  void initializeCounterTimerSlot();
  void ipAddressSlot(QString address);
  void updateRobotPoseSlot(double x, double y, double theta); 
  void updateRobotUncertaintySlot(double sigma11, double sigma12, double sigma13, double sigma21, double sigma22, double sigma23, double sigma31, double sigma32, double sigma33); 
  
signals:
  void startServiceSignal(geometry_msgs::Pose start, geometry_msgs::Pose goal, int numberObstacles, QVector<master_msgs_iele3338::Obstacle> *obstacles);
};

#endif
