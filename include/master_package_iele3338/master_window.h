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
#include <QTimer>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QListWidget>
#include "ros/package.h"
#include "ros/ros.h"
#include "master_package_iele3338/ros_thread.h"
#include "master_msgs_iele3338/Obstacle.h"
#include "master_msgs_iele3338/Covariance.h"


class MainWindow;
class MasterWindow : public QMainWindow
{
    Q_OBJECT

public:
    MasterWindow(int xw, int yw);
    virtual ~MasterWindow();
    
private:
    QGridLayout *mainLayout, *configurationLayout, *readyLayout, *infoLayout, *testLayout;
    QPushButton *startTestButton, *loadConfigFileButton;
    QLabel *appNameLabel, *groupNumberLabel, *startPointLabel, *goalPointLabel, *obstaclesLabel, *masterIpAddressLabel, *robotIpAddressLabel;
    QComboBox *groupNumberComboBox, *startPointComboBox, *goalPointComboBox;
    QListWidget *obstacleList;
    QPlainTextEdit *console;
    QCheckBox *readyCheckBox;
    ros_thread *rosSpinThread;
    master_msgs_iele3338::Obstacle obstacleExample;
    master_msgs_iele3338::Covariance covarianceExample;
};

#endif
