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

#include <QApplication>
#include <QStyle>
#include <QDesktopWidget>
#include "master_package_iele3338/master_window.h"


int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QDesktopWidget dw;
    int x_window=dw.width()*0.3;
    int y_window=dw.height()*0.9;
    MasterWindow masterWindow(x_window, y_window);
    masterWindow.setGeometry(
    QStyle::alignedRect(
        Qt::LeftToRight,
        Qt::AlignCenter,
        masterWindow.size(),
        qApp->desktop()->availableGeometry()
  )
);
    masterWindow.show();
    return app.exec();
}
