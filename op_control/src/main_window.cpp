/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/op_control/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace op_control {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    // ReadSettings();
    setWindowIcon(QIcon(":/images/icon.png"));
    ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    QObject::connect(&qnode, SIGNAL(updateHeadPan(double)), this, SLOT(update_head_pan(double)));
    QObject::connect(&qnode, SIGNAL(updateHeadTilt(double)), this, SLOT(update_head_tilt(double)));
    /*********************
    ** Logging
    **********************/
    ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/

    if ( !qnode.init() )
    {
        showNoMasterMessage();
    }

    _is_feedback = false;
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_play_motion_clicked(bool check )
{
    std::string _motion_str = ui.comboBox_motion_index->currentText().toStdString();
    std::string _delimiter = "::";
    std::string _motion_text;
    int _motion_index;

    int _index_delimiter = _motion_str.find(_delimiter);
    if(_index_delimiter == std::string::npos) return;

    _motion_index = atoi(_motion_str.substr(0, _index_delimiter).c_str());
    _motion_text = _motion_str.substr(_index_delimiter + _delimiter.length());

    qnode.play_motion(_motion_index, _motion_text);
}

void MainWindow::on_slider_head_pan_valueChanged(int value)
{
    if(_is_feedback == true) return;
    int _value = ui.slider_head_pan->value();
    qnode.control_joint(qnode.head_pan_joint_name, qnode.Deg2Rad(_value));
}

void MainWindow::on_slider_head_tilt_valueChanged(int value)
{
    if(_is_feedback == true) return;
    int _value = ui.slider_head_tilt->value();
    qnode.control_joint(qnode.head_tilt_joint_name, qnode.Deg2Rad(_value));
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
    ui.view_logging->scrollToBottom();
}

void MainWindow::update_head_pan(double angle)
{
    _is_feedback = true;
    ui.slider_head_pan->setValue(angle);
    // std::cout << "head pan : " << angle << std::endl;
    _is_feedback = false;
}

void MainWindow::update_head_tilt(double angle)
{
    _is_feedback = true;
    ui.slider_head_tilt->setValue(angle);
    // std::cout << "head tilt : " << angle << std::endl;
    _is_feedback = false;
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::update_motion_combobox()
{
    // not yet
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace op_control

