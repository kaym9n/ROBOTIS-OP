/**
 * @file /include/op_control/main_window.hpp
 *
 * @brief Qt based gui for op_control.
 *
 * @date November 2010
 **/
#ifndef op_control_MAIN_WINDOW_H
#define op_control_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#define M_PI 3.14159265358979

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace op_control {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
    void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
    void on_button_play_motion_clicked(bool check );
    void on_button_stop_motion_clicked(bool check);
    void on_slider_head_pan_valueChanged(int value);
    void on_slider_head_tilt_valueChanged(int value);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void update_head_pan(double angle);
    void update_head_tilt(double angle);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    bool _is_feedback;

    void update_motion_combobox();
};

}  // namespace op_control

#endif // op_control_MAIN_WINDOW_H
