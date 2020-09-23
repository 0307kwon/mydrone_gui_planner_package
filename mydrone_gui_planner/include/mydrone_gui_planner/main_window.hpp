/**
 * @file /include/mydrone_gui_planner/main_window.hpp
 *
 * @brief Qt based gui for mydrone_gui_planner.
 *
 * @date November 2010
 **/
#ifndef mydrone_gui_planner_MAIN_WINDOW_H
#define mydrone_gui_planner_MAIN_WINDOW_H

/*****************************************************************************
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "msgThread.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace mydrone_gui_planner {

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

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/

	void showPanel();
	void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check);
	void on_offboard_button_clicked();
	void on_onboard_button_clicked();
	void on_arm_button_clicked();
	void on_disarm_button_clicked();
	void on_startR_button_clicked();
	void on_stopR_button_clicked();

	void on_checkbox_use_environment_stateChanged(int state);
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);
	QNode* returnQnode(){return &qnode;}


    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
		void updatePlotPoint(double x,double y, double z);
		void updateRPY(double roll,double pitch,double yaw);
		void updateMODE(std::string mode);
		void updateTARGET(float x,float y,float z);
		void updateNOWXYZ(float x,float y,float z);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
	MsgThread msgThread;



};

}  // namespace mydrone_gui_planner

#endif // mydrone_gui_planner_MAIN_WINDOW_H
