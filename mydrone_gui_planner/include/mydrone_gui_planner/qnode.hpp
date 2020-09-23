/**
 * @file /include/mydrone_gui_planner/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef mydrone_gui_planner_QNODE_HPP_
#define mydrone_gui_planner_QNODE_HPP_



/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <fstream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace mydrone_gui_planner{

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);
  float changeToRad(float a);
  float ChangeToDegree(float a);
  void myLocationLogging();
	void clearLogging();
  void myRPYLogging();
  void MODELogging(std::string mode);
  void TARGETLogging(float x,float y,float z);
  void inputToTextfile();
  void CommandArm(bool boolean);
  void CommandOffboard(bool boolean);
  void startRecord();
  void stopRecord();

  //서브스크라이버 콜백함수//
  static void position_cb(const nav_msgs::Odometry::ConstPtr& msg);
  static void state_cb(const mavros_msgs::State::ConstPtr& msg);
  static void odom_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
  static void velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg);

Q_SIGNALS:
	  void loggingUpdated();
    void rosShutdown();
    void sendPlotPoint(double x,double y,double z);
    void sendRPY(double roll, double pitch, double yaw);
    void sendMODE(std::string mode);
    void sendTARGET(float x,float y,float z);
    void sendNOWXYZ(float x,float y,float z);


private:
	int init_argc;
	char** init_argv;

 // 현재 위치 저장
  static double positionX;
  static double positionY;
  static double positionZ;
// 현재 드론 상태 받는 변수
  static mavros_msgs::State current_state;

  //목표지점 저장
  static float target_x;
  static float target_y;
  static float target_z;

  static float target_orient_x;
  static float target_orient_y;
  static float target_orient_z;
  static float target_orient_w;


  //rpy 관련//
  static double roll;
  static double pitch;
  static double yaw;

  //속도(현재 안씀)//
  static double velocity_x;
  static double velocity_y;
  static double velocity_z;



  bool offboard;
  bool arming;
  ros::Subscriber state_sub;
  ros::Subscriber position_sub;
  ros::Subscriber velocity_sub;
  ros::Subscriber odom_sub;

  ros::ServiceClient arming_client;
  ros::ServiceClient land_client;
  ros::Publisher local_pos_pub;
  ros::Publisher tf_pub;

  ros::ServiceClient set_mode_client;
  QStringListModel logging_model;



  //텍스트 파일 저장 관련 변수
  std::string filePath;
  std::ofstream writeFile;
  bool bool_recoding;
  //시간함수 : connect이후부터 시간을 잽니다.//
  ros::WallTime startTime;

  // log함수에서 사용
  QVariant new_row;



  // 끝




};

}  // namespace mydrone_gui_planner

#endif /* mydrone_gui_planner_QNODE_HPP_ */
