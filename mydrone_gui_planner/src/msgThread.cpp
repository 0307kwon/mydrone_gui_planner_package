#include <mydrone_gui_planner/msgThread.hpp>
#include <mydrone_gui_planner/qnode.hpp>

#include <ros/ros.h>


namespace mydrone_gui_planner {

MsgThread::MsgThread(QNode* q)
{
  qnode = q;
}

MsgThread::~MsgThread(){
  Q_EMIT finish();
}

void MsgThread::run(){
  ros::Rate rate(5);

  ros::Time last_request = ros::Time::now();
  ros::Time msg_last_request = ros::Time::now();
  while(ros::ok()){


    qnode->myRPYLogging();

    qnode->myLocationLogging();
/* gui의 로그 화면을 일정시간마다 지우는 코드
    if(ros::Time::now() - last_request > ros::Duration(10)){
      qnode->clearLogging();
      last_request = ros::Time::now();
    }
*/
    if(ros::Time::now()-msg_last_request >ros::Duration(0.1)){
      qnode->inputToTextfile();
      msg_last_request = ros::Time::now();
    }
    ros::spinOnce;
    rate.sleep();
  }
}

}//end namespace
