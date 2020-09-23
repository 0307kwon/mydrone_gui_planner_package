	/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <mydrone_gui_planner/qnode.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/State.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <tf/tf.h>
#include <fstream>
#include <time.h>
//오일러 to 쿼터니언 관련 //
#include <tf2/LinearMath/Quaternion.h>

#define PI 3.14159265358979323846


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace mydrone_gui_planner{

		//정적 변수 초기화
		double QNode::positionX = 0;
		double QNode::positionY = 0;
		double QNode::positionZ = 0;
		mavros_msgs::State QNode::current_state;

		//초기 드론 control 값//
		float QNode::target_x = 0.0;
		float QNode::target_y = 0.0;
		float QNode::target_z = 0.35;

		float QNode::target_orient_x = 0.0;
		float QNode::target_orient_y = 0.0;
		float QNode::target_orient_z = 0.0;
		float QNode::target_orient_w = 1.0;

		double QNode::roll=0;
		double QNode::pitch=0;
		double QNode::yaw=0;
		double QNode::velocity_x=0;
		double QNode::velocity_y=0;
		double QNode::velocity_z=0;


		//


	//서브스크라이버 콜백함수
	void QNode::state_cb(const mavros_msgs::State::ConstPtr& msg){
			current_state = *msg;
	}

	void QNode::position_cb(const nav_msgs::Odometry::ConstPtr& msg){
		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
		tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

		positionX = msg->pose.pose.position.x;
		positionY = msg->pose.pose.position.y;
		positionZ = msg->pose.pose.position.z;
	}

	void QNode::odom_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){
		target_x = msg->position.x;
		target_y = msg->position.y;
		//target_z = msg->position.z;
		target_z = 0.35; //0.35m 로 고정 (임시) //


		//yaw 값을 쿼터니언 값으로 바꿉니다.//
		tf2::Quaternion q_new;
		q_new.setRPY(0,0,msg->yaw);

		target_orient_x = q_new.getX();
		target_orient_y = q_new.getY();
		target_orient_z = q_new.getZ();
		target_orient_w = q_new.getW();

		//std::cout << q_new.getX() <<" "<<q_new.getY() << " " <<q_new.getZ() <<" "<< q_new.getW() <<std::endl;

	}


	void QNode::velocity_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
		velocity_x = msg->twist.linear.x;
		velocity_y = msg->twist.linear.y;
		velocity_z = msg->twist.linear.z;
	}




/*****************************************************************************
** Implementation
*****************************************************************************/



QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{
		arming = false;
		offboard = false;
		bool_recoding = false;
		filePath = "";

	}

	void QNode::inputToTextfile(){

		if(writeFile.is_open() &&
					bool_recoding == true) {
			//ROS_INFO("저장됨");
			/*
			writeFile << ros::WallTime::now()-startTime << "," << positionX << "," << positionY << "," << positionZ
			<< "," << roll << "," << pitch << "," << yaw
			<< "," << target_x << "," << target_y << "," << target_z
			<< "," << velocity_x << "," << velocity_y << "," << velocity_z
			<< "\n";
			*/
		}
	}

QNode::~QNode() {
    if(ros::isStarted()) {

//텍스트 파일 저장 종료//
			if( writeFile.is_open()){
				writeFile.close();
			}
			////


      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}



bool QNode::init() { // ros 노드 실행하는 코드인

	ros::init(init_argc,init_argv,"mydrone_gui_planner");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // 이거 왜필요하지 explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;

	startTime = ros::WallTime::now();//시작시간 기록//


	// Add your ros communications here.
	state_sub = n.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	position_sub = n.subscribe("mavros/local_position/odom", 10, QNode::position_cb);
	velocity_sub = n.subscribe("mavros/local_position/velocity_body",10,velocity_cb);
	odom_sub = n.subscribe<quadrotor_msgs::PositionCommand>("planning/pos_cmd", 10, odom_cb); // 목표지점 받음//

	local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10); // arming 미션//

	set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");// offboard 설정//
	arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming"); // arming
	land_client = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land"); // land

	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) { // qnode 수동설정으로 init

	return true;
}

void QNode::run() { // 여기서 계속 publish loop

	ros::Rate rate(20.0);

	while(ros::ok() && !current_state.connected){

    ros::spinOnce();// 콜백함수 호출을 위한 함수로써, 메시지가 수신되기를 대기, 수신되었을 경우 콜백함수를 실행한다
    rate.sleep();

	}

	geometry_msgs::PoseStamped pose;
	pose.header.stamp = ros::Time::now();
	pose.header.frame_id = "body";

	mavros_msgs::SetMode offb_set_mode;
	offb_set_mode.request.custom_mode = "OFFBOARD";
  mavros_msgs::CommandBool arm_cmd;
	mavros_msgs::CommandTOL land_cmd;

  ros::Time last_request = ros::Time::now();

	std::stringstream ss;
	ss << "initial altitude is " << QNode::target_z << " m";

	log(Info,ss.str());

		while(ros::ok()){
			MODELogging(std::string(current_state.mode));

			// service 날리는 부분 //

			if(offboard == true){ // offboard 명령을 받으면//
				TARGETLogging(target_x,target_y,target_z); //target 로깅//
				//offboard 모드가 아니면 지속적으로 offboard 모드 진입 명령을 보냄//
				offb_set_mode.request.custom_mode = "OFFBOARD";
					if( current_state.mode != "OFFBOARD" &&
							(ros::Time::now() - last_request > ros::Duration(5.0))){
								if( set_mode_client.call(offb_set_mode) &&
									offb_set_mode.response.mode_sent){
										log(Info,"Offboard enabled");
									}
									last_request = ros::Time::now();
					}else{ // offboard로 모드가 바뀌면 //
						//arming이나 disarming 수행
						arm_cmd.request.value = arming;
						if(arm_cmd.request.value == true){ // arm 명령을 줄때 //
            	if( !current_state.armed &&
                	(ros::Time::now() - last_request > ros::Duration(5.0))){
                	if( arming_client.call(arm_cmd) && // arming 신호를 보냄//
                    	arm_cmd.response.success){
                    	log(Info,"Vehicle armed");
                	}
                	last_request = ros::Time::now();// 일정시간마다 service 날림//
            	}
						}else{ // disarm 명령을 줄 때 //
							// land후에 disarmed
							land_cmd.request.altitude = 0;
							if((ros::Time::now() - last_request > ros::Duration(5.0))){
								if(!land_cmd.response.success){
									land_client.call(land_cmd);
								}else{
									if(current_state.armed){
										if(arming_client.call(arm_cmd) && arm_cmd.response.success){
											last_request = ros::Time::now();
											log(Info,"Vehicle disarmed");
										}
									}
								}
								last_request = ros::Time::now(); // 일정시간마다 service 날림//
							}
						}
					}
			}else{ // onboard 명령을 받으면 //
				//STABILIZED 상태로 진입//
				offb_set_mode.request.custom_mode = "STABILIZED";
				if( current_state.mode == "OFFBOARD" &&
						(ros::Time::now() - last_request > ros::Duration(5.0))){
							if( set_mode_client.call(offb_set_mode) &&
								offb_set_mode.response.mode_sent){
									log(Info,"Onboard enabled");
								}
								last_request = ros::Time::now();
				}
			}

			//topic 날리는 부분 //

			if(offboard == true && arm_cmd.request.value == true) {//offboard상태, arming 상태 일때만 위치 publish
				pose.pose.position.x = target_x;
				pose.pose.position.y = target_y;
				pose.pose.position.z = target_z;

				pose.pose.orientation.x = target_orient_x;
				pose.pose.orientation.y = target_orient_y;
				pose.pose.orientation.z = target_orient_z;
				pose.pose.orientation.w = target_orient_w;

				local_pos_pub.publish(pose);
			}

      ros::spinOnce();
      rate.sleep();
    }

}


void QNode::CommandOffboard(bool boolean){
	/*
	if(boolean == true){ //OFFBOARD 모드 명령이 떨어지면 TARGET X,Y,Z를 기본설정해줌//
		target_x = positionX;
		target_y = positionY;
		target_z = positionZ;
	}
	*/
	offboard = boolean;
}

void QNode::CommandArm(bool boolean){
	arming = boolean;
}




float QNode::changeToRad(float a){
	return a*PI/180;
}

float QNode::ChangeToDegree(float a){
	return a*(180/PI);
}

void QNode::myLocationLogging(){
	/*
	std::stringstream ss;
	ss.str("");
	ss << "x :" << std::setprecision(2) <<positionX << " y : " << positionY << " z : " << positionZ;
	log(Info,ss.str());
	*/

	Q_EMIT sendNOWXYZ(positionX,positionY,positionZ);

	Q_EMIT sendPlotPoint(positionX,positionY,positionZ);

}

void QNode::myRPYLogging(){
	Q_EMIT sendRPY(ChangeToDegree(roll),ChangeToDegree(pitch),ChangeToDegree(yaw));
}

void QNode::MODELogging(std::string mode){
	Q_EMIT sendMODE(mode);
}

void QNode::TARGETLogging(float x,float y,float z){
	Q_EMIT sendTARGET(x,y,z);
}

void QNode::clearLogging(){
	logging_model.removeRows(0, logging_model.rowCount());
}

void QNode::startRecord(){
	if(filePath == ""){ // 첫 startRecord버튼 누를시 //
		//파일 새로 생성 //
		//텍스트 파일 저장 관련 변수
				filePath = "/home/kwon/catkin_ws/drone_Info.txt";
				writeFile.open(filePath.data());

				writeFile << "time(sec),"<<"position_x," << "position_y," << "position_z,"
				<< "roll," << "pitch," << "yaw,"
				<< "target_x," << "target_y," << "target_z,"
				<< "velocity_x," << "velocity_y," << "velocity_z,"
				<< "\n";
		// 끝
	}
	bool_recoding = true;
}

void QNode::stopRecord(){
	bool_recoding = false;
}



void QNode::log( const LogLevel &level, const std::string &msg) {
	ros::WallTime timeNow = ros::WallTime::now();
	std::stringstream logging_model_msg;
	logging_model.insertRows(logging_model.rowCount(),1);
	logging_model_msg.str("");
	switch ( level ) {
		case(Debug) : {
				logging_model_msg << "[DEBUG] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Info) : {
				logging_model_msg << "[INFO] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Warn) : {
				logging_model_msg << "[INFO] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Error) : {
				logging_model_msg << "[ERROR] [" << timeNow-startTime << "]: " << msg;
				break;
		}
		case(Fatal) : {
				logging_model_msg << "[FATAL] [" << timeNow-startTime << "]: " << msg;
				break;
		}
	}

	new_row = QString(logging_model_msg.str().c_str());
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // 로그 업데이트 시그널을 줘서 main_window.cpp의 자동스크롤 함수(슬롯)를 call
}

}  // namespace mydrone_gui_planner
