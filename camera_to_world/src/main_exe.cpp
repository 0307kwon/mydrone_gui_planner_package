#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

using namespace std;



tf::Transform transform_base;

void position_cb(const nav_msgs::Odometry::ConstPtr& msg){

  tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  q.normalize();

  transform_base.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z) );
  transform_base.setRotation(q);

}

int main(int argc, char** argv){
  ros::init(argc,argv,"camera_to_world");
  ros::NodeHandle nh;

  tf::TransformBroadcaster br;
  tf::TransformListener listener;

  ros::Subscriber position_sub = nh.subscribe("mavros/local_position/odom", 10, position_cb);

  ros::Publisher tf_pub = nh.advertise<geometry_msgs::PoseStamped>("ctw/tf_pub", 10);


  cout<<"프로그램 시작"<<endl;

  ros::Rate rate(20.0);
  while(nh.ok()){

    br.sendTransform(tf::StampedTransform(transform_base, ros::Time::now(), "world", "fp_base_link"));

    //카메라 tf
    tf::Transform transform_camera;

    transform_camera.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );

    tf::Quaternion q_orig, q_rot, q_new;
    q_orig = tf::Quaternion(0, 0, 0, 1);
    double r=-1.570795;
    double p=0;
    double y=-1.570795;
    q_rot.setRPY(r, p, y);

    q_new = q_rot*q_orig;
    q_new.normalize();

    transform_camera.setRotation( q_new );
    br.sendTransform(tf::StampedTransform(transform_camera, ros::Time::now(), "fp_base_link", "zedm_center_camera_optical_frame"));
    //

        tf::StampedTransform transform;

				try
				{
       		listener.lookupTransform("/world", "zedm_center_camera_optical_frame",
                                ros::Time(0), transform);
     		}catch (tf::TransformException ex){
       		ROS_WARN("%s",ex.what());
       		ros::Duration(1.0).sleep();
          continue;
     		}

        geometry_msgs::PoseStamped camera_tf;
        camera_tf.header.frame_id = "zedm_center_camera_optical_frame";

				camera_tf.header.stamp = ros::Time::now();
				camera_tf.pose.position.x = transform.getOrigin().x();
				camera_tf.pose.position.y = transform.getOrigin().y();
				camera_tf.pose.position.z = transform.getOrigin().z();

				camera_tf.pose.orientation.x = transform.getRotation().x();
				camera_tf.pose.orientation.y = transform.getRotation().y();
				camera_tf.pose.orientation.z = transform.getRotation().z();
				camera_tf.pose.orientation.w = transform.getRotation().w();

        tf_pub.publish(camera_tf);



    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}


///
