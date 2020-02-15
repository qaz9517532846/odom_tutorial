#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <math.h>

double x;
double y;
double th;

float vx;
float vy;
float vth;

float encode_1_vel;
float encode_2_vel;
float encode_vel;
float encode_th;

float vel_right;
float vel_left;

int i = 0;
int a,b;

ros::Time current_time;
ros::Time last_time;

class agv_position {
public:
    ros::NodeHandle n;

    ros::Publisher odom_pub;
    ros::Subscriber encode_1;
    ros::Subscriber encode_2;
    ros::Subscriber vel_1;
    ros::Subscriber vel_2;

    tf::TransformBroadcaster odom_broadcaster;

    void encode1_vel(const std_msgs::Float64::ConstPtr& msg);
    void encode2_vel(const std_msgs::Float64::ConstPtr& msg);
    void vel_A(const std_msgs::Float64::ConstPtr& msg);
    void vel_B(const std_msgs::Float64::ConstPtr& msg);
};

void agv_position::vel_A(const std_msgs::Float64::ConstPtr& msg)
{ 
  vel_right = msg->data;
  if(vel_right < 0)
  {
    a = -1;
  }
  else
  {
    a = 1;
  }
}

void agv_position::vel_B(const std_msgs::Float64::ConstPtr& msg)
{ 
  vel_left = msg->data;
  if(vel_left < 0)
  {
    b = -1;
  }
  else
  {
    b = 1;
  }
}

void agv_position::encode2_vel(const std_msgs::Float64::ConstPtr& msg)
{
  encode_2_vel = msg->data * 0.01 *b;
  ROS_INFO("vy = %f", encode_2_vel);
  encode_vel = (encode_1_vel + encode_2_vel) / 2;
  encode_th = (encode_1_vel - encode_2_vel) / 0.27;
}


void agv_position::encode1_vel(const std_msgs::Float64::ConstPtr& msg)
{
  agv_position pose;

  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  pose.odom_broadcaster.sendTransform(odom_trans);

  encode_1_vel = msg->data * 0.01 *a;
  ROS_INFO("vx = %f", encode_1_vel);

  vx = encode_vel * cos(th) * dt;
  vy = encode_vel * sin(th) * dt;
  vth = encode_th * dt;

  x += vx;
  y += vy;
  th += vth;
  ROS_INFO("x = %f", x);
  ROS_INFO("y = %f", y);
  ROS_INFO("th = %f", th);
  ROS_INFO("dt = %f", dt);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";

  //publish the message
  odom_pub.publish(odom);
  i++;
  last_time = current_time;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_encoder");

  agv_position pose;

  current_time = ros::Time::now();

  pose.odom_pub = pose.n.advertise<nav_msgs::Odometry>("odom", 50);
  
  pose.vel_1 = pose.n.subscribe("velA", 1000, &agv_position::vel_A, &pose);
  pose.vel_2 = pose.n.subscribe("velB", 1000, &agv_position::vel_B, &pose);
  pose.encode_1 = pose.n.subscribe("encoder1_value", 1000, &agv_position::encode1_vel, &pose);
  pose.encode_2 = pose.n.subscribe("encoder2_value", 1000, &agv_position::encode2_vel, &pose);
  ros::spin();
}
