
#include "ros/ros.h"
#include <iostream>
#include "life_msgs/motor.h"
#include "life_msgs/distance.h"
#include "life_msgs/yolo.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include <Eigen/Dense>
#include <queue>
#define ROBOT_WEIGHT 5.5
#define SPEED_WEIGHT 5

using namespace std;
using namespace Eigen;

int dist = 10000;
bool is_find = false;
int step = 0;

struct Position{
  double x = 0;
  double y = 0;
  double z = 0;
  double abs_sum(){
    return abs(x) + abs(y) + abs(z);  
  };
}pos,force;


struct Rotation{
  double r = 0;
  double p = 0;
  double y = 0;
  ros::Time stamp;
}rot;


void DistFun(const life_msgs::distance& msg){
  dist = msg.dist;
};

queue<Rotation> rot_q;
Vector4d target_pos,view_target,unit;

void ImuFun(const sensor_msgs::Imu& msg){
  static int cnt = 0;
  rot.r = msg.orientation.x;
  rot.p = msg.orientation.y;
  rot.y = msg.orientation.z;
  rot.stamp = msg.header.stamp;
  cnt++;
  if(cnt == 10){
    rot_q.push(rot);
    while(rot_q.size()>50)
	rot_q.pop();
    cnt = 0;
  }
  Matrix4d roll,pitch,yaw,rotation;
  roll<< 1,0,0,0,
         0,cos(rot.r),-sin(rot.r),0,
         0,sin(rot.r),cos(rot.r),0,
         0,0,0,1;

  pitch<<cos(rot.p),0,sin(rot.p),0,
         0,1,0,0,
         -sin(rot.p),0,cos(rot.p),0, 
         0,0,0,1;

  yaw<<cos(rot.y),-sin(rot.y),0,0,
       sin(rot.y),cos(rot.y),0,0,
       0,0,1,0,
       0,0,0,1;

  rotation = yaw*pitch*roll;
  rotation(0,3) = pos.x;
  rotation(1,3) = pos.y;
  rotation(2,3) = pos.z;
  view_target = rotation.inverse()*target_pos;
  view_target(0) *= -1;
  view_target(2) *= -1;
  Vector4d x_temp(1,0,0,0),y_temp(0,1,0,0);
  force.x = ROBOT_WEIGHT*msg.linear_acceleration.x;
  force.y = ROBOT_WEIGHT*msg.linear_acceleration.y;
  force.z = ROBOT_WEIGHT*(msg.linear_acceleration.z+1);
  unit = rotation*x_temp + rotation*y_temp;
};

void CameraFun(const life_msgs::yolo& msg){
  Rotation pre = rot;
  if(step == 1)
	is_find = msg.result;
  while(rot_q.size()){
    if(msg.stamp >= rot_q.front().stamp){
      pre = rot_q.front();
      rot_q.pop();  
    }
    else
	break;
  }
  Matrix4d roll,pitch,yaw,rotation;
  roll<< 1,0,0,0,
         0,cos(pre.r),-sin(pre.r),0,
         0,sin(pre.r),cos(pre.r),0,
         0,0,0,1;

  pitch<<cos(pre.p),0,sin(pre.p),0,
         0,1,0,0,
         -sin(pre.p),0,cos(pre.p),0, 
         0,0,0,1;

  yaw<<cos(pre.y),-sin(pre.y),0,0,
       sin(pre.y),cos(pre.y),0,0,
       0,0,1,0,
       0,0,0,1;

  rotation = yaw*pitch*roll;
  rotation(0,3) = pos.x;
  rotation(1,3) = pos.y;
  rotation(2,3) = pos.z;
  double x_angle = -msg.x/320.0*85.0*3.1415/180.0;
  double y_angle = -msg.y/240.0*63.0*3.1415/180.0;
  Vector4d target(sin(x_angle)*dist,cos(x_angle)*dist,tan(y_angle)*dist,1);
  target_pos = rotation*target;
  
};

void OdomFun(const nav_msgs::Odometry& msg){
  static Position init_pos;
  static bool start = false;
  if(!start){
    init_pos.x = msg.pose.pose.position.x;
    init_pos.y = msg.pose.pose.position.y;
    init_pos.z = msg.pose.pose.position.z;
    start = true;
  }
  pos.x = -(msg.pose.pose.position.x - init_pos.x)*100;
  pos.y = (msg.pose.pose.position.y - init_pos.y)*100;
  pos.z = -(msg.pose.pose.position.z - init_pos.z)*100;
};
 

int main(int argc,char** argv){
  cout<<"Hello!"<<endl;
  ros::init(argc,argv,"ROBOT_CORE");
	ros::NodeHandle nh;
  ros::Publisher m_pub = nh.advertise<life_msgs::motor>("/motor", 1);
  ros::Publisher s_pub = nh.advertise<std_msgs::Int32>("/state", 1);
  ros::Subscriber dist_sub = nh.subscribe("/distance", 1 ,DistFun);
  ros::Subscriber imu_sub = nh.subscribe("/imu", 1 ,ImuFun);
  ros::Subscriber cam_sub = nh.subscribe("/cam", 1 ,CameraFun);
  ros::Subscriber odom_sub = nh.subscribe("/odom", 1 ,OdomFun);
  life_msgs::motor m_msg;
  std_msgs::Int32 state;
  
  int turn_cnt = 0;
  bool turn_flag = false;
  ros::Duration(10).sleep();
  while(ros::ok()){
    if(step == 0){
      m_msg.left = 1500;
      m_msg.right = 1500;
      if(abs(unit(2)) < 0.4){
        state.data = 2;      
      }
      else if(abs(unit(2)) < 0.7){
        state.data = 1;      
      }
      else{
        state.data = 0;      
      }
      if(state.data > 0){;
	 cout<<"force : "<<abs(force.abs_sum()-1)*ROBOT_WEIGHT<<endl;
         if(abs(force.abs_sum()-1)*ROBOT_WEIGHT > 500){
            step = 1;
            ros::Duration(2).sleep();      
	 }
      }
    }
    else if(step == 1){
      if(!is_find){
         if(!turn_flag){
           m_msg.left = 1500 + 20 - turn_cnt/10;
           m_msg.right = 1500;
         }
         else{
           m_msg.left = 1500;
           m_msg.right = 1500 + 20 - turn_cnt/10;
         }
         if(turn_cnt > 200){
           m_msg.left = 1500;
           m_msg.right = 1500;
           if(turn_cnt > 250){
             turn_flag = !turn_flag;
             turn_cnt = 0;
           }
         }
         turn_cnt++;
      }
      else
        step = 2;
    }
    else if(step == 2){
      double angle = atan2(view_target(0),view_target(1));
      cout<<"angle : "<<angle<<endl;
      double weight = angle*SPEED_WEIGHT;
      m_msg.left = 1520 + weight;
      m_msg.right = 1480 + weight;
      if(dist < 80){
      	m_msg.left = 1500;
       	m_msg.right = 1500;
        step = 3;
      }
    }
    else if(step == 3){
	ros::Duration(20).sleep();
	turn_cnt = 0;
        step = 4;
    }
    else{
      if(turn_cnt < 100){
         m_msg.left = 1480;
         m_msg.right = 1520;
      }
      else{
        turn_cnt = 0;
        m_msg.left = 1500;
        m_msg.right = 1500;
        step = 0;
        is_find = false;
      }
      turn_cnt++;
    }
    s_pub.publish(state);
    m_pub.publish(m_msg);
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  return 0;
}
