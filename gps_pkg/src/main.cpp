#include "serial_fun.cpp"
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Int32.h"




using namespace std;

int main(int argc,char **argv){
	ros::init(argc,argv,"GPS_LORA");
	ros::NodeHandle nh;
	ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps", 1); //location
	ros::Publisher gps_pub2 = nh.advertise<std_msgs::Int32>("/gps2",1);   //time
	sensor_msgs::NavSatFix gps_msg; 
	std_msgs::Int32 gps_msg2;
	string param;
	if(nh.getParam("/GPS_NODE/port",param))
    		cout<<"port : "<<param<<endl;
	Serial gps(param.c_str(),9600);
	if(!gps.good){
		ROS_INFO("GPS ERROR");
		return 0;
	}
	
	ROS_INFO("START NODE");
	int count = 0;
	vector<float> gps_data;
    while(ros::ok()){
				if(gps.good){
					ROS_INFO("[%d count]",count++);
					gps_data = get_gps(gps);
					if(gps_data.size()){
						gps_msg2.data = gps_data[0]+90000; 
						gps_msg.latitude = gps_data[1];
						gps_msg.longitude = gps_data[2];
						gps_msg.altitude = gps_data[3];
						gps_msg.header.stamp = ros::Time::now();
						gps_pub.publish(gps_msg);
						gps_pub2.publish(gps_msg2);
					}
				}
	}
    return 0; // success
}
