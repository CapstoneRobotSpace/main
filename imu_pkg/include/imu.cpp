#include "imu.h"

IMU::IMU(ros::NodeHandle &nh,std::string port, unsigned int baudrate) : _nh(nh),iMyAhrsPlus(port, baudrate) {
  _pub = _nh.advertise<sensor_msgs::Imu>("/imu",1);
}

IMU::~IMU(){};

bool IMU::initialize() {
        bool ok = false;
        do {
            if(start() == false) break;
            if(cmd_binary_data_format("QUATERNION, IMU") == false) break;
            if(cmd_divider(DIVIDER) == false) break;
            if(cmd_mode("BC") == false) break;
            ok = true;
        } while(0);

        return ok;
}

inline void IMU::get_data(SensorData& data) {
    LockGuard _l(lock);
    data = sensor_data;
}

inline SensorData IMU::get_data() {
    LockGuard _l(lock);
    return sensor_data;
}

void IMU::operate_data(int sensor_id) {
  _imu_msg.header.stamp = ros::Time::now();

  _imu_msg.orientation.x = sensor_data.euler_angle.roll*DEGREE_TO_RADIAN;
  _imu_msg.orientation.y = sensor_data.euler_angle.pitch*DEGREE_TO_RADIAN;
  _imu_msg.orientation.z = sensor_data.euler_angle.yaw*DEGREE_TO_RADIAN;
  _imu_msg.orientation.w = 0;

  _imu_msg.angular_velocity.x = sensor_data.imu.gx;
  _imu_msg.angular_velocity.y = sensor_data.imu.gy;
  _imu_msg.angular_velocity.z = sensor_data.imu.gz;

  _imu_msg.linear_acceleration.x = sensor_data.imu.ax;
  _imu_msg.linear_acceleration.y = sensor_data.imu.ay;
  _imu_msg.linear_acceleration.z = sensor_data.imu.az;

  _pub.publish(_imu_msg);

}

void IMU::OnSensorData(int sensor_id, SensorData data) {
    {
        LockGuard _l(lock);
        sensor_data = data;
        sensor_data.euler_angle = sensor_data.quaternion.to_euler_angle();
        operate_data(sensor_id);
    }
    
}

void IMU::OnAttributeChange(int sensor_id, std::string attribute_name, std::string value) {
    printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
}
