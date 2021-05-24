#include <Servo.h>
#include <ros.h>
#include <life_msgs/motor.h>

ros::NodeHandle  nh;
Servo left,right;


void messageCb( const life_msgs::motor& msg){
  left.writeMicroseconds(msg.left);
  right.writeMicroseconds(msg.right);
}

ros::Subscriber<life_msgs::motor> sub("motor", &messageCb );
void setup() {
  left.attach(5);
  right.attach(6);
  left.writeMicroseconds(1500);
  right.writeMicroseconds(1500);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
}
