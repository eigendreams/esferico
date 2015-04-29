#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>

#include <iostream>
#include "imu_lib.h"
#include <cmath>

#include "volti_msgs/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

///////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_node");
  ros::NodeHandle n;

  ros::Publisher imu1_pub = n.advertise<volti_msgs::float32_3>("imu_pendu_3", 1);
  ros::Publisher imu2_pub = n.advertise<volti_msgs::float32_3>("imu_plate_3", 1);

  //publicar cada 20 ms
  ros::Rate loop_rate(50);

  // modulos de salida en el BBB
  IMU imu_plate(1);
  IMU imu_pendu(2);

  volti_msgs::float32_3 imu_plate_msg;
  volti_msgs::float32_3 imu_pendu_msg;

  int unsigned long times = 0;

  while (ros::ok())
  {
    imu_plate.loop();
    imu_pendu.loop();

    times++;

    if (times % 5 == 1) {

    imu_plate_msg.data[0] = imu_plate.roll;
    imu_plate_msg.data[1] = imu_plate.pitch;
    imu_plate_msg.data[2] = imu_plate.yaw;
    imu1_pub.publish(imu_plate_msg);

    imu_pendu_msg.data[0] = imu_pendu.roll;
    imu_pendu_msg.data[1] = imu_pendu.pitch;
    imu_pendu_msg.data[2] = imu_pendu.yaw;
    imu2_pub.publish(imu_pendu_msg);

    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}