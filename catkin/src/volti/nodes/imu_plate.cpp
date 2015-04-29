#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

#include <sstream>
#include <iostream>
#include <cmath>

#include "imu_lib.h"
#include "volti_msgs/float32_3.h"

using namespace std;

float pi= 3.1415926535897;

int main(int argc, char **argv) {

  ros::init(argc, argv, "imu_plate");
  ros::NodeHandle nh;

  int    rate_num;
  string topic_name;
  int    i2c_num;

  nh.param("rate", rate_num, int(50));
  nh.param("topic", topic_name, string("imu_plate_3"));
  nh.param("i2c", i2c_num, int(2));

  ros::Publisher imu_pub = nh.advertise<volti_msgs::float32_3>(topic_name.c_str(), 1);
  ros::Rate loop_rate(rate_num);
  IMU imu(i2c_num);
  volti_msgs::float32_3 imu_msg;

  unsigned long times = 0;

  while (nh.ok()) {

    imu.loop();

    times++;

    // publish every 100 ms, wait till end of first period
    if ((times % (rate_num / 10)) == 1) {

      imu_msg.data[0] = imu.roll;
      imu_msg.data[1] = imu.pitch;
      imu_msg.data[2] = imu.yaw;
      imu_pub.publish(imu_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
