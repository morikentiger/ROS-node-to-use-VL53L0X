/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include <stdio.h>
#include <stddef.h>	//#define NULL ...
#include <linux/i2c-dev.h>
#include <test_vl53l0x/VL53L0X.h>


VL53L0X sensor;


// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY


void setup()
{
  //Serial.begin(9600);
  //Wire.begin();
  
	//ROS_INFO("testsetup");
  sensor.init();
  //ROS_INFO("testsetup2");
  sensor.setTimeout(500);
  //sensor.setTimeout(2000);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  //ROS_INFO("testsetup3");
  sensor.setSignalRateLimit(0.1);
//  sensor.setSignalRateLimit(0.25);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  //ROS_INFO("testsetup4");
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);//18);
  //ROS_INFO("testsetup5");
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);//14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  //ROS_INFO("testsetup6");
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  //ROS_INFO("testsetup7");
  sensor.setMeasurementTimingBudget(200000);
#endif
}

int main(int argc, char **argv)
{
	////ROS_INFO("test0");
	ros::init(argc, argv, "Single_VL53L0X");
	ros::NodeHandle n;
	printf("test\n");
	//ROS_INFO("test");
	setup();
	
	//ROS_INFO("test2");
	
	//ROS_INFO("test3");
	
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("distance_m", 1000);

	//ROS_INFO("test4");
	ros::Rate loop_rate(10);

	//ROS_INFO("test5");
	int count = 0;
	while (ros::ok())
	{
		//ROS_INFO("test6");
		std_msgs::String msg;
		std_msgs::Float64 distance;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		
		  //ROS_INFO(sensor.readRangeSingleMillimeters());
		  //if (sensor.timeoutOccurred()) { ROS_INFO(" TIMEOUT"); }
		  float distance_m = sensor.readRangeSingleMillimeters();
		distance_m = distance_m/1000;
		  printf("readRangeSingleMillimeters:%lf\n",distance_m);
		  if (sensor.timeoutOccurred()) { printf(" TIMEOUT_loop\n"); }
		  //ROS_INFO("testloopend");
		
		distance.data = distance_m;

		//ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(distance);

		//ROS_INFO("test7");
		ros::spinOnce();

		//ROS_INFO("test8");
		//ROS_INFO("testloop");
		

		//ROS_INFO("test9");
		loop_rate.sleep();
		++count;
	}

	//ROS_INFO("testEnd");
	//ROS_INFO("testEnd");
	printf("testEnd\n");
	return 0;

}
