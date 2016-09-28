#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
 
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <sys/ioctl.h>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	ros::Rate loop_rate(10);

	char *i2cdevName = "/dev/i2c-7";
	const unsigned char i2cAddress = 0x29;
	int fd;

	//struct timeval tv;
	//gettimeofday(tv,NULL);

/* まず，日時・時刻情報を格納するための変数を宣言 */
    struct timeval myTime,myTime2;    // time_t構造体を定義．1970年1月1日からの秒数を格納するもの
    struct tm *time_st,*time_st2;       // tm構造体を定義．年月日時分秒をメンバ変数に持つ構造体
    const char weekName[7][4] = {   // 曜日は数字としてしか得られないので，文字列として用意
        "Sun",
        "Mon",
        "Tue",
        "Wed",
        "Thu",
        "Fri",
        "Sat"
    };

    /* 時刻取得 */
    gettimeofday(&myTime, NULL);    // 現在時刻を取得してmyTimeに格納．通常のtime_t構造体とsuseconds_tに値が代入される
    time_st = localtime(&myTime.tv_sec);    // time_t構造体を現地時間でのtm構造体に変換

	usleep(2000);

    gettimeofday(&myTime2, NULL);    // 現在時刻を取得してmyTimeに格納．通常のtime_t構造体とsuseconds_tに値が代入される
    time_st2 = localtime(&myTime2.tv_sec);    // time_t構造体を現地時間でのtm構造体に変換

    printf("Date : %d/%02d/%02d(%s) %02d:%02d:%02d.%06d\n%d sec\n%d msec\n%d sec*1000\n",     // 現在時刻
                time_st->tm_year+1900,     // 年
                time_st->tm_mon+1,         // 月
                time_st->tm_mday,          // 日
                weekName[time_st->tm_wday],// 曜日
                time_st->tm_hour,          // 時
                time_st->tm_min,           // 分
                time_st->tm_sec,           // 秒
                myTime.tv_usec,            // マイクロ秒
		myTime.tv_sec,
                myTime.tv_usec/1000,
		myTime.tv_sec*1000
		);
	printf("%d ms\n", (myTime.tv_sec ) * 1000 + (myTime.tv_usec ) / 1000);

printf("Date : %d/%02d/%02d(%s) %02d:%02d:%02d.%06d\n%d sec\n%d msec\n%d sec*1000\n",     // 現在時刻
                time_st2->tm_year+1900,     // 年
                time_st2->tm_mon+1,         // 月
                time_st2->tm_mday,          // 日
                weekName[time_st2->tm_wday],// 曜日
                time_st2->tm_hour,          // 時
                time_st2->tm_min,           // 分
                time_st2->tm_sec,           // 秒
                myTime2.tv_usec,            // マイクロ秒
		myTime2.tv_sec,
                myTime2.tv_usec/1000,
		myTime2.tv_sec*1000
		);
	printf("%d ms\n", (myTime2.tv_sec ) * 1000 + (myTime2.tv_usec ) / 1000);

	printf("\n\n%d ms\n\n", (myTime2.tv_sec - myTime.tv_sec ) * 1000 + (myTime2.tv_usec - myTime.tv_usec ) / 1000);
	printf("\n\n%d s\n\n", (myTime2.tv_sec - myTime.tv_sec ) );

	if((fd = open(i2cdevName,O_RDWR)) < 0){
		fprintf(stderr,"Can not open i2c port\n");
		return 1;
	}
	if (ioctl(fd, I2C_SLAVE,i2cAddress) < 0) {
		fprintf(stderr,"Unable to get bus access to talk to slave\n");
		return 1;
	}

	

	int count = 0;
	while (ros::ok())
	{
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		//ROS_INFO("%s", msg.data.c_str());

		chatter_pub.publish(msg);

		ros::spinOnce();


		i2c_smbus_write_byte_data(fd,0x00,0x01);
		//i2c_smbus_read_byte_data(fd,0x1f);
		int cnt = 0;
		while(cnt < 100){
			sleep(0.01);
			__s32 val = i2c_smbus_read_byte_data(fd,0x14);
			val = val & 0xff;
			if(val & 0x01) break;
			cnt += 1;
		}

		__s32 pressxl = i2c_smbus_read_byte_data(fd,0x1f);
		__s32 pressl  = i2c_smbus_read_byte_data(fd,0x1e);
	
		pressxl = pressxl & 0xff;
		pressl  = pressl & 0xff;
	
		__s32 press = (pressl <<8 ) | pressxl ;

		printf("%8.4f\n", (float)press);

		loop_rate.sleep();
		++count;
	}


	return 0;
}
