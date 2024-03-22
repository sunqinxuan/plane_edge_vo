#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<std_msgs/Int64.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#define KEYCODE_SPACE 0x20
#define KEYCODE_Q 0x71

bool flag_stop;
 
void callback()
{
	char c;
	int kfd=0;

	read(kfd, &c, 1);

	if(c==KEYCODE_Q)
	{
		flag_stop=true;
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "go_three_second");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("RosAria/cmd_vel", 1000);
	ros::Publisher pub_flag = nh.advertise<std_msgs::Int64>("flag", 1000);
	geometry_msgs::Twist msg;
	std_msgs::Int64 flag;

	double BASE_SPEED = 0.2, MOVE_TIME = 3.0, CLOCK_SPEED = 3.0;
	int count = 0;
	ros::Rate rate(CLOCK_SPEED);
	flag_stop=false;

	// Make the robot stop (robot perhaps has a speed already)
	msg.linear.x = 0;
	msg.linear.y = 0;
	msg.linear.z = 0;
	msg.angular.x = 0;
	msg.angular.y = 0;
	msg.angular.z = 0;
	pub.publish(msg);

//	ros::Subscriber sub = nh.subscribe("flag",1,callback);
//	message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
//	syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(5));
//	syncExact->registerCallback(boost::bind(&callback, this));

	while(ros::ok())// && count<1000)//MOVE_TIME/CLOCK_SPEED + 1)
	{
//		if(flag_stop)
//		{
//			msg.linear.x=0;
//			msg.angular.z=0;
//			flag.data=99999999999;
//			pub.publish(msg);
//			pub_flag.publish(flag);
//			break;
//		}

		if(count<1000000)
		{
			msg.linear.x = 0.2;
		}
		else if(count<2020170)
		{
			msg.linear.x = 0.2;
		}
		else
		{
			msg.linear.x=0.1;
			msg.angular.z=0.1;
		}

		if(count>4092056)
			break;
		pub.publish(msg);
		//ROS_INFO_STREAM("The robot is now moving forward!");

		count++;
		flag.data=count;
		pub_flag.publish(flag);

		std::cout<<count<<"\t"<<flag.data<<std::endl;
		//ros::spinOnce();
		//rate.sleep();
	}

	// make the robot stop
	for (int i = 0; i < 2; i++)
	{  
		msg.linear.x=0;
		msg.angular.z=0;
		pub.publish(msg);
		pub_flag.publish(flag);
	}
	//ROS_INFO_STREAM("The robot finished moving forward three seconds!");

	// Guard, make sure the robot stops.
	rate.sleep();
	msg.linear.x=0;
	msg.angular.z=0;
	pub.publish(msg); 
	pub_flag.publish(flag);

}
