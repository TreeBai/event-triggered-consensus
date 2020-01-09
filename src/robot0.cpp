#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/UInt32.h"

#include "consensus/triggertime.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"

#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "math.h"


double k_x = 0.15, k_yaw = 1.0;
const double distance = 0.5;
const double delta = 0.02;

static double r0_x, r0_y, r1_x, r1_y, r2_x, r2_y, r3_x, r3_y;

static double r0_x_s, r0_y_s, r1_x_s, r1_y_s, r2_x_s, r2_y_s, r3_x_s, r3_y_s;

static double r0_d_s, r1_d_s, r2_d_s, r3_d_s;

static double r0_roll, r0_pitch, r0_yaw, r1_roll, r1_pitch, r1_yaw, r2_roll, r2_pitch, r2_yaw, r3_roll, r3_pitch, r3_yaw;

static double r0_v, r0_w, r1_v, r1_w, r2_v, r2_w, r3_v, r3_w;

static geometry_msgs::Twist cmd_value0, cmd_value1, cmd_value2, cmd_value3;

static int  r0_time,r1_time,r2_time,r3_time,flag_0,flag_1,flag_2,flag_3;

static consensus::triggertime tr_0,tr_1,tr_2,tr_3;


void robot0Callback(const nav_msgs::Odometry& odom)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r0_roll, r0_pitch, r0_yaw);
    
	r0_x = odom.pose.pose.position.x;
	r0_y = odom.pose.pose.position.y;
	r0_v = odom.twist.twist.linear.x;
	r0_w = odom.twist.twist.angular.z;
    
	if( sqrt( pow(r0_x - r0_x_s,2) + pow(r0_y - r0_y_s,2)) - delta> exp(-0.5*0.1*r0_time) )
	{
		r0_x_s = r0_x; 
		r0_y_s = r0_y;

		++flag_0;
	}

	double r0_d_s = sqrt(pow(r1_y_s - r0_y_s,2)+ pow(r1_x_s - r0_x_s,2) );
    /*
	double d = sqrt(pow(r1_y-r0_y,2)+ pow(r1_x-r0_x,2) );
	if(  fabs(r0_d_s - d) -distance > exp(-0.5*0.1*r0_time)    ){
		r0_d_s = d;
		++flag_0;
	}
    */
	++r0_time;

	tr_0.trigger = flag_0;
	tr_0.sample = r0_time;

	ROS_INFO("Robot1 sample times : %d; Robot1 trigger times: %d",r0_time, flag_0);

	double ref_yaw = atan2(r1_y-r0_y,r1_x-r0_x);
	

	cmd_value0.linear.y = 0;
	cmd_value0.linear.z = 0;
	cmd_value0.angular.x = 0;
	cmd_value0.angular.y = 0;
    //ROS_INFO("R0_yaw: %.2f",r0_yaw);
	//ROS_INFO("ref_yaw: %.2f",ref_yaw);
   // ROS_INFO( "dif_yaw: %.2f",fabs(r0_yaw - ref_yaw) );
   /*
	if( fabs(r0_yaw - ref_yaw) > 0.02 )
	{
        if(is_first) {cmd_value0.linear.x =  0;is_first=false;}
	    cmd_value0.angular.z = -k_yaw*(r0_yaw - ref_yaw);
	}
	else{
		cmd_value0.linear.x = k_x * sqrt(pow(r1_y-r0_y,2)+ pow(r1_x-r0_x,2)) ;
		cmd_value0.angular.z = 0;
	}
	*/
	cmd_value0.angular.z = -k_yaw*(r0_yaw - ref_yaw);
	cmd_value0.linear.x = k_x * (r0_d_s  -distance) ;
}

void robot1Callback(const nav_msgs::Odometry& odom)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r1_roll, r1_pitch, r1_yaw);

	r1_x = odom.pose.pose.position.x;
	r1_y = odom.pose.pose.position.y;
	r1_v = odom.twist.twist.linear.x;
	r1_w = odom.twist.twist.angular.z;

	 
	if( sqrt( pow(r1_x - r1_x_s,2) + pow(r1_y - r1_y_s,2)) - delta > exp(-0.5*0.1*r1_time) )
	{
		r1_x_s = r1_x; 
		r1_y_s = r1_y;

		++flag_1;
	}

	double r1_d_s = sqrt(pow(r1_y_s - r0_y_s,2)+ pow(r1_x_s - r0_x_s,2) );

	/*
	double d = sqrt(pow(r1_y-r0_y,2)+ pow(r1_x-r0_x,2) );
	if(  fabs(r1_d_s - d) -distance > exp(-0.5*0.1*r1_time)    ){
		r1_d_s = d;
		++flag_1;
	}
	*/
	++r1_time;

	tr_1.trigger = flag_1;
    tr_1.sample = r1_time;
	ROS_INFO("Robot2 sample times : %d; Robot2 trigger times: %d",r1_time, flag_1);

    double ref_yaw = atan2(r0_y-r1_y,r0_x-r1_x);

	cmd_value1.linear.y = 0;
	cmd_value1.linear.z = 0;
	cmd_value1.angular.x = 0;
	cmd_value1.angular.y = 0;
    /*
	if( fabs(r1_yaw - ref_yaw) > 0.05 )
	{
        cmd_value1.linear.x =  0;
	    cmd_value1.angular.z = -k_yaw*(r1_yaw - ref_yaw);
	}
	else{
		cmd_value1.linear.x = k_x *( r1_d_s - distance) ;
		cmd_value1.angular.z = 0;
	}
	*/
	
	cmd_value1.angular.z = -k_yaw*(r1_yaw - ref_yaw);
	cmd_value1.linear.x = k_x *( r1_d_s - distance) ;

}

void robot2Callback(const nav_msgs::Odometry& odom)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r2_roll, r2_pitch, r2_yaw);

	r2_x = odom.pose.pose.position.x;
	r2_y = odom.pose.pose.position.y;
	r2_v = odom.twist.twist.linear.x;
	r2_w = odom.twist.twist.angular.z;
	
	 
	if( sqrt( pow(r2_x - r2_x_s,2) + pow(r2_y - r2_y_s,2)) - delta > exp(-0.5*0.1*r2_time) )
	{
		r2_x_s = r2_x; 
		r2_y_s = r2_y;
		++flag_2;
	}

	double r2_d_s = sqrt(pow(r2_y_s - r1_y_s,2)+ pow(r2_x_s - r1_x_s,2) );
	/*
	double d = sqrt(pow(r1_y-r2_y,2)+ pow(r1_x-r2_x,2));
	if( fabs(r2_d_s - d) -distance > exp(-0.5*0.1*r2_time)  ){
		r2_d_s = d;
		++flag_2;
	}
	*/
	++r2_time;

	tr_2.trigger = flag_2;
    tr_2.sample = r2_time;
	ROS_INFO("Robot3 sample times : %d; Robot3 trigger times: %d",r2_time, flag_2);

    double ref_yaw = atan2(r1_y-r2_y,r1_x-r2_x);

	cmd_value2.linear.y = 0;
	cmd_value2.linear.z = 0;
	cmd_value2.angular.x = 0;
	cmd_value2.angular.y = 0;
    /*
	if( fabs(r2_yaw - ref_yaw) > 0.01 )
	{
        cmd_value2.linear.x =  0;
	    cmd_value2.angular.z = -k_yaw*(r2_yaw - ref_yaw);
	}
	else{
		cmd_value2.linear.x = k_x * sqrt(pow(r1_y-r2_y,2)+ pow(r1_x-r2_x,2)) ;
		cmd_value2.angular.z = 0;
	}
	*/
	cmd_value2.angular.z = -k_yaw*(r2_yaw - ref_yaw);
	cmd_value2.linear.x = k_x * ( r2_d_s -distance);
}

void robot3Callback(const nav_msgs::Odometry& odom)
{
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(r3_roll, r3_pitch, r3_yaw);

	r3_x = odom.pose.pose.position.x;
	r3_y = odom.pose.pose.position.y;
	r3_v = odom.twist.twist.linear.x;
	r3_w = odom.twist.twist.angular.z;
	
	 
	if( sqrt( pow(r3_x - r3_x_s,2) + pow(r3_y - r3_y_s,2)) - delta> exp(-0.5*0.1*r3_time) )
	{
		r3_x_s = r3_x; 
		r3_y_s = r3_y;
		++flag_3;
	}

	double r3_d_s = sqrt(pow(r2_y_s - r3_y_s,2)+ pow(r2_x_s - r3_x_s,2) );
	/*
	double d = sqrt(pow(r3_y-r2_y,2)+ pow(r3_x-r2_x,2));
	if(  fabs(r3_d_s - d) -distance > exp(-0.5*0.1*r3_time)    ){
		r3_d_s = d;
		++flag_3;
	}
	*/
	++r3_time;

	tr_3.trigger = flag_3;
    tr_3.sample = r3_time;
	ROS_INFO("Robot4 sample times : %d; Robot4 trigger times: %d",r3_time, flag_3);

    double ref_yaw = atan2(r2_y-r3_y,r2_x-r3_x);

	cmd_value3.linear.y = 0;
	cmd_value3.linear.z = 0;
	cmd_value3.angular.x = 0;
	cmd_value3.angular.y = 0;
    /*
	if( fabs(r3_yaw - ref_yaw) > 0.01 )
	{
        cmd_value3.linear.x =  0;
	    cmd_value3.angular.z = -k_yaw*(r3_yaw - ref_yaw);
	}
	else{
		cmd_value3.linear.x = k_x * sqrt(pow(r3_y-r2_y,2)+ pow(r3_x-r2_x,2)) ;
		cmd_value3.angular.z = 0;
	}
	*/
	cmd_value3.angular.z = -k_yaw*(r3_yaw - ref_yaw);
	cmd_value3.linear.x = k_x * ( r3_d_s -distance) ;
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "robot0");

	ros::NodeHandle n;
  
	ros::Publisher pub0 = n.advertise<geometry_msgs::Twist>("tb3_0/cmd_vel", 10);
    ros::Publisher pub1 = n.advertise<geometry_msgs::Twist>("tb3_1/cmd_vel", 10);
    ros::Publisher pub2 = n.advertise<geometry_msgs::Twist>("tb3_2/cmd_vel", 10);
	ros::Publisher pub3 = n.advertise<geometry_msgs::Twist>("tb3_3/cmd_vel", 10);
    
    ros::Publisher pub_t1 = n.advertise<consensus::triggertime>("triggertime1", 10);
	ros::Publisher pub_t2 = n.advertise<consensus::triggertime>("triggertime2", 10);
	ros::Publisher pub_t3 = n.advertise<consensus::triggertime>("triggertime3", 10);
	ros::Publisher pub_t4 = n.advertise<consensus::triggertime>("triggertime4", 10);
	
	ros::Subscriber sub0 = n.subscribe("tb3_0/odom", 10, robot0Callback);
	ros::Subscriber sub1 = n.subscribe("tb3_1/odom", 10, robot1Callback);
	ros::Subscriber sub2 = n.subscribe("tb3_2/odom", 10, robot2Callback);
	ros::Subscriber sub3 = n.subscribe("tb3_3/odom", 10, robot3Callback);

	
	ros::Rate loop_rate(10);

	
	
	while (ros::ok())
	{
		
		
		pub0.publish(cmd_value0);
		pub1.publish(cmd_value1);
		pub2.publish(cmd_value2);
		pub3.publish(cmd_value3);
        
		pub_t1.publish(tr_0);
		pub_t2.publish(tr_1);
		pub_t3.publish(tr_2);
		pub_t4.publish(tr_3);
        
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}