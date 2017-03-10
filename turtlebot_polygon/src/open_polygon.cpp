#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <getopt.h>


int main(int argc, char* argv[])
{
        ros::init(argc, argv, "openpolygon");
	ros::NodeHandle n;

	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
	ros::Rate loop_rate(60);

	int option, num_sides, length;
        //TODO: Add some flags to ensure that both arguments are obtained from the command line

	while ((option = getopt(argc, argv, "n:d:")) != -1)
	{
	  
	  switch(option)
	  {
	    case 'n':
	       // cout<< "The number of sides are :" << atof(optarg)<<endl;
               num_sides = atoi(optarg);
	       break;
	    case 'd':
	       // cout<< "The length of each side is :" << atof(optarg)<<endl;
               length = atoi(optarg);
	       break;
	    default:
	      std::cout<< "Didn't get the expected inputs"<<std::endl; 

	  }

	}
        //TODO: error checking for verifying the values of the input

	float poly_rot_angle_curr = 3.146 * 360/ (180* num_sides);
       	float poly_angle = 3.146 - poly_rot_angle_curr;

        //use MAX velocity and angular velocity
        double velocity = 0.5;
        double angular_velocity = 0.5;

        int number_of_intervals = 2 * num_sides;
        double* time_keeper = new double[number_of_intervals];
        double* lin_vel = new double[number_of_intervals];
        double* ang_vel = new double[number_of_intervals];

        //Adjust this parameter
        double initial_time_delay = 1.0;
        time_keeper[0] = poly_angle / angular_velocity + initial_time_delay;
        lin_vel[0] = 0.0;
        ang_vel[0] = angular_velocity;

        //Populate the time, linear velocity and angular velocity array
        for (int i = 1; i < number_of_intervals; i++)
        {
           if (i % 2 == 0)
           {
             time_keeper[i] = poly_rot_angle_curr / angular_velocity;
             lin_vel[i] = 0.0;
             ang_vel[i] = -angular_velocity;
           } 
	   else
           { 
             time_keeper[i] = length / velocity;
             lin_vel[i] = velocity;
             ang_vel[i] = 0.0;
           }      
         }
        
        geometry_msgs::Twist msg;
        int count = 0;
        double t;

	while (ros::ok() && count < number_of_intervals)
	{
	   ROS_INFO("count = %d \n", count);
	   ROS_INFO("lin_vel = %f \n", lin_vel[count]);
	   ROS_INFO("ang_vel = %f \n", ang_vel[count]);
           msg.linear.x = lin_vel[count];
           msg.angular.z = ang_vel[count];
           t = ros::Time::now().toSec();
           while (ros::Time::now().toSec() - t < time_keeper[count] )
                  pub.publish(msg);

           loop_rate.sleep();
           count++;
	}

        delete[] time_keeper;
        delete[] lin_vel;
        delete[] ang_vel;

        return 0;

}

