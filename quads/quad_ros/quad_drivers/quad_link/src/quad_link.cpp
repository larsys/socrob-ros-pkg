#include "ros/ros.h"
#include <sys/wait.h>


/* 
 * Open the link to quadrotor
 */

int main(int argc, char **argv)
{
	// Initializing ROS
	ros::init(argc, argv, "quad_link");
	ros::NodeHandle nh("~");
  
	system("/home/quadbase/paparazzi/sw/ground_segment/tmtc/link -s 57600 -d /dev/Zigbee_Link");

	//ros::spin();


  return 0;
  
/*	char programPath[] = "/home/quadbase/paparazzi/sw/ground_segment/tmtc/link";
	char arg1[] = "57600";
	char arg2[] = "/dev/Zigbee_Link";

	ros::Rate loop_rate(10);

	pid_t pid = fork(); 

	switch (pid) {
		case -1: 
			ROS_FATAL("Quad_Link: Fork Failed");
			exit(1);
			break;
		case 0: 
			//close(1);
			execlp(programPath,"link", "-s", arg1, "-d",arg2,NULL); 
			ROS_FATAL("Quad_Link: Can't run 'link' program"); 
			exit(3);
			break;
		default: 
			int status;
			while (ros::ok()){
				ros::spinOnce();
				loop_rate.sleep();
				waitpid(pid, &status, WNOHANG); 
				if(status!=0){
					//ROS_FATAL("Para");
					 break;
				 }
			}
			
			//ROS_FATAL("'dapsijdksaj");
			break;
	}
	return 0;*/
}
