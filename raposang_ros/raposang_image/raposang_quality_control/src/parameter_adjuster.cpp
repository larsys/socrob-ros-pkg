/***

If you have some question about this program, please send me message.
keiya.okada(at mark)gmail.com

Copyright (C) 2011 Keiya Okada All Rights Reserved.

***/

/*

This program can adjust parameters (now jpeg_quality ratio for each camera) automatically.
But this program uses "system" function to adjust parameters because "dynparam" of "dynamic_raconfigure" is not yet supported.
Please see bellow link;
http://www.ros.org/wiki/hokuyo_node/Tutorials/UsingDynparamToChangeHokuyoLaserParameters#Using_dynparam_from_C.2B-.2B-_code

**Caution**
This program has not been tested whether work well or not.

***Future Works***
This program uses two "system" functions because now republisher of RAPOSA subscribes and publishes each image from right and left in several.
I think these images should be subscribes and publishes in same topic.
And jpeg_quality also should be integrated and changed simultaneously in same topic.

This program should be integrated with 

!!THIS PROGRAM SHOULD BE PUT ON RAPOSA!!

*/

/***********/
/* Include */
/***********/

// setting for usign queue //

#include <iostream>
#include <queue>

// setting unistd //

#include <unistd.h> // is to use system functuion

// includes from depedend packages //

#include <ros/ros.h>  
#include <ros/time.h> // to get current time as ROS time. " http://www.ros.org/wiki/roscpp/Overview/Time " 

// includes message type //

#include "std_msgs/Time.h"
#include "std_msgs/Int32.h"   // is used to get the time difference
#include "std_msgs/Float32.h" // is used to publish RTT for another node especially GUI 

/**********/
/* Define */
/**********/

#define size_of_queue 10 // length of queue
#define decrease_ratio 5 // decreasing rate of jpeg_qualityn ratio
#define increase_ratio 5 // increasing rate of jpeg_quality ratio
#define max_comp 80      // maximal jpeg_quality ratio
#define min_comp 5       // minimal jpeg_quality ratio
#define check_period 5   // grace_period. now set as 5 sec
#define rtt_min 10.0     // if RTT between RAPOSA and teleop pc is lower than this value, jpeg_quality is increased 
#define rtt_max 100.0    // if RTT between RAPOSA and teleop pc is larger than this value, jpeg_quality is decreased

/*************/
/* Namespace */
/*************/

using namespace std_msgs; // shortens "std_msgs::Int32"
using namespace std;      // namespace to use queue

/****************/
/* Global value */
/****************/

ros::Publisher rtt_pub; // declares gloval publisher to send moving average of RTT for another node especially GUI
ros::Time getting_time;     // this value is used in "request_received" only but I put it on here because I wanted to remove extra time
Float32 sending_rtt;    // is message to publish moving average of RTT for another node especially GUI
queue<float> qu;        // value of queue to put RTT
int ping_count;         // denominator of average RTT
int grace_period;       // if this value reachs "check_period", it is possible for jpeg_quality to be changed
int jpeg_quality;       // jpeg_quality of image
float average_rtt;      // is used to calculates moving average of RTT
char left_quality[1024], right_quality[1024]; // to put command line to use system function

/**********************/
/* moving_average_rtt */
/**********************/

/* 
This function is to calculate RTT with moving average concept.
Data type of function is "float" 
Parameter of average is current (latest) RTT.
*/

float moving_average_rtt(float current_rtt){
    
    if(ping_count < size_of_queue){ // calculates average RTT for first some pings 
        average_rtt += current_rtt; // calculates total value of RTT
        qu.push(current_rtt);       // pushes current RTT on queue
        ping_count++;               // increases the total number of pings
    }
    
    else{                                                     // calculates average RTT for "size_of_queue";
        average_rtt = average_rtt - qu.front() + current_rtt; // subtract olest RTT from total value of RTT and add current RTT
        qu.pop();                                             // trashes oldest FPS  
        qu.push( current_rtt );                               // pushes current RTT on queue
    }
    return ( average_rtt / (float)ping_count ); // return average RTT
}

/***********************/
/* compressed_decrease */
/***********************/

/* 
This function is to decrease jpeg_quality automatically.
Data type of function is "int"
"system" function is used in this function. 
Parameter of average is current jpeg_quality".
*/

int compressed_decrease(int compressed_quality){
    ROS_INFO("called decrease");
    if(compressed_quality >= (decrease_ratio + min_comp)){        // checks current jpeg_quality not to exceed "min_comp"
        compressed_quality = compressed_quality - decrease_ratio; // decreases jpeg_quality
        
        // write command line in left_quality[1024] and right_quality[1024] with updated jpeg_quality// 
               
        sprintf(left_quality, "rosrun dynamic_reconfigure dynparam set /bb2/left_for_video/compressed jpeg_quality %d", compressed_quality);
        sprintf(right_quality, "rosrun dynamic_reconfigure dynparam set /bb2/right_for_video/compressed jpeg_quality %d", compressed_quality);
        
        // change jpeg_quality directly //
        
        system(left_quality);
        system(right_quality);
    } 
    ROS_INFO("compressed_quality: %d", compressed_quality);
    return compressed_quality; // return updated jpeg_quality
  
}

/***********************/
/* compressed_increase */
/***********************/

/* 
This function is to increase jpeg_quality automatically.
Data type of function is "int"
"system" function is used in this function. 
Parameter of average is current jpeg_quality".
*/

int compressed_increace(int compressed_quality){
    ROS_INFO("called increase");
    if((compressed_quality + increase_ratio) <= max_comp){        // checks current jpeg_quality not to exceed "max_comp"
        compressed_quality = compressed_quality + increase_ratio; // increases jpeg_quality
        
        // write command line in left_quality[1024] and right_quality[1024] with updated jpeg_quality//
        
        sprintf(left_quality, "rosrun dynamic_reconfigure dynparam set /bb2/left_for_video/compressed jpeg_quality %d", compressed_quality);
        sprintf(right_quality, "rosrun dynamic_reconfigure dynparam set /bb2/right_for_video/compressed jpeg_quality %d", compressed_quality);
        
        // change jpeg_quality directly //
        
        system(left_quality);
        system(right_quality);
    }
    ROS_INFO("compressed_quality: %d", compressed_quality);
    return compressed_quality; // return updated jpeg_quality
   
}

/*********************/
/* response_received */
/*********************/

/* 
This function receives request from streaming_prove.cpp
This function also publishes average RTT for another node. 
*/

void response_received(const TimeConstPtr& get_request)
{
    getting_time = ros::Time::now(); // gets current time and put it as data on message 

    double rtt = 0.0;     // declares value to put current rtt and formats it
    float ave_rtt = 0.0; // declares value to put average rtt and formats it 
    
    //rtt = (getting_time.secs-get_request->data.secs)
    
    rtt = ((getting_time - get_request->data).toSec() * 1000); // calculates current RTT. 
                                                                      // getting_time.data has current time.
                                                                      // get_request->data has time when request was sent by streaming_manager.cpp
    ave_rtt = moving_average_rtt(rtt); // calculates average RTT 

    // check average RTT is larger than "rtt_max" and lower than "rtt_min" and ping count is larger than "check_period" //

    if((ave_rtt > rtt_max) && (grace_period >= check_period)){ // average RTT is larger than "rtt_max" and ping count is larger than "check_period"
        jpeg_quality = compressed_decrease(jpeg_quality);      // calls "compressed_decrease" function
        grace_period = 0; // formats grace_period
    }
    else if((ave_rtt < rtt_min) && (grace_period >= check_period)){ // average RTT is lower than "rtt_min" and ping count is larger than "check_period"
        jpeg_quality = compressed_increace(jpeg_quality);           // calls "compressed_increase" function
        grace_period = 0; // formats grace_period
    }
    
    grace_period++; // increases grace_period
    
    sending_rtt.data = rtt;       // puts average RTT on message  
    rtt_pub.publish(sending_rtt); // publishes average RTT for another node
    
    //ros::spinOnce();
    ROS_INFO("rtt: %f ms  average of %d pings: %f ms  compression ratio: %d", rtt, ping_count, ave_rtt, jpeg_quality);
}

/********/
/* main */
/********/

int main(int argc, char **argv)
{
    ping_count = 0;   // formats ping_count
    grace_period = 0; // formats grace_period

	ros::init(argc, argv, "parameter_adjuster"); // declares name of node
    ros::NodeHandle nh;                          // nodehandle for this program
    
    nh.getParam("/bb2/right_for_video/compressed/jpeg_quality", jpeg_quality); // puts "/bb2/right_for_video/compressed/jpeg_quality" parameter on string "s".
                                                                               // this line is temporary!! Please read "Future Works" written on the top of this program.
  	ROS_INFO("jpeg_quality: %d", jpeg_quality); // shows jpeg_quality
  	
  	//jpeg_quality = 1; // test part. This line should be deleated.

    rtt_pub = nh.advertise<Float32>("average_rtt", 1);                           // tells the master that we are going to be publishing a message of type std_msgs/Float32
    ros::Subscriber get_request= nh.subscribe<Time>("response", 1, response_received); // subscribes response from streaming_prove.cpp
	ros::spin();
}
