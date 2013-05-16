#include <stdio.h>
#include <sys/termios.h>
#include <sys/ioctl.h>
#include <cstring>
#include <unistd.h>
#include <cstdlib>
#include <time.h>
#include <errno.h>
#include <bitset>
#include <math.h>
#include <sys/types.h>
#include <fcntl.h>
#include "ros/ros.h"

struct str_ExternControl
{
unsigned char ID;	 //two 1/0-buttons, not used atm
unsigned char N; // ? something to control the lcd menues?!
unsigned char Throttle;			
signed char   Yaw;
signed char   Pitch;
signed char   Roll;
signed char   Aux1;
signed char   Aux2;
signed char   Aux3;
signed char   Aux4;
};

extern struct str_ExternControl ExternControl;	//create an instance of the struct

void InitSerialInterface (std::string serialport_name_, uint32_t serialport_speed_);
void CloseSerialInterface (void);
speed_t bitrate (int Bitrate);
void SendOutData(unsigned char ID, struct str_ExternControl cmd, unsigned char N);
void AddCRC_CAN(unsigned int frame_length);
int sendStringToCom (unsigned char *output, int len);
	

