// ------------------- INCLUDE -------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include <iostream>

#include "ros/ros.h"

// -------------------- DEFINE --------------------

#define ODOMETRY_FREQ 10

#define MAX_ANGLE  30.0
#define MAX_ANGLE_DIF  5.0
#define MIN_ANGLE -20.0
#define CLICK_TO_DIST 0.000035069
#define ANGLE_TO_VALUE 4.1875
#define ANGLE_TO_VALUE_OFFSET 477.0
#define TIME_BEFORE_STOP 1

#define BAUDRATE 			B57600
#define MSG_SIZE 			8
#define BUF_SIZE 			512
#define WAIT_PROBE 	  100000
#define WAIT_READ    	15000
#define USB_MAX 			10
#define NP 						2

#define SET_VELOCITY	'V'
#define GET_DELTA_XY	'J'
#define CLR_DELTA_XY  'K'
#define SET_ARMANGLE	'B'
#define GET_ARMANGLE	'H'

#define GET_BATTERY_STATS	'Q' 
#define SET_RELES_CONTROL 'C' // Not Implemented (don't wanna screw the system :s)

#define is_bigendian 					((*(char*)&test_endian)==0)

#define PI                    3.14159265
#define value_from_vel(x)			((970*(x/0.091))/(2*3.14159))

//#define value_from_angle(x)		(4.1875*x+477.0)
//#define angle_from_value(x)		((x-477.0)/4.1875)

// ----------------- STRUCT ENUM ------------------

enum { MOTORS, RELAYS };

struct floatlr {
  float left;
	float right;
};

struct shortlr {
  short left;
	short right;
};

struct battery_status {

	// EB: Electronics Battery
	// MB: Motors Battery
	// ES: External Source

	short EB_Voltage;
	short MB_Voltage;
	short ES_Voltage;
	short Relay_Status;
	bool is_ES_On;
	bool is_EB_Relay_On;
	bool is_MB_Relay_On;
	bool is_ES_Relay_On;
};

// ------------------- FUNCTION -------------------

int RAPOSA_USB_Serial_Open(const char *filename, speed_t baud);

void RAPOSA_USB_Serial_Close(int fd);

void RAPOSA_Error_Exit(const char * msg, const int p);

short RAPOSA_Convert_to_BigEndian(short s);

short RAPOSA_Convert_from_BigEndian(short s);

void RAPOSA_USB_Serial_Write(int fd, unsigned char *command, int msg_size);

int RAPOSA_USB_Serial_Read(int fd, unsigned char *response, int msg_size);

int RAPOSA_Get_Command_Size(char command);

int RAPOSA_Get_Response_Size(char command);

bool RAPOSA_Verify_Checksum(unsigned char* buf, int msg_size);

void RAPOSA_Dumper(unsigned char *out, int len);

bool RAPOSA_USB_Serial_Transaction(int fd, unsigned char* cmd, int cmd_size, unsigned char* msg, int msg_size);

void RAPOSA_Set_Velocity(const int p, short v_left, short v_right);

struct shortlr RAPOSA_Get_Delta_XY_Clicks(const int p);

void RAPOSA_Clear_Delta_XY_Clicks(const int p);

void RAPOSA_Set_Arm_Inclination(const int p, short arm_value);

short RAPOSA_Get_Arm_Inclination(const int p);

battery_status RAPOSA_Get_Battery_Status(const int p);

void RAPOSA_Set_Reles(const int p, unsigned char order);

bool RAPOSA_Check_Ports(int fd, int type);
