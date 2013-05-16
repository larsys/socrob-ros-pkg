#include <raposang_driver.h>

const int test_endian = 1;

// ------------------------------------------------
// Function: [Serial Open]
// ------------------------------------------------
int RAPOSA_USB_Serial_Open(const char *filename, speed_t baud) {

  int r = open(filename, O_RDWR|O_NOCTTY);
  if (r!=-1) {
    struct termios term;
    tcgetattr(r, &term);
    cfmakeraw(&term);
		cfsetspeed(&term, baud);
    tcsetattr(r, TCSANOW, &term);
    tcflush(r, TCIOFLUSH);
  }
  return r;
}

// ------------------------------------------------
// Function: [Serial Close]
// ------------------------------------------------
void RAPOSA_USB_Serial_Close(int fd) {

  fsync(fd);
  close(fd);
}

// ------------------------------------------------
// Function: [Error Exit]
// ------------------------------------------------
void RAPOSA_Error_Exit(const char * msg, const int p) {

	char err[BUF_SIZE];
  char* err_str = strerror_r(errno, err, BUF_SIZE);

	//fprintf(stderr, "[RAPOSA] ERROR - %s (%s)\n", msg, err_str);

	for(int i=0;i<NP;i++) {
		if (p != -1)
			RAPOSA_USB_Serial_Close(p);
	}
	exit(0);
}

// ------------------------------------------------
// Function: [Convert to BigEndian]
// ------------------------------------------------
short RAPOSA_Convert_to_BigEndian(short s) {

  unsigned char c1, c2;
  if (is_bigendian) {
    return s;
  } else {
    c1 = s & 255;
    c2 = (s >> 8) & 255;
    return (c1 << 8) + c2;
  }
}

// ------------------------------------------------
// Function: [Convert from BigEndian]
// ------------------------------------------------
short RAPOSA_Convert_from_BigEndian(short s) {

  return RAPOSA_Convert_to_BigEndian(s);
}

// ------------------------------------------------
// Function: [USB Serial Write]
// ------------------------------------------------
void RAPOSA_USB_Serial_Write(int fd, unsigned char *command, int msg_size) {
	
	write(fd, command, msg_size*sizeof(unsigned char));
}

// ------------------------------------------------
// Function: [USB Serial Read]
// ------------------------------------------------
int RAPOSA_USB_Serial_Read(int fd, unsigned char *response, int msg_size) {

	int sizeread = 0;
	int value = 0;
  fd_set rfds;
	struct timeval timeout = {1, 0};

  FD_ZERO(&rfds);
	FD_SET(fd, &rfds);

	if(select(fd+1, &rfds, NULL, NULL, &timeout)>0 && FD_ISSET(fd, &rfds)) {
		while(sizeread < msg_size) {
			value = read(fd, &response[sizeread], (msg_size-sizeread)*sizeof(unsigned char));
			if(value < 0) 
				return -1;
			sizeread += value;
		} 
	} else {
		return 0;
	}
    
	return sizeread;
}

// ------------------------------------------------
// Function: [Get Command Size]
// ------------------------------------------------

int RAPOSA_Get_Command_Size(char command) {

	switch(command) {
		// Motors
		case SET_VELOCITY: return 5;
		case GET_DELTA_XY: return 1;
		case CLR_DELTA_XY: return 1;
		case SET_ARMANGLE: return 3;
		case GET_ARMANGLE: return 1;
		// Relays
		case GET_BATTERY_STATS: return 1;
		case SET_RELES_CONTROL: return 2;
	}
	return 0;
}

// ------------------------------------------------
// Function: [Get Response Size]
// ------------------------------------------------

int RAPOSA_Get_Response_Size(char command) {

	switch(command) {
		// Motors
		case SET_VELOCITY: return 4;
		case GET_DELTA_XY: return 8;
		case CLR_DELTA_XY: return 4;
		case SET_ARMANGLE: return 4;
		case GET_ARMANGLE: return 6;
		// Relays
		case GET_BATTERY_STATS: return 8;
		case SET_RELES_CONTROL: return 4;
	}
	return 0;
}

// ------------------------------------------------
// Function: [Verify Checksum]
// ------------------------------------------------
bool RAPOSA_Verify_Checksum(unsigned char* buf, int msg_size) {

	int i = 0;
	short value = 0;
	unsigned char checkvalue[2];

	for(i=0;i<msg_size-2;i++) {
		value += (short) buf[i];
	}
	value = RAPOSA_Convert_to_BigEndian(value);

	memcpy(checkvalue,&value,sizeof(short));

	if((buf[msg_size-2] == checkvalue[0]) && (buf[msg_size-1] == checkvalue[1])) 
		return true;
	else
		return false;
}

// ------------------------------------------------
// Function: [Dumper]
// ------------------------------------------------
void RAPOSA_Dumper(unsigned char *out, int len) {
  int j;

  printf("[RAPOSA] Dumper: ");
  for (j=0 ; j<len ; j++) {
    printf(" 0x%02x", out[j]);
  }
  printf("   ");
  for (j=0 ; j<len ; j++) {
    putchar(isprint(out[j])?out[j]:'.');
  }
  printf("\n");
}

// ------------------------------------------------
// Function: [RAPOSA_USB_Serial_Transaction]
// ------------------------------------------------

bool RAPOSA_USB_Serial_Transaction(int fd, unsigned char* cmd, int cmd_size, unsigned char* msg, int msg_size) {

	for(int i=0;i<msg_size;i++) {
		msg[i]='A';
	}

	RAPOSA_USB_Serial_Write(fd, cmd, cmd_size);

	usleep(WAIT_READ);

	int sread = RAPOSA_USB_Serial_Read(fd, msg, msg_size);

	if((sread==msg_size) && (msg[0] == cmd[0]) && RAPOSA_Verify_Checksum(msg, msg_size)) 
		return true;	

	ROS_ERROR("[RAPOSANG-Driver] Non-sucessfull reading (%d of %d).", sread, msg_size);

	return false;
}

// ------------------------------------------------
// Function: [Set Velocity]
// ------------------------------------------------

void RAPOSA_Set_Velocity(const int p, short v_left, short v_right) {

	unsigned char cmd[MSG_SIZE];
	unsigned char msg[MSG_SIZE];

	short v_left_BE  = RAPOSA_Convert_to_BigEndian(v_left);
	short v_right_BE = RAPOSA_Convert_to_BigEndian(v_right);

	cmd[0] = SET_VELOCITY;
	memcpy(&(cmd[1]), &v_left_BE,  sizeof(short));
	memcpy(&(cmd[3]), &v_right_BE, sizeof(short));

	RAPOSA_USB_Serial_Transaction(p, 
								cmd, RAPOSA_Get_Command_Size(cmd[0]), 
								msg, RAPOSA_Get_Response_Size(cmd[0]));
}

// ------------------------------------------------
// Function: [Get Delta XY Clicks]
// ------------------------------------------------

struct shortlr RAPOSA_Get_Delta_XY_Clicks(const int p) {

	unsigned char cmd[MSG_SIZE];
	unsigned char msg[MSG_SIZE];

	struct shortlr clicks_BE, clicks;

	cmd[0] = GET_DELTA_XY;

	RAPOSA_USB_Serial_Transaction(p, 
								cmd, RAPOSA_Get_Command_Size(cmd[0]), 
								msg, RAPOSA_Get_Response_Size(cmd[0]));

	memcpy(&(clicks_BE.left) , &(msg[1]), sizeof(short));
	memcpy(&(clicks_BE.right), &(msg[3]), sizeof(short));

	clicks.left  = RAPOSA_Convert_from_BigEndian(clicks_BE.left);
	clicks.right = RAPOSA_Convert_from_BigEndian(clicks_BE.right);

	return clicks;
}

// ------------------------------------------------
// Function: [Clear Delta XY Clicks]
// ------------------------------------------------

void RAPOSA_Clear_Delta_XY_Clicks(const int p) {

	unsigned char cmd[MSG_SIZE];
	unsigned char msg[MSG_SIZE];

	cmd[0] = CLR_DELTA_XY;

	RAPOSA_USB_Serial_Transaction(p, 
								cmd, RAPOSA_Get_Command_Size(cmd[0]), 
								msg, RAPOSA_Get_Response_Size(cmd[0]));
}

// ------------------------------------------------
// Function: [Set Arm Inclination]
// ------------------------------------------------

void RAPOSA_Set_Arm_Inclination(const int p, short arm_value) {

	unsigned char cmd[MSG_SIZE];
	unsigned char msg[MSG_SIZE];

	short arm_value_BE  = RAPOSA_Convert_to_BigEndian(arm_value);

	cmd[0] = SET_ARMANGLE;
	memcpy(&(cmd[1]), &arm_value_BE, sizeof(short));

	RAPOSA_USB_Serial_Transaction(p, 
								cmd, RAPOSA_Get_Command_Size(cmd[0]), 
								msg, RAPOSA_Get_Response_Size(cmd[0]));
}


// ------------------------------------------------
// Function: [Get Arm Inclination]
// ------------------------------------------------

short RAPOSA_Get_Arm_Inclination(const int p) {

	unsigned char cmd[MSG_SIZE];
	unsigned char msg[MSG_SIZE];

	short arm_value_BE;
	cmd[0] = GET_ARMANGLE;

	RAPOSA_USB_Serial_Transaction(p, 
								cmd, RAPOSA_Get_Command_Size(cmd[0]), 
								msg, RAPOSA_Get_Response_Size(cmd[0]));

	memcpy(&arm_value_BE, &(msg[1]), sizeof(short));

	return RAPOSA_Convert_from_BigEndian(arm_value_BE);
}

// ------------------------------------------------
// Function: [Get Battery Status]
// ------------------------------------------------

battery_status RAPOSA_Get_Battery_Status(const int p) {

	unsigned char cmd[MSG_SIZE];
	unsigned char msg[MSG_SIZE];

	battery_status bat;

	cmd[0] = GET_BATTERY_STATS;

	RAPOSA_USB_Serial_Transaction(p, 
								cmd, RAPOSA_Get_Command_Size(cmd[0]), 
								msg, RAPOSA_Get_Response_Size(cmd[0]));

	// From experimental tests, contrary to RAPOSA manual:
	//  1 - Motors Batt.
  //	2 - Electronic Batt.
	//  3 - Cable Batt.

	bat.MB_Voltage = (short) msg[1];
	bat.EB_Voltage = (short) msg[2];
	bat.ES_Voltage = (short) msg[3];
	bat.Relay_Status = (short) msg[4];

	bat.is_ES_On       = ((msg[4] & 128)==128)?true:false;
	bat.is_EB_Relay_On = ((msg[4] &   4)==  4)?true:false;
	bat.is_ES_Relay_On = ((msg[4] &   2)==  2)?true:false;
	bat.is_MB_Relay_On = ((msg[4] &   1)==  1)?true:false;

	return bat;
} 

// ------------------------------------------------
// Function: [Get Battery Status]
// ------------------------------------------------

void RAPOSA_Set_Reles(const int p, unsigned char order) {

	unsigned char cmd[MSG_SIZE];
	unsigned char msg[MSG_SIZE];

	cmd[0] = SET_RELES_CONTROL;
	memcpy(&(cmd[1]), &order, sizeof(char));

	RAPOSA_USB_Serial_Transaction(p, 
								cmd, RAPOSA_Get_Command_Size(cmd[0]), 
								msg, RAPOSA_Get_Response_Size(cmd[0]));
}

// ------------------------------------------------
// Function: [Check Ports]
// ------------------------------------------------
bool RAPOSA_Check_Ports(int fd, int type) {

	unsigned char buf[MSG_SIZE];
	unsigned char cmd = '\0';
	int msg_size;
	int sizeread = 0;
	int value = 0;

	switch(type) {
		case MOTORS: 
			cmd = GET_DELTA_XY; 
			break;
		case RELAYS: 
			cmd = GET_BATTERY_STATS; 
			break;
	}

	msg_size = RAPOSA_Get_Response_Size(cmd);
  RAPOSA_USB_Serial_Write(fd, &cmd, 1);
	usleep(WAIT_PROBE);

	while(sizeread < msg_size) {
		value = read(fd, &buf[sizeread], (msg_size-sizeread)*sizeof(unsigned char));
		if(value > 0) 
			sizeread += value;
	} 

	//RAPOSA_Dumper(buf, msg_size);
	if((sizeread==msg_size) && (buf[0] == cmd) && RAPOSA_Verify_Checksum(buf, msg_size)) 
		return true;

	return false;
}

/* EOF */
