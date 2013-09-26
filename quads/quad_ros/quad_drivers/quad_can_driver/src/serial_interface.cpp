#include "quad_can_driver/serial_interface.h"

int dev_;
speed_t serialport_baud_;
unsigned char tx_buffer[150];

speed_t bitrate (int Bitrate){
	
    switch (Bitrate){
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      case 230400:
        return B230400;
      default:                 // invalid bitrate
        return B0;
    }
}

void InitSerialInterface (std::string serialport_name_, uint32_t serialport_speed_){
	struct termios tio;
      serialport_baud_ = bitrate (serialport_speed_);
      ROS_INFO ("Initializing serial port...");

      dev_ = open(serialport_name_.c_str (),O_RDWR | O_NOCTTY | O_NDELAY);
      ROS_ASSERT_MSG (dev_ != -1, "Failed to open serial port: %s %s", serialport_name_.c_str (), strerror (errno));

      ROS_ASSERT_MSG (tcgetattr (dev_, &tio) == 0, "Unknown Error: %s", strerror (errno));

      cfsetispeed (&tio, serialport_baud_);
      cfsetospeed (&tio, serialport_baud_);

      tio.c_iflag = 0;
      tio.c_iflag &= ~(BRKINT | ICRNL | IMAXBEL);
      tio.c_iflag |= IGNBRK;

      tio.c_oflag = 0;
      tio.c_oflag &= ~(OPOST | ONLCR);

      tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8;
      tio.c_cflag &= ~(PARENB | CRTSCTS | CSTOPB);

      tio.c_lflag = 0;
      tio.c_lflag |= NOFLSH;
      tio.c_lflag &= ~(ISIG | IEXTEN | ICANON | ECHO | ECHOE);

      ROS_ASSERT_MSG (tcsetattr (dev_, TCSADRAIN, &tio) == 0, "Unknown Error: %s", strerror (errno));

      tio.c_cc[VMIN] = 0;
      tio.c_cc[VTIME] = 0;

      tcflush (dev_, TCIOFLUSH);

      ROS_ASSERT_MSG (dev_ != -1, "Could not open serial port %s", serialport_name_.c_str ());
      ROS_INFO ("Successfully connected to %s, Baudrate %d\n", serialport_name_.c_str (), serialport_speed_);
 }

void CloseSerialInterface (void){
    ROS_INFO ("Destroying CAN Interface");
    close (dev_);
}

void AddCRC_CAN(unsigned int frame_length){ //length of #,adr,cmd,data
	
	unsigned short int tmpCRC = 0;
	unsigned int i;

	for (i=0; i < frame_length;i++)
	{
		tmpCRC += tx_buffer[i+3];
	}

	tx_buffer[16] = 0x00FF & tmpCRC;
	tx_buffer[17] = tmpCRC >> 8;
}
  
int sendStringToCom (unsigned char *output, int len){
	  
    int i;
    i = write (dev_, output, len);
	return i;
}

void SendOutData(unsigned char ID, str_ExternControl cmd, unsigned char N){

	int j=0;

	tx_buffer[0] = 0x55;               // Start-Byte
	tx_buffer[1] = 0x41;
	tx_buffer[2] = 0x56;
	tx_buffer[3] = ID;                // Adress
	tx_buffer[4] = 0X00;               
	tx_buffer[5] = 0X00;
	tx_buffer[6] = 0x00;
	tx_buffer[7] = N;
	tx_buffer[8] = cmd.Throttle;        // Commands
	tx_buffer[9] = (unsigned char) cmd.Yaw;
	tx_buffer[10] = (unsigned char) cmd.Pitch;
	tx_buffer[11] = (unsigned char) cmd.Roll;
	tx_buffer[12] = (unsigned char) cmd.Aux1;
	tx_buffer[13] = (unsigned char) cmd.Aux2;
	tx_buffer[14] = (unsigned char) cmd.Aux3;
	tx_buffer[15] = (unsigned char) cmd.Aux4;
	
	AddCRC_CAN(13);

	j=sendStringToCom(tx_buffer, 16+2);	//whole frame length is pt+3
										//#,adr,cmd,data ; crc1,crc2,\r
										
	if (j<0) ROS_WARN("Failed to write to COM");
}

