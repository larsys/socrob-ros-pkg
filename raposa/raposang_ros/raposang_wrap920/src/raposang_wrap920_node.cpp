
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>
 
 /*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
  *
  * If you need this, please have your distro update the kernel headers.
  */
 #ifndef HIDIOCSFEATURE
 #define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
 #define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
 #endif
 
 /* Unix */
 #include <sys/ioctl.h>
 #include <sys/types.h>
 #include <sys/stat.h>
 #include <fcntl.h>
 #include <unistd.h>
 
 /* C */
 #include <stdio.h>
 #include <string.h>
 #include <stdlib.h>
 #include <errno.h>

#include "ros/ros.h"
#include "raposang_msgs/ImuRaw.h"

#define DEFAULT_DEV_PATH "/dev/wrap920"
 
const char *bus_str(int bus);
 
 
int main(int argc, char **argv) {
  int fd;
  int i, res, desc_size = 0;
  unsigned char buf[256];
  char ncbuf[256];
  struct hidraw_report_descriptor rpt_desc;
  struct hidraw_devinfo info;
  const char *dev_path = DEFAULT_DEV_PATH;

  int *ptr;
  short *lala;
  int aux;
  char * pEnd;
  long int lil, li2, li3, li4;

  if (argc>1) dev_path=argv[1];
 
  /* Open the Device with blocking reads. In real life,
     don't use a hard coded path; use libudev instead. */
		
  printf("Opening device %s\n", dev_path);
  fd = open(dev_path, O_RDONLY); /* O_RDWR|O_NONBLOCK); */

  if (fd < 0) {
    perror("Unable to open device");
    return 1;
  }
 
     
  memset(&rpt_desc, 0x0, sizeof(rpt_desc));
  memset(&info, 0x0, sizeof(info));
  memset(buf, 0x0, sizeof(buf));
 
  /* Get Report Descriptor Size */
  res = ioctl(fd, HIDIOCGRDESCSIZE, &desc_size);
  if (res < 0)
    perror("HIDIOCGRDESCSIZE");
  else
    printf("Report Descriptor Size: %d\n", desc_size);
  
  /* Get Report Descriptor */
  rpt_desc.size = desc_size;
  res = ioctl(fd, HIDIOCGRDESC, &rpt_desc);
  if (res < 0) {
    perror("HIDIOCGRDESC");
  } else {
    printf("Report Descriptor:\n");
    for (i = 0; i < rpt_desc.size; i++)
      printf("%hhx ", rpt_desc.value[i]);
    puts("\n");
  }
  
  /* Get Raw Name */
  res = ioctl(fd, HIDIOCGRAWNAME(256), buf);
  if (res < 0)
    perror("HIDIOCGRAWNAME");
  else
    printf("Raw Name: %s\n", buf);
  
  /* Get Physical Location */
  res = ioctl(fd, HIDIOCGRAWPHYS(256), buf);
  if (res < 0)
    perror("HIDIOCGRAWPHYS");
  else
    printf("Raw Phys: %s\n", buf);
  
  /* Get Raw Info */
  res = ioctl(fd, HIDIOCGRAWINFO, &info);
  if (res < 0) {
    perror("HIDIOCGRAWINFO");
  } else {
    printf("Raw Info:\n");
    printf("\tbustype: %d (%s)\n", info.bustype, bus_str(info.bustype));
    printf("\tvendor: 0x%04hx\n", info.vendor);
    printf("\tproduct: 0x%04hx\n", info.product);
  }

  /* Set Feature */
  buf[0] = 0x9; /* Report Number */
  buf[1] = 0xff;
  buf[2] = 0xff;
  buf[3] = 0xff;
  res = ioctl(fd, HIDIOCSFEATURE(4), buf);
  if (res < 0)
    perror("HIDIOCSFEATURE");
  else
    printf("ioctl HIDIOCGFEATURE returned: %d\n", res);

  /* Get Feature */
  buf[0] = 0x9; /* Report Number */
  res = ioctl(fd, HIDIOCGFEATURE(256), buf);
  if (res < 0) {
    perror("HIDIOCGFEATURE");
  } else {
    printf("ioctl HIDIOCGFEATURE returned: %d\n", res);
    printf("Report data (not containing the report number):\n\t");
    for (i = 0; i < res; i++)
      printf("%hhx ", buf[i]);
    puts("\n");
  }
  
  /* Get a report from the device */
  res = read(fd, buf, 42);
  if (res < 0) {
    perror("read");
  } else {
    printf("read() read %d bytes:\n\t", res);  
    
  }
  
  ros::init(argc, argv, "raposang_wrap920");
  ros::NodeHandle n;  
  /* WTF? ros::NodeHandle nh("~"); */
	
  ros::Publisher pubImu = n.advertise<raposang_msgs::ImuRaw>("wrap920", 5);		
  
  while (ros::ok()) {
    res = read(fd, buf, 42);
    if (res > 0) {

      /* DEBUG STUFF */
      if (false) {
        int i;

        if (buf[0]==0x01 && buf[1]==0x80) {
          printf("---");
          for (i=0 ; i<42 ; i++)
            printf(" %02x", buf[i]);
          printf("\r");
        } else {
          printf("\n***");
          for (i=0 ; i<42 ; i++)
            printf(" %02x", buf[i]);
          printf("\n");
        }
      }

      if (buf[0]==0x01 && buf[1]==0x80) {
	raposang_msgs::ImuRaw imu;

        imu.header.stamp = ros::Time::now();

        lala = (short *) (buf+2); 
        imu.magnetometer[0] = *lala;     
        lala = (short *) (buf+4);           
        imu.magnetometer[1] = *lala;    
        lala = (short *) (buf+6);  
        imu.magnetometer[2] = *lala; 
      
        lala = (short *) (buf+8); 
        imu.accelerometer[0] = *lala;     
        lala = (short *) (buf+10);           
        imu.accelerometer[1] = *lala;    
        lala = (short *) (buf+12);  
        imu.accelerometer[2] = *lala;
 
        lala = (short *) (buf+18); 
        imu.gyroscope[0] = *lala;     
        lala = (short *) (buf+16);           
        imu.gyroscope[1] = *lala;    
        lala = (short *) (buf+14);  
        imu.gyroscope[2] = *lala;
        
        pubImu.publish(imu);
      } else {
        printf("***");
        for (int i=0 ; i<8 ; i++)
          printf(" %02x", buf[i]);
        printf("\n");
      }
          
    }
  }
  
  
  close(fd);
  return 0;
}
 
 const char *
 bus_str(int bus)
 {
         switch (bus) {
         case BUS_USB:
                 return "USB";
                 break;
         case BUS_HIL:
                 return "HIL";
                 break;
         case BUS_BLUETOOTH:
                 return "Bluetooth";
                 break;
         case BUS_VIRTUAL:
                 return "Virtual";
                 break;
         default:
                return "Other";
               break;
         }1; 
   
}
