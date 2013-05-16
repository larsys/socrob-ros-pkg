#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <triclops.h>
#include <pnmutils.h>

#include <time.h>

#include "Bumblebee_Parameters.h"

#include <ros/ros.h>

#include <dc1394/dc1394.h>

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include "raposang_msgs/RaposaStereo.h"

#define  BUMBLEBEE2_SIZE_DMA                    8          
#define  BUMBLEBEE2_FLUSH                       true
#define  BUMBLEBEE2_AUTOGAIN                    1           //in seconds
#define  BUMBLEBEE2_NR_ATTEMPTS_TRANSMITION     5           
#define  BUMBLEBEE2_SLEEP_TIME_TRANSMITION      50000       //in microseconds

#define REG_CONFIG_LENGTH         	0x1FFC 
#define REG_CONFIG_DATA           	0x2000 
#define REG_UNIT_DIRECTORY_OFFSET   0x0424  

#define RECTIFY

#ifndef  PGR_REGISTERS_H
#define  BAYER_TILE_MAPPING_REGISTER 	        (0x1040)
#define  SENSOR_BOARD_INFO_REGISTER  	        (0x1f28)
#define  IMAGE_DATA_FORMAT_REGISTER	          (0x1048)
#endif 

//=============================================================================
// Define Structs
//=============================================================================

typedef enum {
   UNKNOWN_CAMERA,
   BUMBLEBEE,
   BUMBLEBEE2,
   BUMBLEBEEXB3
} PGRStereoCameraModel_t;

typedef struct {
   bool flush;
   bool rectify;
   bool normal;
   bool stereo;
   bool new_bb2_file;
   int fps;
   int dma;
} Params;

typedef struct {
   dc1394camera_t*		  camera;
   PGRStereoCameraModel_t model;
   dc1394color_filter_t	  bayerTile;
   bool			          bColor;
   unsigned int		      nRows;
   unsigned int		      nCols;
   unsigned int		      nBytesPerPixel;
} PGRStereoCamera_t;

typedef unsigned long long int bumblebee2_timestamp;

typedef struct Bumblebee2_buffer_{

   unsigned int   size;
   unsigned char* RGB;
   unsigned char* deinterlaced;
          
} Bumblebee2_buffer;

typedef struct Bumblebee2_{

   PGRStereoCamera_t    info;
   Bumblebee2_buffer    buffer; 
   
} Bumblebee2;

//=============================================================================
// Globals
//=============================================================================

Params p;

bool rectify_images;

sensor_msgs::Image bb_left;
sensor_msgs::Image bb_right;
sensor_msgs::Image rc_left;
sensor_msgs::Image rc_right;
sensor_msgs::CameraInfo bb_left_info;
sensor_msgs::CameraInfo bb_right_info;
sensor_msgs::CameraInfo rc_left_info;
sensor_msgs::CameraInfo rc_right_info;

raposang_msgs::RaposaStereo camera_stereo;

ros::Publisher camera_stereo_pub;

image_transport::CameraPublisher bb_left_pub;
image_transport::CameraPublisher bb_right_pub;
image_transport::CameraPublisher rc_left_pub;
image_transport::CameraPublisher rc_right_pub;

TriclopsInput  pTriclopsInputR;
TriclopsInput  pTriclopsInputL;
TriclopsContext triclops;

cv::Mat right_f_x;
cv::Mat right_f_y;	
cv::Mat left_f_x;
cv::Mat left_f_y;	

cv::Mat mat_unrect;
cv::Mat mat_rect;
cv::Mat mat_low;

//=============================================================================
// Functions
//=============================================================================

Bumblebee2 Bumblebee2_Initialize();
void Bumblebee2_Show_Info(Bumblebee2 );
void Bumblebee2_Get_Images(Bumblebee2 *);
void Bumblebee2_Cleanup_and_Exit(dc1394camera_t *, const char *);
void Bumblebee2_Close(Bumblebee2);


dc1394error_t
writeTriclopsConfigFromCameraToFile( dc1394camera_t* camera,
                                     const char* outputFile) {
   dc1394error_t err;
   uint32_t 	ulQuadlet;

   err = dc1394_get_control_register( camera, REG_CONFIG_LENGTH, &ulQuadlet );
   if ( err != DC1394_SUCCESS )
   {
      fprintf(stderr, "dc1394_get_control_register(REG_CONFIG_LENGTH) failed\n");
      return err;
   }
   
   // the length of the config file
   unsigned long ulFileSizeBytes = ulQuadlet;
   if( ulFileSizeBytes == 0 )
   {
      fprintf( stderr, "File size == 0!\n" );
      return DC1394_FAILURE;
   }
   
   FILE* pfile = fopen( outputFile, "w" );
   if ( !pfile )
   {
      fprintf( stderr, "Can't open temporary file\n" );
      return DC1394_FAILURE;
   }

   // Read the config file, and save it to the output file,
   // while fixing endianness.
   for( unsigned long offset = 0 ; offset < ulFileSizeBytes; offset += 4 )
   {
      err = dc1394_get_control_register( camera,
				      REG_CONFIG_DATA + offset, 
				      &ulQuadlet );
      
      if( err != DC1394_SUCCESS )
      {
	 fprintf( stderr, 
		  "Can't get control register 0x%x\n", 
		  (int) (REG_CONFIG_DATA+offset) );
	 fclose( pfile );
	 return err;
      }


      for( int i = 24; i >= 0; i -= 8 )
      {
	 fputc( ( (ulQuadlet>>i) & 0xFF ), pfile );
      }
   }
   
   fclose( pfile );

   return DC1394_SUCCESS;
}

TriclopsError getTriclopsContextFromCamera( PGRStereoCamera_t* stereoCamera, const char* tempFile, 
			      TriclopsContext* pTriclops ) {
   // note: if you have multi-users you should use a more robust temporary file name
   // creation method to avoid permission conflicts

   dc1394error_t err;
   err = writeTriclopsConfigFromCameraToFile( stereoCamera->camera, tempFile );
   if ( err != DC1394_SUCCESS )
   {
      fprintf( stderr, "Couldn't write config data to file\n" );
      return TriclopsErrorSystemError;
   }

   TriclopsError  tErr;
   tErr = triclopsGetDefaultContextFromFile( pTriclops, (char*) tempFile );
   if ( tErr != TriclopsErrorOk )
   {
      fprintf( stderr, "triclopsGetDefaultContextFromFile failed!\n" );
   }
   return tErr;
}


Bumblebee2 Bumblebee2_Initialize() {

    Bumblebee2 bumblebee;

    dc1394_t              * d;
    dc1394camera_list_t   * list;
    dc1394camera_t        * camera;  
    dc1394switch_t          status = DC1394_OFF;  
    dc1394color_coding_t    coding;

    unsigned int i;
    unsigned int value;
    
    d = dc1394_new();
    
    if (dc1394_camera_enumerate(d, &list) != DC1394_SUCCESS) {    
        ROS_FATAL( "[BB2] Unable to look for cameras. Please check if the kernel modules (ieee1394,raw1394,ohci1394)"
                         "are loaded or if you have read/write access to /dev/raw1394  ");
        exit(0);
    }
    
    if (list->num == 0) {    
        ROS_FATAL( "[BB2] No camera found!  ");
        exit(0);
    }
    
    ROS_INFO("[BB2] %d camera(s) found attached to your PC  ", list->num );
    
    for (i=0; i<list->num; i++) {
    
        camera = dc1394_camera_new(d, list->ids[i].guid);
    
        if(!camera) {
            ROS_INFO("[BB2] Failed to initialize camera with guid %llx", list->ids[i].guid);
            continue;
        }
        
        ROS_INFO("[BB2] Found: Camera %d model = '%s'  ", i, camera->model);
    
        if (!strncmp(camera->model, "Bumblebee2", strlen("Bumblebee2"))) {        
            ROS_INFO( "[BB2] Its a Bumblebee2 Camera! Using this camera!  " );
            break;
        }
        dc1394_camera_free(camera);
    }
    
    if (i == list->num) {
        ROS_INFO( "[BB2] No stereo camera detected  " );
        exit(0);
    }
    
    dc1394_camera_free_list(list);
        
    // set the camera handle and find out what base model camera we have
    bumblebee.info.camera           = camera;
    bumblebee.info.model            = BUMBLEBEE2;
    bumblebee.info.nBytesPerPixel	= 2;
    
    if (dc1394_get_control_register( camera, SENSOR_BOARD_INFO_REGISTER, &value ) != DC1394_SUCCESS ) 
       Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Could not query the Sensor Info Register!" );

    switch( 0xf & value ) {
      default:
	     Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Illegal sensor board info detected!" );
      case 0xA:	// color 640x480
	     bumblebee.info.bColor	= true;
	     bumblebee.info.nRows	= 480;
	     bumblebee.info.nCols	= 640;
	     break;
      case 0xB:	// mono 640x480
	     bumblebee.info.bColor	= false;
	     bumblebee.info.nRows	= 480;
	     bumblebee.info.nCols	= 640;
	     break;
      case 0xC:	// color 1024x768
	     bumblebee.info.bColor	= true;
	     bumblebee.info.nRows	= 768;
	     bumblebee.info.nCols	= 1024;
	     break;
      case 0xD:	// mono 1024x768
	     bumblebee.info.bColor	= false;
	     bumblebee.info.nRows	= 768;
	     bumblebee.info.nCols	= 1024;
	     break;
      case 0xE:	// color 1280x960
	     bumblebee.info.bColor	= true;
	     bumblebee.info.nRows	= 960;
	     bumblebee.info.nCols	= 1280;
	     break;
      case 0xF:	// mono 1280x960
	     bumblebee.info.bColor	= false;
	     bumblebee.info.nRows	= 960;
	     bumblebee.info.nCols	= 1280;
	     break;
    }      
        
    ROS_INFO( "[BB2] Setting stereo video capture mode..  " );
    
    coding = bumblebee.info.bColor ? DC1394_COLOR_CODING_RAW16 : DC1394_COLOR_CODING_MONO16;
    
	// Load the factory defaults - This is auto-everything
	if (dc1394_memory_load( bumblebee.info.camera, 0 ) != DC1394_SUCCESS ) 
	   Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Can't load default memory channel." );	   

	// set 16-bit transmission to be PGR-default little endian mode
	if (dc1394_set_control_register( bumblebee.info.camera, IMAGE_DATA_FORMAT_REGISTER, 0x80000000) != DC1394_SUCCESS ) 
	   Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera , "Can't set Bumblebee2 into little-endian mode." );
	   
	   
	dc1394_video_set_iso_speed( bumblebee.info.camera, DC1394_ISO_SPEED_400 );
	dc1394_video_set_mode( bumblebee.info.camera, DC1394_VIDEO_MODE_FORMAT7_3 );


	if (dc1394_format7_set_roi( bumblebee.info.camera, DC1394_VIDEO_MODE_FORMAT7_3, coding, 
	                            DC1394_USE_MAX_AVAIL, 0, 0, bumblebee.info.nCols, bumblebee.info.nRows) != DC1394_SUCCESS)
	    Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Can't setup Bumblebee2 capture.");


	if (dc1394_capture_setup( bumblebee.info.camera, BUMBLEBEE2_SIZE_DMA, DC1394_CAPTURE_FLAGS_DEFAULT ) != DC1394_SUCCESS )
	    Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Can't setup Bumblebee capture." );


    // get the bayer tile info so we will know how to color process this mode
    
    if (dc1394_get_control_register( bumblebee.info.camera, BAYER_TILE_MAPPING_REGISTER, &value ) != DC1394_SUCCESS ) 
        Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Could not query the Sensor Info Register!");
    
    switch(value) {     // Ascii R=52 G=47 B=42 Y=59
      default:
      case 0x59595959:	// YYYY	 
	    bumblebee.info.bayerTile  = (dc1394color_filter_t) 0; break;
      case 0x52474742:	// RGGB
	    bumblebee.info.bayerTile  = DC1394_COLOR_FILTER_RGGB; break;
      case 0x47425247:	// GBRG
	    bumblebee.info.bayerTile  = DC1394_COLOR_FILTER_GBRG; break;
      case 0x47524247:	// GRBG
	    bumblebee.info.bayerTile  = DC1394_COLOR_FILTER_GRBG; break;
      case 0x42474752:	// BGGR
	    bumblebee.info.bayerTile  = DC1394_COLOR_FILTER_BGGR; break;
    }    
    
    ROS_INFO( "[BB2] Start transmission...  " );    
    
    // Have the camera start transmitting data.
    if (dc1394_video_set_transmission( bumblebee.info.camera, DC1394_ON ) != DC1394_SUCCESS ) 
        Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Unable to start camera iso transmission." );

    ROS_INFO( "[BB2] - Waiting for transmission...  " );
    
    //  Sleep until the camera has the transmission status ON;
    for (i = 0; i <= BUMBLEBEE2_NR_ATTEMPTS_TRANSMITION; i++) {
    
        usleep(BUMBLEBEE2_SLEEP_TIME_TRANSMITION);
        
        //  Check the transmittion status.
        if (dc1394_video_get_transmission( bumblebee.info.camera, &status ) != DC1394_SUCCESS ) 
	       Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Unable to get transmision status." );

        //  If the transmition status is ON, break the cycle.
        if ( status == DC1394_ON ) break;
        
        //  If it always gives status OFF, close.
        if(i == BUMBLEBEE2_NR_ATTEMPTS_TRANSMITION)
	       Bumblebee2_Cleanup_and_Exit( bumblebee.info.camera, "Camera doesn't seem to want to turn on!" );
    }
  
    // Sleep-time for autogain!
    sleep(BUMBLEBEE2_AUTOGAIN);
    
    // Initialize variables and buffers!
  
    bumblebee.buffer.size  = bumblebee.info.nRows * bumblebee.info.nCols * bumblebee.info.nBytesPerPixel;   
    bumblebee.buffer.RGB   = bumblebee.info.bColor ? (unsigned char *) malloc(3*bumblebee.buffer.size) : NULL;
    bumblebee.buffer.deinterlaced  =                 (unsigned char *) malloc(  bumblebee.buffer.size);
    
    return bumblebee; 
    
}

void Bumblebee2_Show_Info(Bumblebee2 bumblebee) {
    
    ROS_INFO("[BB2] Info - Camera Model: %s - %dpx x %dpx - Channels: %d - Bytes per Pixel: %d",
           bumblebee.info.camera->model,
           bumblebee.info.nCols,
           bumblebee.info.nRows,
           (bumblebee.info.bColor ? 3 : 1),
           bumblebee.info.nBytesPerPixel);
       
}

void Bumblebee2_Get_Images(Bumblebee2 * bumblebee) { 

    dc1394error_t err;
    dc1394video_frame_t* frame;
    dc1394video_frame_t* frame2;
   
    bool endFound = false;
    unsigned int nrFrames = 0;   
    unsigned int nrPixels = bumblebee->info.nRows * bumblebee->info.nCols;   
     
    dc1394_capture_dequeue( bumblebee->info.camera,DC1394_CAPTURE_POLICY_WAIT,&frame );
   
    if (BUMBLEBEE2_FLUSH) { 
         
       while (!endFound) {
          err = dc1394_capture_dequeue(bumblebee->info.camera,DC1394_CAPTURE_POLICY_POLL, &frame2);
          if (frame2 && err==DC1394_SUCCESS) {
              dc1394_capture_enqueue(bumblebee->info.camera, frame);
              frame = frame2;
              nrFrames++;
          } 
          else endFound = true;                                                                  
       }

       if (nrFrames == BUMBLEBEE2_SIZE_DMA-2) {
           dc1394_capture_enqueue(bumblebee->info.camera, frame);      
           dc1394_capture_dequeue(bumblebee->info.camera,DC1394_CAPTURE_POLICY_WAIT, &frame);
       }  
    } 

		bb_right.header.stamp = bb_left.header.stamp = 
		rc_right.header.stamp = rc_left.header.stamp = 
		camera_stereo.right.header.stamp = camera_stereo.left.header.stamp = 
		ros::Time::now();

    dc1394_deinterlace_stereo( frame->image,
												       bumblebee->buffer.deinterlaced,
												       bumblebee->info.nCols,
												       2*bumblebee->info.nRows );

    dc1394_bayer_decoding_8bit( bumblebee->buffer.deinterlaced,
		       	                    bumblebee->buffer.RGB,
						                 		bumblebee->info.nCols,
						                 		2*bumblebee->info.nRows,
				                   			bumblebee->info.bayerTile,
				                   			DC1394_BAYER_METHOD_NEAREST); 

		if(rectify_images) {		
			mat_unrect.data = bumblebee->buffer.RGB;		
			mat_rect.data = &rc_right.data[0];			
			remap(mat_unrect, mat_rect, right_f_y, right_f_x, cv::INTER_LINEAR);		
			
			if(p.stereo)
				memcpy(&camera_stereo.right.data[0], &mat_rect.data[0], 3*nrPixels);
		
			mat_unrect.data = bumblebee->buffer.RGB + 3*nrPixels;		
			mat_rect.data = &rc_left.data[0];
			remap(mat_unrect, mat_rect, left_f_y, left_f_x, cv::INTER_LINEAR);	
			
			if(p.stereo)					
				memcpy(&camera_stereo.left.data[0],  &mat_rect.data[0], 3*nrPixels);
		
		}
		
		if(p.normal){
			memcpy(&bb_right.data[0], bumblebee->buffer.RGB,              3*nrPixels);
			memcpy(&bb_left.data[0],  bumblebee->buffer.RGB + 3*nrPixels, 3*nrPixels);
		}
		
    dc1394_capture_enqueue( bumblebee->info.camera, frame );
   
    return;
}


void Bumblebee2_Close(Bumblebee2 bumblebee) {

    ROS_INFO( "Stop transmission  " );
    
    if (dc1394_video_set_transmission(bumblebee.info.camera, DC1394_OFF) != DC1394_SUCCESS ) 
        ROS_FATAL( "Couldn't stop the camera.  ");
    
    if (bumblebee.buffer.RGB   != NULL) free(bumblebee.buffer.RGB);
    free(bumblebee.buffer.deinterlaced);
    
    Bumblebee2_Cleanup_and_Exit(bumblebee.info.camera, NULL);
}

void Bumblebee2_Cleanup_and_Exit( dc1394camera_t* camera, const char * msg ) {

    if (msg!=NULL) ROS_FATAL("[BB2] %s",msg);
    dc1394_capture_stop(camera);    
    dc1394_video_set_transmission(camera, DC1394_OFF );
    dc1394_camera_free(camera);
    exit(0);
}
 
void Bumblebee2_Initialize_ROS_Messages(ros::NodeHandle n, Bumblebee2 bumblebee) {

		unsigned int nrPixels, nCols, nRows;

		nCols = bumblebee.info.nCols;
		nRows = bumblebee.info.nRows;

		image_transport::ImageTransport img_trans(n);

		nrPixels = nCols*nRows*3;

		// Image Raw

		if (p.normal) {		
						
			bb_left_pub  = img_trans.advertiseCamera("output_left_raw",  1);
			bb_right_pub = img_trans.advertiseCamera("output_right_raw", 1);

			bb_right.encoding = bb_left.encoding = sensor_msgs::image_encodings::RGB8;
			bb_right.height   = bb_left.height   = bumblebee.info.nRows;		
			bb_right.width    = bb_left.width    = bumblebee.info.nCols;
			bb_right.step     = bb_left.step     = bumblebee.info.nCols*3;
					
			bb_left.data.resize(nrPixels);
			bb_right.data.resize(nrPixels);
		}
		
		// Image Rect	
		
		if (rectify_images) {
		
			mat_unrect.create(nRows, nCols, CV_8UC3);
			mat_rect.create(nRows, nCols, CV_8UC3);  
					
			right_f_x.create(nRows, nCols, CV_32FC1);
			right_f_y.create(nRows, nCols, CV_32FC1);		
		
			left_f_x.create(nRows, nCols, CV_32FC1);
			left_f_y.create(nRows, nCols, CV_32FC1);		
		
			if (p.rectify) {	 				
				rc_left_pub  = img_trans.advertiseCamera("output_left_rect",  1);
				rc_right_pub = img_trans.advertiseCamera("output_right_rect", 1);

				rc_right.encoding = rc_left.encoding = sensor_msgs::image_encodings::RGB8;
				rc_right.height   = rc_left.height   = bumblebee.info.nRows;	
				rc_right.width    = rc_left.width    = bumblebee.info.nCols;
				rc_right.step     = rc_left.step     = bumblebee.info.nCols*3;
			
				rc_left.data.resize(nrPixels);
				rc_right.data.resize(nrPixels);		
	 
				rc_left_info.distortion_model = 
				rc_right_info.distortion_model = "plumb_bob";

				rc_left_info.D.resize(5);
				rc_right_info.D.resize(5); 
			}
			
			if (p.stereo) {	 
				
				camera_stereo_pub  = n.advertise<raposang_msgs::RaposaStereo>("output_stereo_rect",  1);
				
				camera_stereo.left.encoding = camera_stereo.right.encoding = sensor_msgs::image_encodings::RGB8;
				camera_stereo.left.height   = camera_stereo.right.height = bumblebee.info.nRows;	  
				camera_stereo.left.width    = camera_stereo.right.width = bumblebee.info.nCols;
				camera_stereo.left.step     = camera_stereo.right.step = bumblebee.info.nCols*3; 
				camera_stereo.left.data.resize(nrPixels);
				camera_stereo.right.data.resize(nrPixels);
				
			}				
						
		}

		// Other

		double left_D_data[] = D_LEFT;
    double left_K_data[] = K_LEFT;
	  double left_R_data[] = R_LEFT;
	  double left_P_data[] = P_LEFT;

	  double right_D_data[] = D_RIGHT;
    double right_K_data[] = K_RIGHT;
	  double right_R_data[] = R_RIGHT;
	  double right_P_data[] = P_RIGHT;

		bb_left_info.distortion_model = 
		bb_right_info.distortion_model = "plumb_bob";

		bb_left_info.D.resize(5);
		bb_right_info.D.resize(5);

    memcpy(&bb_left_info.D[0],  &left_D_data[0],  sizeof(left_D_data));
	  memcpy(&bb_left_info.K[0],  &left_K_data[0],  sizeof(left_K_data));
	  memcpy(&bb_left_info.R[0],  &left_R_data[0],  sizeof(left_R_data));
	  memcpy(&bb_left_info.P[0],  &left_P_data[0],  sizeof(left_P_data));
	  memcpy(&bb_right_info.D[0], &right_D_data[0], sizeof(right_D_data));
    memcpy(&bb_right_info.K[0], &right_K_data[0], sizeof(right_K_data));
    memcpy(&bb_right_info.R[0], &right_R_data[0], sizeof(right_R_data));
    memcpy(&bb_right_info.P[0], &right_P_data[0], sizeof(right_P_data));

} 

int main(int argc, char **argv) {

		int nrPixels, nRows, nCols, k;
		register int r, c;
		float rawr, rawc;
		float f, crow, ccol, baseline;
					
		std::string frame_id_left, frame_id_right, bb2_file_path;		
		
  	ros::init(argc, argv, "raposang_bumblebee2");
  	ros::NodeHandle n;
    ros::NodeHandle nh("~");    
 
		nh.param("fps", p.fps, 30);	 
		
		nh.param("publish_rectified_images", p.rectify, true);	 
		nh.param("publish_raw_images",  p.normal, true);	
		nh.param("publish_stereo_pack",  p.stereo, true);			 		
		nh.param("flush",   p.flush,   true);	
		nh.param("dma_ring_size",     p.dma,     8);	

		nh.param("new_bb2_file",  p.new_bb2_file,  true);	
		nh.param<std::string>("bb2_file_path",  bb2_file_path,  "/tmp/triclops.cal");	
		nh.param<std::string>("frame_id_left",  frame_id_left,  "bb2_left");	
		nh.param<std::string>("frame_id_right", frame_id_right, "bb2_right");			
		
		rectify_images = p.rectify || p.stereo;
								     
    Bumblebee2 bumblebee = Bumblebee2_Initialize();
        
    Bumblebee2_Show_Info(bumblebee);

		Bumblebee2_Initialize_ROS_Messages(n, bumblebee);

		nrPixels = bumblebee.info.nRows * bumblebee.info.nCols; 			
		nCols = bumblebee.info.nCols;
		nRows = bumblebee.info.nRows;		
		
		if (rectify_images) {
		
			if (p.new_bb2_file)
				getTriclopsContextFromCamera(&bumblebee.info, bb2_file_path.c_str(), &triclops);
			else
				triclopsGetDefaultContextFromFile( &triclops, (char *) bb2_file_path.c_str());
				
			triclopsSetResolution(triclops, nRows, nCols);
			triclopsSetSubpixelInterpolation(triclops,1); 
			
			triclopsGetFocalLength(triclops, &f);
			triclopsGetImageCenter(triclops, &crow, &ccol);
			triclopsGetBaseline(triclops, &baseline);								
									
			float * r_r_ptr = (float *) &right_f_x.data[0];
			float * c_r_ptr = (float *) &right_f_y.data[0];
			float * r_l_ptr = (float *) &left_f_x.data[0];
			float * c_l_ptr = (float *) &left_f_y.data[0];
	
			k = 0;	
			for(r = 0; r < nRows; r++)	{
				for(c = 0; c < nCols; c++)	{

					triclopsUnrectifyPixel( triclops, TriCam_RIGHT, r, c, &rawr, &rawc );			
					r_r_ptr[k] = rawr*2;
					c_r_ptr[k] = rawc;

					triclopsUnrectifyPixel( triclops, TriCam_LEFT,  r, c, &rawr, &rawc );			
					r_l_ptr[k] = rawr*2;
					c_l_ptr[k] = rawc;

					k++;		
				}
			} 
			
			if (p.rectify) {	 
						
				rc_left.header.frame_id = rc_left_info.header.frame_id = frame_id_left;
				rc_right.header.frame_id = rc_right_info.header.frame_id = frame_id_right;
				
				rc_left_info.K[0] = rc_right_info.K[0] = 
				rc_left_info.K[4] = rc_right_info.K[4] = f;
				rc_left_info.K[2] = rc_right_info.K[2] = ccol;
				rc_left_info.K[5] = rc_right_info.K[5] = crow;	
				rc_left_info.K[8] = rc_right_info.K[8] = 1.0;

				rc_left_info.R[0] = rc_right_info.R[0] = 
				rc_left_info.R[4] = rc_right_info.R[4] = 
				rc_left_info.R[8] = rc_right_info.R[8] = 1.0;

				rc_left_info.P[0] = rc_right_info.P[0] = 
				rc_left_info.P[5] = rc_right_info.P[5] = f;
				rc_left_info.P[2] = rc_right_info.P[2] = ccol;
				rc_left_info.P[6] = rc_right_info.P[6] = crow;	
				rc_left_info.P[10] = rc_right_info.P[10] = 1.0;
				
				rc_left_info.P[7] =	f*baseline;		
								
			}
	
			if (p.stereo) {	 
				camera_stereo.focal_length      = f;
				camera_stereo.image_center_row  = crow;	  
				camera_stereo.image_center_col  = ccol;
				camera_stereo.baseline     			= baseline; 
				camera_stereo.left.header.frame_id  = frame_id_left;
				camera_stereo.right.header.frame_id = frame_id_right;				
			}
					
		}  

		if (p.normal) {

			bb_left.header.frame_id  = bb_left_info.header.frame_id  = frame_id_left;
			bb_right.header.frame_id = bb_right_info.header.frame_id = frame_id_right;	
			bb_left_info.header.frame_id  = frame_id_left;
			bb_right_info.header.frame_id = frame_id_right;	
					  
		}
					  
		ros::Rate ra(p.fps);
		
    while(ros::ok()) {

        Bumblebee2_Get_Images(&bumblebee);

				if (p.normal) {    
					bb_right_pub.publish(bb_right, bb_right_info);
					bb_left_pub.publish(bb_left, bb_left_info);	
				}	
				
				if (p.rectify) {
					rc_right_pub.publish(rc_right, rc_right_info);
					rc_left_pub.publish(rc_left, rc_left_info);				
				}
				
				if (p.stereo) 
					camera_stereo_pub.publish(camera_stereo);
					
				ra.sleep();
    } 

    Bumblebee2_Close(bumblebee);
     
    return 0;

}
