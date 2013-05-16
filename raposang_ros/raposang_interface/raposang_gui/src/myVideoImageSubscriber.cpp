#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include <gtk/gtk.h>
#include <gdk/gdkkeysyms.h>
#include <ros/rate.h>	// In order to use Rate class.
#include <time.h>		// In order to calculate time intervals.
#include <math.h>		// In order to print pows (10^-9).

#define DEBUG_MESSAGES 0						// Colour code for debug: \033[22;31m	To cancel: \033[01;37m
#define MY_FUNCTIONS_ENTRANCE_CONTROL 0			// Colour code for debug: \033[01;33m	To cancel: \033[01;37m
#define INNER_FUNCTION_CONTROL 0				// Colour code for debug: \033[01;34m	To cancel: \033[01;37m
#define TIME_CONTROL 0

#define MAX_NUMBER_OF_CAMERAS 4
#define ROW_NUMBER_IN_IMAGE_TABLE 2
#define COLUMN_NUMBER_IN_IMAGE_TABLE 2
#define IMAGES_HORIZONTAL_PLACEMENT 0		
#define IMAGES_VERTIACL_PLACEMENT 0        

typedef enum { CREATE, REFRESH } functionFlag;
typedef enum { LEFT, RIGHT, KINECT, KINECT_DEPTH } cameraFlag;
typedef enum { ONE_CAMERA, FOUR_CAMERAS } numOfCameras;
typedef enum { SHOW_SINGLE_IMAGE, SHOW_GRID, TRANSITION_FROM_IMAGE_TO_GRID, TRANSITION_FROM_GRID_TO_IMAGE } StateMachineStatus;
typedef enum { NOT_CLICKED, CLICKED } ClickingStatus;
typedef enum { ONE, TWO, THREE, FOUR, FIVE, SIX, SEVEN, EIGHT } ImageIndex;

// Image menagment struct
typedef struct imageManagmentStruct
{
	IplImage*  pSrcIplImage;
    IplImage*  pDstIplImage;
    IplImage*  pResizedIplImage;
    GdkPixbuf* pGtkPixbuf;
    GtkWidget* pGtkImgWindow; 
    GtkWidget* pEventBoxForImage;   
    int        previousImageSize;
    int        left_attach;
    int        right_attach;
    int        top_attach;
    int        bottom_attach;
    ClickingStatus clickedOrNot;
    ImageIndex imageIndex;  
} ImageManagmentStruct;


// Function prototypes
IplImage* resizeImage(const IplImage *origImg, int newWidth, int newHeight, bool keepAspectRatio);
GtkWidget* convertOpenCv2Gtk(ImageManagmentStruct* pToImageManagmentStruct, functionFlag flag);


/** Global arrays, that holds all the information needed to maintain an 
 *  image from a certain camera.
 *  Each cell of the array hold a pointer to ImageManagmentStruct.
 */
// An array that is used to hold all the information for 4x4 grid of images.
ImageManagmentStruct* globalArrayOfImagesForGrid[MAX_NUMBER_OF_CAMERAS];
// An array that is used to hold all the information for single image representation.
ImageManagmentStruct* globalArrayForSingleImages[MAX_NUMBER_OF_CAMERAS];

// Other global variables.
GtkWidget* globalTopLevelWindow;
GtkWidget* pGlobalGtkTable;
GtkWidget* pGlobalFixedContainer;
double globalResizingFactor;


/** --------------------------------------------------------------------------------------------------------
 *  A global parameter which indicates in which state the "clicking state machine" is: 
 *  1. SHOW_SIMGLE_IMAGE - showing a single image.
 *  2. SHOW_GRID - showing a grid of images.
 *  3. TRANSITION_IMAGE_GRID - transition, between one image and a grid of 4x4 cameras, should be performed.
 *  4. TRANSITION_GRID_IMAGE - transition, between a grid of 4x4 cameras and one image, should be performed.
 *  
 *  NOTE 1: This variable may be changed from different functions in the code.
 *  NOTE 2: The variable is checked in the ROS callback, and according to it, the next state is determined.
 * --------------------------------------------------------------------------------------------------------*/
StateMachineStatus stateMachineStatus = SHOW_GRID;


/***********************************************************************
 * Description: Calculation of time interval between two given times.
 * ---------------------------------------------------------------------
 * Input:  start - start time.
 *         end - end time.
 * Output: the time interval between the start time and the end time.
 * --------------------------------------------------------------------- 
 * Notes:  1. All the times are represented by timespec struct.
 *         2. This function may be used in order to check the 
 *            performances of this module in real time.
 * ********************************************************************/
timespec diff ( timespec start, timespec end )
{
	timespec temp;	 
	
	if ( ( end.tv_nsec - start.tv_nsec ) < 0 ) 
	{
		temp.tv_sec = end.tv_sec - start.tv_sec-1;
		temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
	} 
	else 
	{
		temp.tv_sec = end.tv_sec - start.tv_sec;
		temp.tv_nsec = end.tv_nsec - start.tv_nsec;
	} 
	return temp;
}

 
 
/***********************************************************************
 * Description: Callback function for a "mouse click" event.
 *              In this function the state of all the system is changed.
 * ---------------------------------------------------------------------
 * Input:  pToEventBox - the event box that wraps the image.
 *         event - the event that is received from the user (press, 
 *                 relase, ...).
 *         pToData - data that is delivered to the callback.
 * Output: TRUE/FALSE
 * --------------------------------------------------------------------- 
 * Notes:  The possible states that the system should pass to after 
 *         click are:
 *         1. TRANSITION_IMAGE_GRID.
 *         2. TRANSITION_GRID_IMAGE.
 *         Generally there are 2 more possible states of the system
 *         (SHOW_SINGLE_IMAGE, SHOW_GRID ).
 * ********************************************************************/
 static gboolean clickingCallback (GtkWidget* pToEventBox, GdkEventButton* event, gpointer pToData)
 {
	 
	 //static int clickCount = 0;		// Variable that counts clicks.
	 
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"clickingCallback\".\033[01;37m\n" ); }
	 
	 // Update number of clicks.
	 //clickCount++;
	 
	 // Avoiding increasing grouth of the counter.
	 //clickCount = clickCount%4;	
	 
	 // Update the inner status of the structs in both the arrays (globalArrayOfImagesForGrid and globalArrayForSingleImages).
	 switch ( ((ImageManagmentStruct*)pToData)->imageIndex )
	 {
		 case ONE:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[0]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[0]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[0]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[0]->clickedOrNot = CLICKED;					 
			 }
			 break;
		 }
		 case TWO:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[1]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[1]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[1]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[1]->clickedOrNot = CLICKED;					 
			 }			 
			 break;
		 }
		 case THREE:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[2]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[2]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[2]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[2]->clickedOrNot = CLICKED;					 
			 }			 			 
			 break;
		 }
		 case FOUR:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[3]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[3]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[3]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[3]->clickedOrNot = CLICKED;					 
			 }			 			 
			 break;
		 }		 
		 case FIVE:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[0]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[0]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[0]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[0]->clickedOrNot = CLICKED;					 
			 }				 
			 break;
		 }
		 case SIX:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[1]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[1]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[1]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[1]->clickedOrNot = CLICKED;					 
			 }				 
			 break;
		 }
		 case SEVEN:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[2]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[2]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[2]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[2]->clickedOrNot = CLICKED;					 
			 }				 
			 break;
		 }
		 case EIGHT:
		 {
		 	 if ( ((ImageManagmentStruct*)pToData)->clickedOrNot == CLICKED )
             {
				 globalArrayOfImagesForGrid[3]->clickedOrNot = NOT_CLICKED;
				 globalArrayForSingleImages[3]->clickedOrNot = NOT_CLICKED;
             }
             else
             {
				 globalArrayOfImagesForGrid[3]->clickedOrNot = CLICKED;
				 globalArrayForSingleImages[3]->clickedOrNot = CLICKED;					 
			 }				 
			 break;
		 }		 
		 default:
		 {
			 ROS_ERROR ("\033[01;33mclickingCallback: The image index is wrong. Aborting...\033[01;37m\n"); 
			 ros::shutdown(); 
		 }	 		 		 		 		 
	 } 
#if 0	 
	 // Debug information about the inner status of the system.
	 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: the status of the images ufter the click is:.\033[01;37m\n"); }
	 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: in \"globalArrayOfImagesForGrid\": %d, %d, %d, %d\033[01;37m\n",
	                                 globalArrayOfImagesForGrid[0]->clickedOrNot, 
	                                 globalArrayOfImagesForGrid[1]->clickedOrNot,
	                                 globalArrayOfImagesForGrid[2]->clickedOrNot,
	                                 globalArrayOfImagesForGrid[3]->clickedOrNot
	                                ); }
	 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: in \"globalArrayForSingleImages\": %d, %d, %d\033[01;37m\n",
	                                 globalArrayForSingleImages[0]->clickedOrNot, 
	                                 globalArrayForSingleImages[1]->clickedOrNot,
	                                 globalArrayForSingleImages[2]->clickedOrNot,
	                                 globalArrayForSingleImages[3]->clickedOrNot
	                                ); }	                    
	 
	 // Changing the state according to the "clickCount" parameter.
	 switch ( clickCount%2 )
	 {
		 /** If a click event was accepted and the "clickEvent" flag is odd (1,3,5,...),
		  *  it means that the system is showing a grid of 4 images. 
		  *  Than the state should be changed to TRANSITION_FROM_GRID_TO_IMAGE.
		  *  Assumption: only one image can be clicked at a time. */
		 case 1:
		 {
			 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: clickCount == %d.\033[01;37m\n", clickCount); }
			 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: clickedOrNot == %d.\033[01;37m\n", ((ImageManagmentStruct*)pToData)->clickedOrNot); }
			 if ( INNER_FUNCTION_CONTROL ) 
			 { 
				 printf ("\033[01;33mclickingCallback: the clicked image is == \033[01;37m"); 
				 switch ( ((ImageManagmentStruct*)pToData)->imageIndex )
				 {
					 case ONE:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mONE\033[01;37m\n"); }   break; }
					 case TWO:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mTWO\033[01;37m\n"); }  break; }
					 case THREE:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mTHREE\033[01;37m\n"); } break; }
					 case FOUR:        { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mFOUR\033[01;37m\n"); } break; }					 
					 case FIVE:        { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mFIVE\033[01;37m\n"); }    break; }
					 case SIX:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mSIX\033[01;37m\n");  }  break; }
					 case SEVEN:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mSEVEN\033[01;37m\n"); }   break; }
					 case EIGHT:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mSEVEN\033[01;37m\n"); }   break; }					 
					 default:          { if ( INNER_FUNCTION_CONTROL ) { ROS_ERROR ("\033[01;33mclickingCallback: The image index is wrong. Aborting...\033[01;37m\n"); ros::shutdown();} }							 
				 }				 
		     }
			 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: status changed to TRANSITION_FROM_GRID_TO_IMAGE.\033[01;37m\n" ); }
			 stateMachineStatus = TRANSITION_FROM_GRID_TO_IMAGE;
			 break;
		 }
		 
		 /** If a click event was accepted and the "clickEvent" flag is even (0,2,4,...),
		  *  it means that the system is showing a single image. 
		  *  Than the state should be changed to TRANSITION_FROM_IMAGE_TO_GRID.
		  *  Assumption: only one image can be clicked at a time. */		 
		 case 0:
		 {
			 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: clickCount == %d.\033[01;37m\n", clickCount); }
			 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: clickedOrNot == %d.\033[01;37m\n", ((ImageManagmentStruct*)pToData)->clickedOrNot); }
			 if ( INNER_FUNCTION_CONTROL ) 
			 { 
				 printf ("\033[01;33mclickingCallback: the clicked image is == \033[01;37m"); 
				 switch ( ((ImageManagmentStruct*)pToData)->imageIndex )
				 {
					 case ONE:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mONE\033[01;37m\n"); }   break; }
					 case TWO:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mTWO\033[01;37m\n"); }  break; }
					 case THREE:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mTHREE\033[01;37m\n"); } break; }
					 case FOUR:        { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mFOUR\033[01;37m\n"); } break; }	
					 case FIVE:        { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mFIVE\033[01;37m\n"); }    break; }
					 case SIX:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mSIX\033[01;37m\n");  }  break; }
					 case SEVEN:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mSEVEN\033[01;37m\n"); }   break; }
					 case EIGHT:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mSEVEN\033[01;37m\n"); }   break; }
					 default:          { if ( INNER_FUNCTION_CONTROL ) { ROS_ERROR ("\033[01;33mclickingCallback: The image index is wrong. Aborting...\033[01;37m\n"); ros::shutdown();} }										 
				 }				 
		     }			 
			 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mclickingCallback: status changed to TRANSITION_FROM_IMAGE_TO_GRID.\033[01;37m\n" ); }
			 stateMachineStatus = TRANSITION_FROM_IMAGE_TO_GRID;
			 break;
		 }
		 default:
		 {
			ROS_ERROR ("clickingCallback: There was an error in modulo operation. Aborting...\n");
			ros::shutdown();			 
		 }
	 }	 
	 #endif
	 
	 switch ( stateMachineStatus )
	 {
		 case SHOW_GRID:
		 {
			 stateMachineStatus = TRANSITION_FROM_GRID_TO_IMAGE;
			 break;
		 }
		 case SHOW_SINGLE_IMAGE:
		 {
			 stateMachineStatus = TRANSITION_FROM_IMAGE_TO_GRID;
			 break;
		 }		 
		 default:
		 {
		 }
	 }
	 
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"clickingCallback\".\033[01;37m\n\n" ); }
	 return TRUE;
 }

// ---------- Code for HMD ----------
 
// Globals for full screen window
bool hmd_enabled = false;
GtkWidget* hmdTopLevelWindow;
GtkWidget* hmdGtkTable;
GtkWidget* hmdFixedContainer;
ImageManagmentStruct globalArrayForHMD[2];
gint hmdWidth, hmdHeight;

void hmdDataInitialization(void) {
  for (int i=0 ; i<2 ; i++) {
    globalArrayForHMD[i].pSrcIplImage  = NULL;
    globalArrayForHMD[i].pDstIplImage  = NULL;
    globalArrayForHMD[i].pResizedIplImage = NULL;
    globalArrayForHMD[i].pGtkPixbuf    = NULL;
    globalArrayForHMD[i].pGtkImgWindow = NULL;
    globalArrayForHMD[i].pEventBoxForImage = NULL;
    globalArrayForHMD[i].previousImageSize = -1;
    globalArrayForHMD[i].left_attach = i;
    globalArrayForHMD[i].right_attach = i+1;
    globalArrayForHMD[i].top_attach = 0;
    globalArrayForHMD[i].bottom_attach = 1;
    globalArrayForHMD[i].clickedOrNot = NOT_CLICKED;
    globalArrayForHMD[i].imageIndex = (ImageIndex)i;
  }
}

void createHMDInterfaceWindow(void) {
  GdkScreen *screen = gdk_screen_get_default();
  int n = gdk_screen_get_n_monitors(screen);
  int p = gdk_screen_get_primary_monitor(screen);
  int i;
  GdkRectangle geo;

  hmdDataInitialization();

  printf("%d monitors found, primary is #%d\n", n, p);
  for (i=0 ; i<n ; i++) {
    gdk_screen_get_monitor_geometry(screen, i, &geo);
    printf("monitor #%d: (%d,%d) %dx%d\n", i, geo.x, geo.y, geo.width, geo.height);
  }
  for (i=0 ; i<n ; i++) if (i!=p) break;
  printf("Using non-primary monitor #%d\n", i);
  gdk_screen_get_monitor_geometry(screen, i, &geo);
  hmdWidth  = geo.width/2;
  hmdHeight = geo.height;

  hmdTopLevelWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);     
  gtk_window_set_title ( GTK_WINDOW (hmdTopLevelWindow), "RAPOSA GUI"); 
  gtk_window_set_decorated( GTK_WINDOW (hmdTopLevelWindow), FALSE);
  gtk_window_set_default_size ( GTK_WINDOW(hmdTopLevelWindow), geo.width, geo.height);
  gtk_window_move( GTK_WINDOW(hmdTopLevelWindow), geo.x, geo.y);
  g_signal_connect( G_OBJECT(hmdTopLevelWindow), "destroy", G_CALLBACK(gtk_main_quit), NULL );
  hmdFixedContainer = gtk_fixed_new();
  gtk_container_add ( GTK_CONTAINER(hmdTopLevelWindow), hmdFixedContainer );
  hmdGtkTable = gtk_table_new(1, 2, TRUE);
  gtk_fixed_put( GTK_FIXED(hmdFixedContainer), hmdGtkTable, 0, 0);
  gtk_widget_show_all(hmdTopLevelWindow);
}

static gboolean keyCallback (GtkWidget* pToEventBox, GdkEventKey* event, gpointer pToData) {
  if (event->keyval==GDK_KEY_space) {
    if (hmd_enabled) {
	printf("INOP: Disabling HMD\n");
	// hmd_enabled = false;
      } else {
	printf("Enabling HMD\n");
	createHMDInterfaceWindow();
	hmd_enabled = true;
      }
  }

  return TRUE;
}

void callbackHMD (const sensor_msgs::ImageConstPtr& msg, int index) {
  sensor_msgs::CvBridge bridge;

  // printf("showingHMD() called index=%d\n", index);

  ImageManagmentStruct *pToImageManagmentStruct = &globalArrayForHMD[index];

  // Convert the ROS image message to an OpenCV image with BGR pixel encoding.
  try {
    pToImageManagmentStruct->pSrcIplImage = bridge.imgMsgToCv ( msg, "bgr8" );
  }
  catch (sensor_msgs::CvBridgeException& e) {
    ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
    
  if ( pToImageManagmentStruct->pResizedIplImage != NULL ) {
    cvReleaseImage ( &(pToImageManagmentStruct->pResizedIplImage) );
  }
  
  // Resizing the image.
  pToImageManagmentStruct->pResizedIplImage = resizeImage(pToImageManagmentStruct->pSrcIplImage, hmdWidth, hmdHeight, true);
    
  /** In the next lines, the next actions are taken:
   *  1. Check the size of the received image.
   *  2. Compare it to the previous image size.
   *  3. Decide if a new image should be created, or it should only be refreshed.
   */
  if ( pToImageManagmentStruct->pResizedIplImage->imageSize != pToImageManagmentStruct->previousImageSize ) {
    /** Creation part */		 
    // Updating the image size variable.
    pToImageManagmentStruct->previousImageSize = pToImageManagmentStruct->pResizedIplImage->imageSize;
    
    // The GDK window should be created from skretch.
    pToImageManagmentStruct->pGtkImgWindow = convertOpenCv2Gtk (pToImageManagmentStruct, CREATE); 
          
    // Attach the created image to an appropriate place in the GtkGrid.
    gtk_table_attach_defaults ( GTK_TABLE(hmdGtkTable), 
                                pToImageManagmentStruct->pEventBoxForImage,
                                pToImageManagmentStruct->left_attach,
                                pToImageManagmentStruct->right_attach,
                                pToImageManagmentStruct->top_attach,
                                pToImageManagmentStruct->bottom_attach);
    
    gtk_widget_show_all(hmdTopLevelWindow);

    printf("attached GtkImgWindow (0x%08x) to GtkTable (0x%08x)\n", pToImageManagmentStruct->pGtkImgWindow, hmdGtkTable);

  } else {
    /** Refreshing part */
    // The image inside the main window should only be refreshed.	
    pToImageManagmentStruct->pGtkImgWindow = convertOpenCv2Gtk ( pToImageManagmentStruct, REFRESH ); 
  }

}

 
 /***********************************************************************
 * Description: Function that does 3 major operations:
 *              1. Creates an event box which wraps an image.
 *              2. Conncts the image to the created event box.
 *              3. Connects a "button_press_event" signal to the created
 *                 event box.
 * ---------------------------------------------------------------------
 * Input: pToImageManagmentStruct - a pointer to an ImageManagmentStruct
 *                                  the image of this struct will be 
 *                                  connected to it´s event box.
 * Output: NONE.
 * --------------------------------------------------------------------- 
 * Notes: NONE.  
 * ********************************************************************/
 void connectImageToButtonPressEvent ( ImageManagmentStruct* pToImageManagmentStruct )
 {
   if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"connectImageToButtonPressEvent\".\033[01;37m\n" ); }
	 
    // Creating an GtkEventBox in order to allow the images to receive events.
    pToImageManagmentStruct->pEventBoxForImage = gtk_event_box_new ();
    
    // Add the image widget to the created GtkEventBox.
    gtk_container_add ( GTK_CONTAINER (pToImageManagmentStruct->pEventBoxForImage), pToImageManagmentStruct->pGtkImgWindow );
    
    // Conecting the "button_press_event" to the created GtkEventBox.
    g_signal_connect ( G_OBJECT (pToImageManagmentStruct->pEventBoxForImage),
					   "button_press_event",
					   G_CALLBACK (clickingCallback),
					   pToImageManagmentStruct
					 );	 
	 
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"connectImageToButtonPressEvent\".\033[01;37m\n\n" ); }
	 return;
 }
 
 
 
/***********************************************************************
 * Description: Function that initializes the main data structures of 
 *              the program.
 * ---------------------------------------------------------------------
 * Input:  The function receives 8 pointers to ImageManagmentStruct.
 *         Each one of them holds an information of a specific image.
 *         Four of the pointers used to contain information of images 
 *         for 4x4 grid representation.
 *         while the other four used to contain information of images 
 *         for single image representation.
 * Output: None (void).
 * ---------------------------------------------------------------------
 * Notes:  NONE.
 **********************************************************************/ 
 void dataStructuresInitialization ( ImageManagmentStruct* pLeftCameraImageStructForGrid, 
                                     ImageManagmentStruct* pRightCameraImageStructForGrid,
                                     ImageManagmentStruct* pKinectCameraImageStructForGrid,
                                     ImageManagmentStruct* pKinectDepthCameraImageStructForGrid,
                                     ImageManagmentStruct* pLeftCameraImageStructOneImage, 
                                     ImageManagmentStruct* pRightCameraImageStructOneImage,
                                     ImageManagmentStruct* pKinectCameraImageStructOneImage,
                                     ImageManagmentStruct* pKinectDepthCameraImageStructOneImage
                                   )
 {
	 int i = 0;		// Iterator.
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"dataStructuresInitialization\".\033[01;37m\n" ); }
	 
	 // Initializing the global arrays that hold the information about the images.
	 for ( i = 0; i <= MAX_NUMBER_OF_CAMERAS-1; i++ )
	 {
		 globalArrayOfImagesForGrid[i] = NULL;
		 globalArrayForSingleImages[i] = NULL;
	 }
	 
	 /** Initialization of data structures for FOUR BY FOUR GRID representation */
	 // Initializing the data structure of the left BB camera.
	 pLeftCameraImageStructForGrid->pSrcIplImage  = NULL;
	 pLeftCameraImageStructForGrid->pDstIplImage  = NULL;
	 pLeftCameraImageStructForGrid->pResizedIplImage = NULL;
	 pLeftCameraImageStructForGrid->pGtkPixbuf    = NULL;
	 pLeftCameraImageStructForGrid->pGtkImgWindow = NULL;
	 pLeftCameraImageStructForGrid->pEventBoxForImage = NULL;
	 pLeftCameraImageStructForGrid->previousImageSize = -1;
	 pLeftCameraImageStructForGrid->left_attach = 0;
	 pLeftCameraImageStructForGrid->right_attach = 1;
	 pLeftCameraImageStructForGrid->top_attach = 0;
	 pLeftCameraImageStructForGrid->bottom_attach = 1;
	 pLeftCameraImageStructForGrid->clickedOrNot = NOT_CLICKED;
	 pLeftCameraImageStructForGrid->imageIndex = ONE;
	 
	 // Initializing the data structure of the right BB camera.
	 pRightCameraImageStructForGrid->pSrcIplImage  = NULL;
	 pRightCameraImageStructForGrid->pDstIplImage  = NULL;
	 pRightCameraImageStructForGrid->pResizedIplImage = NULL;
	 pRightCameraImageStructForGrid->pGtkPixbuf    = NULL;
	 pRightCameraImageStructForGrid->pGtkImgWindow = NULL;
	 pRightCameraImageStructForGrid->pEventBoxForImage = NULL;
	 pRightCameraImageStructForGrid->previousImageSize = -1;
	 pRightCameraImageStructForGrid->left_attach = 1;
	 pRightCameraImageStructForGrid->right_attach = 2;
	 pRightCameraImageStructForGrid->top_attach = 0;
	 pRightCameraImageStructForGrid->bottom_attach = 1;
	 pRightCameraImageStructForGrid->clickedOrNot = NOT_CLICKED;
	 pRightCameraImageStructForGrid->imageIndex = TWO;
	 
	 // Initializing the data structure of the Kinect camera.
	 pKinectCameraImageStructForGrid->pSrcIplImage  = NULL;
	 pKinectCameraImageStructForGrid->pDstIplImage  = NULL;
	 pKinectCameraImageStructForGrid->pGtkPixbuf    = NULL;
	 pKinectCameraImageStructForGrid->pResizedIplImage = NULL;
	 pKinectCameraImageStructForGrid->pGtkImgWindow = NULL;
	 pKinectCameraImageStructForGrid->pEventBoxForImage = NULL;
	 pKinectCameraImageStructForGrid->previousImageSize = -1;
	 pKinectCameraImageStructForGrid->left_attach = 0;
	 pKinectCameraImageStructForGrid->right_attach = 1;
	 pKinectCameraImageStructForGrid->top_attach = 1;
	 pKinectCameraImageStructForGrid->bottom_attach = 2;
	 pKinectCameraImageStructForGrid->clickedOrNot = NOT_CLICKED;
	 pKinectCameraImageStructForGrid->imageIndex = THREE;
	 
	 // Initializing the data structure of the Kinect Depth camera.
	 pKinectDepthCameraImageStructForGrid->pSrcIplImage  = NULL;
	 pKinectDepthCameraImageStructForGrid->pDstIplImage  = NULL;
	 pKinectDepthCameraImageStructForGrid->pGtkPixbuf    = NULL;
	 pKinectDepthCameraImageStructForGrid->pResizedIplImage = NULL;
	 pKinectDepthCameraImageStructForGrid->pGtkImgWindow = NULL;
	 pKinectDepthCameraImageStructForGrid->pEventBoxForImage = NULL;
	 pKinectDepthCameraImageStructForGrid->previousImageSize = -1;
	 pKinectDepthCameraImageStructForGrid->left_attach = 1;
	 pKinectDepthCameraImageStructForGrid->right_attach = 2;
	 pKinectDepthCameraImageStructForGrid->top_attach = 1;
	 pKinectDepthCameraImageStructForGrid->bottom_attach = 2;
	 pKinectDepthCameraImageStructForGrid->clickedOrNot = NOT_CLICKED;
	 pKinectDepthCameraImageStructForGrid->imageIndex = FOUR;	 
	 
	 // Inserting the initialized data structures to the global array.
	 globalArrayOfImagesForGrid[0] = pLeftCameraImageStructForGrid;
	 globalArrayOfImagesForGrid[1] = pRightCameraImageStructForGrid;
	 globalArrayOfImagesForGrid[2] = pKinectCameraImageStructForGrid;
	 globalArrayOfImagesForGrid[3] = pKinectDepthCameraImageStructForGrid;
	 	 
     /** Initialization of data structures for ONE IMAGE representation */     
	 // Initializing the data structure of the left BB camera.
	 pLeftCameraImageStructOneImage->pSrcIplImage  = NULL;
	 pLeftCameraImageStructOneImage->pDstIplImage  = NULL;
	 pLeftCameraImageStructOneImage->pResizedIplImage = NULL;
	 pLeftCameraImageStructOneImage->pGtkPixbuf    = NULL;
	 pLeftCameraImageStructOneImage->pGtkImgWindow = NULL;
	 pLeftCameraImageStructOneImage->pEventBoxForImage = NULL;
	 pLeftCameraImageStructOneImage->previousImageSize = -1;
	 pLeftCameraImageStructOneImage->left_attach = 0;
	 pLeftCameraImageStructOneImage->right_attach = 1;
	 pLeftCameraImageStructOneImage->top_attach = 0;
	 pLeftCameraImageStructOneImage->bottom_attach = 1;
	 pLeftCameraImageStructOneImage->clickedOrNot = NOT_CLICKED;
	 pLeftCameraImageStructOneImage->imageIndex = FIVE;
	 
	 // Initializing the data structure of the right BB camera.
	 pRightCameraImageStructOneImage->pSrcIplImage  = NULL;
	 pRightCameraImageStructOneImage->pDstIplImage  = NULL;
	 pRightCameraImageStructOneImage->pResizedIplImage = NULL;
	 pRightCameraImageStructOneImage->pGtkPixbuf    = NULL;
	 pRightCameraImageStructOneImage->pGtkImgWindow = NULL;
	 pRightCameraImageStructOneImage->pEventBoxForImage = NULL;
	 pRightCameraImageStructOneImage->previousImageSize = -1;
	 pRightCameraImageStructOneImage->left_attach = 1;
	 pRightCameraImageStructOneImage->right_attach = 2;
	 pRightCameraImageStructOneImage->top_attach = 0;
	 pRightCameraImageStructOneImage->bottom_attach = 1;
	 pRightCameraImageStructOneImage->clickedOrNot = NOT_CLICKED;
	 pRightCameraImageStructOneImage->imageIndex = SIX;
	 
	 // Initializing the data structure of the Kinect camera.
	 pKinectCameraImageStructOneImage->pSrcIplImage  = NULL;
	 pKinectCameraImageStructOneImage->pDstIplImage  = NULL;
	 pKinectCameraImageStructOneImage->pGtkPixbuf    = NULL;
	 pKinectCameraImageStructOneImage->pResizedIplImage = NULL;
	 pKinectCameraImageStructOneImage->pGtkImgWindow = NULL;
	 pKinectCameraImageStructOneImage->pEventBoxForImage = NULL;
	 pKinectCameraImageStructOneImage->previousImageSize = -1;
	 pKinectCameraImageStructOneImage->left_attach = 0;
	 pKinectCameraImageStructOneImage->right_attach = 1;
	 pKinectCameraImageStructOneImage->top_attach = 1;
	 pKinectCameraImageStructOneImage->bottom_attach = 2;
	 pKinectCameraImageStructOneImage->clickedOrNot = NOT_CLICKED;
	 pKinectCameraImageStructOneImage->imageIndex = SEVEN;
	 
	 // Initializing the data structure of the Kinect Depth camera.
	 pKinectDepthCameraImageStructOneImage->pSrcIplImage  = NULL;
	 pKinectDepthCameraImageStructOneImage->pDstIplImage  = NULL;
	 pKinectDepthCameraImageStructOneImage->pGtkPixbuf    = NULL;
	 pKinectDepthCameraImageStructOneImage->pResizedIplImage = NULL;
	 pKinectDepthCameraImageStructOneImage->pGtkImgWindow = NULL;
	 pKinectDepthCameraImageStructOneImage->pEventBoxForImage = NULL;
	 pKinectDepthCameraImageStructOneImage->previousImageSize = -1;
	 pKinectDepthCameraImageStructOneImage->left_attach = 1;
	 pKinectDepthCameraImageStructOneImage->right_attach = 2;
	 pKinectDepthCameraImageStructOneImage->top_attach = 1;
	 pKinectDepthCameraImageStructOneImage->bottom_attach = 2;
	 pKinectDepthCameraImageStructOneImage->clickedOrNot = NOT_CLICKED;
	 pKinectDepthCameraImageStructOneImage->imageIndex = EIGHT;	 	 
	 
	 // Inserting the initialized data structures to the global array.
	 globalArrayForSingleImages[0] = pLeftCameraImageStructOneImage;
	 globalArrayForSingleImages[1] = pRightCameraImageStructOneImage;
	 globalArrayForSingleImages[2] = pKinectCameraImageStructOneImage;
	 globalArrayForSingleImages[3] = pKinectDepthCameraImageStructOneImage;
	      
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"dataStructuresInitialization\".\033[01;37m\n\n" ); }
	 return;
 }


 
/***********************************************************************
 * Description: Returns a new image that is a cropped version 
 *              (rectangular cut-out) of the original image.
 * ---------------------------------------------------------------------
 * Input:  img - original image, which will be cropped.
 *         region - the region of interest.
 * Output: the time interval between the start time and the end time.
 * --------------------------------------------------------------------- 
 * Notes:  The function was implemented by Shervin Emami.
 *         http://www.shervinemami.co.cc/
 * ********************************************************************/ 
IplImage* cropImage ( const IplImage *img, const CvRect region )
{
	IplImage *imageCropped;
	CvSize size;

	if ( img->width <= 0 || img->height <= 0 || region.width <= 0 || region.height <= 0 ) 
	{		
		ROS_ERROR ("cropImage: invalid dimensions. Aborting...\n");
		exit(1);
	}

	if (img->depth != IPL_DEPTH_8U) 
	{
		ROS_ERROR ("cropImage: image depth is not 8. Aborting...\n");
		exit(1);
	}

	// Set the desired region of interest.
	cvSetImageROI ( (IplImage*)img, region );
	
	// Copy region of interest into a new iplImage and return it.
	size.width = region.width;
	size.height = region.height;
	imageCropped = cvCreateImage ( size, IPL_DEPTH_8U, img->nChannels );
	cvCopy (img, imageCropped);	// Copy just the region.
	return imageCropped;
}



/***********************************************************************
 * Description: Creates a new image copy that is of a desired size. 
 *              The aspect ratio will be kept constant if 
 *              'keepAspectRatio' is true, by cropping undesired parts
 *              so that only pixels of the original image are shown, 
 *              instead of adding extra blank space.               
 * ---------------------------------------------------------------------
 * Input:  origImg - original image.
 *         newWidth - the desired width.
 *         newHeight - the desired height. 
 *         keepAspectRatio - boolean value, as depicted in the 
 *                           description.
 * Output: the time interval between the start time and the end time.
 * --------------------------------------------------------------------- 
 * Notes:  1. The function was implemented by Shervin Emami.
 *            http://www.shervinemami.co.cc/
 *         2. Remember to free the new image later.
 * ********************************************************************/ 
IplImage* resizeImage ( const IplImage *origImg, 
                        int newWidth,
                        int newHeight, 
                        bool keepAspectRatio
                      )
{
	IplImage *outImg = 0;
	int origWidth = 0;
	int origHeight = 0;
	if (origImg) {
		origWidth = origImg->width;
		origHeight = origImg->height;
	}
	if ( newWidth <= 0 || newHeight <= 0 || origImg == 0 || origWidth <= 0 || origHeight <= 0 ) 
	{
		ROS_ERROR ("resizeImage: Bad desired image size of %d, %d.\n", newWidth, newHeight );
		exit(1);
	}

	if ( keepAspectRatio ) 
	{
		// Resize the image without changing its aspect ratio,
		// by cropping off the edges and enlarging the middle section.
		CvRect r;
		
		// Input aspect ratio.
		float origAspect = ( origWidth / (float)origHeight );
		
		// Output aspect ratio.
		float newAspect = ( newWidth / (float)newHeight );
		
		// Crop width to be origHeight X newAspect.
		if ( origAspect > newAspect ) {
			int tw = ( origHeight * newWidth ) / newHeight;
			r = cvRect ( (origWidth - tw)/2, 0, tw, origHeight );
		}
		else 
		{	
			// Crop height to be origWidth / newAspect.
			int th = ( origWidth * newHeight ) / newWidth;
			r = cvRect ( 0, (origHeight - th)/2, origWidth, th );
		}
		IplImage *croppedImg = cropImage ( origImg, r );

		// Call this function again, with the new aspect ratio image.
		// Will do a scaled image resize with the correct aspect ratio.
		outImg = resizeImage ( croppedImg, newWidth, newHeight, false );
		cvReleaseImage( &croppedImg );
	}
	else 
	{
		// Scale the image to the new dimensions, even if the aspect ratio will be changed.
		outImg = cvCreateImage ( cvSize (newWidth, newHeight),
		                         origImg->depth, 
		                         origImg->nChannels
		                       );
		                       
		if ( (newWidth > origImg->width) && (newHeight > origImg->height) ) 
		{
			// Make the image larger.
			cvResetImageROI((IplImage*)origImg);
			
			// CV_INTER_LINEAR: good at enlarging.
			// CV_INTER_CUBIC: good at enlarging.			
			cvResize(origImg, outImg, CV_INTER_LINEAR);
		}
		else 
		{
			// Make the image smaller
			cvResetImageROI((IplImage*)origImg);
			
			// CV_INTER_AREA: good at shrinking (decimation) only.
			cvResize(origImg, outImg, CV_INTER_AREA);
		}
	}
	return outImg;
}


 
/***********************************************************************
 * Description: Create and show the main GUI window.
 * ---------------------------------------------------------------------
 * Input:  None 
 * Output: None
 * --------------------------------------------------------------------- 
 * Notes:  The function creates a GTK window, connects a "destroy" 
 *         signal to it, and shows it to the user.
 * ********************************************************************/
 void createMainInterfaceWindow ()
 {
	 
	 // Parameters connected to time maintenance.
	 timespec startTime, endTime;
	 timespec  timeIntrval;	 	
	
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"createMainInterfaceWindow\".\033[01;37m\n" ); }

	 // Time check.
	 if ( TIME_CONTROL ) {
	   if ( clock_gettime ( CLOCK_REALTIME, &startTime ) == -1 ) {
	     printf ("\033[01;32mERROR: quering start time in \"createMainInterfaceWindow\" failed...\033[01;37m\n" );
	   }
	 }
	 
	 globalTopLevelWindow = gtk_window_new (GTK_WINDOW_TOPLEVEL);     
	 gtk_window_set_title ( GTK_WINDOW (globalTopLevelWindow), "RAPOSA GUI"); 
	 gtk_window_set_default_size ( GTK_WINDOW(globalTopLevelWindow), 230, 150);
	 gtk_window_set_position ( GTK_WINDOW(globalTopLevelWindow), GTK_WIN_POS_CENTER);
	 g_signal_connect( G_OBJECT(globalTopLevelWindow), "destroy", G_CALLBACK(gtk_main_quit), NULL );
	 g_signal_connect( G_OBJECT(globalTopLevelWindow), "key-press-event", G_CALLBACK(keyCallback), NULL);
     
     // Time check.
	 if ( TIME_CONTROL ) 
	 {
	     if ( clock_gettime ( CLOCK_REALTIME, &endTime ) == -1 )
	     {
		     printf ("\033[01;32mERROR: time differences could not be calculated...\033[01;37m\n" );
	     }
	     else
	     {
		     // Calculating the time interval and printing it
		     timeIntrval = diff ( startTime, endTime ); 
		     printf ("\033[01;32mThe \"createMainInterfaceWindow\" function lasted %f seconds.\033[01;37m\n", (double)(timeIntrval.tv_nsec)*pow(10,-9) );
	     }		
     } 
     if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited: \"createMainInterfaceWindow\".\033[01;37m\n" ); }
     return;
 }


/***********************************************************************
 * Description: Creating a fixed container which will contain all the
 *              widgedts of the interface, and adding it to the main 
 *              interface window.
 * ---------------------------------------------------------------------
 * Input:  None 
 * Output: None
 * --------------------------------------------------------------------- 
 * Notes:  The container is needed in order to place the camera images
 *         in the wanted location inside the main GUI window.
 * ********************************************************************/
 void createFixedContainer ()
 {
	 // Parameters connected to time maintenance.
	 timespec startTime, endTime;
	 timespec  timeIntrval;	 
	 
	 // Time check.
	 if ( TIME_CONTROL ) 
     {
	  	 if ( clock_gettime ( CLOCK_REALTIME, &startTime ) == -1 )
		 {
	  		 printf ("\033[01;32mERROR: quering start time in \"createGtkTable\" failed...\033[01;37m\n" );
		 }
	 }
	 
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"createFixedContainer\".\033[01;37m\n" ); }
	 
	 pGlobalFixedContainer = gtk_fixed_new ();
     gtk_container_add ( GTK_CONTAINER(globalTopLevelWindow), pGlobalFixedContainer );
     
     // Time check.
	 if ( TIME_CONTROL ) 
	 {
	     if ( clock_gettime ( CLOCK_REALTIME, &endTime ) == -1 )
	     {
		     printf ("\033[01;32mERROR: time differences could not be calculated...\033[01;37m\n" );
	     }
	     else
	     {
		     // Calculating the time interval and printing it
		     timeIntrval = diff ( startTime, endTime ); 
		     printf ("\033[01;32mThe \"createGtkTable\" function lasted %f seconds.\033[01;37m\n", (double)(timeIntrval.tv_nsec)*pow(10,-9) );
	     }		
     }      
     
     if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited: \"createMainInterfaceWindow\".\033[01;37m\n" ); }
     return;
 }
 
 
/***********************************************************************
 * Description: Create a help structure (GtkTable), which will contain 
 *              the images.
 * ---------------------------------------------------------------------
 * Input:  None.
 * Output: None.
 * --------------------------------------------------------------------- 
 * Notes:  None.
 * ********************************************************************/
 void createGtkTable ()
 {	 
	 // Parameters connected to time maintenance.
	 timespec startTime, endTime;
	 timespec  timeIntrval;	 	
	
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"createGtkTable\".\033[01;37m\n" ); }

	 // Time check.
	 if ( TIME_CONTROL ) 
     {
	  	 if ( clock_gettime ( CLOCK_REALTIME, &startTime ) == -1 )
		 {
	  		 printf ("\033[01;32mERROR: quering start time in \"createGtkTable\" failed...\033[01;37m\n" );
		 }
	 }
	 
	 // Creating a new table widget with one row and two columns.
     pGlobalGtkTable = gtk_table_new ( ROW_NUMBER_IN_IMAGE_TABLE, COLUMN_NUMBER_IN_IMAGE_TABLE, TRUE );
     
     // Time check.
	 if ( TIME_CONTROL ) 
	 {
	     if ( clock_gettime ( CLOCK_REALTIME, &endTime ) == -1 )
	     {
		     printf ("\033[01;32mERROR: time differences could not be calculated...\033[01;37m\n" );
	     }
	     else
	     {
		     // Calculating the time interval and printing it
		     timeIntrval = diff ( startTime, endTime ); 
		     printf ("\033[01;32mThe \"createGtkTable\" function lasted %f seconds.\033[01;37m\n", (double)(timeIntrval.tv_nsec)*pow(10,-9) );
	     }		
     } 
     if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited: \"createGtkTable\".\033[01;37m\n" ); }
     return;
 }
 


/***********************************************************************
 * Description: Function that converts an IplImage to GtkWidget.
 * ---------------------------------------------------------------------
 * Input:  srcImage - pointer to an IplImage struct.
 * Output: GtkWidget - a GtkWidget, that contains the IplImage.
 * ---------------------------------------------------------------------
 * Notes:  Part of the variables used in this function are global.
 * ********************************************************************/
GtkWidget* convertOpenCv2Gtk ( ImageManagmentStruct* pToImageManagmentStruct,
                               functionFlag flag 
                             )
{		
	// Parameters connected to time maintenance.
    timespec startTime, endTime;
	timespec  timeIntrval;	
	
	if ( TIME_CONTROL ) 
    {
		if ( clock_gettime ( CLOCK_REALTIME, &startTime ) == -1 )
		{
			printf ("\033[01;32mERROR: quering start time in \"convertOpenCv2Gtk\" failed...\033[01;37m\n" );
		}
	}
	
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"convertOpenCv2Gtk\".\033[01;37m\n" ); }	
	
	if ( flag == CREATE )
	{
		/** Crating all the objects (widgets and pixbuf) for the ONE time */			
		// Creating the destionation image.
		pToImageManagmentStruct->pDstIplImage = cvCreateImage ( cvSize(pToImageManagmentStruct->pResizedIplImage->width,
		                                                               pToImageManagmentStruct->pResizedIplImage->height), 
		                                                        IPL_DEPTH_8U, 
		                                                        3
		                                                      );  
	
	    // Converting the format of the picture from BGR to RGB.
	    cvCvtColor ( pToImageManagmentStruct->pResizedIplImage, 
	                 pToImageManagmentStruct->pDstIplImage, 
	                 CV_BGR2RGB 
	               );	
	
	    // Creates a new GdkPixbuf out of in-memory image data.
	    pToImageManagmentStruct->pGtkPixbuf = gdk_pixbuf_new_from_data ( (guchar*)(pToImageManagmentStruct->pDstIplImage->imageData),
	                                                                     GDK_COLORSPACE_RGB,
	                                                                     FALSE,
	                                                                     pToImageManagmentStruct->pDstIplImage->depth,
	                                                                     pToImageManagmentStruct->pDstIplImage->width,
	                                                                     pToImageManagmentStruct->pDstIplImage->height,
	                                                                     (pToImageManagmentStruct->pDstIplImage->widthStep),
	                                                                     NULL,
	                                                                     NULL
	                                                                   );      

	    // Create new GtkImage displaying pixbuf.
	    pToImageManagmentStruct->pGtkImgWindow = gtk_image_new_from_pixbuf ( pToImageManagmentStruct->pGtkPixbuf );
	    
	    // Creating a GtkEventBox and connecting it to a "button_press_event" event.
	    // Needed in order to change the representation as a result of a single mouse click.
	    connectImageToButtonPressEvent ( pToImageManagmentStruct );
	}
	else
	{
		/** Refreshing all the objects (widgets and pixbuf) */		
	    // Converting the format of the picture from BGR to RGB (refreshing the pDstIplImage).	    
	    cvCvtColor ( pToImageManagmentStruct->pResizedIplImage, 
	                 pToImageManagmentStruct->pDstIplImage, 
	                 CV_BGR2RGB 
	               );
	    
	    // Refreshing the GDK window, which contains the image.
	    gtk_widget_queue_draw ( pToImageManagmentStruct->pGtkImgWindow );	    
	}		
	
	if ( TIME_CONTROL ) 
    {
		if ( clock_gettime ( CLOCK_REALTIME, &endTime ) == -1 )
	    {
			 printf ("\033[01;32mERROR: time differences could not be calculated...\033[01;37m\n" );
		}
		else
		{
			// Calculating the time interval and printing it
			timeIntrval = diff ( startTime, endTime ); 
			printf ("\033[01;32mThe \"convertOpenCv2Gtk\" function lasted %f seconds.\033[01;37m\n", (double)(timeIntrval.tv_nsec)*pow(10,-9) );
		}		
	 }
	 
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"convertOpenCv2Gtk\".\033[01;37m\n" );	}
	
	// Returning the created/refreshed image.
	return pToImageManagmentStruct->pGtkImgWindow ;
}



/***********************************************************************
 * Description: Showing one image which changes in real time.
 * ---------------------------------------------------------------------
 * Input:  msg - a ROS message from the publisher (const pointer to the 
 *               message).
 *         pToImageManagmentStruct - pointer to a ImageManagmentStruct,
 *         which represents the image that should be shown.
 * Output: None.
 * ---------------------------------------------------------------------
 * Notes:  NONE.
 **********************************************************************/
void showingSingleImage ( const sensor_msgs::ImageConstPtr& msg,
                          ImageManagmentStruct* pToImageManagmentStruct                     
                        )
{
	sensor_msgs::CvBridge bridge;
	
    // Parameters connected to time maintenance.
    timespec startTime, endTime;
	timespec  timeIntrval;	
	
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"showingSingleImage\".\033[01;37m\n" ); }
	
	if ( TIME_CONTROL ) 
    {
		if ( clock_gettime ( CLOCK_REALTIME, &startTime ) == -1 )
		{
			printf ("\033[01;32mERROR: quering start time in \"showingSingleImage\" failed...\033[01;37m\n" );
		}
	}	
	
	// Debug messages print.
	if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mshowingSingleImage: the index of the current image is:\033[01;37m" ); }
    switch ( pToImageManagmentStruct->imageIndex )
    {
		case ONE:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mONE\033[01;37m\n"); }   break; }
		case TWO:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mTWO\033[01;37m\n"); }  break; }
		case THREE:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mTHREE\033[01;37m\n"); } break; }
		case FOUR:        { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mFOUR\033[01;37m\n"); } break; }					 
		case FIVE:        { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mFIVE\033[01;37m\n"); }    break; }
		case SIX:         { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mSIX\033[01;37m\n");  }  break; }
		case SEVEN:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mSEVEN\033[01;37m\n"); }   break; }
		case EIGHT:       { if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mSEVEN\033[01;37m\n"); }   break; }								 
	    default:          { if ( INNER_FUNCTION_CONTROL ) { ROS_ERROR ("\033[01;34mclickingCallback: The image index is wrong. Aborting...\033[01;37m\n"); ros::shutdown();} }			    
    }	
    if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mshowingSingleImage: the \"clickedOrNot\" field is: %d.\033[01;37m\n", pToImageManagmentStruct->clickedOrNot ); }
	
	// Checking if the image that was received is the clicked one.
	if ( pToImageManagmentStruct->clickedOrNot == NOT_CLICKED )
	{
		if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mshowingSingleImage: the current image was not clicked, waiting for the next one.\033[01;37m\n" ); }
		return;
	}
	
	// Convert the ROS image message to an OpenCV image with BGR pixel encoding.
    try 
    {
	    pToImageManagmentStruct->pSrcIplImage = bridge.imgMsgToCv ( msg, "bgr8" );
    }
	catch (sensor_msgs::CvBridgeException& e)
    {
	   ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
	if ( pToImageManagmentStruct->pResizedIplImage != NULL )
	{
		cvReleaseImage ( &(pToImageManagmentStruct->pResizedIplImage) );
	}
    pToImageManagmentStruct->pResizedIplImage = resizeImage(pToImageManagmentStruct->pSrcIplImage, 
                                                            (int)(((pToImageManagmentStruct->pSrcIplImage->width)/globalResizingFactor)*2), 
                                                            (int)(((pToImageManagmentStruct->pSrcIplImage->height)/globalResizingFactor)*2), 
                                                            true
                                                           );
    
    /** In the next lines, the next actions are taken:
     *  1. Check the size of the received image.
     *  2. Compare it to the previous image size.
     *  3. Decide if a new image should be created, or it should only be refreshed.
     */
     if ( pToImageManagmentStruct->pResizedIplImage->imageSize != pToImageManagmentStruct->previousImageSize )
     {
		 /** Creation part */		 
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mEntered: \"Creation\" section in \"showingSingleImage\".\033[01;37m\n" ); }
		 // Updating the image size variable.
		 pToImageManagmentStruct->previousImageSize = pToImageManagmentStruct->pResizedIplImage->imageSize;
		 
		 // The GDK window should be created from skretch.
         pToImageManagmentStruct->pGtkImgWindow = convertOpenCv2Gtk ( pToImageManagmentStruct,
                                                                      CREATE
                                                                    ); 
          
		 // Attaching the created windget to the upper level container, to the same place where the table was.
	     gtk_fixed_put ( GTK_FIXED (pGlobalFixedContainer), 
	                     pToImageManagmentStruct->pEventBoxForImage, 
	                     IMAGES_HORIZONTAL_PLACEMENT, 
	                     IMAGES_VERTIACL_PLACEMENT
	                   );
		 
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mExited: \"Creation\" section in \"showingSingleImage\".\033[01;37m\n" ); }
	 }
	 else
	 {
		 /** Refreshing part */
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mEnter: \"Refresh\" section in \"showingSingleImage\".\033[01;37m\n" ); }    
		 
		 // The image inside the main window should only be refreshed.	
         pToImageManagmentStruct->pGtkImgWindow = convertOpenCv2Gtk ( pToImageManagmentStruct, 
                                                                      REFRESH 
                                                                    ); 
                                                                    
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mExited: \"Refresh\" section in \"showingSingleImage\".\033[01;37m\n" ); }                                                                    
  	 }
     
	 if ( TIME_CONTROL ) 
     {
		 if ( clock_gettime ( CLOCK_REALTIME, &endTime ) == -1 )
		 {
			 printf ("\033[01;32mERROR: time differences could not be calculated...\033[01;37m\n" );
		 }
		 else
		 {
			 // Calculating the time interval and printing it
			 timeIntrval = diff ( startTime, endTime ); 
			 printf ("\033[01;32mThe \"showingGrid\" function lasted %f seconds.\033[01;37m\n", (double)(timeIntrval.tv_nsec)*pow(10,-9) );
		 }		
	 } 
         	 
     if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"showingGrid\".\033[01;37m\n\n" ); }
     return;
}



/***********************************************************************
 * Description: Showing a 4x4 grid of images in real time.
 * ---------------------------------------------------------------------
 * Input:  msg - a ROS message from the publisher (const pointer to the 
 *               message).
 *         pToImageManagmentStruct - pointer to ImageManagmentStruct, 
 *         which contains all the information of a current image.
 * Output: None.
 * ---------------------------------------------------------------------
 * Notes:  NONE.
 **********************************************************************/
void showingGrid ( const sensor_msgs::ImageConstPtr& msg,
                   ImageManagmentStruct* pToImageManagmentStruct                     
                 )
{
	sensor_msgs::CvBridge bridge;
	
    // Parameters connected to time maintenance.
    timespec startTime, endTime;
	timespec  timeIntrval;	
	
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"showingGrid\".\033[01;37m\n" ); }
	
	if ( TIME_CONTROL ) 
    {
		if ( clock_gettime ( CLOCK_REALTIME, &startTime ) == -1 )
		{
			printf ("\033[01;32mERROR: quering start time in \"showingGrid\" failed...\033[01;37m\n" );
		}
	}	
	
	// Convert the ROS image message to an OpenCV image with BGR pixel encoding.
    try 
    {
	    pToImageManagmentStruct->pSrcIplImage = bridge.imgMsgToCv ( msg, "bgr8" );
    }
	catch (sensor_msgs::CvBridgeException& e)
    {
	   ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
	if ( pToImageManagmentStruct->pResizedIplImage != NULL )
	{
		cvReleaseImage ( &(pToImageManagmentStruct->pResizedIplImage) );
	}
	
	// Resizing the image.
    pToImageManagmentStruct->pResizedIplImage = resizeImage(pToImageManagmentStruct->pSrcIplImage, 
                                                            (int)((pToImageManagmentStruct->pSrcIplImage->width)/globalResizingFactor), 
                                                            (int)((pToImageManagmentStruct->pSrcIplImage->height)/globalResizingFactor), 
                                                            true
                                                           );

    
    /** In the next lines, the next actions are taken:
     *  1. Check the size of the received image.
     *  2. Compare it to the previous image size.
     *  3. Decide if a new image should be created, or it should only be refreshed.
     */
     if ( pToImageManagmentStruct->pResizedIplImage->imageSize != pToImageManagmentStruct->previousImageSize )
     {
		 /** Creation part */		 
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mEntered: \"Creation\" section in \"showingGrid\".\033[01;37m\n" ); }
		 // Updating the image size variable.
		 pToImageManagmentStruct->previousImageSize = pToImageManagmentStruct->pResizedIplImage->imageSize;
		 
		 // The GDK window should be created from skretch.
         pToImageManagmentStruct->pGtkImgWindow = convertOpenCv2Gtk ( pToImageManagmentStruct,
                                                                      CREATE
                                                                    ); 
          
         // Attach the created image to an appropriate place in the GtkGrid.
         gtk_table_attach_defaults ( GTK_TABLE(pGlobalGtkTable), 
                                     pToImageManagmentStruct->pEventBoxForImage,
                                     pToImageManagmentStruct->left_attach,
                                     pToImageManagmentStruct->right_attach,
                                     pToImageManagmentStruct->top_attach,
                                     pToImageManagmentStruct->bottom_attach
                                   );
		 
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mExited: \"Creation\" section in \"showingGrid\".\033[01;37m\n" ); }
	 }
	 else
	 {
		 /** Refreshing part */
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mEnter: \"Refresh\" section in \"showingGrid\".\033[01;37m\n" ); }    
		 // The image inside the main window should only be refreshed.	
         pToImageManagmentStruct->pGtkImgWindow = convertOpenCv2Gtk ( pToImageManagmentStruct, 
                                                                      REFRESH 
                                                                    ); 
		 if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mExited: \"Refresh\" section in \"showingGrid\".\033[01;37m\n" ); }                                                                    
  	 }
     
	 if ( TIME_CONTROL ) 
     {
		 if ( clock_gettime ( CLOCK_REALTIME, &endTime ) == -1 )
		 {
			 printf ("\033[01;32mERROR: time differences could not be calculated...\033[01;37m\n" );
		 }
		 else
		 {
			 // Calculating the time interval and printing it
			 timeIntrval = diff ( startTime, endTime ); 
			 printf ("\033[01;32mThe \"showingGrid\" function lasted %f seconds.\033[01;37m\n", (double)(timeIntrval.tv_nsec)*pow(10,-9) );
		 }		
	 } 
         	 
     if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"showingGrid\".\033[01;37m\n\n" ); }
     return;
}



/***********************************************************************
 * Description: Function that checks if any of the images of the 4X4
 *              grid was clicked, and returns the index of this image
 *              in the appropriate global array.
 * ---------------------------------------------------------------------
 * Input:  NONE.
 * Output: -1 - if no image was clicked.
 *          An index of the pointer to struct (ImageManagmentStruct), 
 *          that represents the clicked image in the global array.
 * ---------------------------------------------------------------------
 * Notes: None.     
 **********************************************************************/
 int returnIndexOfClickedImage ()
 {
	 int i = 0;
	 int indexOfClickedImage = -1;
	 
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"returnIndexOfClickedImage\".\033[01;37m\n" ); }
	 
 	 // Diagnosis loop.
 	 for ( i = 0; i <= MAX_NUMBER_OF_CAMERAS-1; i++ )
	 {
		 // Check if any of the images was clicked.
		 if ( globalArrayOfImagesForGrid[i]->clickedOrNot == CLICKED )
		 {
			 indexOfClickedImage = i;
			 break;		/** Only one image can be clicked at a certain point of time. */
		 }
	 } 
	 
	 if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"returnIndexOfClickedImage\".\033[01;37m\n\n" ); }
	 return indexOfClickedImage;	
 }
 
 
 
/***********************************************************************
 * Description: Performing a transition between 4x4 grid of images, to 
 *              one image representation.
 * ---------------------------------------------------------------------
 * Input:  msg - a ROS message from the publisher (const pointer to the 
 *               message).
 * Output: NONE.
 * ---------------------------------------------------------------------
 * Notes:  Part of the information that is used in this function is 
 *         extracted from the global arrays of images.
 **********************************************************************/
void transitionFromGridToImage ( const sensor_msgs::ImageConstPtr& msg )
{
	int indexOfClickedImage = -1;
	sensor_msgs::CvBridge bridge;
	
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"transitionFromGridToImage\".\033[01;37m\n" ); }	
	
	// Destroying the table that represents a grid.
	gtk_widget_destroy ( pGlobalGtkTable );
	
	if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mtransitionFromGridToImage: destroyed the grid.\033[01;37m\n" ); }
	
	// Determine which image among the four was clicked, and check correctness.
	indexOfClickedImage = returnIndexOfClickedImage();
	if ( returnIndexOfClickedImage() == -1 )
	{
		if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;33mtransitionFromGridToImage: no image was clicked. There is a bug, aborting...\033[01;37m\n"); }
		ros::shutdown();
	}
	
	if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mtransitionFromGridToImage: The index of the image in the global array is %d.\033[01;37m\n", indexOfClickedImage ); }
	
    // Convert the ROS image message to an OpenCV image with BGR pixel encoding.
    try 
    {
	    globalArrayForSingleImages[indexOfClickedImage]->pSrcIplImage = bridge.imgMsgToCv ( msg, "bgr8" );
    }
	catch (sensor_msgs::CvBridgeException& e)
    {
	   ROS_ERROR ("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
    
	if ( globalArrayForSingleImages[indexOfClickedImage]->pResizedIplImage != NULL )
	{
		cvReleaseImage ( &(globalArrayForSingleImages[indexOfClickedImage]->pResizedIplImage) );
	}	
	
	if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mtransitionFromGridToImage: Checked the memory of ResizedIplImage, and it is OK.\033[01;37m\n" ); }
	
	// Using the existing IplImage of the clicked image in the "globalArrayOfImagesForGrid"	
	// in order to create a new and resized IplImage in "globalArrayForSingleImages"
	globalArrayForSingleImages[indexOfClickedImage]->pResizedIplImage = 
	                                     resizeImage ( globalArrayForSingleImages[indexOfClickedImage]->pSrcIplImage,
			                                           (int)(((globalArrayForSingleImages[indexOfClickedImage]->pSrcIplImage->width)/globalResizingFactor)*2),
													   (int)(((globalArrayForSingleImages[indexOfClickedImage]->pSrcIplImage->height)/globalResizingFactor)*2),
													   true
													 );	   
    if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mtransitionFromGridToImage: the resize operation is done successfully.\033[01;37m\n" ); }													                                                                               
													                     
    // Updating the size of the image (needed for the next iterations).
    globalArrayForSingleImages[indexOfClickedImage]->previousImageSize = globalArrayForSingleImages[indexOfClickedImage]->pResizedIplImage->imageSize;
    
    if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mtransitionFromGridToImage: size update was done successfully.\033[01;37m\n" ); }
    
	// Create the GtkWidget which will contain the IplImage.
	globalArrayForSingleImages[indexOfClickedImage]->pGtkImgWindow = convertOpenCv2Gtk ( globalArrayForSingleImages[indexOfClickedImage],
																                         CREATE
																                        );
	if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mtransitionFromGridToImage: GtkWidget that contains the image was created successfully.\033[01;37m\n" ); }																                        
    																                        
	// Attaching the created windget to the upper level container, to the same place where the table was.
    gtk_fixed_put ( GTK_FIXED (pGlobalFixedContainer), 
                    globalArrayForSingleImages[indexOfClickedImage]->pEventBoxForImage, 
                    IMAGES_HORIZONTAL_PLACEMENT, 
                    IMAGES_VERTIACL_PLACEMENT
                  );                
	
	// Change the status of the system (state machine) to show one image.
	stateMachineStatus = SHOW_SINGLE_IMAGE;
	
	if ( INNER_FUNCTION_CONTROL ) { printf ("\033[01;34mtransitionFromGridToImage: The status of the state machine was changed.\033[01;37m\n" ); } 
	
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"transitionFromGridToImage\".\033[01;37m\n\n" ); }
	return;
}



/***********************************************************************
 * Description: Performing a transition between one image representation
 *              to 4x4 grid of images.
 * ---------------------------------------------------------------------
 * Input:  msg - a ROS message from the publisher (const pointer to the 
 *               message).
 * Output: NONE.
 * ---------------------------------------------------------------------
 * Notes:  All the information that is used in this function is 
 *         extracted from the global arrays of images.
 **********************************************************************/
void transitionFromImageToGrid ( const sensor_msgs::ImageConstPtr& msg )
{
	int i = 0;
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"transitionFromImageToGrid\".\033[01;37m\n" ); }	
	
	// Checking which of the images was shown, it´s data structures should be destroyed.
	 for ( i = 0; i <= MAX_NUMBER_OF_CAMERAS-1; i++ )
	 {
		 if ( GTK_IS_WIDGET (globalArrayForSingleImages[i]->pEventBoxForImage) )
		 {
			 gtk_widget_destroy ( globalArrayForSingleImages[i]->pEventBoxForImage );
		 }
	 }
	 
	// Initializing the whole data structures for a new cicle.
	dataStructuresInitialization ( globalArrayOfImagesForGrid[0],
	                               globalArrayOfImagesForGrid[1],
	                               globalArrayOfImagesForGrid[2],
	                               globalArrayOfImagesForGrid[3], 
	                               globalArrayForSingleImages[0],
	                               globalArrayForSingleImages[1],
	                               globalArrayForSingleImages[2],
	                               globalArrayForSingleImages[3]
	                             );
	
	// Change the state of the state machine.
	stateMachineStatus = SHOW_GRID;	
	
	// Creating a GtkGrid widget, which will contain all the images (it was destroyed in TRANSITION_FROM_GRID_TO_IMAGE state).
    createGtkTable ();
    
    // Attaching the created table to the Container.
    gtk_fixed_put (GTK_FIXED (pGlobalFixedContainer), pGlobalGtkTable, IMAGES_HORIZONTAL_PLACEMENT, IMAGES_VERTIACL_PLACEMENT);
	
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"transitionFromImageToGrid\".\033[01;37m\n\n" ); }
	return;
}                              



/***********************************************************************
 * Description: Wrapper callback function. 
 *              This function activates the "real" callback functions  
 *              according to the state of the state machine.
 * ---------------------------------------------------------------------
 * Input:  msg - a ROS message from the publisher (const pointer to the 
 *               message).
 *         pToImageManagmentStructForGrid - pointer to 
 *         ImageManagmentStruct, represents the current image for 4x4 
 *         image representation.
 *         pToImageManagmentStructForOneImage - pointer to 
 *         ImageManagmentStruct, represents the current image for one 
 *         image representation.
 * Output: None.
 * ---------------------------------------------------------------------
 * Notes:  After the activation of this function, a GDK + window should 
 *         appear, with an image inside it.
 **********************************************************************/
void imageCallbackWrapper ( const sensor_msgs::ImageConstPtr& msg,
                            ImageManagmentStruct* pToImageManagmentStructForGrid,
                            ImageManagmentStruct* pToImageManagmentStructForOneImage                    
                          )
{	
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mEntered: \"imageCallbackWrapper\".\033[01;37m\n" ); }
	
	switch ( stateMachineStatus )
	{
		case SHOW_SINGLE_IMAGE:
		{
			showingSingleImage ( msg, pToImageManagmentStructForOneImage );
			break;
		}
		case SHOW_GRID:
		{
			showingGrid ( msg, pToImageManagmentStructForGrid );
			break;
		}
		case TRANSITION_FROM_IMAGE_TO_GRID:
		{
			transitionFromImageToGrid ( msg );
			break;
		}
		case TRANSITION_FROM_GRID_TO_IMAGE:
		{
			transitionFromGridToImage( msg );
			break;
		}
		default:
		{
			ROS_ERROR ("The status of the state machine is wrong. Aborting...\n");
			ros::shutdown();
		}
	}

        // Check if HMD is active to show image there
        if (hmd_enabled)
          switch (pToImageManagmentStructForGrid->imageIndex) {
          case ONE:
            callbackHMD(msg, 0);
            break;
          case TWO:
            callbackHMD(msg, 1);
            break;
          default:
            // ignoring
            break;
          }
	     	 
    if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited:  \"imageCallbackWrapper\".\033[01;37m\n\n" ); }
    return;
} 


/***********************************************************************
 * Description: this function and the main function together, run the 
 *              main loop of the program.
 *              This solution doesn´t uses busy loop and doesn´t 
 *              enforsing the system to sleep.
 * --------------------------------------------------------------------
 * NOTE: Check out this link:
 * http://www.gtkforums.com/viewtopic.php?f=3&t=55652&p=72481#p72481
 **********************************************************************/
gboolean do_ros ( gpointer data )
{
	// Showing the main window, with the attached image.
	if ( GTK_IS_WIDGET (globalTopLevelWindow) )
	{
		gtk_widget_show_all ( globalTopLevelWindow ); 		
	}
	else
	{
		ROS_INFO ("The top level window of the application was closed...Exiting the program.\n");
        if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited from the main function normally.\033[01;37m\n" ); }
        return 0;
	} 
	
    if (ros::ok())
    {
        ros::spinOnce();
	}
    else
    {
        gtk_main_quit();
	}

    return TRUE;
}


 
/***********************************************************************
 * Description: the main function of the "Subscriber" module.
 * ---------------------------------------------------------------------
 * Input:  argc - the number of the parameters 
 *         argv - the list of the parameters.
 * Output: 0 - if the program returned normally.
 * ---------------------------------------------------------------------
 * Notes:  The main function subscribes to a topic, and calls a callback
 *         function, that should show an image in a GDK+ window.
 **********************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::NodeHandle nhh("~");
	ros::Rate loop_rate(100);		// Rate class instance.	
	ImageManagmentStruct LeftCameraImageStructForGrid, RightCameraImageStructForGrid, KinectImageStructForGrid, KinectDepthImageStructForGrid;		
	ImageManagmentStruct LeftCameraImageStructOneImage, RightCameraImageStructOneImage, KinectImageStructOneImage, KinectDepthImageStructOneImage;
		
    
    // Getting the resizing parameter from the launch file.
    if ( !nhh.getParam("resizing_factor", globalResizingFactor) )
    {
        ROS_ERROR ("The resizing factor was not loaded succesfully. Aborting...\n");
        ros::shutdown();
    }

    // Initializing the image/camera data structures.   
    dataStructuresInitialization ( &LeftCameraImageStructForGrid, 
				   &RightCameraImageStructForGrid, 
				   &KinectImageStructForGrid,
				   &KinectDepthImageStructForGrid,
				   &LeftCameraImageStructOneImage,
				   &RightCameraImageStructOneImage,
				   &KinectImageStructOneImage,
				   &KinectDepthImageStructOneImage );
    	
    // GTK library initialization
    gtk_init ( &argc, &argv );

    // Creating a top level window and connecting it to a sygnal.    
    createMainInterfaceWindow (); 

    // Creating a fixed container, which will hold all the widgets of the GUI.
    createFixedContainer ();
    
    // Creating a GtkGrid widget, which will contain all the images.
    createGtkTable ();
    
    // Attaching the created table to the Container.
    gtk_fixed_put (GTK_FIXED (pGlobalFixedContainer), pGlobalGtkTable, IMAGES_HORIZONTAL_PLACEMENT, IMAGES_VERTIACL_PLACEMENT);
                            
    // Subscribing in order to receive images.
    ros::Subscriber subLeft = nh.subscribe<sensor_msgs::Image> ("leftCameraTopic", 2, boost::bind(imageCallbackWrapper, _1, globalArrayOfImagesForGrid[0], globalArrayForSingleImages[0])); 	
    ros::Subscriber subRight = nh.subscribe<sensor_msgs::Image> ("rightCameraTopic", 2, boost::bind(imageCallbackWrapper, _1, globalArrayOfImagesForGrid[1], globalArrayForSingleImages[1])); 
    ros::Subscriber subKinect = nh.subscribe<sensor_msgs::Image> ("kinectCameraTopic", 2, boost::bind(imageCallbackWrapper, _1, globalArrayOfImagesForGrid[2], globalArrayForSingleImages[2]));
    ros::Subscriber subKinectDepth = nh.subscribe<sensor_msgs::Image> ("kinectDepthCameraTopic", 2, boost::bind(imageCallbackWrapper, _1, globalArrayOfImagesForGrid[3], globalArrayForSingleImages[3]));
    
    g_timeout_add(1000 / 100, do_ros, NULL);

    gtk_main();
    	 
	if ( MY_FUNCTIONS_ENTRANCE_CONTROL ) { printf ("\033[01;33mExited from the main function normally.\033[01;37m\n" ); }
    return 0;
}
