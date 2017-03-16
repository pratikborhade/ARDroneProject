#ifndef _IHM_STAGES_O_GTK_H
#define _IHM_STAGES_O_GTK_H

#include <config.h>
#include <VP_Api/vp_api_thread_helper.h>


PROTO_THREAD_ROUTINE(video_stage, data);
PROTO_THREAD_ROUTINE(GUIThread, data);

//Opencv includes
#include <cv.h>
#include <highgui.h>

static vp_os_mutex_t  control_data_lock = PTHREAD_MUTEX_INITIALIZER;	//change the control data
static vp_os_mutex_t  control_video_lock = PTHREAD_MUTEX_INITIALIZER;   //change the video stream
static vp_os_mutex_t  dst_video_lock = PTHREAD_MUTEX_INITIALIZER;   //change the Destination stream
static vp_os_mutex_t ip_data_lock = PTHREAD_MUTEX_INITIALIZER;

typedef struct control_data_t						//define the structure of control data;
{
    float roll;
    float pitch;
    float gaz;
    float yaw;
    int start;
    double timestamp;
}control_data_t; 

IplImage *g_src_bottom_img;							//the source image
IplImage *g_dst_img;							//the result image
IplImage *g_src_front_img;						//src front image

int end_all_threads;							//all threads control

control_data_t ip_bottom_data,control_data, ip_front_data;

CvSize g_imgsize_bottom;
CvSize g_imgsize_front;


#endif // _IHM_STAGES_O_GTK_H
