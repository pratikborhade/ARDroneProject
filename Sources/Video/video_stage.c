/*
 * @video_stage.c
 * @author marc-olivier.dzeukou@parrot.com
 * @date 2007/07/27
 *
 * ihm vision thread implementation
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>

#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <VP_Api/vp_api_picture.h>
#include <VP_Stages/vp_stages_io_file.h>
#include <VP_Stages/vp_stages_i_camif.h>

#include <config.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Os/vp_os_malloc.h>
#include <VP_Os/vp_os_delay.h>
#include <VP_Stages/vp_stages_yuv2rgb.h>
#include <VP_Stages/vp_stages_buffer_to_picture.h>
#include <VLIB/Stages/vlib_stage_decode.h>

#include <ardrone_tool/ardrone_tool.h>
#include <ardrone_tool/Com/config_com.h>

#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Control/ardrone_control_configuration.h>
#include <ardrone_tool/ardrone_tool_configuration.h>

#include "Video/video_stage.h"
#include <cv.h>
#include <highgui.h>


#define NB_STAGES 10
//#define delay

//Global Variables

PIPELINE_HANDLE pipeline_handle;
static vp_os_mutex_t  video_update_lock = PTHREAD_MUTEX_INITIALIZER;
ZAP_VIDEO_CHANNEL nextChannel = ZAP_CHANNEL_HORI;
CvSize vs_src_size;


//***************************************************GUI**********************************************************

void keyboard_control(char c)
{
	if( c=='s' || c=='S')
	{
		vp_os_mutex_lock(&control_data_lock);
		uint32_t altMax = 3000;
		uint32_t altMin = 50;

		ardrone_at_set_flat_trim();
		ARDRONE_TOOL_CONFIGURATION_ADDEVENT (altitude_max, &altMax, NULL); 
		ARDRONE_TOOL_CONFIGURATION_ADDEVENT (altitude_min, &altMin, NULL);

		if( control_data.start==0 )
			control_data.start = 1;
		else 
			control_data.start = 0;
  		vp_os_mutex_unlock(&control_data_lock); 
		vp_os_delay(50);
 	}  

	else if( c=='J' || c=='j')				//turn left 
	{
		vp_os_mutex_lock(&control_data_lock); 
		control_data.roll = -0.1;
		vp_os_mutex_unlock(&control_data_lock);
		vp_os_delay(20);
	}
	else if( c=='L' || c=='l')				//turn right 
	{
		vp_os_mutex_lock(&control_data_lock); 
		control_data.roll = 0.1;
		vp_os_mutex_unlock(&control_data_lock);
		vp_os_delay(20);
	}	
	else if( c=='I' || c=='i')				//go forward
	{	
		vp_os_mutex_lock(&control_data_lock); 
		control_data.pitch = -0.5;
		vp_os_mutex_unlock(&control_data_lock);
		vp_os_delay(20);
	}
	else if( c=='K' || c=='k')				//go backward		
	{
		vp_os_mutex_lock(&control_data_lock); 
		control_data.pitch = 0.5;
		vp_os_mutex_unlock(&control_data_lock);
		vp_os_delay(20);
	}
	  else if( c=='O' || c=='o')				//go yaw		
	{
			vp_os_mutex_lock(&control_data_lock); 
			control_data.yaw = -0.1;
			vp_os_mutex_unlock(&control_data_lock);
			vp_os_delay(20);
	}
	else if( c=='P' || c=='p')				
	{
		vp_os_mutex_lock(&control_data_lock); 
		MakeControlZero(&control_data);
		vp_os_mutex_unlock(&control_data_lock); 
	}
	else if( c == '1' )
	{
		nextChannel = ZAP_CHANNEL_VERT;
		g_imgsize_bottom = cvSize( 176,144 );
		vs_src_size = cvSize( 176,144 );
		ardrone_tool_configuration_addevent_video_channel( &nextChannel,NULL );
		vp_os_delay(500);    
	}
	else if( c == '2' )
	{
		nextChannel = ZAP_CHANNEL_HORI;
		g_imgsize_front = cvSize( QVGA_WIDTH,QVGA_HEIGHT );
		vs_src_size = cvSize( QVGA_WIDTH,QVGA_HEIGHT );
		vp_os_mutex_lock(&control_video_lock);
		g_src_front_img->width = QVGA_WIDTH;
		g_src_front_img->height = QVGA_HEIGHT;
		vp_os_mutex_unlock(&control_video_lock);
		ardrone_tool_configuration_addevent_video_channel( &nextChannel,NULL );
		vp_os_delay(500);
	}
	else if( c == '3' )
	{
		nextChannel = ZAP_CHANNEL_LARGE_HORI_SMALL_VERT;
		g_imgsize_front = cvSize(QVGA_WIDTH,QVGA_HEIGHT);
		g_imgsize_bottom = cvSize( 88,72 );
		vs_src_size = cvSize( QVGA_WIDTH,QVGA_HEIGHT );
		/*vp_os_mutex_lock(&control_video_lock);
		g_src_bottom_img->width = g_imgsize_bottom.width;
		g_src_bottom_img->height = g_imgsize_bottom.height;
		g_src_front_img->width = g_imgsize_front.width;
		g_src_front_img->height = g_imgsize_front.height;
		vp_os_mutex_unlock(&control_video_lock);*/
		ardrone_tool_configuration_addevent_video_channel( &nextChannel,NULL );
		vp_os_delay(500);
	}
	else if( c == '4' )
	{
		nextChannel = ZAP_CHANNEL_LARGE_VERT_SMALL_HORI;
		g_imgsize_bottom = cvSize(QVGA_WIDTH,QVGA_HEIGHT);
		vp_os_mutex_lock(&control_video_lock);
		g_src_bottom_img->width = QVGA_WIDTH;
		g_src_bottom_img->height = QVGA_HEIGHT;
		vp_os_mutex_unlock(&control_video_lock);
		ardrone_tool_configuration_addevent_video_channel( &nextChannel,NULL );
		vp_os_delay(500);
	}
	else if( c==27 )
		end_all_threads = 1;
}

int IsSizeEqual( CvSize x, CvSize y );

DEFINE_THREAD_ROUTINE(GUIThread,data)
{
	IplImage *src_bottom=NULL,*dst=NULL, *src_front=NULL;
	char c;
	printf("GUI Started\n");
	while( end_all_threads==0 )
	{
		//printf( "GUI RUNNING\n" );
		#ifdef delay
			double start_time = cvGetTickCount(), t_freq = cvGetTickFrequency(), end_time, tot_delay;
		#endif

		vp_os_mutex_lock(&control_video_lock);
		if( g_src_bottom_img && g_src_front_img )
		{
			src_bottom = cvCreateImage( g_imgsize_bottom,8,3 );
			src_front = cvCreateImage( g_imgsize_front,8,3 );
			cvCopy( g_src_bottom_img,src_bottom,NULL );
			cvCopy( g_src_front_img,src_front,NULL );
		}
		else if( g_src_bottom_img )
		{
			src_bottom = cvCreateImage( g_imgsize_bottom,8,3 );
			src_front = NULL;
			cvCopy( g_src_bottom_img,src_bottom,NULL );
		}
		else if( g_src_front_img )
		{
			src_bottom = NULL;
			src_front = cvCreateImage( g_imgsize_front,8,3 );
			cvCopy( g_src_front_img,src_front,NULL );
		}
		else
		{
			vp_os_mutex_unlock(&control_video_lock);
			vp_os_delay( 100 );
			continue;
		}

		vp_os_mutex_unlock(&control_video_lock);
		if( src_bottom )
			cvShowImage( "Bottom",src_bottom );
		if( src_front )
			cvShowImage( "Front",src_front );
		if( g_dst_img )
			cvShowImage( "dst",g_dst_img );

		c = cvWaitKey(30);
		keyboard_control(c);
		cvReleaseImage(&src_bottom);
		cvReleaseImage(&src_front);
		//cvReleaseImage(&dst);

		#ifdef delay
			end_time = cvGetTickCount();
			tot_delay = ((double)end_time - (double)start_time)/ ((float)t_freq);
			tot_delay = tot_delay/1000;
			printf("GUI BLOCK = %f millisecond\n",(float)tot_delay );
		#endif

	}
	return (SUCCESS);
}


//*********************************************************************************************************************


int IsSizeEqual( CvSize x, CvSize y )
{
	if( x.width == y.width && x.height == y.height )
		return(1);
	return(0);
}

C_RESULT output_gtk_stage_open( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
 
}
			

#ifdef delay
	double fc_start_time, fc_t_freq, fc_end_time, fc_tot_delay;
#endif

C_RESULT output_gtk_stage_transform( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
	/* Get a reference to the last decoded picture */
	IplImage *src,*temp;
	temp = cvCreateImageHeader( vs_src_size,8,3 );
	src = cvCreateImage( vs_src_size,8,3 );
	//src->widthStep = QVGA_WIDTH*3;
	temp->widthStep = QVGA_WIDTH*3;

	vp_os_mutex_lock(&video_update_lock);			//video update
	temp->imageData = (uint8_t*)in->buffers[0];
	cvCopy( temp,src,NULL );
	vp_os_mutex_unlock(&video_update_lock);
	cvCvtColor(src, src, CV_RGB2BGR );

	if( nextChannel == ZAP_CHANNEL_VERT )
	{
		vp_os_mutex_lock(&control_video_lock); 
		g_src_front_img = NULL;
		if( g_src_bottom_img==NULL || !IsSizeEqual(cvGetSize(src),cvGetSize(g_src_bottom_img))  )
			g_src_bottom_img = cvCreateImage(cvGetSize(src),8,3 );		//refresh the matrix size
		cvCopy( src,g_src_bottom_img,NULL );					//make sure the size is right before execute the copying
		vp_os_mutex_unlock(&control_video_lock); 
	}
	else if( nextChannel == ZAP_CHANNEL_HORI )
	{
		vp_os_mutex_lock(&control_video_lock);
		g_src_bottom_img = NULL;
		if( g_src_front_img==NULL || !IsSizeEqual(cvGetSize(src),cvGetSize(g_src_front_img))  )
			g_src_front_img = cvCreateImage(cvGetSize(src),8,3 );		//refresh the matrix size
		cvCopy( src,g_src_front_img,NULL );					//make sure the size is right before execute the copying
		vp_os_mutex_unlock(&control_video_lock); 
	}
	else if( nextChannel == ZAP_CHANNEL_LARGE_HORI_SMALL_VERT )
	{
		vp_os_mutex_lock(&control_video_lock); 
		temp->imageData = src->imageData;
		temp->width = g_imgsize_bottom.width;
		temp->height = g_imgsize_bottom.height;
		temp->widthStep = src->widthStep;

		if( g_src_bottom_img==NULL || !IsSizeEqual(cvGetSize(temp),cvGetSize(g_src_bottom_img)) )
			g_src_bottom_img = cvCreateImage(cvGetSize(temp),8,3 );		//refresh the matrix size

		if( g_src_front_img==NULL || !IsSizeEqual(cvGetSize(src),cvGetSize(g_src_front_img))  )
			g_src_front_img = cvCreateImage(cvGetSize(src),8,3 );		//refresh the matrix size

		cvCopy( temp,g_src_bottom_img,NULL );
		cvZero( temp );
		cvCopy( src,g_src_front_img,NULL );					//make sure the size is right before execute the copying
		vp_os_mutex_unlock(&control_video_lock); 
	}


	cvReleaseImageHeader(&temp);
	cvReleaseImage(&src);

	#ifdef delay
		fc_end_time = cvGetTickCount();
		fc_t_freq = cvGetTickFrequency();
		fc_tot_delay = ((double)fc_end_time - (double)fc_start_time)/ ((float)fc_t_freq);
		fc_tot_delay = fc_tot_delay/1000;
		printf("Image Capturing Block = %f millisecond\n",(float)fc_tot_delay );
		fc_start_time = cvGetTickCount();
	#endif

	if( end_all_threads!=0 )
		return(C_FAIL);

	return (SUCCESS);
}

C_RESULT output_gtk_stage_close( void *cfg, vp_api_io_data_t *in, vp_api_io_data_t *out)
{
  return (SUCCESS);
}


const vp_api_stage_funcs_t vp_stages_output_gtk_funcs =
{
  NULL,
  (vp_api_stage_open_t)output_gtk_stage_open,
  (vp_api_stage_transform_t)output_gtk_stage_transform,
  (vp_api_stage_close_t)output_gtk_stage_close
};

DEFINE_THREAD_ROUTINE(video_stage, data)
{
  C_RESULT res;

  vp_api_io_pipeline_t    pipeline;
  vp_api_io_data_t        out;
  vp_api_io_stage_t       stages[NB_STAGES];

  vp_api_picture_t picture;

  video_com_config_t              icc;
  vlib_stage_decoding_config_t    vec;
  vp_stages_yuv2rgb_config_t      yuv2rgbconf;
  /// Picture configuration
  picture.format        = PIX_FMT_YUV420P;

  picture.width         = QVGA_WIDTH;
  picture.height        = QVGA_HEIGHT;
  picture.framerate     = 30;

  picture.y_buf   = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT     );
  picture.cr_buf  = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT / 4 );
  picture.cb_buf  = vp_os_malloc( QVGA_WIDTH * QVGA_HEIGHT / 4 );

  picture.y_line_size   = QVGA_WIDTH;
  picture.cb_line_size  = QVGA_WIDTH / 2;
  picture.cr_line_size  = QVGA_WIDTH / 2;

  vp_os_memset(&icc,          0, sizeof( icc ));
  vp_os_memset(&vec,          0, sizeof( vec ));
  vp_os_memset(&yuv2rgbconf,  0, sizeof( yuv2rgbconf ));

  icc.com                 = COM_VIDEO();
  icc.buffer_size         = 100000;
  icc.protocol            = VP_COM_UDP;
  COM_CONFIG_SOCKET_VIDEO(&icc.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);

  vec.width               = QVGA_WIDTH;
  vec.height              = QVGA_HEIGHT;
  vec.picture             = &picture;
  vec.block_mode_enable   = TRUE;
  vec.luma_only           = FALSE;

  yuv2rgbconf.rgb_format = VP_STAGES_RGB_FORMAT_RGB24;

  pipeline.nb_stages = 0;

  stages[pipeline.nb_stages].type    = VP_API_INPUT_SOCKET;
  stages[pipeline.nb_stages].cfg     = (void *)&icc;
  stages[pipeline.nb_stages].funcs   = video_com_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_FILTER_DECODER;
  stages[pipeline.nb_stages].cfg     = (void*)&vec;
  stages[pipeline.nb_stages].funcs   = vlib_decoding_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_FILTER_YUV2RGB;
  stages[pipeline.nb_stages].cfg     = (void*)&yuv2rgbconf;
  stages[pipeline.nb_stages].funcs   = vp_stages_yuv2rgb_funcs;

  pipeline.nb_stages++;

  stages[pipeline.nb_stages].type    = VP_API_OUTPUT_SDL;
  stages[pipeline.nb_stages].cfg     = NULL;
  stages[pipeline.nb_stages].funcs   = vp_stages_output_gtk_funcs;

  pipeline.nb_stages++;

  pipeline.stages = &stages[0];

  //change the camera
  //nextChannel = ZAP_CHANNEL_VERT;
  //ardrone_tool_configuration_addevent_video_channel(&nextChannel,NULL);
  g_imgsize_bottom = cvSize(QVGA_WIDTH,QVGA_HEIGHT);
  g_dst_img = cvCreateImage( g_imgsize_bottom,8,1 );
  g_src_bottom_img = NULL;
  g_src_front_img = NULL;
  vs_src_size = cvSize( QVGA_WIDTH,QVGA_HEIGHT );
  g_imgsize_bottom = cvSize( 0,0 );
  g_imgsize_front = cvSize( QVGA_WIDTH,QVGA_HEIGHT );
  cvNamedWindow( "Bottom", CV_WINDOW_AUTOSIZE );
  cvNamedWindow( "Front", CV_WINDOW_AUTOSIZE );
  cvNamedWindow( "dst", CV_WINDOW_AUTOSIZE );

  /* Processing of a pipeline */
  if( !ardrone_tool_exit() )				//running the pipeline
  {
    PRINT("\n   Video stage thread initialisation1\n\n");

    res = vp_api_open(&pipeline, &pipeline_handle);

    if( SUCCEED(res) )
    {
      int loop = SUCCESS;
      out.status = VP_API_STATUS_PROCESSING;

      while( !ardrone_tool_exit() && (loop == SUCCESS) )
      {
          if( SUCCEED(vp_api_run(&pipeline, &out)) ) {
            if( (out.status == VP_API_STATUS_PROCESSING || out.status == VP_API_STATUS_STILL_RUNNING) ) {
              loop = SUCCESS;
            }
          }
          else loop = -1; // Finish this thread
      }

      vp_api_close(&pipeline, &pipeline_handle);
    }
  }

  PRINT("   Video stage thread ended\n\n");

  return (THREAD_RET)0;
}

