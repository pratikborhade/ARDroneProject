/**
 * @file main.c
 * @author sylvain.gaeremynck@parrot.com
 * @date 2009/07/01
 */
#include <ardrone_testing_tool.h>

//ARDroneLib
#include <ardrone_tool/ardrone_time.h>
#include <ardrone_tool/Navdata/ardrone_navdata_client.h>
#include <ardrone_tool/Control/ardrone_control.h>
#include <ardrone_tool/UI/ardrone_input.h>

//Common
#include <config.h>
#include <ardrone_api.h>

//VP_SDK
#include <ATcodec/ATcodec_api.h>
#include <VP_Os/vp_os_print.h>
#include <VP_Api/vp_api_thread_helper.h>
#include <VP_Os/vp_os_signal.h>

//Local project
//#include <UI/gamepad.h>
#include <Video/video_stage.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <fcntl.h> 
#include <math.h>

#include <ardrone_tool/Control/ardrone_control_configuration.h>
#include <ardrone_tool/ardrone_tool_configuration.h>

//****************************************** VARIABLES *******************************************************

#define COLOR 175,180,160,0
#define DecayConst 10
#define RollFactor 3
#define PIXEL_THRESH 150
#define PIXEL_FRONT_THRESH 300
#define AUTO 0
#define Front_Line_Seach_Depth 10
int g_bottom_lineDetected=0, g_front_lineDetected=0;
//#define delay

//*********************************************************************************************************************


//*************************************************** Memory Module *********************************************

typedef struct Q
{
	control_data_t control;
	struct Q *next,*prev;
}Q;

void QAdd( Q **curr, control_data_t control )
{
	//PRINT("Q_ADD\n");
	(*curr)->next = (Q *)vp_os_malloc(sizeof(Q));
	(*curr)->next->prev = (*curr);
	(*curr)=(*curr)->next;
	(*curr)->control = control;
	(*curr)->control.timestamp = cvGetTickCount();
}

Q* QInit()
{
	Q *new;
	new = (Q *)vp_os_malloc(sizeof(Q));
	new->next = NULL;
	new->prev = NULL;
	return( new );
}

void QDel( Q **curr )
{
	//PRINT("Q_DEL\n");
	Q *prev;
	prev = (*curr);
	(*curr)=(*curr)->next;
	(*curr)->prev = NULL;
	vp_os_free(prev);
}

control_data_t QPop( Q **curr )
{
	control_data_t local_data;
	local_data = (*curr)->control;
		if ( (*curr)->prev == NULL )
		{
			//ardrone_tool_set_ui_pad_start( 0 );
			//ardrone_tool_shutdown_custom();
		}
	(*curr) = (*curr)->prev;
	vp_os_free( (*curr)->next );
	(*curr)->next = NULL;
	return(local_data);
}

Q *g_Qstart = NULL,*g_Qend = NULL;

void MemoryClearRoutine( )
{
	double curr_time = cvGetTickCount();
	double total_time = (double)(curr_time - g_Qend->control.timestamp )/ cvGetTickFrequency();
	if(  total_time > 5000000. && g_Qend != g_Qstart )
		QDel( &g_Qend );
}

//***************************************************************************************************************



//******************************************** CONTROL FUNCTIONS************************************************************

void MakeControlZero(control_data_t *l_control)
{
	l_control->roll = 0;
	l_control->pitch = 0;
	l_control->gaz = 0;
	l_control->yaw = 0;

}

void MakeControlTendZero(control_data_t *l_control)
{
	l_control->roll = l_control->roll - l_control->roll/5;
	l_control->pitch = l_control->pitch - l_control->pitch/5;
	l_control->gaz = l_control->gaz - l_control->gaz/5;
	l_control->yaw = l_control->yaw - l_control->yaw/5;

}

control_data_t GradualChangeOfControl( control_data_t l_control,control_data_t prev_control,int grad ) //to change the control gradually
{
	control_data_t control_value;
	control_value.roll = prev_control.roll + (l_control.roll - prev_control.roll)/grad;
	control_value.pitch = prev_control.pitch + (l_control.pitch - prev_control.pitch)/(grad);
	control_value.gaz = prev_control.gaz + (l_control.gaz - prev_control.gaz)/grad;
	control_value.yaw = prev_control.yaw +(l_control.yaw - prev_control.yaw)/grad;
	control_value.start = prev_control.start;

	if( ( l_control.roll < 0 && prev_control.roll >0 ) || ( l_control.roll > 0 && prev_control.roll < 0 ) )
		control_value.roll = 0;

	if( ( l_control.pitch < 0 && prev_control.pitch >0 ) || ( l_control.pitch > 0 && prev_control.pitch < 0 ) )
		control_value.pitch = 0;

	if( ( l_control.gaz < 0 && prev_control.gaz >0 ) || ( l_control.gaz > 0 && prev_control.gaz < 0 ) )
		control_value.gaz = 0;

	if( ( l_control.yaw < 0 && prev_control.yaw >0 ) || ( l_control.yaw > 0 && prev_control.yaw < 0 ) )
		control_value.yaw = 0;
	return(control_value);
}

control_data_t GradualChangeOfControlFine( control_data_t l_control,control_data_t prev_control,int gradroll,int gradpitch, int gradgaz, int gradyaw  ) //to change the control gradually
{
	control_data_t control_value;
	control_value.roll = prev_control.roll + (float)(l_control.roll - prev_control.roll)/(float)gradroll;
	control_value.pitch = prev_control.pitch + (float)(l_control.pitch - prev_control.pitch)/(float)gradpitch;
	control_value.gaz = prev_control.gaz + (float)(l_control.gaz - prev_control.gaz)/(float)gradgaz;
	control_value.yaw = prev_control.yaw +(float)(l_control.yaw - prev_control.yaw)/(float)gradyaw;
	control_value.start = prev_control.start;
	return(control_value);
}

control_data_t calcNegativeControl( control_data_t src )
{
	control_data_t dst;
	dst.roll = -src.roll;
	dst.pitch = -src.pitch;
	dst.gaz = 0;
	dst.yaw = -src.yaw;
	dst.start = src.start;
	dst.timestamp = src.timestamp;
	return( dst );
}

void printControlData(control_data_t src)
{
	PRINT( "ROLL:%f , PITCH:%f, GAZ:%f, YAW:%f\n\n",src.roll,src.pitch,src.gaz,src.yaw );
}

static int32_t exit_ihm_program = 1;

//********************************************IP FUNCTIONS ************************************************************

void FindGrayPixels( IplImage *src,IplImage *dst,int threshold )
{
    uchar *ptr_src = src->imageData, *ptr_dst = dst->imageData;
    int i,j;

    for( i=0;i<src->width;i++ )
    for( j=0;j<src->height;j++ )
    {
        if( ptr_src[1] < 50 && ptr_src[2] > threshold )
            *ptr_dst = 255;
        else
            *ptr_dst = 0;
        ptr_src = ptr_src + src->nChannels;
        ptr_dst = ptr_dst + dst->nChannels;
    }
} 


int IfZero( IplImage *src )
{
    int i,j,k;
    uchar *ptr;
    ptr = (uchar *)src->imageData;
    for( i=0;i<src->width;i++ )
    for( j=0;j<src->height;j++ )
    for( k=0;k<src->nChannels;k++ )
    {
        if( *ptr>127)
            return(0);
        ptr++;
    }
    return(1);
}

void Skeletonisation( IplImage *src, IplImage *dst )
{
    IplConvKernel* ker;
    IplImage *temp,*prev_img,*ans_iteration,*answer;

    temp = cvCreateImage( cvGetSize(src),src->depth,src->nChannels );
    ker = cvCreateStructuringElementEx( 3,3,1,1,CV_SHAPE_CROSS,NULL );
    prev_img = cvCreateImage( cvGetSize(src),src->depth,src->nChannels );
    ans_iteration = cvCreateImage( cvGetSize(src),src->depth,src->nChannels );
    answer = cvCreateImage( cvGetSize(src),src->depth,src->nChannels );

        //cvErode( src,dst, ker,1 )
    cvZero(answer);
    cvCopy( src,temp,NULL );

    while( IfZero( temp )==0 )
    {
        cvCopy( temp,prev_img,NULL );
        cvErode( temp,temp, ker,1 );
        cvDilate( temp,temp, ker,1 );
        cvSub( prev_img, temp, ans_iteration,NULL );
        cvAdd( answer,ans_iteration,answer,NULL );
                cvCopy( prev_img,temp,NULL );
                cvErode( temp,temp, ker,1 );
                //cvShowImage( "Out",answer );
                //cvWaitKey(100);
    }
        cvCopy(answer,dst,NULL);
    cvReleaseImage(&temp);
    cvReleaseImage(&prev_img);
    cvReleaseImage(&ans_iteration);

} 

float calculateDegree( float degree )
{
	if( degree > 90 )
	{
		return( degree-180 );
	}
	else
		return( degree );
}

float calculateRoll( float rho, int width )
{
	if( rho < ((float)width/2) )
		return( -1*(float)(((float)width/2) - rho)/((float)width/2) );
	else 
		return( (float)(rho - ((float)width/2))/((float)width/2) );
}

typedef struct structure_COM
{
    int x;
    int y;
    long int n;
}structure_COM;

float calculateRollByPow( int width,structure_COM COM,int div )
{
	float dist = COM.x - width/2.;
	if( dist < 0 )
		return( -1 * pow( ((float) abs(dist)/(float)(width/3.)),1.5)/div );
	else 
		return( pow( ((float) abs(dist)/(float)(width/3.)),1.5)/div );
}


structure_COM calcCenterOfMass( IplImage *color_pos ) // Calculate Center of Mass
{
	int i,j;
	float x=0,y=0,n=0;
	structure_COM center;

	for( i=0;i<color_pos->width;i++ )
        	for( j=0;j<color_pos->height;j++ )
        	{
			uchar *ptr_src;
                	ptr_src = (uchar *) color_pos->imageData + j * color_pos->widthStep + i * color_pos->nChannels;
               		if( *ptr_src > 127 )
			{
                    		x = x+i;
				y = y+j;
				n++;
			}			
        	}
	center.x = (int) x/n;
	center.y = (int) y/n;
	center.n = n;
	return(center);
}

void MakeCOMZero( structure_COM *com )
{
	com->x=0;
	com->y=0;
	com->n=0;
}

float calcDistHSI( CvScalar pix,CvScalar sclr) //Calculate the Distance between two Scalars
{
    int x;
    x = pix.val[0]-sclr.val[0];
    if( abs(x) > 90 )
	x = 180-abs(x);
    return( ( abs(x)+abs(pix.val[1]-sclr.val[1])/3 ) );  // abs( h1-h2 ) + abs( (s1-s2)/3 ) -- More dependent on Colour which is in Hue Space 
}


int BackProjectHSI( IplImage *src,IplImage *dst, CvScalar sclr, int thresh ) // Get a Image corresponding to Specific Image
{
    int i,j;
    CvScalar pix;
    IplImage *src_hsi;
    src_hsi = cvCreateImage( cvGetSize(src),src->depth,src->nChannels );
    cvCvtColor( src,src_hsi,CV_BGR2HSV );

    for( i=0;i<src->width;i++ )
        for( j=0;j<src->height;j++ )
        {
		uchar *ptr_src,*ptr_dst;
                ptr_src = (uchar *) src_hsi->imageData + j * src_hsi->widthStep + i * src_hsi->nChannels;
                ptr_dst = (uchar *) dst->imageData + j * dst->widthStep + i * dst->nChannels;
                pix = cvScalar( ptr_src[0],ptr_src[1],ptr_src[2],0 );
		//if( i==src->width/2 && j==src->height/2 )
		//	printf( "Pix : %f,%f,%f\n" ,ptr_src[0],ptr_src[1],ptr_src[2] );
                if( calcDistHSI(pix,sclr)<thresh )
                    *ptr_dst = 255;
		else
			*ptr_dst = 0;
        }
    cvReleaseImage( &src_hsi );
    return(1);
}

float calculateRhoFromTheta(int i,int j,float degree)
{
	float rho;
	rho = i * cos( degree/28.662420 ) + j * sin( degree/28.662420 );
	return( rho );
}

CvPoint *calcLinePoints( float rho, float degree , CvSize size)
{
	CvPoint *points;
	int i=0,j=0,flag = 0;
	points = ( CvPoint * ) vp_os_malloc( 2*sizeof(CvPoint) );
	
	for( i=0;i<size.width;i++ )
		for( j=0;j<size.height;j++ )
			if( abs( (float) calculateRhoFromTheta(i,j,degree) - (float) rho ) < 5 )
				if( flag == 0 )
				{
					flag = 1;
					points->x = i;
					points->y = j;
				}
				else
				{
					points[1].x = i;
					points[1].y = j;
				}
	return( points );
}

void traceline(IplImage* src, IplImage* BW){
	//int i;	
	float* line1;
	float* line2;
	float degree;
	float rho;
	structure_COM COM;
	CvPoint *points;

	control_data_t local_data;
	IplImage* dst = cvCreateImage(cvGetSize(src),src->depth,1);

	CvSeq* lines = 0;
	CvMemStorage* storage = cvCreateMemStorage(0);
	MakeControlZero( &local_data );
	cvCopy(src,dst,NULL);

	COM = calcCenterOfMass( BW );
	if( COM.n > PIXEL_THRESH )
	{
		local_data.roll = calculateRoll( COM.x,src->width )/RollFactor;
		g_bottom_lineDetected = 1;
	}
	else
	{
		g_bottom_lineDetected = 0;
		vp_os_delay(50);
		return;
	}	

	lines = cvHoughLines2( dst, storage, CV_HOUGH_STANDARD , 1, CV_PI/90, 30, 0, 0 );	//hough transform
	if(lines->total > 0)	//printf("lines= %f\n",line[1]);
	{	
		if(lines->total > 1)
		{
			line1 = (float*)cvGetSeqElem(lines,0);
			line2 = (float*)cvGetSeqElem(lines,1);
			degree = (line1[1] + line2[1])*28.66;
			rho = (line1[0] + line2[0])/2.0;
		}
		else
		{
			line1 = (float*)cvGetSeqElem(lines,0);
			degree = line1[1]*57.32;
			rho = line1[0];
		}
		//printf( "rho :%f , Deg: %f\n",rho,degree );
		//points = calcLinePoints( rho, degree , cvGetSize(dst) );
		local_data.yaw = calculateDegree( degree )/90.;
	}

	
	//if( COM.y < (float) 2*dst->height/3.5 && AUTO)
	//	local_data.pitch = -0.1;

	vp_os_mutex_lock(&ip_data_lock);
	ip_bottom_data = local_data;
	//printf("\nData %f,%f,%f\n\n",ip_bottom_data.roll,ip_bottom_data.pitch,ip_bottom_data.yaw);
	ip_bottom_data.timestamp = cvGetTickCount();
	vp_os_mutex_unlock(&ip_data_lock);
	vp_os_delay(20); 
	cvReleaseMemStorage(&storage);

///*	vp_os_mutex_lock(&dst_video_lock);   // ----------------------------------------------------- TO keep Destination Image as Bottom Camera
	/*if( g_dst_img==NULL || !IsSizeEqual(cvGetSize(g_dst_img),cvGetSize(dst)) )
			g_dst_img = cvCreateImage( g_imgsize_bottom,dst->depth,3 );

	{
		//printf("Going IN\n");
		uchar *ptrBW,*ptrDst,*ptrg_dst;
		int i,j;
		for( i=0;i<BW->width;i++ )
			for( j=0;j<BW->height;j++ )
			{
				ptrBW = (uchar *) BW->imageData + j * BW->widthStep + i * BW->nChannels;
				ptrDst = (uchar *) dst->imageData + j * dst->widthStep + i * dst->nChannels;
				ptrg_dst = (uchar *) g_dst_img->imageData + j * g_dst_img->widthStep + i * g_dst_img->nChannels;

				if( ptrDst[0] > 127 )
				{
					//float rho_t;
					ptrg_dst[0] = 255;
					ptrg_dst[1] = 255;
					ptrg_dst[2] = 255;
					//rho_t = calculateRhoFromTheta( i,j,degree);
					//printf( "rho :%f , Deg: %f , calcR: %f\n",rho,degree,rho_t );
				}
				else if( ptrBW[0] > 127 )
				{
					ptrg_dst[0] = 120;
					ptrg_dst[1] = 120;
					ptrg_dst[2] = 120;
				}
				else
				{
					ptrg_dst[0] = 0;
					ptrg_dst[1] = 0;
					ptrg_dst[2] = 0;
				}
			}
	}
	cvCircle( g_dst_img,cvPoint(COM.x,COM.y),3,CV_RGB(255,0,0),CV_FILLED,8,0 );
//	cvLine( g_dst_img,points[0],points[1],CV_RGB(0,0,255),1,8,0 );
//	vp_os_mutex_unlock(&dst_video_lock);*/
	cvReleaseImage(&dst);

}




//image object threshold
void LineFollower(IplImage* src, IplImage* dst){					//threshold the image based on RGB channel

	IplImage *BW =  cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,1);
	IplImage *s = cvCreateImage(cvGetSize(src),IPL_DEPTH_8U,3);
	CvSize size = cvGetSize(src);
	cvZero(BW);

	BackProjectHSI( src,BW, cvScalar(COLOR), 40 );
	cvCanny( BW, dst, 50, 200, 3 );					//edge dectection
	
	traceline( dst,BW );								//trace the line algorithm
	cvCopy(BW,dst,NULL);
	cvReleaseImage(&s);
	cvReleaseImage(&BW);

}

int RemoveAllComponentExcept( IplImage *src,IplImage *dst,int val )
{
    int i,j;
    for( i=0;i<src->width;i++ )
        for( j=0;j<src->height;j++ )
        {
            uchar *ptr_src,*ptr_dst;
            ptr_src = (uchar *) src->imageData + j * src->widthStep + i * src->nChannels;
            ptr_dst = (uchar *) dst->imageData + j * dst->widthStep + i * dst->nChannels;
            if( *ptr_src == val )
                *ptr_dst = 255;
            else
                *ptr_dst = 0;
        }

}

int CalcConnectedComponents( IplImage *src, IplImage *dst, int window ) // Calculate the Connected Componenets in an Image
{
	cvZero( dst );
	int curr_component = 1; 
	int p,q;
	int i,j;
	int change = 1;

	cvZero(dst);
	while( change==1 )
	{
            change = 0;

            for( i=0;i<src->width;i++ )
                    for( j=0;j<src->height;j++ )
                    {
                            uchar *ptr_src,*ptr_dst;
                            ptr_src = (uchar *) src->imageData + j * src->widthStep + i * src->nChannels;
                            ptr_dst = (uchar *) dst->imageData + j * dst->widthStep + i * dst->nChannels;

                            if( *ptr_src < 127 )
			    {
				    *ptr_dst = 0;
                                    continue;
			    }

                            for( p=-window/2;p<window/2;p++ )
                            for( q=-window/2;q<window/2;q++ )
                            {
                                    uchar *ptr_curr;
                                    if( (i+p)<0 || (i+p)>src->width || (j+q)<0 || (j+q)>src->height )
                                            continue;
                                    if( p==0 && q==0 )
                                            continue;

                                    ptr_curr = (uchar *) dst->imageData + (j+q) * dst->widthStep + (i+p) * dst->nChannels;
                                    if( *ptr_curr < *ptr_dst && *ptr_curr != 0 )
                                    {
                                            *ptr_dst = *ptr_curr;
                                            change = 1;
                                    }
                                    else if( *ptr_dst==0 && *ptr_curr>0 )
                                    {
                                            *ptr_dst = *ptr_curr;
                                            change = 1;
                                    }
                            }
                            if( *ptr_dst == 0 )
                            {
                                    *ptr_dst = curr_component++;
                                    change = 1;
                            }
                    }
	}
	return(0);
}

void cvShowDiffComponents( IplImage *src, IplImage *dst )  //----------------------------------- This Function Gives Graylevels to Different Componenets from the Map from calcConnectedComp
{
	int levels;
	float multiplier;
	int i,j;

	levels = 0;

	for( i=0;i<src->width;i++ )
	for( j=0;j<src->height;j++ )
	{
		uchar *ptr_src,*ptr_dst;
		ptr_src = (uchar *)src->imageData+j*src->widthStep+i*src->nChannels;
		ptr_dst = (uchar *)dst->imageData+j*dst->widthStep+i*dst->nChannels;
		if( *ptr_dst > levels )
			levels = *ptr_dst;
	}

	if( levels == 0 )
		levels = 255;

	multiplier = 255/levels;
	
	for( i=0;i<src->width;i++ )
	for( j=0;j<src->height;j++ )
	{
		uchar *ptr_src,*ptr_dst;
		ptr_src = (uchar *)src->imageData+j*src->widthStep+i*src->nChannels;
		ptr_dst = (uchar *)dst->imageData+j*dst->widthStep+i*dst->nChannels;
		*ptr_dst = (int)((*ptr_src) * multiplier);
	}
}

structure_COM FindLineinFrontImage(IplImage *src,IplImage *dst )
{
	IplImage *temp,*connected_comp;
	uchar *ptr_src,*ptr_conn;
	int i,j,last_comm=0;

	temp = cvCreateImage( cvGetSize(src),src->depth,src->nChannels );
	connected_comp = cvCreateImage( cvGetSize(src),src->depth,src->nChannels );
	CalcConnectedComponents( src, connected_comp,6 );
	//cvShowDiffComponents( connected_comp,dst );

	for( i=src->width-1; i>=0;i-- )
		for( j=src->height-1; j>= src->height-Front_Line_Seach_Depth;j-- )
		{
			structure_COM COM;
			ptr_src = (uchar *)src->imageData + j * src->widthStep + i * src->nChannels;
			ptr_conn = (uchar *)connected_comp->imageData + j * src->widthStep + i * src->nChannels;
			if( *ptr_src > 127 && (*ptr_conn)!=last_comm )
			{
				RemoveAllComponentExcept( connected_comp,temp,*ptr_conn );
				COM = calcCenterOfMass( temp );
				if( COM.n > PIXEL_FRONT_THRESH )
				{
					cvCopy( temp,dst,NULL );
					return( COM );
				}
			}
			last_comm = *ptr_conn;
		}
	cvReleaseImage(&temp);
	cvReleaseImage(&connected_comp);
}

//********************************************************************************************************************

//****************************************Image Processing Thread Block***********************************************
/*
int IsSizeEqual( CvSize src,CvSize dst )
{
	if( src.width == dst.width && src.height == dst.height )
		return(1);
	else
		return(0);
}*/

DEFINE_THREAD_ROUTINE( IP_BOTTOM_PROC_BLOCK, data )
{
	IplImage *dst = NULL ,*src = NULL;
	CvSize size;
    	uchar* ptr_src ;
	PRINT("STARTING IP Bottom THREAD\n");
    	while( end_all_threads == 0 )
    	{
		#ifdef delay
			double start_time = cvGetTickCount(), t_freq = cvGetTickFrequency(), end_time, tot_delay;
		#endif

		vp_os_mutex_lock( &control_video_lock );
	    	if( g_src_bottom_img==NULL )
	    	{
			vp_os_mutex_unlock( &control_video_lock );
			vp_os_delay(20);
			continue;
	    	}

	    	/*size = cvGetSize(g_src_bottom_img);
	    	if( src == NULL || !IsSizeEqual(cvGetSize(g_src_bottom_img),cvGetSize(src)) )
		{
			if( src!=NULL )
				cvReleaseImage(&src);
			//if( dst!=NULL )
			//	cvReleaseImage(&dst);

	    		src = cvCreateImage( g_imgsize_bottom,g_src_bottom_img->depth,g_src_bottom_img->nChannels );
			dst = cvCreateImage( g_imgsize_bottom,g_src_bottom_img->depth,g_src_bottom_img->nChannels );
		}
	    	if( dst == NULL || !IsSizeEqual(cvGetSize(g_src_bottom_img),cvGetSize(dst)) )
	    		dst = cvCreateImage( g_imgsize_bottom,g_src_bottom_img->depth,1 );*/

		src = cvCreateImage( g_imgsize_bottom,g_src_bottom_img->depth,g_src_bottom_img->nChannels );
		dst = cvCreateImage( g_imgsize_bottom,g_src_bottom_img->depth,1 );

	    	cvCopy( g_src_bottom_img,src,NULL );
	    	vp_os_mutex_unlock( &control_video_lock );

	    	cvZero( dst );
	    	LineFollower(src,dst);

	    	//if( g_dst_img==NULL || !IsSizeEqual(cvGetSize(g_dst_img),cvGetSize(dst)) )
		//	g_dst_img = cvCreateImage( g_imgsize_bottom,dst->depth,dst->nChannels );
	    	//cvCopy(dst,g_dst_img,NULL );
		cvReleaseImage(&src);
    		cvReleaseImage(&dst);
		#ifdef delay
			end_time = cvGetTickCount();
			tot_delay = ((double)end_time - (double)start_time)/ ((float)t_freq);
			tot_delay = tot_delay/1000;
			printf("Image Processing Delay = %f millisecond\n",(float)tot_delay );
		#endif
   	 }
	return (SUCCESS);
}

DEFINE_THREAD_ROUTINE( IP_FRONT_PROC_BLOCK, data )
{
	IplImage *dst = NULL ,*front = NULL,*BW;
	structure_COM fr_com;
	control_data_t l_control;
	while( end_all_threads == 0 )
	{
		#ifdef delay
			double start_time = cvGetTickCount(), t_freq = cvGetTickFrequency(), end_time, tot_delay;
		#endif

		vp_os_mutex_lock( &control_video_lock );
		if( g_src_front_img==NULL )
		{
			vp_os_mutex_unlock( &control_video_lock );
			vp_os_delay(20);
			continue;
		}

		cvZero(dst);
		cvZero(BW);
		front = cvCreateImage( g_imgsize_front,g_src_front_img->depth,g_src_front_img->nChannels );
		dst = cvCreateImage( g_imgsize_front,g_src_front_img->depth,1 );
		BW = cvCreateImage( g_imgsize_front,g_src_front_img->depth,1 );
		cvCopy( g_src_front_img,front,NULL );
		vp_os_mutex_unlock( &control_video_lock );

		MakeCOMZero( &fr_com );
		BackProjectHSI( front,BW, cvScalar(COLOR), 25 );
		cvErode( BW,BW, NULL,2 );
		cvDilate( BW,BW, NULL,2 );
		fr_com = FindLineinFrontImage( BW,dst );

		if( fr_com.n>PIXEL_FRONT_THRESH )
			g_front_lineDetected = 1;
		else
		{
			g_front_lineDetected = 0;
			continue;
		}

		l_control.yaw = (float)(fr_com.x-(QVGA_WIDTH/2))/(QVGA_WIDTH/2);
		vp_os_mutex_lock( &ip_data_lock );
			ip_front_data.yaw = l_control.yaw;
			ip_front_data.timestamp = cvGetTickCount();
		vp_os_mutex_unlock( &ip_data_lock );

		if( g_dst_img==NULL || !IsSizeEqual(cvGetSize(g_dst_img),cvGetSize(dst)) )
			g_dst_img = cvCreateImage( g_imgsize_front,dst->depth,1 );
		cvCopy( dst,g_dst_img,NULL );

		#ifdef delay
			end_time = cvGetTickCount();
			tot_delay = ((double)end_time - (double)start_time)/ ((float)t_freq);
			tot_delay = tot_delay/1000;
			printf("Image Processing Delay = %f millisecond\n",(float)tot_delay );
		#endif
		cvReleaseImage(&front);
		cvReleaseImage(&dst);
		cvReleaseImage(&BW);
	}
	return (SUCCESS);
}

//*********************************************************************************************************************


//***********************************************CONTROL SYSTEM********************************************************

control_data_t control_data_Factorise_sinc( control_data_t src,float factor , float decayConst )
{
	control_data_t dst;
	dst.roll = src.roll/3 * (float)sin( 2 * 3.142 * (float)factor / (float)decayConst )/( (float)factor / decayConst );
	dst.pitch = src.pitch * (float)sin( 2 * 3.142 * (float)factor / (float)decayConst )/( (float)factor / decayConst );
	dst.gaz = src.gaz * (float)sin( 2 * 3.142 * (float)factor / (float)decayConst )/( (float)factor / decayConst );
	dst.yaw = src.yaw * (float)sin( 2 * 3.142 * (float)factor / (float)decayConst )/( (float)factor / decayConst );
	return(dst);
}

void control_data_Factorise_sinc_roll_sinc( control_data_t *src, control_data_t *dst,float factor, float decayConst )
{
	dst->roll = src->roll/4 * (float)sin( 2 * 3.142 * (float)factor / (float)decayConst )/( (float)factor / decayConst );
}

void control_data_Factorise_sinc_yaw_sinc( control_data_t *src, control_data_t *dst,float factor, float decayConst )
{
	dst->yaw = src->yaw/2 * (float)sin( 2 * 3.142 * (float)factor / (float)decayConst )/( (float)factor / decayConst );
}

control_data_t control_data_Factorise_inv( control_data_t src,float factor )
{
	control_data_t dst;
	dst.roll = src.roll/(factor*2);
	dst.pitch = src.pitch/factor;
	dst.gaz = src.gaz/factor;
	dst.yaw = src.yaw/(factor*2);
	return(dst);
}

void control_data_Factorise_sinc_roll_inv( control_data_t *src, control_data_t *dst,float factor )
{
	dst->roll = src->roll/(factor*2);
}

void control_data_Factorise_sinc_yaw_inv( control_data_t *src, control_data_t *dst,float factor )
{
	dst->yaw = src->yaw/(factor*2);
}

void Print_ControlData(control_data_t l_control)
{
	PRINT( "ROLL:%f , PITCH:%f, GAZ:%f, YAW:%f\n\n",l_control.roll,l_control.pitch,l_control.gaz,l_control.yaw );
}

DEFINE_THREAD_ROUTINE(ControlHandler,data)
{
	Q *roll_prev_vals;
	int prev_bottom_line_detected = 0;
	while(end_all_threads == 0)
	{

		#ifdef delay
			double start_time = cvGetTickCount(), t_freq = cvGetTickFrequency(), end_time, tot_delay;
		#endif

		control_data_t l_control,l_ip_bottom_data,l_ip_front_data;
		double curr_time,freq,time1;
		float factor;
		vp_os_mutex_lock( &ip_data_lock );
			l_ip_bottom_data = ip_bottom_data;
			l_ip_front_data = ip_front_data;
		vp_os_mutex_unlock( &ip_data_lock );

		Print_ControlData(ip_bottom_data);
		curr_time = cvGetTickCount();
		freq = cvGetTickFrequency();
		vp_os_delay(10);
		time1 = cvGetTickCount();
		MakeControlZero(&l_control);

		if( g_bottom_lineDetected )
		{
			prev_bottom_line_detected = 1;
			factor = ((float)curr_time - (float)l_ip_bottom_data.timestamp)/ ((float)freq);
			factor = factor / 1000;
			//factor = ((float)curr_time - (float)l_ip_bottom_data.timestamp)/ ((float)DecayConst*freq/1000);
			if( factor > DecayConst )
			{
				control_data_Factorise_sinc_roll_sinc( &l_ip_bottom_data,&l_control,factor,50 );
				control_data_Factorise_sinc_yaw_sinc( &l_ip_bottom_data,&l_control,factor,50 );
				l_control.yaw = l_control.yaw / 5;
			}
			else
			{
				l_control.roll = l_ip_bottom_data.roll/4;
				l_control.yaw = l_ip_bottom_data.yaw/5;
			}
		}
		else
		{
			if( g_Qstart == NULL )
				continue;

			if( prev_bottom_line_detected == 1 )
				roll_prev_vals = g_Qstart;

			prev_bottom_line_detected = 0;
			if( roll_prev_vals == NULL || roll_prev_vals->prev == NULL )
				continue;

			roll_prev_vals = roll_prev_vals->prev;

			if( roll_prev_vals == NULL )
				continue;
			l_control.roll = -1 * roll_prev_vals->control.roll;
		}


		if( g_front_lineDetected )
		{
			factor = ((float)curr_time - (float)l_ip_front_data.timestamp)/ ((float)freq);
			factor = factor / 1000;
			if( factor > DecayConst )
			{
				control_data_Factorise_sinc_yaw_sinc( &l_ip_front_data,&l_control,factor,100 );
			}
			else
				l_control.yaw = l_ip_front_data.yaw;
		}

		if( !g_bottom_lineDetected && !g_front_lineDetected )
		{
			if( g_Qstart != NULL && g_Qstart->prev != NULL )
			{
				control_data_t past_control;
				past_control = QPop( &g_Qstart );
				l_control = calcNegativeControl( past_control );
			}
		}

		//PRINT( "ROLL:%f , PITCH:%f, GAZ:%f, YAW:%f,Factor: %f\n\n",l_control.roll,l_control.pitch,l_control.gaz,l_control.yaw,factor );
		//printf( "\n%d - ",g_bottom_lineDetected  );
		//printControlData( l_control );
		if( g_bottom_lineDetected || g_front_lineDetected )
		{
			l_control.pitch = -0.05;
		}


		vp_os_mutex_lock( &control_data_lock );
			l_control.start = control_data.start;
			control_data = l_control;
		vp_os_mutex_unlock( &control_data_lock );

		MemoryClearRoutine( );

		vp_os_delay(10);
		#ifdef delay
			end_time = cvGetTickCount();
			tot_delay = ((double)end_time - (double)start_time)/ ((float)t_freq);
			tot_delay = tot_delay/1000;
			printf("Control Handler Block = %f millisecond\n",(float)tot_delay );
		#endif

	}
	ardrone_tool_shutdown_custom();
	return (SUCCESS);
}

//*************************************************************************************************************************


//****************************************Communication Thread****************************************************************
//thread for sending the control cmd
DEFINE_THREAD_ROUTINE( comm_control, data )
{
    int seq_no = 0,prev_start = 0;
    control_data_t l_control;

    PRINT( "Initilizing Thread 1\n" );
    while( end_all_threads == 0 )
    {
		#ifdef delay
			double start_time = cvGetTickCount(), t_freq = cvGetTickFrequency(), end_time, tot_delay;
		#endif
		
		vp_os_delay(20);

		vp_os_mutex_lock( &control_data_lock ); //Taking Control Mutex
		l_control = control_data;
		vp_os_mutex_unlock( &control_data_lock );

		if( prev_start == 0 && l_control.start == 1 ) //To Start Drone
		{
			ardrone_tool_set_ui_pad_start( 1 );
			prev_start = 1;
		}
		else if( prev_start == 1 && l_control.start == 0 ) //To Stop the Drone
		{
			ardrone_tool_set_ui_pad_start( 0 );
			prev_start = 0;
		}
		ardrone_at_set_progress_cmd( ++seq_no,l_control.roll,l_control.pitch,l_control.gaz,l_control.yaw); //command to make drone move.

		if( g_bottom_lineDetected || g_front_lineDetected )
			QAdd(&g_Qstart,l_control);
		//PRINT( "\nROLL:%f , PITCH:%f, GAZ:%f, YAW:%f \n\n",l_control.roll,l_control.pitch,l_control.gaz,l_control.yaw );
		
		#ifdef delay
			end_time = cvGetTickCount();
			tot_delay = ((double)end_time - (double)start_time)/ ((double)t_freq);
			tot_delay = tot_delay/1000;
			printf("Communication Block = %f millisecond\n",(float)tot_delay );
		#endif
    }
    PRINT( "Communication Thread Ending\n" );
    return C_OK;
}
/* Implementing Custom methods for the main function of an ARDrone application */
//*************************************************************************************************************************




//******************************************MAIN Funtion******************************************************

/* The delegate object calls this method during initialization of an ARDrone application */
C_RESULT ardrone_tool_init_custom(int argc, char **argv)
{
  /* Registering for a new device of game controller */
 // ardrone_tool_input_add( &gamepad );

  /* Start all threads of your application */
  g_src_bottom_img = NULL;
  g_dst_img = NULL;
  g_Qstart = QInit();
  g_Qend = g_Qstart;
  end_all_threads = 0;
  
  START_THREAD( video_stage, NULL );
  START_THREAD( IP_BOTTOM_PROC_BLOCK,NULL);
  START_THREAD( IP_FRONT_PROC_BLOCK,NULL);
  START_THREAD( comm_control,NULL);
  START_THREAD( GUIThread, NULL );
  START_THREAD( ControlHandler, NULL );
  //START_THREAD( navdata_update, NULL );
    
  return C_OK;
}

/* The delegate object calls this method when the event loop exit */
C_RESULT ardrone_tool_shutdown_custom()
{
  /* Relinquish all threads of your application */
 // JOIN_THREAD( video_stage );

  /* Unregistering for the current device */
  //ardrone_tool_input_remove( &gamepad );
  PRINT("EXIT SEQUENCE STARTING\n");
  JOIN_THREAD( video_stage );
  JOIN_THREAD( IP_BOTTOM_PROC_BLOCK );
  JOIN_THREAD( IP_FRONT_PROC_BLOCK );
  JOIN_THREAD( comm_control);
  JOIN_THREAD( GUIThread );
  JOIN_THREAD( ControlHandler );
  JOIN_THREAD( navdata_update );
  JOIN_THREAD( ardrone_control );
  ardrone_tool_set_ui_pad_start(0);

  return C_OK;
}

/* The event loop calls this method for the exit condition */
bool_t ardrone_tool_exit()
{
  return exit_ihm_program == 0;
}

C_RESULT signal_exit()
{
  exit_ihm_program = 0;

  return C_OK;
}

/* Implementing thread table in which you add routines of your application and those provided by the SDK */
BEGIN_THREAD_TABLE
  THREAD_TABLE_ENTRY( ardrone_control, 20 )
  THREAD_TABLE_ENTRY( navdata_update, 20 )
  THREAD_TABLE_ENTRY( video_stage, 20 )
  THREAD_TABLE_ENTRY( IP_BOTTOM_PROC_BLOCK, 20 )
  THREAD_TABLE_ENTRY( IP_FRONT_PROC_BLOCK, 20 )
  THREAD_TABLE_ENTRY( comm_control, 20 )
  THREAD_TABLE_ENTRY( GUIThread, 20 )
  THREAD_TABLE_ENTRY( ControlHandler, 20 )
END_THREAD_TABLE

/*
PROTO_THREAD_ROUTINE(thread1,nomParams)
{
  PRINT("Thread 1 Start\n");

  while(1)
    {
      vp_os_delay(100);
      PRINT("Thread 1 Loop\n");
    }
}*/





