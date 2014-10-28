#include <BSystem.h>
#include <BCamera.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/background_segm.hpp>
#include <cmath>

using namespace cv;

baumer::BCamera* g_cam = 0;
baumer::BSystem* g_system  = 0;

int width = 696;
int height = 524;
char key;
int n_boards = 0;
const int board_dt = 20;
int board_w;
int board_h;

// callbacks
void init_camera();
IplImage* open_stream(int width, int height);

int main() {

	// initialize baumer camera
	init_camera();

	board_w = 7; // Board width in squares
	board_h = 10; // Board height 
	n_boards = 15; // Number of boards
	int board_n = board_w * board_h;
	CvSize board_sz = cvSize( board_w, board_h );

	cvNamedWindow( "Calibration" );
	// Allocate Sotrage
	CvMat* image_points		= cvCreateMat( n_boards*board_n, 2, CV_32FC1 );
	CvMat* object_points		= cvCreateMat( n_boards*board_n, 3, CV_32FC1 );
	CvMat* point_counts			= cvCreateMat( n_boards, 1, CV_32SC1 );
	CvMat* intrinsic_matrix		= cvCreateMat( 3, 3, CV_32FC1 );
	CvMat* distortion_coeffs	= cvCreateMat( 5, 1, CV_32FC1 );

	CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
	int corner_count;
	int successes = 0;
	int step, frame = 0;


	IplImage *image = open_stream(width, height);
	//IplImage *gray_image = cvCreateImage( cvSize( 1392,1044 ), 8, 1 );

	/*
	while( successes < n_boards ){
		// Skp every board_dt frames to allow user to move chessboard
		if( frame++ % board_dt == 0 ){
			// Find chessboard corners:
			int found = cvFindChessboardCorners( image, board_sz, corners,
				&corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );

			// Get subpixel accuracy on those corners
			//cvCvtColor( image, gray_image, CV_BGR2GRAY );
			cvFindCornerSubPix( image, corners, corner_count, cvSize( 11, 11 ), 
				cvSize( -1, -1 ), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			// Draw it
			cvDrawChessboardCorners( image, board_sz, corners, corner_count, found );
			cvShowImage( "Calibration", image );

			// If we got a good board, add it to our data
			if( corner_count == board_n ){
				step = successes*board_n;
				for( int i=step, j=0; j < board_n; ++i, ++j ){
					CV_MAT_ELEM( *image_points, float, i, 0 ) = corners[j].x;
					CV_MAT_ELEM( *image_points, float, i, 1 ) = corners[j].y;
					CV_MAT_ELEM( *object_points, float, i, 0 ) = j/board_w;
					CV_MAT_ELEM( *object_points, float, i, 1 ) = j%board_w;
					CV_MAT_ELEM( *object_points, float, i, 2 ) = 0.0f;
				}
				CV_MAT_ELEM( *point_counts, int, successes, 0 ) = board_n;
				successes++;
			}
		} 

		// Handle pause/unpause and ESC
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
		if( c == 27 )
			return 0;
		image = open_stream(width, height); // Get next image
	} // End collection while loop

	std::cout << "success!!!!!!" << std::endl;
	
	// Allocate matrices according to how many chessboards found
	CvMat* object_points2 = cvCreateMat( successes*board_n, 3, CV_32FC1 );
	CvMat* image_points2 = cvCreateMat( successes*board_n, 2, CV_32FC1 );
	CvMat* point_counts2 = cvCreateMat( successes, 1, CV_32SC1 );
	
	// Transfer the points into the correct size matrices
	for( int i = 0; i < successes*board_n; ++i ){
		CV_MAT_ELEM( *image_points2, float, i, 0) = CV_MAT_ELEM( *image_points, float, i, 0 );
		CV_MAT_ELEM( *image_points2, float, i, 1) = CV_MAT_ELEM( *image_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 0) = CV_MAT_ELEM( *object_points, float, i, 0 );
		CV_MAT_ELEM( *object_points2, float, i, 1) = CV_MAT_ELEM( *object_points, float, i, 1 );
		CV_MAT_ELEM( *object_points2, float, i, 2) = CV_MAT_ELEM( *object_points, float, i, 2 );
	}

	for( int i=0; i < successes; ++i ){
		CV_MAT_ELEM( *point_counts2, int, i, 0 ) = CV_MAT_ELEM( *point_counts, int, i, 0 );
	}
	cvReleaseMat( &object_points );
	cvReleaseMat( &image_points );
	cvReleaseMat( &point_counts );

	// At this point we have all the chessboard corners we need
	// Initiliazie the intrinsic matrix such that the two focal lengths
	// have a ratio of 1.0

	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0;

	// Calibrate the camera
	cvCalibrateCamera2( object_points2, image_points2, point_counts2, cvGetSize( image ), 
		intrinsic_matrix, distortion_coeffs, NULL, NULL, CV_CALIB_FIX_ASPECT_RATIO ); 

	// Save the intrinsics and distortions
	//cvSave( "Intrinsics.xml", intrinsic_matrix );
	//cvSave( "Distortion.xml", distortion_coeffs );
	*/
	// Example of loading these matrices back in
	CvMat *intrinsic1 = (CvMat*)cvLoad( "data/kalibrate_inside/Intrinsics.xml" );
	CvMat *distortion1 = (CvMat*)cvLoad( "data/kalibrate_inside/Distortion.xml" );
	
	CvMat *intrinsic2 = (CvMat*)cvLoad( "data/kalibrate_outside/Intrinsics.xml" );
	CvMat *distortion2 = (CvMat*)cvLoad( "data/kalibrate_outside/Distortion.xml" );

	// Build the undistort map that we will use for all subsequent frames
	IplImage* mapx1 = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	IplImage* mapy1 = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic1, distortion1, mapx1, mapy1 );

	IplImage* mapx2 = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	IplImage* mapy2 = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic2, distortion2, mapx2, mapy2 );

	// Run the camera to the screen, now showing the raw and undistorted image
	cvNamedWindow( "Undistort_inside" );
	cvNamedWindow( "Undistort_outside" );

	while( image ){
		IplImage *t1 = cvCloneImage( image );
		IplImage *t2 = cvCloneImage( image );
		cvShowImage( "Calibration", image ); // Show raw image
		cvRemap( t1, image, mapx1, mapy1 ); // undistort image
		cvReleaseImage( &t1 );
		cvShowImage( "Undistort_inside", image ); // Show corrected image
		cvRemap( t2, image, mapx2, mapy2 ); // undistort image
		cvReleaseImage( &t2 );
		cvShowImage( "Undistort_outside", image ); // Show corrected image

		// Handle pause/unpause and esc
		int c = cvWaitKey( 15 );
		if( c == 'p' ){
			c = 0;
			while( c != 'p' && c != 27 ){
				c = cvWaitKey( 250 );
			}
		}
		if( c == 27 )
			break;
		image = open_stream(width, height);
	}

	return 0;

}

void init_camera() {
	g_system = new baumer::BSystem;
	g_system->init();
	if(g_system->getNumCameras() == 0) {
		std::cerr << "Error: no camera found" << std::endl;
		return;
	}

	g_cam = g_system->getCamera(0 /*the first camera*/, false /*use not rgb*/, true /*not fastest mode but full resolution*/);
	
	width = g_cam->getWidth();
	height = g_cam->getHeight();
}

IplImage* open_stream(int width, int height) {
	while (g_cam->capture() == NULL) 
	{
		std::cout << "cam not initialized yet" << std::endl;
		key = cvWaitKey(10);
	}

	if (!g_cam->capture() == NULL) {
		if (g_cam->capture() == NULL) {
				std::cout << "Error: stream is empty" << std::endl;
			
		}
		else {
			// save the data stream in each frame
			IplImage* frame = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 1);
			frame -> imageData = (char*) g_cam-> capture();

			// show stream
			//cvShowImage("Stream", frame);

	        key = cvWaitKey(100); // throws a segmentation fault (?)
	        return frame;
		}		
	}
	
	return cvCreateImage( cvSize( 696, 524 ), 8, 1 ); 
}
