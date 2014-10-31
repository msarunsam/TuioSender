#include "bgapi2_genicam.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/background_segm.hpp>
#include <cmath>

using namespace cv;

char key;
int n_boards = 0;
const int board_dt = 20;
int board_w;
int board_h;


int 					m_camWidth;
int 					m_camHeight;

CvSize 					m_imageSize(cvSize(696, 520));
IplImage* 				m_frame(cvCreateImage(m_imageSize, IPL_DEPTH_8U, 1));

BGAPI2::SystemList* 	m_systemList(NULL);
BGAPI2::System* 		m_systemMem(NULL);
BGAPI2::String 			m_systemID("");

BGAPI2::InterfaceList*	m_interfaceList(NULL);
BGAPI2::Interface* 		m_interface(NULL);
BGAPI2::String			m_interfaceID("");

BGAPI2::DeviceList*		m_deviceList(NULL);
BGAPI2::Device*			m_device(NULL);
BGAPI2::String			m_deviceID("");

BGAPI2::DataStreamList*	m_datastreamList(NULL);
BGAPI2::DataStream*		m_datastream(NULL);
BGAPI2::String			m_datastreamID("");

BGAPI2::BufferList*		m_bufferList(NULL);
BGAPI2::Buffer*			m_buffer(NULL);


// callbacks
void init_camera();
IplImage* open_stream();
void distortion();
void illuminationCorrection();
void illuminationCorrection2();
void illuminationCorrection3();
void perspectiveCorrection();

int main() {

	// initialize baumer camera
	init_camera();


	

	distortion();
	//illuminationCorrection3();
	//perspectiveCorrection();
}

void perspectiveCorrection() {
	/*
	Mat src=imread("image_work.png");
	Mat thr;
	cvtColor(src,thr,CV_BGR2GRAY);
	threshold( thr, thr, 70, 255,CV_THRESH_BINARY );

	vector< vector <Point> > contours; // Vector for storing contour
	vector< Vec4i > hierarchy;
	int largest_contour_index=0;
	int largest_area=0;

	Mat dst(src.rows,src.cols,CV_8UC1,Scalar::all(0)); //create destination image
	findContours( thr.clone(), contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE ); // Find the contours in the image
	for( int i = 0; i< contours.size(); i++ ){
		double a=contourArea( contours[i],false);  //  Find the area of contour
		if(a>largest_area){
		largest_area=a;
	    largest_contour_index=i;                //Store the index of largest contour
	    }
	}

	drawContours( dst,contours, largest_contour_index, Scalar(255,255,255),CV_FILLED, 8, hierarchy );
	vector<vector<Point> > contours_poly(1);
	approxPolyDP( Mat(contours[largest_contour_index]), contours_poly[0],100, true );
	Rect boundRect=boundingRect(contours[largest_contour_index]);
	
	if(contours_poly[0].size()==4){
	    std::vector<Point2f> quad_pts;
	    std::vector<Point2f> squre_pts;
	    quad_pts.push_back(Point2f(contours_poly[0][0].x,contours_poly[0][0].y));
	    quad_pts.push_back(Point2f(contours_poly[0][1].x,contours_poly[0][1].y));
	    quad_pts.push_back(Point2f(contours_poly[0][3].x,contours_poly[0][3].y));
	    quad_pts.push_back(Point2f(contours_poly[0][2].x,contours_poly[0][2].y));
	    squre_pts.push_back(Point2f(boundRect.x,boundRect.y));
	    squre_pts.push_back(Point2f(boundRect.x,boundRect.y+boundRect.height));
	    squre_pts.push_back(Point2f(boundRect.x+boundRect.width,boundRect.y));
	    squre_pts.push_back(Point2f(boundRect.x+boundRect.width,boundRect.y+boundRect.height));
		
		Mat transformed = Mat::zeros(src.rows, src.cols, CV_8UC3);
	    Mat transmtx = getPerspectiveTransform(quad_pts,squre_pts);

	    //imwrite("data/perspectiveCorrection.png", transmtx);
	    Mat perspectiveCorrection;

	    cv::FileStorage storage1("data/perspectiveCorrection.yml", cv::FileStorage::WRITE);
	    storage1 << "transmtx" << transmtx;
	    storage1.release();  
	} else
	{
    	cout << "Make sure that your are getting 4 corner using approxPolyDP..." << endl;	
	}
	*/
	Mat src=imread("image_work.png");
    Mat transformed = Mat::zeros(src.rows, src.cols, CV_8UC3);
	Mat perspectiveCorrection;

    //Reading from file
    cv::FileStorage storage2("data/perspectiveCorrection.yml", cv::FileStorage::READ);
    storage2["transmtx"] >> perspectiveCorrection;
    storage2.release();

    warpPerspective(src, transformed, perspectiveCorrection, src.size());
      
    imshow("quadrilateral", transformed);
    
    waitKey();

}

void distortion()
{
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


	IplImage *image = open_stream();
	//IplImage *gray_image = cvCreateImage( cvSize( 1392,1044 ), 8, 1 );

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
		{
			std::cout << "ERROR "<< std::endl;
			break;
		}
		image = open_stream(); // Get next image
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
	cvSave( "Intrinsics.xml", intrinsic_matrix );
	cvSave( "Distortion.xml", distortion_coeffs );
	

	// Example of loading these matrices back in
	CvMat *intrinsic = (CvMat*)cvLoad( "Intrinsics.xml" );
	CvMat *distortion = (CvMat*)cvLoad( "Distortion.xml" );
	
	// Build the undistort map that we will use for all subsequent frames
	IplImage* mapx = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	IplImage* mapy = cvCreateImage( cvGetSize( image ), IPL_DEPTH_32F, 1 );
	cvInitUndistortMap( intrinsic, distortion, mapx, mapy );

	// Run the camera to the screen, now showing the raw and undistorted image
	cvNamedWindow( "Undistort" );

	while( image ){
		IplImage *t1 = cvCloneImage( image );
		IplImage *imageTemp = cvCloneImage( image );
		cvShowImage( "Calibration", image ); // Show raw image
		
		cvRemap( t1, image, mapx, mapy ); // undistort image
		cvReleaseImage( &t1 );
		cvShowImage( "Undistort", image ); // Show corrected image
		if (char(key) == 32) { // Space saves the current image
			std::cout << "pflaggi" << std::endl;
			cvSaveImage("undistort.png", image);
			cvSaveImage("distort.png", imageTemp);
		}
		cvReleaseImage( &imageTemp );

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
		image = open_stream();
	}
}

void illuminationCorrection2(){

	Mat img = imread("image.png", CV_LOAD_IMAGE_COLOR); //open and read the image


	if (img.empty())
	{
	std::cout << "Image cannot be loaded..!!" << std::endl;
	
	}

	Mat imgH;
	img.convertTo(imgH, -1, 3, 0); //increase the contrast (double)

	Mat imgL;
	img.convertTo(imgL, -1, 0.5, 0); //decrease the contrast (halve)

	//create windows
	namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
	namedWindow("High Contrast", CV_WINDOW_AUTOSIZE);
	namedWindow("Low Contrast", CV_WINDOW_AUTOSIZE);

	//show the image
	imshow("Original Image", img);
	imshow("High Contrast", imgH);
	imshow("Low Contrast", imgL);

	waitKey(0); //wait for key press

	destroyAllWindows(); //destroy all open windows
}

void illuminationCorrection3(){

	Mat img = imread("image.png", CV_LOAD_IMAGE_COLOR); //open and read the image


	if (img.empty())
	{
	std::cout << "Image cannot be loaded..!!" << std::endl;
	
	}
	cvtColor(img, img, CV_BGR2GRAY); //change the color image to grayscale image

	Mat img_hist_equalized;
	equalizeHist(img, img_hist_equalized); //equalize the histogram

	//create windows
	namedWindow("Original Image", CV_WINDOW_AUTOSIZE);
	namedWindow("Histogram Equalized", CV_WINDOW_AUTOSIZE);

	//show the image
	imshow("Original Image", img);
	imshow("Histogram Equalized", img_hist_equalized);

	waitKey(0); //wait for key press

	destroyAllWindows(); //destroy all open windows

}

void illuminationCorrection() {
	// READ RGB color image and convert it to Lab
    cv::Mat bgr_image = cv::imread("image.png");
    cv::Mat lab_image;
    cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

   // convert back to RGB
   cv::Mat image_clahe;
   cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

   // display the results  (you might also want to see lab_planes[0] before and after).
   cv::imshow("image original", bgr_image);
   cv::imshow("image CLAHE", image_clahe);
   cv::waitKey();




}

void init_camera() {
	// SYSTEM  
    m_systemList = BGAPI2::SystemList::GetInstance();
    m_systemList->Refresh();
    std::cout << "Detected systems: " << m_systemList->size() << std::endl;

    m_systemList->begin()->second->Open();
    m_systemID = m_systemList->begin()->first;
    if(m_systemID == "") {
        std::cout << "Error: no system found" << std::endl;
    }
    else {
        m_systemMem = (*m_systemList)[m_systemID];
        std::cout << "SystemID:  " << m_systemID << std::endl;
    }

    //INTERFACE
    m_interfaceList = m_systemMem->GetInterfaces();
    m_interfaceList->Refresh(100);
    std::cout << "Detected interfaces: " << m_interfaceList->size() << std::endl;

    for (BGAPI2::InterfaceList::iterator interfaceIter = m_interfaceList->begin(); interfaceIter != m_interfaceList->end(); interfaceIter++) {
        interfaceIter->second->Open();
        m_deviceList = interfaceIter->second->GetDevices();
        m_deviceList->Refresh(100);

        if (m_deviceList->size() > 0) {
            std::cout << "Detected Devices: " << m_deviceList->size() << std::endl;
            m_interfaceID = interfaceIter->first;
            m_interface = interfaceIter->second;
            break;
        }
        else {
            interfaceIter->second->Close();
        }
    }

    // DEVICE
    m_device  = m_deviceList->begin()->second;
    m_device->Open();
    m_deviceID = m_deviceList->begin()->first;
    if(m_deviceID == "") {
        std::cout << "Error: no camera found" << std::endl;
    }
    else {
        m_device = (*m_deviceList)[m_deviceID];
        std::cout << "DeviceID: " << m_deviceID << std::endl;
    }

    // CONFIG
    //Set camera features based on config file


    m_device->GetRemoteNode("Gain")->SetDouble(15.56);
    m_device->GetRemoteNode("TriggerMode")->SetString("On");
    m_device->GetRemoteNode("TriggerSource")->SetValue("Line0");
    m_device->GetRemoteNode("ExposureTime")->SetDouble(13000);

    //Set cam resolution to halve reolution [696, 520]
    //std::cout << "1 " << m_device->GetRemoteNode("TriggerSource")->GetDescription()  << std::endl;
    //std::cout << "2 " << m_device->GetRemoteNode("TriggerSource")->GetInterface()  << std::endl;
    m_device->GetRemoteNode("BinningHorizontal")->SetInt( 2);
    m_device->GetRemoteNode("BinningVertical")->SetInt( 2);

    // GET CAM RESOLUTION
    m_camWidth = m_device->GetRemoteNode("Width")->GetInt();
    m_camHeight = m_device->GetRemoteNode("Height")->GetInt();
    std::cout << "Cam resolution : " << m_camWidth << "  " <<  m_camHeight << std::endl;


    // DATASTREAM
    m_datastreamList = m_device->GetDataStreams();
    m_datastreamList->Refresh();
    std::cout << "Detected datastreams: " << m_datastreamList->size() << std::endl;

    m_datastreamList->begin()->second->Open();
    m_datastreamID = m_datastreamList->begin()->first;
    if(m_datastreamID == "") {
        std::cout << "Error: no datastream found" << std::endl;
    }
    else{
        m_datastream = (*m_datastreamList)[m_datastreamID];
        std::cout << "DatastreamID: " << m_datastreamID << std::endl;
    }

    // BUFFER
    m_bufferList = m_datastream->GetBufferList();
    for(int i=0; i<(4); i++) { // 4 buffers using internal buffers
         m_buffer = new BGAPI2::Buffer();
         m_bufferList->Add(m_buffer);
    }
    std::cout << "Announced buffers: " << m_bufferList->size() << std::endl;
    for (BGAPI2::BufferList::iterator buf = m_bufferList->begin(); buf != m_bufferList->end(); buf++) {
         buf->second->QueueBuffer();
    }
    std::cout << "Queued buffers: " << m_bufferList->GetQueuedCount() << std::endl;

    // START DATASTREAM AND FILL BUFFER
    m_datastream->StartAcquisitionContinuous();
    m_device->GetRemoteNode("AcquisitionStart")->Execute();

}

IplImage* open_stream() {
	char* img = nullptr;

    BGAPI2::Buffer* m_bufferFilled = NULL;
    m_bufferFilled = m_datastream->GetFilledBuffer(1000);
    if(m_bufferFilled == NULL){
        std::cout << "Error: buffer timeout" << std::endl;
    }

    img = (char*)m_bufferFilled->GetMemPtr();

    m_bufferFilled->QueueBuffer();

    IplImage* frameTemp = cvCreateImageHeader(cvSize(m_camWidth, m_camHeight), IPL_DEPTH_8U, 1);
    cvSetData(frameTemp, img, m_camWidth);

    cvCopy(frameTemp, m_frame, NULL);
    cvReleaseImageHeader(&frameTemp);

    cvShowImage("Stream", m_frame);
	key = cvWaitKey(100);
    return m_frame;
}
