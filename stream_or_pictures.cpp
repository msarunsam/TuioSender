/*
#include <BSystem.h>
#include <BCamera.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <cmath>

#include "TUIOSender.h"

using namespace cv;

const int CROP[] = {250, 200, 930, 600};

int SYSTEM_INPUT = 1; // 0 uses a picture; change it to 1 to use the stream
int ALGORITHM = 1; // 0 uses only contours; change it to 1 to use MSER or to 2 to use both

baumer::BCamera* g_cam = 0;
baumer::BSystem* g_system  = 0;

int width = 640;
int height = 480;

Mat frameMat, back, maxI, max_intens, maskI, maskI2, maskMinI, img0, ellipses, temp, gr, b1, b2, temp2, temp3, minI, previous, roi;
int calc_every_second_frame = 0;
char key;
Ptr<BackgroundSubtractor> pMOG;
int skip_first_frame = 0;
std::vector<RotatedRect> min_rect;
bool set_min_rect = true;
std::vector<std::vector<Point> > min_r;
RotatedRect min_box, max_rect;
float prev_x, prev_y;
int i;


int largest_area=0;
int largest_contour_index=0;
Rect bounding_rect;



// mser values
int _delta=1; 				// default: 1; 			good: 1 						[1; infinity]
int _min_area=4; //600			// default: 60; 		good: 60 						[1; infinity]
int _max_area=400; //200000		// default: 14 400; 	good: 20 000 for fingertips		[1; infinity]
double _max_variation=.05; 	// default: 0.25;		good: 0.03 - 0.05				[0; 1]
double _min_diversity=.8;	// default: 0.2;		good: 0.5 - 0.7					[0; 1]

//used only with colored images
int _max_evolution=200;
double _area_threshold=0.5;
double _min_margin=0.003; 
int _edge_blur_size=5;

// callbacks
void cleanup();
void init_camera();
void open_stream(int width, int height, Ptr<BackgroundSubtractor> pMOG);
void get_contours(Mat img_cont);
void mser_algo(Mat temp_img);
void draw_ellipses(vector<vector<Point> > contours, Mat ellipses, Mat img0);
void get_min_box(vector<Point> r, RotatedRect box);
bool is_bigger(RotatedRect box_1, RotatedRect box_2);


//////// TUIOSENDER //////
TUIOSender sender("127.0.0.1", 3333);
int interpolation=INTER_LINEAR;

int main() {

	/////////// STREAM //////////
	if (SYSTEM_INPUT == 1) {
		//namedWindow("Stream", CV_WINDOW_AUTOSIZE);
		//namedWindow("Ellipses", CV_WINDOW_AUTOSIZE );

		pMOG = new BackgroundSubtractorMOG();

		// initialize baumer camera
		init_camera();

		// infinte loop for the stream
		open_stream(width, height, pMOG);
	}
	
	////////// PICTURE //////////
	else if (SYSTEM_INPUT == 0) {
		//namedWindow("Contour", CV_WINDOW_AUTOSIZE);

		// load image
		Mat img;
		img = imread("binary.png", CV_LOAD_IMAGE_COLOR);
		if (!img.data) {
			std::cout << "Error: couldn't load image" << std::endl;
			return 0;
		}

		if (ALGORITHM == 0) {
		    get_contours(img);
		}
		else if (ALGORITHM == 1) {
			mser_algo(img);
		}
		else if (ALGORITHM == 2) {
			get_contours(img);
			mser_algo(img);
		}
		else {
			std::cout << "Error: invalide algorithm number" << std::endl;
		}

		// wait until any key is pressed
		cvWaitKey(0);
	}
	else {
		std::cout << "Error: invalide system input number" << std::endl;
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

	g_cam = g_system->getCamera(0 , false , false );
	
	width = g_cam->getWidth();
	height = g_cam->getHeight();
}

void open_stream(int width, int height, Ptr<BackgroundSubtractor> pMOG) {
	Mat thr;
	Mat dst;

	while (true) {
		if (!g_cam->capture() == NULL) {
			if (g_cam->capture() == NULL) {
				std::cout << "Error: stream is empty" << std::endl;
			}
			else {
				// save the data stream in each frame
				IplImage* frame = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 1);
				frame -> imageData = (char*) g_cam-> capture();

///////////////////////
				// SET VIEW OF INTEREST
				cvSetImageROI(frame, cvRect(CROP[0], CROP[1], CROP[2], CROP[3]));
				frameMat = Mat(frame);
				//resize(frameMat, frameMat, Size(10,20), 0, 0, interpolation);
///////////////////////
				
				flip(frameMat, frameMat, -1);


				// show stream
				//cvShowImage("Stream", frame);
   
		        if (char(key) == 27) { // ESC breaks the loop
		            break;      
		        }
		        else if (char(key) == 32) { // Space saves the current image
		        	//cvSaveImage("current.png", frame);
		        	cvSaveImage("min.png", frame);
		        	minI = imread("min.png", CV_LOAD_IMAGE_GRAYSCALE);
		        	//maskI = imread("mask4.bmp", CV_LOAD_IMAGE_GRAYSCALE);
		        	//bitwise_not(maskI, maskI);

		        }
		        else if (char(key) == 10) { // Enter takes an image of the background
		        	//maxI = imread("background.png", CV_LOAD_IMAGE_GRAYSCALE);
		        	threshold(maxI, maskI, 25, 255, CV_THRESH_BINARY); //65
		        	//pMOG->operator()(background, MaskMOG);
		        	
		        }
				else if (char(key) == 112) {
					cvSaveImage("background.png", frame);
				}
		        else if(char(key) ==  49 ) {
		        	++_delta;
		        }
		        else if(char(key) ==  33 ) {
		        	--_delta;
		        }
		        else if(char(key) ==  50 ) {
		        	_min_area += 100;
		        }
		        else if(char(key) ==  34 ) {
		        	_min_area -= 100;
		        }
		        else if(char(key) ==  51 ) {
		        	_max_area += 100;
		        }
		        else if(char(key) ==  35 ) {
		        	_max_area -= 100;
		        }
		        else if(char(key) ==  52 ) {
		        	_max_variation += 0.2;
		        }
		        else if(char(key) ==  36 ) {
		        	_max_variation -= 0.2;
		        }
		        else if(char(key) ==  53 ) {
		        	_max_variation += 0.1;
		        }
		        else if(char(key) ==  37 ) {
		        	_max_variation -= 0.1;
		        }
		        else if(char(key) ==  54 ) {
		        	g_cam->setGain(1.05 * g_cam->getGain());
		        }
		        else if(char(key) ==  38 ) {
		        	g_cam->setGain(0.95 * g_cam->getGain());
		        }
		        else if (char(key) == 43) {
		        	max_intens = Mat(frame);
		        }
		        
		        Mat black(frameMat.rows, frameMat.cols, frameMat.type(), Scalar::all(0));
		        //black.copyTo(frameMat, maskI);

		        if(maxI.data && minI.data) {
		        	subtract(Mat(frame), minI, temp2, noArray(), -1);
			    	subtract(maxI, minI, temp3, noArray(), -1);
			    	frameMat = ( temp2 / temp3) * pow(2, 8);
			    }
			    else {
			    	frameMat = Mat(frame);
			    }

			    if (ALGORITHM == 0) {
			    	get_contours(frameMat);
			    }
				else if (ALGORITHM == 1) {
					mser_algo(frameMat);
				}
				else if (ALGORITHM == 2) {
					get_contours(frameMat);
					mser_algo(frameMat);
				}
				else if (ALGORITHM == 3)
				{
					//blob_detection(frameMat)
				}
				else {
					std::cout << "Error: invalide algorithm number" << std::endl;
				}

		        key = cvWaitKey(10); // throws a segmentation fault (?)
	    	}
		}		
	}
}

void blob_detection(Mat img)
{
	/*
	// set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 50.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 20.0f;
	params.maxArea = 500.0f;
	// ... any other params you don't want default value

	// set up and create the detector using the parameters
	cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
	blob_detector->create("SimpleBlob");

	// detect!
	vector<cv::KeyPoint> keypoints;
	blob_detector->detect(image, keypoints);

	// extract the x y coordinates of the keypoints: 

	for (int i=0; i<keypoints.size(); i++){
	    float X=keypoints[i].pt.x; 
	    float Y=keypoints[i].pt.y;
	}
	

}

void get_contours(Mat img_cont) {
	
	Mat img2;
	
	if (SYSTEM_INPUT == 0) {
		cvtColor(img_cont, img_cont, CV_BGR2GRAY);
		threshold(img_cont, img2, 65, 255, CV_THRESH_BINARY); //65
	}
	else {
		threshold(img_cont, img2, 20, 255, CV_THRESH_BINARY); // smaller threshold possible because of the background substraction
	}

	// detect contours
	vector<vector<Point> > contours;
	vector<vector<Point> > contours0;

	findContours( img2, contours0, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE );

	contours.resize(contours0.size());
	for( size_t k = 0; k < contours0.size(); k++ )
		approxPolyDP(Mat(contours0[k]), contours[k], 3, true);

	Mat cnt_img = Mat::zeros(img2.rows, img2.cols, CV_8UC3);

	if (SYSTEM_INPUT == 0) {
		img_cont.copyTo(cnt_img); // when it is used with the stream. it shows the same image as the mser algorithm
	}

	drawContours(cnt_img, contours, -1, Scalar(128,255,255));

	imshow("Contour", cnt_img);
}

void mser_algo(Mat temp_img)  {
	vector<vector<Point> > contours;

	sender.initFrame();
	//sender.cleanCursors();

	if (SYSTEM_INPUT == 0) {
		cvtColor(temp_img, temp_img, CV_BGR2GRAY);
	}

	temp_img.copyTo(ellipses);

	MSER ms(_delta, _min_area, _max_area, _max_variation, _min_diversity, _max_evolution, _area_threshold, _min_margin, _edge_blur_size);
	ms(temp_img, contours, Mat());

	cvtColor(ellipses, ellipses, CV_GRAY2BGR);
	
	if (SYSTEM_INPUT == 0) {
		draw_ellipses(contours, ellipses, img0);
		
	}
	else {
		if (skip_first_frame == 1) {
			draw_ellipses(contours, ellipses, img0);
		}

		if(skip_first_frame == 0) {
			++skip_first_frame;
		}
	}

	//show contours
	Mat cnt_img = Mat::zeros(temp_img.rows, temp_img.cols, CV_8UC3);
	drawContours(cnt_img, contours, -1, Scalar(128,255,255));
	//imshow("Contour", cnt_img);

	//++test1;
	//std::cout << "Frame: " << test1 << std::endl;
	sender.updateCursors();
	sender.drawEllipses(ellipses, CROP[2], CROP[3]);
	sender.sendCursors();
	imshow( "Ellipses", ellipses);
}

void draw_ellipses(vector<vector<Point> > contours, Mat ellipses, Mat img0) {
	std::vector<Point> r;
	std::vector<RotatedRect> boxes;
	min_r.clear();
	min_rect.clear();

	for( int i = (int)contours.size()-1; i >= 0; i-- ) {	// for each contour get on box
		r = contours[i];
		boxes.push_back(fitEllipse( r ));
	}

	sort(boxes.begin(), boxes.end(), is_bigger);	// sort from smallest to biggest box
	
	if (!boxes.empty()) {
		for (std::vector<RotatedRect>::iterator i = boxes.begin(); i != boxes.end() ; ++i) {	//get only the smallest touchpoints
			get_min_box(r, *i);
			ellipse( ellipses, *i, Scalar(125,125,0), 2 ); // color ellipses
		}
	}

	if (!min_rect.empty() || !min_r.empty()) {
		if (SYSTEM_INPUT == 1) {
			for (auto const& i : boxes) {
				//min_box = fitEllipse( min_r[i] );
				double scaled_touch_x = (i.center.x / CROP[2]) ;
				float scaled_touch_y = (i.center.y / CROP[3]) ;

				sender.addTuioCursor(scaled_touch_x, scaled_touch_y);

				/*
				if (std::fabs(scaled_touch_x - prev_x) > 0.0001) {
					scaled_touch_x = prev_x;
				}
				if (std::fabs(scaled_touch_y - prev_y) > 0.0001) {
					scaled_touch_y = prev_y;
				}*/
				//prev_x = scaled_touch_x;
				//prev_y = scaled_touch_y;
				//std::cout << "Touchpoint " << i << ": " << "[" << std::fabs(scaled_touch_x) << ", " << std::fabs(scaled_touch_y) << "]" << std::endl;
/*
			}				
			if (min_r.size() > 0) {
				/*for (int i = 0; i < int(min_rect.size()); ++i) {
					min_box = fitEllipse( min_r[i] );
					double scaled_touch_x = (min_box.center.x / CROP[2]) ;
					float scaled_touch_y = (min_box.center.y / CROP[3]) ;

					sender.addTuioCursor(scaled_touch_x, scaled_touch_y);

					/*
					if (std::fabs(scaled_touch_x - prev_x) > 0.0001) {
						scaled_touch_x = prev_x;
					}
					if (std::fabs(scaled_touch_y - prev_y) > 0.0001) {
						scaled_touch_y = prev_y;
					}*/
					//prev_x = scaled_touch_x;
					//prev_y = scaled_touch_y;
					//std::cout << "Touchpoint " << i << ": " << "[" << std::fabs(scaled_touch_x) << ", " << std::fabs(scaled_touch_y) << "]" << std::endl;

				/*}
				
			}
		}
		else {
			for (int i = 0; i < int(min_rect.size()); ++i) {
				min_box = fitEllipse( min_r[i] );			// draw an ellipse for each touchpoint

				float scaled_touch_x = min_box.center.x / CROP[2];		// framesize; hardcoded (!!!)
				float scaled_touch_y = min_box.center.y / CROP[3];

				sender.addTuioCursor(scaled_touch_x, scaled_touch_y);

				std::cout << "Touchpoint " << i << ": " << "[" << scaled_touch_x << ", " << scaled_touch_y << "]" << std::endl;
			}
		}
		
	}
}

void get_min_box(vector<Point> r, RotatedRect box) {

	if ((box.size.width < 500) && (box.size.width < 500)) {
		if (min_rect.size() == 0)	{		// first rect
			min_rect.push_back(box);
			min_r.push_back(r);
		}
		else {							// every other rect
			bool is_inside = true;
			for (int i = 0; i < int(min_rect.size()); ++i) {
				if (box.boundingRect().contains(min_rect[i].center)) { 	// if hierachical boxes break; else add box
					is_inside = true;
					break;
				}
				else {
					is_inside = false;
				}
			}
			if (is_inside == false) {
				min_rect.push_back(box);
				min_r.push_back(r);
			}
		}				
	}
}

bool is_bigger(RotatedRect box_1, RotatedRect box_2) {
	return box_1.boundingRect().size().area() < box_2.boundingRect().size().area();
}
*/