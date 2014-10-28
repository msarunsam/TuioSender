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

// callbacks
void init_camera();
void open_stream(int width, int height);

int main() {

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

void get_stream(int width, int height) {
	while (true) 
	{
		if (!g_cam->capture() == NULL) {
			if (g_cam->capture() == NULL) {
					std::cout << "Error: stream is empty" << std::endl;
				
			}
			else {
				// save the data stream in each frame
				IplImage* frame = cvCreateImageHeader(cvSize(width, height), IPL_DEPTH_8U, 1);
				frame -> imageData = (char*) g_cam-> capture();

				Mat grey_image(frame);
				Mat lab_image;

				cv::cvtColor( grey_image, lab_image, CV_BGR2Lab );

				// show stream
				cvShowImage("Stream", frame);

		        // Extract the L channel
	            std::vector<Mat> lab_planes(3);
	            cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

	            // apply the CLAHE algorithm to the L channel
	            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	            clahe->setClipLimit(4);
	            Mat dst;
	            clahe->apply(lab_planes[0], dst);

	            // Merge the color planes back into an Lab image
	            dst.copyTo(lab_planes[0]);
	            cv::merge(lab_planes, lab_image);

	           // convert back to RGB
	           Mat image_clahe;
	           cv::cvtColor(lab_image, image_clahe, CV_BGR2GRAY);

	           // display the results  (you might also want to see lab_planes[0] before and after).
	           cv::imshow("image original", grey_image);
	           cv::imshow("image CLAHE", image_clahe);
	           
	           key = cvWaitKey(10);

			}		
		}
	}
}
