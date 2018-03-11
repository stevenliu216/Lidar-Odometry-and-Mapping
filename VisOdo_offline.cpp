

#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "vo_featuresGPU.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/core/cuda_types.hpp"
// #include "opencv2/cudacodec.hpp"

#include <thread>

/*to do list
1. change back to kitti, try to get direct to GPU reading
2. Create two versions of feature detection, tracking, getOdom, CPU and GPU
3. Add in timers for all sections
4. Compare GPU and CPU
5. Add in other feature detection algorithms
6. Speed up find essential matrix
7. Page locked memory to speed up CPU2GPU memory transfer rates by double?
*/

using namespace cv;
using namespace std;

#define MAX_FRAME 2000

int main(int argc, char** argv)
{
	// open the default camera, use something different from 0 otherwise;	
	//cuda::HostMem page_locked(Size(1241, 376), CV_8U); //allocate page locked memory
	//Mat cap = page_locked; //no copy, just header
	
    VideoCapture cap("/Users/groh/Documents/KITTI_VO/data_odometry_gray/dataset/sequences/00/image_0/%06d.png");
    if (!cap.isOpened())
    {
		cout << "OpenCV did not open file" << endl; waitKey(0);
		return 101;
	}
    
    //Grab First two images
    Mat prevImage, currImage;
    cap>>prevImage;
    cap>>currImage;
    if (!prevImage.data || !currImage.data) {
        std::cout << " --(!) Error reading images " << std::endl; return -1;
    }
    //Display first two images
    namedWindow("First Image", WINDOW_AUTOSIZE);
    imshow("First Image",prevImage);
    waitKey(0);
    imshow("First Image",currImage);
    waitKey(0);
    
    // plotting initialization and setup
    Mat traj = Mat::zeros(650, 650, CV_8UC3);
    Mat RefTraj = Mat::zeros(650, 650, CV_8UC3);
    Mat bothTraj = Mat::zeros(650, 650, CV_8UC3);
    Mat KeyPointVisualization;
    vector<KeyPoint> keyPointsTracked;
    
	/*//Set Camera Settings for Logitech USB webcam
	double focal = 522.7536;
	double scale = 1.0;
	cv::Point2d pp(319.5, 239.5); */

	//Set Camera Settings for KITTI
	double focal = 718.8560;
	cv::Point2d pp(607.1928, 185.2157);

	//Reference Visual Odometry
    double scale = 1.0;
    int ScaleCount = 0;
    char region1[25];
    sprintf(region1, "Region 1");
    
    
	//vectors to store the coordinates of the feature points
	vector<Point2f> currFeatures, prevFeatures;
	vector<uchar> status;
    
	//feature detection, tracking for first two images	
	featureDetectionCPU(prevImage, prevFeatures);        //detect features
    featureTrackingCPU(prevImage, currImage, prevFeatures, currFeatures, status);
    
	//Inialize rotation, translation, and mask, the final rotation and translation vectors
	Mat R, t, mask, R_f, t_f;
	//recover the pose and the essential matrix for the first two images
    Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.99, 1, mask);
    recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
	//getOdometryCPU(prevImage, currImage, prevFeatures, currFeatures, status, R, t, mask, focal, pp);

	// Final Rotation and Translation
	R_f = R.clone();
	t_f = t.clone();
    float error_sum = 0;
    
	clock_t start = clock();
	double loopTime;
    float distance =0;
    Ptr<CLAHE> clahe = createCLAHE();

    // MAIN LOOP
	for (int numFrame = 2; numFrame < MAX_FRAME; numFrame++)
    {
		// Check VideoCapture documentation.
		start = clock();
		if (!cap.isOpened())
			return 0;

		//Grab Frame
		cap >> currImage;
        //equalizeHist( currImage, currImage );
        clahe->apply(currImage, currImage);

		if (currImage.empty()) break; // end of video stream
		if (waitKey(1) == 27) break; // stop capturing by pressing ESC 

		//Odometry Function
        checkNtrackFeature(prevImage, currImage, prevFeatures, currFeatures, status, region1, 3500 );
        distance = (calcMeanFlow(prevFeatures, currFeatures));
        //
        clock_t start2 = clock();
        
        if (distance < 3.0)
        {
            //if mean optical flow is less than X, dont update distance
            cout << "Average Pixel Distance is:" << distance << endl;
            waitKey(0);
        }
        else
        {
            Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.99, 1, mask);
            recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
            scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));
            setScale(scale, R, t, R_f, t_f, ScaleCount);
        }
        
        // Move currImage to prevImage, currFeatures to prevFeatures
        prevImage = currImage.clone();
        prevFeatures = currFeatures;
        //Calculate Loop Time
        cout << "getOdometry Time: " << (((double)clock() - start2) / CLOCKS_PER_SEC) * 1000 << "ms" << endl;

		loopTime = (((double)clock() - start) / CLOCKS_PER_SEC) * 1000;
		cout << "Loop Time:" << loopTime << "ms | " << prevFeatures.size() << " | " << prevFeatures[prevFeatures.size() - 1].x << endl;
		// END FUNCTION

        
        //PLOTTING
        plotOdometry(traj, t_f, numFrame, error_sum);

		cv::KeyPoint::convert(currFeatures,keyPointsTracked);
		drawKeypoints(currImage, keyPointsTracked, KeyPointVisualization);
		imshow("KeyPoint Visualization", KeyPointVisualization);
	}
	waitKey(0);
	imwrite("images/RefvsCalculated.png", traj);
	// the camera will be closed automatically upon exit
	// cap.close();
	return 0;
}
