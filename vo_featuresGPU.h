

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/core/cuda_types.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudafeatures2d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;

#define MIN_NUM_FEAT 1500
#define REGION1_X 400
#define REGION2_X 441
#define REGION_Y 376

static void download(const cuda::GpuMat& d_mat, vector<Point2f>& vec)
{
	vec.resize(d_mat.cols);
	Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
	d_mat.download(mat);
}

static void download(const cuda::GpuMat& d_mat, vector<uchar>& vec)
{
	vec.resize(d_mat.cols);
	Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
	d_mat.download(mat);
}

void featureDetectionCPU(Mat img_1, vector<Point2f>& points1)	{   //uses FAST
	vector<KeyPoint> keypoints_1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;
	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

void featureDetectionGPU(cuda::GpuMat img_1, vector<Point2f>& points1)	{   //uses FAST
	vector<KeyPoint> d_points1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;

	Ptr<cuda::FastFeatureDetector> d_fast = cuda::FastFeatureDetector::create(fast_threshold, nonmaxSuppression);
	d_fast->detect(img_1, d_points1);
	cout << "Fast Keypoints: " << d_points1.size() << endl;
	KeyPoint::convert(d_points1, points1, vector<int>());
}

void featureTrackingGPU(cuda::GpuMat img_1, cuda::GpuMat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 
//this function automatically gets rid of points for which tracking fails
  cuda::GpuMat d_points1(points1);
  cuda::GpuMat d_points2;
  cuda::GpuMat d_status(status);				
  Size winSize=Size(21,21);																								
  Ptr<cuda::SparsePyrLKOpticalFlow> d_pyrLK = cuda::SparsePyrLKOpticalFlow::create(winSize, \
  3, 100);

  d_pyrLK->calc(img_1, img_2, d_points1, d_points2, d_status);
  download(d_points2, points2);
  download(d_status, status);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0)){
     		  	status.at(i) = 0;}
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}
     }
}

void featureTrackingCPU(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{ 
//this function automatically gets rid of points for which tracking fails
  vector<float> err;					
  Size winSize=Size(21,21);																							
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);  

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, winSize, 3, termcrit, 0, 0.001);

  //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = points2.at(i- indexCorrection);
     	if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
     		  if((pt.x<0)||(pt.y<0)){status.at(i) = 0;}
     		  points1.erase (points1.begin() + (i - indexCorrection));
     		  points2.erase (points2.begin() + (i - indexCorrection));
     		  indexCorrection++;
     	}
     }
}



void featureDetectionFASTgpu(cuda::GpuMat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> d_points1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;

	Ptr<cuda::FastFeatureDetector> d_fast = cuda::FastFeatureDetector::create(fast_threshold, nonmaxSuppression);
	d_fast->detect(img_1, d_points1);
	cout << "Fast Keypoints: " << d_points1.size() << endl;
	KeyPoint::convert(d_points1, points1, vector<int>());
}

void featureDetectionFASTcpu(Mat img_1, vector<Point2f>& points1)	{   //uses FAST as of now, modify parameters as necessary
	vector<KeyPoint> keypoints_1;
	int fast_threshold = 20;
	bool nonmaxSuppression = true;

	FAST(img_1, keypoints_1, fast_threshold, nonmaxSuppression);
	KeyPoint::convert(keypoints_1, points1, vector<int>());
}

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)	{
	string line;
	int i = 0;
	ifstream myfile("/Users/groh/Documents/KITTI_VO/data_odometry_gray/dataset/poses/00.txt");
	double x = 0, y = 0, z = 0, x_prev, y_prev, z_prev;
	if (myfile.is_open()){
		while ((getline(myfile, line)) && (i <= frame_id)){
			z_prev = z;
			x_prev = x;
			y_prev = y;
			std::istringstream in(line);
			//cout << line << '\n';
			for (int j = 0; j<12; j++)  {
				in >> z;
				if (j == 7){ y = z; }
				if (j == 3) { x = z; }
			}
			i++;
		}
		myfile.close();
	}
	else {
		cout << "Unable to open file";
		return 0;
	}
	//debugging
	// cout << "x = " << x << '\n';
	// cout << "y = " << y << '\n';
	// cout << "z = " << z << '\n';

	// scale is calculated per frame and is the magnitude of the change in x, y, and z
	return sqrt((x - x_prev)*(x - x_prev) + (y - y_prev)*(y - y_prev) + (z - z_prev)*(z - z_prev));
}

void setScale(double scale, InputArray _R, InputArray _t, InputOutputArray _R_f, InputOutputArray _t_f, int& ScaleCount){
	Mat t = _t.getMat();
	Mat R = _R.getMat();
	Mat R_f = _R_f.getMat();
	Mat t_f = _t_f.getMat();
	cout << "Scale is: " << scale << endl;

	//check scaling factor and check that majority of movement is forward, else do not update R and t
	//Z>x and Z>Y && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
	if ((scale > 0.1))  {
		t_f = t_f + scale*(R_f*t);
		R_f = R*R_f;
	}
	else{
		cout << "scale below 0.1, or incorrect translation" << endl;
		/*
		cout << "R_f = " << R_f << endl;
		cout << "R_f(0) = " << R_f.at<double>(0) << endl;
		cout << "R_f(1) = " << R_f.at<double>(1) << endl;
		cout << "R_f(2) = " << R_f.at<double>(2) << endl;
		cout << "R_f(3) = " << R_f.at<double>(3) << endl;
		*/
		//thetaX = atan2(R_f.at<double>(), R_f.at<double>());
		ScaleCount++;
		waitKey(0);

		//DO SOMETHING when movement is not forward
		//count number of frames where frame doesnt update
	}
}

float calcMeanFlow(vector<Point2f> prevFeatures, vector<Point2f> currFeatures){
	float distance = 0;
    
	for (int i = 0; i < prevFeatures.size(); i++)
    {
		distance += abs(prevFeatures.at(i).x - currFeatures.at(i).x) + abs(prevFeatures.at(i).y - currFeatures.at(i).y);
	}
	distance /= prevFeatures.size();
	return distance;
}

void checkNtrackFeature(Mat prevImage, Mat currImage, vector<Point2f>& prevFeatures, vector<Point2f>& currFeatures, vector<uchar>& status, char* region, int minFeat){
	//Redetection is triggered when number of tracked features are too low
	vector<Point2f> tempFeatures;
	if (prevFeatures.size() < minFeat)	{
		cout << region << ": Number of tracked Features is: " << prevFeatures.size() << "<"<< minFeat <<" Features, Redetecting!" << endl;
		featureDetectionCPU(prevImage, tempFeatures); //find new features
		//Check to see if there are more features in the redetect before updating
		if (tempFeatures.size() > prevFeatures.size()){
			prevFeatures = tempFeatures;
			cout << region << ": New Features: " << prevFeatures.size() << " in " << endl;
		}
		else{
			cout << region << " Redetect has less features, not updating"<< endl;
			//waitKey(0);
		}
		featureTrackingCPU(prevImage, currImage, prevFeatures, currFeatures, status);
		cout << region << ": Tracked Features: " << currFeatures.size() << endl;
	}
	else {
		featureTrackingCPU(prevImage, currImage, prevFeatures, currFeatures, status);
		cout << region << ": Tracked Features: " << currFeatures.size() << endl;
	}
}

void getOdometryGPU(cuda::GpuMat prevImageGPU, cuda::GpuMat currImageGPU, vector<Point2f>& prevFeatures, vector<Point2f>& currFeatures, vector<uchar>& status, OutputArray R, OutputArray t, InputOutputArray mask, double focal = 1.0, Point2d pp = Point2d(0, 0)){
	//Feature Tracking -> Essential Matrix -> Recover Pose
	featureTrackingGPU(prevImageGPU, currImageGPU, prevFeatures, currFeatures, status);

	clock_t start2 = clock();

	Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.99, 1.0, mask);
	cout << "getOdometry Time: " << (((double)clock() - start2) / CLOCKS_PER_SEC)*1000<< "ms" << endl;
	recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask); // 0.244 s
}
/*
void getOdometryCPU(Mat prevImage, Mat currImage, vector<Point2f>& prevFeatures, vector<Point2f>& currFeatures, vector<uchar>& status, OutputArray R, OutputArray t, InputOutputArray mask, double focal = 1.0, Point2d pp = Point2d(0, 0))
{
    //Feed in previous found features into Feature Tracking -> Estimate Essential Matrix w/RANSAC -> Recover Pose
    //Input previous image and features, load new image and track old features to new
    
    //Redetection is triggered when number of tracked features are too low
    //checkNtrackFeature(prevImage, currImage, prevFeatures, currFeatures, status, text);
    char region1[25];
    sprintf(region1, "Region 1");
    
    checkNtrackFeature(prevImage, currImage, prevFeatures, currFeatures, status, region1, 2500 );
    float distance = (calcMeanFlow(prevFeatures, currFeatures));
    
    clock_t start2 = clock();
    if (distance < 4.0){
        //if mean optical flow is less than X, dont update distance
        cout << "Average Pixel Distance is:" << distance << endl;
        waitKey(0);
    }
    else{
        Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.99, 1, mask);
        recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
        scale = getAbsoluteScale(numFrame, 0, t.at<double>(2));
        //setScale(scale, R, t, R_f, t_f, ScaleCount);
    }
}
*/



void getOdometryCPU_old(Mat prevImage, Mat currImage, vector<Point2f>& prevFeatures, vector<Point2f>& currFeatures, vector<uchar>& status, OutputArray R, OutputArray t, InputOutputArray mask, double focal = 1.0, Point2d pp = Point2d(0, 0)){
	//Feed in previous found features into Feature Tracking -> Estimate Essential Matrix w/RANSAC -> Recover Pose
		//Input previous image and features, load new image and track old features to new

	//Redetection is triggered when number of tracked features are too low
	//checkNtrackFeature(prevImage, currImage, prevFeatures, currFeatures, status, text);

	// Mean Optical Flow Check
	float distance = calcMeanFlow(prevFeatures, currFeatures);
	cout << "Distance = " << distance << endl;

	clock_t start2 = clock();
	if (distance < 8){
		cout << "Average Pixel Distance is:" << distance <<  endl;
		waitKey(0);
	}
	else{
		Mat E = findEssentialMat(currFeatures, prevFeatures, focal, pp, RANSAC, 0.99, 1, mask);		// findEssentialMat slows significantly sometimes
		recoverPose(E, currFeatures, prevFeatures, R, t, focal, pp, mask);
	}
	cout << "getOdometry Time: " << (((double)clock() - start2) / CLOCKS_PER_SEC) * 1000 << "ms" << endl;
}


void getReferenceVO(int frame_id, int sequence_id, float& VOx, float& VOy, float& VOz)	{
	string line;
	int i = 0;
	ifstream myfile("/Users/groh/Documents/KITTI_VO/data_odometry_gray/dataset/poses/00.txt");
	float x = 0, y = 0, z = 0;

	if (myfile.is_open()){
		while ((getline(myfile, line)) && (i <= frame_id)){
			std::istringstream in(line);
			//cout << line << '\n';
			// in the reference pose file, the last column is z, the 8th column is y, and the 4th is x
			for (int j = 0; j<12; j++)  {
				in >> z;
				if (j == 7){y = z;}
				if (j == 3){x = z;}
			}
			i++;
		}
		myfile.close();
	}
	else {
		cout << "Unable to open file";
	}
	VOx = x;
	VOy = y;
	VOz = z;

	//debugging
	//cout << "x = " << x << '\n';
	//cout << "y = " << y << '\n';
	//cout << "z = " << z << '\n';
}

void calcRefDisplacement(int frame_id, int sequence_id, double& displacement)	{
	string line;
	int i = 0;
	ifstream myfile("/Users/groh/Documents/KITTI_VO/data_odometry_gray/dataset/poses/00.txt");
	double x = 0, y = 0, z = 0, z_prev = z,	x_prev = x;

	if (myfile.is_open()){
		while ((getline(myfile, line)) && (i <= frame_id)){
			z_prev = z;
			x_prev = x;
			std::istringstream in(line);
			// in the reference pose file, the last column is z, the 8th column is y, and the 4th is x
			for (int j = 0; j<12; j++)  {
				in >> z;
				if (j == 7){ y = z; }
				if (j == 3){ x = z; }
			}
			i++;
			displacement = displacement + sqrt((x - x_prev)*(x - x_prev) + (z - z_prev)*(z - z_prev));
		}
		myfile.close();
	}
	else {
		cout << "Unable to open file";
	}
}

void calcDisplacement(Mat t, Mat t_f, double& displacement)	{
	displacement = displacement + sqrt((t.at<double>(2) - t_f.at<double>(2))*(t.at<double>(2) - t_f.at<double>(2)) + (t.at<double>(0) - t_f.at<double>(0))*(t.at<double>(0) - t_f.at<double>(0)));
}

void concatFeat(vector<Point2f> Feat_1, vector<Point2f> Feat_2, vector<Point2f> Feat_3, vector<Point2f>& Feat){
	//	prevFeatures.reserve(prevFeat_1.size() + prevFeat_2.size() + prevFeat_3.size());
	//prevFeatures.insert(prevFeat_1.begin, prevFeat_1.end, prevFeat_2.begin, prevFeat_2.end, prevFeat_3.begin, prevFeat_3.end);
	//	prevFeatures.insert(prevFeat_1.begin, prevFeat_1.end);

	vector<Point2f> temp;
	for (int i = 0; i < Feat_2.size(); i++){
		Feat_2.at(i).x += REGION1_X;
	}
	for (int i = 0; i < Feat_3.size(); i++){
		Feat_3.at(i).x += REGION1_X + REGION2_X;
	}

	hconcat(Feat_1, Feat_2, temp);
	hconcat(temp, Feat_3, Feat);
}

void splitRegion(Mat image, Mat& image_1, Mat& image_2, Mat& image_3 ){
	Rect region1(0, 0, REGION1_X, REGION_Y); //start point x,y and region size x,y
	Rect region2(REGION1_X, 0, REGION2_X, REGION_Y);
	Rect region3(REGION1_X + REGION2_X, 0, (1241 - REGION1_X - REGION2_X), REGION_Y);

	image_1 = image(region1);
	image_2 = image(region2);
	image_3 = image(region3);
}

void plotOdometry(Mat& traj, Mat t_f, int numFrame, float& error_sum){
	int fontFace = FONT_HERSHEY_PLAIN;
	float fontScale = 1;
	int thickness = 1;
	char text[100];
	float VOx, VOy, VOz;
	
	//Plotting
	rectangle(traj, Point(10, 10), Point(650, 120), CV_RGB(0, 0, 0), CV_FILLED);

	int x = int(t_f.at<double>(0)) + 300; //x 
	int y = -1 * int(t_f.at<double>(2)) + 550; //z
	circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);
	sprintf(text, "Coords: x = %.2fm y = %02fm z = %.2fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
	putText(traj, text, Point(10, 30), fontFace, fontScale, Scalar::all(255), thickness, 8);

	getReferenceVO(numFrame, 0, VOx, VOy, VOz);
	int VOx_plot = VOx + 300;
	int VOz_plot = -1 * VOz + 550;
	circle(traj, Point(VOx_plot, VOz_plot), 1, CV_RGB(255, 255, 0), 2);
	sprintf(text, "Coords: VOx = %.2fm VOy = %.2fm VOz = %.2fm", VOx, VOy, VOz);
	putText(traj, text, Point(10, 50), fontFace, fontScale, Scalar::all(255), thickness, 8);

	circle(traj, Point(VOx_plot, VOz_plot), 1, CV_RGB(255, 255, 0), 2);
	sprintf(text, "Difference: x = %.2fm y = %.2fm z = %.2fm", abs(VOx - t_f.at<double>(0)), VOy, abs(VOz - t_f.at<double>(2)));
	putText(traj, text, Point(10, 70), fontFace, fontScale, Scalar::all(255), thickness, 8);

	//Error Calculation for Z and X
	float eucl_dist = sqrt((pow(VOx - t_f.at<double>(0), 2) + pow(VOz - t_f.at<double>(2), 2)));
	error_sum = error_sum + eucl_dist;
	float MAE = error_sum / numFrame;
	sprintf(text, "Mean Euclidean Error = %.2fm, Framewise Euclidean Distance = %.2fm ", MAE, eucl_dist);
	putText(traj, text, Point(10, 90), fontFace, fontScale, Scalar::all(255), thickness, 8);

	//	calcDisplacement(t, t_f, displacement);
	//	calcRefDisplacement(numFrame, 0, refDisp);

	imshow("Trajectory", traj);
}

/*
void featureDetectionORBgpu(cuda::GpuMat img_1, vector<Point2f>& points1)	{
vector<KeyPoint> d_points1;

int 	nfeatures = 500; //The maximum number of features to retain.
float 	scaleFactor = 1.2f; // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
int 	nlevels = 8; //	The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).
int 	edgeThreshold = 31; //This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
int 	firstLevel = 0;
int 	WTA_K = 2; 	//The number of points that produce each element of the oriented BRIEF descriptor.The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0 / 1 response.Other possible values are 3 and 4. For example, 3 means that we take 3 random points(of course, those point coordinates are random, but they are generated from the pre - defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner(0, 1 or 2).Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2(2 bits per bin).When WTA_K = 4, we take 4 random points to compute each bin(that will also occupy 2 bits with possible values 0, 1, 2 or 3).
int 	scoreType = cv::ORB::HARRIS_SCORE; //	The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
int 	patchSize = 31; //size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.
int 	fastThreshold = 20;

Ptr<cuda::ORB> d_ORB = cuda::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType);
d_ORB->detect(img_1, d_points1);
cout << "ORB Keypoints: " << d_points1.size() << endl;
KeyPoint::convert(d_points1, points1, vector<int>());
}

void featureDetectionORBcpu(Mat img_1, vector<Point2f>& points1, OutputArray descriptors)	{   //ORB, modify parameters as necessary
// Find Keypoints and Descriptors

vector<KeyPoint> keypoints_1;
Mat mask;
int 	nfeatures = 500; //The maximum number of features to retain.
float 	scaleFactor = 1.2f; // Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.
int 	nlevels = 8; //	The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).
int 	edgeThreshold = 31; //This is size of the border where the features are not detected. It should roughly match the patchSize parameter.
int 	firstLevel = 0;
int 	WTA_K = 2; 	//The number of points that produce each element of the oriented BRIEF descriptor.The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0 / 1 response.Other possible values are 3 and 4. For example, 3 means that we take 3 random points(of course, those point coordinates are random, but they are generated from the pre - defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner(0, 1 or 2).Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2(2 bits per bin).When WTA_K = 4, we take 4 random points to compute each bin(that will also occupy 2 bits with possible values 0, 1, 2 or 3).
int 	scoreType = cv::ORB::HARRIS_SCORE; //	The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.
int 	patchSize = 31; //size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.
int 	fastThreshold = 20;

Ptr<cv::ORB> d_ORB = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel, WTA_K, scoreType);

Mat descriptors;
d_ORB->detectAndCompute(img_1, mask, keypoints_1, descriptors); //
//d_ORB->detect(img_1, keypoints_1);

cout << "ORB Keypoints: " << keypoints_1.size() << endl;
KeyPoint::convert(keypoints_1, points1, vector<int>());
} */

/*
void featureTrackingORB(Mat img_1, Mat img_2, vector<Point2f>& points1, vector<Point2f>& points2, vector<uchar>& status)	{
Mat descript1, descript2;

featureDetectionORBcpu(img_1, points1, descript1);
featureDetectionORBcpu(img_2, points2, descript2);

FlannBasedMatcher matcher;
vector< DMatch > matches;
matcher.match(descript1, descript2, matches);

double max_dist = 0; double min_dist = 100;

//-- Quick calculation of max and min distances between keypoints
for (int i = 0; i < descript1.rows; i++)
{
double dist = matches[i].distance;
if (dist < min_dist) min_dist = dist;
if (dist > max_dist) max_dist = dist;
}

std::vector< DMatch > good_matches;

for (int i = 0; i < descriptors_1.rows; i++)
{
if (matches[i].distance <= max(2 * min_dist, 0.02))
{
good_matches.push_back(matches[i]);
}
}

Mat img_matches;
drawMatches()
drawMatches(img_1, points1, img_2, points2,good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),	vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
}*/


