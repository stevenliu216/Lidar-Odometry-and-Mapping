/*
  The main objective of scanRegistration is to preprocess point cloud and IMU data. 
  This particular approach uses the curvature of point clouds to separate into various
  features (Edge, Plane, or NaN). 
  
  See equation (1) for curvature equation in LOAM paper by Ji Zhang.
*/

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;

/* Magic Numbers - These will be different when running on KITTI dataset */
const float scanPeriod = 0.1;
const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

/* Declaring pcl containers */
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsSharp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr cornerPointsLessSharp(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsFlat(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlat(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr surfPointsLessFlatScanDS(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr imuTrans(new pcl::PointCloud<pcl::PointXYZ>(4, 1));
/* Original dataset uses VLP-16, hence the magic number 16 
 * The KITTI dataset uses HDL-64E, we would use 64 here at the minimum.
 * Needs more consideration for how to sort through the scans for HDL-64E.
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudScans[16];

/* The size of these would need to be increased for KITTI dataset
 * because of the HDL-64E having over 2 million points per scan
 */
float cloudCurvature[40000];
int cloudSortInd[40000];
int cloudNeighborPicked[40000];
int cloudLabel[40000];

/* Change 16 to 64 for KITTI */
int scanStartInd[16];
int scanEndInd[16];

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;

float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;

float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;

float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

/* Declare ros publishers */
ros::Publisher* pubLaserCloudPointer;
ros::Publisher* pubCornerPointsSharpPointer;
ros::Publisher* pubCornerPointsLessSharpPointer;
ros::Publisher* pubSurfPointsFlatPointer;
ros::Publisher* pubSurfPointsLessFlatPointer;
ros::Publisher* pubImuTransPointer;

void ShiftToStartIMU(float pointTime)
{
  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

void VeloToStartIMU()
{
  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}

void TransformToStartIMU(pcl::PointXYZI *p)
{
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

  p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}

void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < 0.1) {

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}

/* laserCloudHandler is the main routine in scanRegistration. 
 * According to LOAM paper, equation (1), features are found based on the point cloud
 * curvature. Then the features are divided into a few categories, mainly edge/planar.
 * 
 * Observation: The code doesn't look like the math in equation (1)
 */
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudIn2)
{
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    }
    return;
  }
  
  double timeScanCur = laserCloudIn2->header.stamp.toSec();

  pcl::fromROSMsg(*laserCloudIn2, *laserCloudIn);
  int cloudSize = laserCloudIn->points.size();

  /* Starting angle and ending angle */
  float startOri = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
  float endOri = -atan2(laserCloudIn->points[cloudSize - 1].y, 
                        laserCloudIn->points[cloudSize - 1].x) + 2 * PI;

  if (endOri - startOri > 3 * PI) {
    endOri -= 2 * PI;
  } else if (endOri - startOri < PI) {
    endOri += 2 * PI;
  }


  /* Iterate through all point clouds !!!
   * Divide into different lines based on the resulting angle calculation.
   * 1 - Find angle
   * 2 - Find start/end indices
   * 3 - Insert IMU data
   * 4 - Store in pcl containers
   */
  bool halfPassed = false;
  pcl::PointXYZI point;

  /* You can tell 3D lidar is very much different from 2D where you can get 
   * a (range,bearing) every scan. 3D requires much more calculations.
   */


  /* This likely must be reworked for HDL-64E
   */
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn->points[i].y;
    point.y = laserCloudIn->points[i].z;
    point.z = laserCloudIn->points[i].x;

    float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / PI;
    int scanID = int(0.75 * angle + 0.5) + 7;
    if (angle < 0) {
      scanID--;
    }

    float ori = -atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - PI / 2) {
        ori += 2 * PI;
      } else if (ori > startOri + PI * 3 / 2) {
        ori -= 2 * PI;
      }

      if (ori - startOri > PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * PI;

      if (ori < endOri - PI * 3 / 2) {
        ori += 2 * PI;
      } else if (ori > endOri + PI / 2) {
        ori -= 2 * PI;
      } 
    }

    float relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + 0.1 * relTime;


	/* IMU data is inserted here */
    if (imuPointerLast >= 0) {
      float pointTime = relTime * scanPeriod;
      while (imuPointerFront != imuPointerLast) {
        if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
          break;
        }
        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
      }

      if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
        imuRollCur = imuRoll[imuPointerFront];
        imuPitchCur = imuPitch[imuPointerFront];
        imuYawCur = imuYaw[imuPointerFront];

        imuVeloXCur = imuVeloX[imuPointerFront];
        imuVeloYCur = imuVeloY[imuPointerFront];
        imuVeloZCur = imuVeloZ[imuPointerFront];

        imuShiftXCur = imuShiftX[imuPointerFront];
        imuShiftYCur = imuShiftY[imuPointerFront];
        imuShiftZCur = imuShiftZ[imuPointerFront];
      } else {
        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

        imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
        imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
        if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * PI) * ratioBack;
        } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * PI) * ratioBack;
        } else {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
        }

        imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
        imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
        imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

        imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
        imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
        imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
      }

      if (i == 0) {
        imuRollStart = imuRollCur;
        imuPitchStart = imuPitchCur;
        imuYawStart = imuYawCur;

        imuVeloXStart = imuVeloXCur;
        imuVeloYStart = imuVeloYCur;
        imuVeloZStart = imuVeloZCur;

        imuShiftXStart = imuShiftXCur;
        imuShiftYStart = imuShiftYCur;
        imuShiftZStart = imuShiftZCur;
      } else {
		/* !! 
		 * Not in the first frame. Thus the IMU data is converted to the first frame of the IMU coordinates
		 */
        ShiftToStartIMU(pointTime);
        VeloToStartIMU();
        TransformToStartIMU(&point);
      }
    }

    laserCloudScans[scanID]->push_back(point);
  }

  /* KITTI must use 64 here 
   * This loop just updates the entire laserCloud
   */
  for (int i = 0; i < 16; i++) {
    *laserCloud += *laserCloudScans[i];
  }

  /* Below, calculate the curvature of the plane formed by a point and it's neighboring 10 points.
   * Recall that radius of curvature = 1/curvature -> curvature = 1/R
   * R*R is actually used to represent the curvature. So the bigger R*R is, the more curved.
   */
  int scanCount = -1;
  for (int i = 5; i < cloudSize - 5; i++) {
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x 
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x 
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x 
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y 
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y 
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y 
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z 
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z 
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z 
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;
    
    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    if (int(laserCloud->points[i].intensity) != scanCount) {
      scanCount = int(laserCloud->points[i].intensity);

	  /* Should there be additional condition of scanCount < 16 or 64?  */
      if (scanCount > 0) {
		/* The start and end indices of the scan
		 * Arbitrarily filter out the first and last 5 points (KITTI needs tuning)
		 */
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
      }
    }
  }
  /* We arbitrarily filtered out the first and last 5 points, so adjust accordingly here for KITTI */
  scanStartInd[0] = 5;
  scanEndInd[15] = cloudSize - 5;

  /* According to LOAM paper for throwing out bad features (See Fig 4):
   * 1. Plane/straight edges that are approximately parallel to the laser (a)
   * 2. Occuluded edge points (b)
   */
  for (int i = 5; i < cloudSize - 6; i++) {
    
	float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;

	/* Find the distance squared between points[i+1] and points[i] */
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

	/* 0.1 is a magic number, potentially needs tuning for KITTI */
    if (diff > 0.1) {

      /* depth of points[i] */
      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);
	  /* depth of points[i+1] */
      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

	  /* This corresponds to (b) in Fig 4 
	   * X(i+1)-X(i)*（|X(i+1)|/|X(i)|）
	   */
      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;
		/* Think triangles */

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
		  /* If the angle between X(i+1) and X(i) is less than 0.1.. or ~ 5.7 degrees */
          cloudNeighborPicked[i - 5] = 1;
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

	/* 
	 * This corresponds to (a) in Fig 4.
	 * Lots of magical variable names and numbers here
	 */
    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
	  /* This 0.0002 is important, it is about 0.0115 degrees */
      cloudNeighborPicked[i] = 1;
    }
  }

  /* Now, we should have a well organized set of point clouds, with the features being stored.
   * We divide one scan into 4 sub-regions, and each sub-region provides a max of two edge and four 
   * planars. Thresholds must be predefined in order to easily categorize these points. In the following
   * code, each line gets divided into 6 segments and the segments are sorted in ascending order by curvature.
   */
  /* Lots of magic parameters to tune per above. Again, the 16 should be 64 for KITTI */
  for (int i = 0; i < 16; i++) {
    surfPointsLessFlatScan->clear();
    for (int j = 0; j < 6; j++) {
	  /* This is where a line gets split into 6 segments */
	  /* sp = start position, ep = end position */
      int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
      int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;

	  /* Sort by curvature in ascending order */
      for (int k = sp + 1; k <= ep; k++) {
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > 0.1) {
        
          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp->push_back(laserCloud->points[ind]);
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;
            cornerPointsLessSharp->push_back(laserCloud->points[ind]);
          } else {
            break;
          }

		  /* Unclear from the paper what this is.
		   * But it looks like checking the neighboring points for potential features.
		   */
          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x 
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y 
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z 
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x 
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y 
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z 
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;
          surfPointsFlat->push_back(laserCloud->points[ind]);

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x 
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y 
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z 
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x 
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y 
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z 
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {
          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

	/* The pcl library is used here for downsampling of lessFlatScan */
    surfPointsLessFlatScanDS->clear();
    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
	/* What is with these magic numbers? */
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.filter(*surfPointsLessFlatScanDS);

    *surfPointsLessFlat += *surfPointsLessFlatScanDS;
  }

  /* Initializing the topics to publish */
  sensor_msgs::PointCloud2 laserCloud2;
  pcl::toROSMsg(*laserCloud, laserCloud2);
  laserCloud2.header.stamp = laserCloudIn2->header.stamp;
  laserCloud2.header.frame_id = "/camera";
  pubLaserCloudPointer->publish(laserCloud2);

  sensor_msgs::PointCloud2 cornerPointsSharp2;
  pcl::toROSMsg(*cornerPointsSharp, cornerPointsSharp2);
  cornerPointsSharp2.header.stamp = laserCloudIn2->header.stamp;
  cornerPointsSharp2.header.frame_id = "/camera";
  pubCornerPointsSharpPointer->publish(cornerPointsSharp2);

  sensor_msgs::PointCloud2 cornerPointsLessSharp2;
  pcl::toROSMsg(*cornerPointsLessSharp, cornerPointsLessSharp2);
  cornerPointsLessSharp2.header.stamp = laserCloudIn2->header.stamp;
  cornerPointsLessSharp2.header.frame_id = "/camera";
  pubCornerPointsLessSharpPointer->publish(cornerPointsLessSharp2);

  sensor_msgs::PointCloud2 surfPointsFlat2;
  pcl::toROSMsg(*surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudIn2->header.stamp;
  surfPointsFlat2.header.frame_id = "/camera";
  pubSurfPointsFlatPointer->publish(surfPointsFlat2);

  sensor_msgs::PointCloud2 surfPointsLessFlat2;
  pcl::toROSMsg(*surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudIn2->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/camera";
  pubSurfPointsLessFlatPointer->publish(surfPointsLessFlat2);

  imuTrans->points[0].x = imuPitchStart;
  imuTrans->points[0].y = imuYawStart;
  imuTrans->points[0].z = imuRollStart;

  imuTrans->points[1].x = imuPitchCur;
  imuTrans->points[1].y = imuYawCur;
  imuTrans->points[1].z = imuRollCur;

  imuTrans->points[2].x = imuShiftFromStartXCur;
  imuTrans->points[2].y = imuShiftFromStartYCur;
  imuTrans->points[2].z = imuShiftFromStartZCur;

  imuTrans->points[3].x = imuVeloFromStartXCur;
  imuTrans->points[3].y = imuVeloFromStartYCur;
  imuTrans->points[3].z = imuVeloFromStartZCur;

  sensor_msgs::PointCloud2 imuTrans2;
  pcl::toROSMsg(*imuTrans, imuTrans2);
  imuTrans2.header.stamp = laserCloudIn2->header.stamp;
  imuTrans2.header.frame_id = "/camera";
  pubImuTransPointer->publish(imuTrans2);

  laserCloudIn->clear();
  laserCloud->clear();
  cornerPointsSharp->clear();
  cornerPointsLessSharp->clear();
  surfPointsFlat->clear();
  surfPointsLessFlat->clear();

  /* Use 64 for KITTI */
  for (int i = 0; i < 16; i++) {
    laserCloudScans[i]->points.clear();
  }
}

/* imuHandler is another main routine in this node
 * The code is much easier to read than the above laserCloudHandler
 */
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(imuIn->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
  float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
  float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
  imuYaw[imuPointerLast] = yaw;
  imuAccX[imuPointerLast] = accX;
  imuAccY[imuPointerLast] = accY;
  imuAccZ[imuPointerLast] = accZ;

  AccumulateIMUShift();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;

  for (int i = 0; i < 16; i++) {
    laserCloudScans[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/velodyne_cloud", 2, laserCloudHandler);

  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> 
                                 ("/velodyne_cloud_2", 2);

  ros::Publisher pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2> 
                                        ("/laser_cloud_sharp", 2);

  ros::Publisher pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2> 
                                            ("/laser_cloud_less_sharp", 2);

  ros::Publisher pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2> 
                                       ("/laser_cloud_flat", 2);

  ros::Publisher pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2> 
                                           ("/laser_cloud_less_flat", 2);

  ros::Publisher pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);

  pubLaserCloudPointer = &pubLaserCloud;
  pubCornerPointsSharpPointer = &pubCornerPointsSharp;
  pubCornerPointsLessSharpPointer = &pubCornerPointsLessSharp;
  pubSurfPointsFlatPointer = &pubSurfPointsFlat;
  pubSurfPointsLessFlatPointer = &pubSurfPointsLessFlat;
  pubImuTransPointer = &pubImuTrans;

  ros::spin();

  return 0;
}
