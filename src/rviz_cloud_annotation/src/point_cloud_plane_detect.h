#include <sys/stat.h>
#include <sys/types.h>
// STL
#include <dirent.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <stdint.h>
#include <stdio.h>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
// ROS
#include <interactive_markers/interactive_marker_server.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64MultiArray.h>
#include <termios.h>
// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/colors.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define _lowerBound -15
#define _upperBound 15
#define _numOfRings 16

#define _radiusRadio 6.9
#define _srcLenThreshold 0.2
#define _arcNumThreshold 7
#define _disScaleThreshold 0.8

#define _windowsize 100

typedef int64_t int64;
typedef uint64_t uint64;
typedef unsigned short ushort;
typedef std::vector<uint64> Uint64Vector;
typedef std::vector<int64> Int64Vector;

typedef pcl::PointXYZRGBNormal PointXYZRGBNormal;
typedef pcl::PointCloud<PointXYZRGBNormal> PointXYZRGBNormalCloud;

class PointCloudPlaneDetect
{
private:
  PointXYZRGBNormalCloud* mPlaneVector;
  PointXYZRGBNormalCloud* mCurvesVector;
  int64 PLANE_ID = 0;

public:
  Uint64Vector mCurvesId[_numOfRings];
  PointXYZRGBNormalCloud* SearchCurves(PointXYZRGBNormalCloud& PointCloud);
  PointXYZRGBNormalCloud CurveDensityFilter(PointXYZRGBNormalCloud& Curve, int64 ringID, Uint64Vector& curveid);
  PointXYZRGBNormalCloud CurveSizeFilter(PointXYZRGBNormalCloud& Curve, int64 ringID, Uint64Vector& curveid);

  float InverseSqrt(float x)
  {
    float half_x = 0.5 * x;
    int i = *((int*)&x);               // 以整数方式读取X
    i = 0x5f3759df - (i >> 1);         // 神奇的步骤
    x = *((float*)&i);                 // 再以浮点方式读取i
    x = x * (1.5 - (half_x * x * x));  // 牛顿迭代一遍提高精度
    return x;
  }

  float getVar(float x[], int len)
  {
    int m = len;
    float sum = 0;
    for (int i = 0; i < m; i++)
    {  // 求和
      sum += x[i];
    }
    float dAve = sum / m;  // 求平均值
    float dVar = 0;
    for (int i = 0; i < m; i++)
    {  // 求方差
      dVar += (x[i] - dAve) * (x[i] - dAve);
    }
    return dVar / m;
  }

  int64 GetScanringID(const float& angle);
  float GetScanringRadius(int64 ID);
};