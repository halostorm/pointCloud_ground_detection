#include "point_cloud_plane_curves_extract.h"

#define _sectorNum 180
#define _sentorAngleResolution 2

struct Sentor
{
  float conf;
  int smooth;
  float plane[4];
  float edge[2];
  PointXYZRGBNormalCloud Points;
};

typedef std::vector<Sentor> sensorVector;

class PointCloudPlaneSegment
{
public:
  Sentor mPlane[_planeRings][_numOfAngleGrid];

  void PlaneSegment(const PointCloudPlaneCurvesExtract *pcpce);

  float InverseSqrt(float x)
  {
    float half_x = 0.5 * x;
    int i = *((int *)&x);              // 以整数方式读取X
    i = 0x5f3759df - (i >> 1);         // 神奇的步骤
    x = *((float *)&i);                // 再以浮点方式读取i
    x = x * (1.5 - (half_x * x * x));  // 牛顿迭代一遍提高精度
    return x;
  }
};
