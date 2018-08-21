#ifndef POINT_CLOUD_PLANE_SEGMENT
#define POINT_CLOUD_PLANE_SEGMENT

#include "point_cloud_plane_curves_extract.h"

struct Sentor
{
  bool isGround;
  int conf[2]; // Ground Confidence  = number of points that on the sentor
  float smooth;
  float radiusEdge[2];
  Eigen::VectorXf planeParams;
  PointXYZRGBNormalCloud oneLinePoints;
  PointXYZRGBNormalCloud twoLinePoints;
};
typedef std::vector<Sentor> sensorVector;

class PointCloudPlaneSegment
{
public:
  Sentor mPlane[_planeRings][_numOfAngleGrid];

  void PlaneSegment(const PointCloudPlaneCurvesExtract *pcpce);

  float getInclination(const float A[3], const float B[3])
  {
    float inclination =
        std::atan2((A[2] - B[2]), m_sqrt((A[0] - B[0]) * (A[0] - B[0]) + (A[1] - B[1]) * (A[1] - B[1])));
    return inclination;
  }

  void getPlaneRansac(PointXYZRGBNormalCloud::Ptr cloud ,Eigen::VectorXf &params);
};
#endif