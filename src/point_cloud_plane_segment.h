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

  void getPlaneRansac(PointXYZRGBNormalCloud::Ptr cloud ,Eigen::VectorXf &params);
};
#endif