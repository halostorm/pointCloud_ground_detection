#include "point_cloud_plane_curves_extract.h"

#define _sectorNum 180
#define _sentorAngleResolution 2

struct Sentor
{
  float conf;
  float edge[2];
};

typedef std::vector<Sentor> sensorVector;

class PointCloudPlaneSegment
{
public:
  Sentor mPlane[_planeRings][_numOfAngleGrid];
  void PlaneSegment(const PointCloudPlaneCurvesExtract *pcpce);
};
