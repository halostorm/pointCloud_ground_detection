#include "point_cloud_plane_segment.h"

void PointCloudPlaneSegment::PlaneSegment(const PointCloudPlaneCurvesExtract *pcpce)
{
  for (int i = 0; i < _planeRings; i++)
  {
    for (int j = 0; j < pcpce->mSizeCurvesVector[i].size(); j++)
    {
      uint AngleGridId = pcpce->mSizeCurvesVector[i][j].rgba; // pointCloud 's rgba -> AngleGridId
      if (AngleGridId >= _numOfAngleGrid)
      {
        continue;
      }
      mPlane[i][AngleGridId].Points.push_back(pcpce->mSizeCurvesVector[i][j]);
      mPlane[i][AngleGridId].edge[0] = i > 0 ? pcpce->mSentorMeanRadius[i - 1][AngleGridId] : 0;
      mPlane[i][AngleGridId].edge[1] = pcpce->mSentorMeanRadius[i][AngleGridId];
      mPlane[i][AngleGridId].conf = mPlane[i][AngleGridId].Points.size() / 20.0;
    }
  }
  for (int i = 0; i < _planeRings; i++)
  {
    for (int j = 0; j < _numOfAngleGrid; j++)
    {
      float smooth = 0;
      if (mPlane[i][j].Points.size() < 3)
        continue;
      else
      {
        for (int k = 0; k < mPlane[i][j].Points.size() - 1; k++)
        {
          float rk = mPlane[i][j].Points[k].curvature;
          float rk1 = mPlane[i][j].Points[k + 1].curvature;
          smooth = (smooth * k + fabs(rk - rk1)) / (k + 1);
        }
      }
      if (i == 0)
        mPlane[i][j].smooth = smooth;
      else
      {
        mPlane[i][j].smooth = (smooth + mPlane[i - 1][j].smooth) * 0.5;
      }
    }
  }
}