#include "point_cloud_plane_segment.h"

void PointCloudPlaneSegment::PlaneSegment(const PointCloudPlaneCurvesExtract *pcpce)
{
  for (int i = 0; i < _planeRings; i++)
  {
    for (int j = 0; j < pcpce->mSizeCurvesVector[i].size(); j++)
    {
      uint AngleGridId = pcpce->mSizeCurvesVector[i][j].rgba;  // pointCloud 's rgba -> AngleGridId
      if (AngleGridId >= _numOfAngleGrid)
      {
        continue;
      }
      mPlane[i][AngleGridId].Points.push_back(pcpce->mSizeCurvesVector[i][j]);
      mPlane[i][AngleGridId].radiusEdge[0] = i > 0 ? pcpce->mSentorMeanRadius[i - 1][AngleGridId] : 0;
      mPlane[i][AngleGridId].radiusEdge[1] = pcpce->mSentorMeanRadius[i][AngleGridId];
      mPlane[i][AngleGridId].conf = mPlane[i][AngleGridId].Points.size();
    }
  }
  for (int i = 0; i < _planeRings; i++)
  {
    for (int j = 0; j < _numOfAngleGrid; j++)
    {
      float smooth = 0;
      float center[3] = { 0 };
      if (mPlane[i][j].conf < 3)
      {
        mPlane[i][j].smooth = -1;
        mPlane[i][j].inclination = -1;
        mPlane[i][j].curveCenter[0] = center[0];
        mPlane[i][j].curveCenter[1] = center[1];
        mPlane[i][j].curveCenter[2] = center[2];
        continue;
      }
      else
      {
        for (int k = 0; k < mPlane[i][j].Points.size() - 1; k++)
        {
          float rk = mPlane[i][j].Points[k].curvature;
          float rk1 = mPlane[i][j].Points[k + 1].curvature;

          float x = mPlane[i][j].Points[k].x;
          float y = mPlane[i][j].Points[k].y;
          float z = mPlane[i][j].Points[k].z;

          center[0] = (center[0] * k + x) / (k + 1);
          center[1] = (center[1] * k + y) / (k + 1);
          center[2] = (center[2] * k + z) / (k + 1);

          smooth = (smooth * k + fabs(rk - rk1)) / (k + 1);
        }
        mPlane[i][j].curveCenter[0] = center[0];
        mPlane[i][j].curveCenter[1] = center[1];
        mPlane[i][j].curveCenter[2] = center[2];
      }
      if (i == 0)
      {
        mPlane[i][j].smooth = smooth;
        float c[3] = { 0 };
        mPlane[i][j].inclination = getInclination(mPlane[i][j].curveCenter, c);
      }
      else
      {
        mPlane[i][j].smooth = (smooth + mPlane[i - 1][j].smooth) * 0.5;
        mPlane[i][j].inclination = getInclination(mPlane[i][j].curveCenter, mPlane[i - 1][j].curveCenter);
      }
      // printf("point_cloud_plane_segment: mPlane[%d][%u]: conf %d smooth %f inclination %f radiusEdge[%f, %f]\n", i, j,
      //        mPlane[i][j].conf, mPlane[i][j].smooth, mPlane[i][j].inclination, mPlane[i][j].radiusEdge[0],
      //        mPlane[i][j].radiusEdge[1]);
    }
  }
}