#include "point_cloud_plane_segment.h"

void PointCloudPlaneSegment::PlaneSegment(const PointCloudPlaneCurvesExtract *pcpce)
{
  for (int i = 0; i < _planeRings; i++)
  {
    for (int j = 0; j < _numOfAngleGrid; j++)
    {
      Sentor st;
      st.edge[0] = pcpce->mSentorMeanRadius[i][j];
      if (i > 0)
        st.edge[1] = pcpce->mSentorMeanRadius[i - 1][j];
      else
        st.edge[1] = 0;
      float conf = pcpce->mSentorIds[i][j].size() / 20.0;
      st.conf = conf;
      mPlane[i][j] = st;
      // ROS_INFO("PointCloudPlaneSegment: mPlane[%d][%d]: edge 0 %f, edge 1 %f, conf %f", i, j,st.edge[0], st.edge[1],
      // st.conf);
    }
  }
}