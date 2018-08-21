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
      mPlane[i][AngleGridId].oneLinePoints.push_back(pcpce->mSizeCurvesVector[i][j]);
      mPlane[i][AngleGridId].radiusEdge[0] = i > 0 ? pcpce->mSentorMeanRadius[i - 1][AngleGridId] : 0;
      mPlane[i][AngleGridId].radiusEdge[1] = pcpce->mSentorMeanRadius[i][AngleGridId];
      mPlane[i][AngleGridId].conf[0] = mPlane[i][AngleGridId].oneLinePoints.size();
      if (i == 0)
        mPlane[i][AngleGridId].conf[1] = 1000;
      else
        mPlane[i][AngleGridId].conf[1] = mPlane[i - 1][AngleGridId].conf[0];
    }
  }

  for (int i = 0; i < _planeRings; i++)
  {
    for (int j = 0; j < _numOfAngleGrid; j++)
    {
      float smooth = 0;
      mPlane[i][j].twoLinePoints.insert(mPlane[i][j].twoLinePoints.end(), mPlane[i][j].oneLinePoints.begin(),
                                        mPlane[i][j].oneLinePoints.end());
      if (i != 0)
      {
        mPlane[i][j].twoLinePoints.insert(mPlane[i][j].twoLinePoints.end(), mPlane[i - 1][j].oneLinePoints.begin(),
                                          mPlane[i - 1][j].oneLinePoints.end());
      }
      mPlane[i][j].planeParams = Eigen::VectorXf::Zero(4, 1);

      if (mPlane[i][j].conf[0] < 3 || mPlane[i][j].conf[1] < 3) //不是平面
      {
        mPlane[i][j].isGround = false;
        mPlane[i][j].smooth = -1;
        continue;
      }
      else
      {
        mPlane[i][j].isGround = true;
        float meanHeight = 0;
        for (int k = 0; k < mPlane[i][j].oneLinePoints.size() - 1; k++)
        {
          float rk = mPlane[i][j].oneLinePoints[k].curvature;
          float rk1 = mPlane[i][j].oneLinePoints[k + 1].curvature;

          float x = mPlane[i][j].oneLinePoints[k].x;
          float y = mPlane[i][j].oneLinePoints[k].y;
          float z = mPlane[i][j].oneLinePoints[k].z;

          meanHeight = (meanHeight * k + z) / (k + 1);

          smooth = (smooth * k + fabs(rk - rk1)) / (k + 1);
        }

        PointXYZRGBNormalCloud::Ptr cloud(new PointXYZRGBNormalCloud(mPlane[i][j].twoLinePoints));
        getPlaneRansac(cloud, mPlane[i][j].planeParams);
      }
      if (i == 0)
      {
        mPlane[i][j].smooth = smooth;
      }
      else
      {
        mPlane[i][j].smooth = (smooth + mPlane[i - 1][j].smooth) * 0.5;
      }
      // // printf("point_cloud_plane_segment: mPlane[%d][%u]: conf %d smooth %f inclination %f radiusEdge[%f, %f]\n", i,
      // // j,
      // //        mPlane[i][j].conf, mPlane[i][j].smooth, mPlane[i][j].inclination, mPlane[i][j].radiusEdge[0],
      // //        mPlane[i][j].radiusEdge[1]);
      // printf("params: 1 %f 2 %f 3 %f 4 %f\n", mPlane[i][j].planeParams[0], mPlane[i][j].planeParams[1],
      //        mPlane[i][j].planeParams[2], mPlane[i][j].planeParams[3]);
    }
  }
}

void PointCloudPlaneSegment::getPlaneRansac(PointXYZRGBNormalCloud::Ptr cloud, Eigen::VectorXf &params)
{
  // 保存局内点索引
  std::vector<int> inliers;
  pcl::SampleConsensusModelPlane<PointXYZRGBNormal>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<PointXYZRGBNormal>(cloud));
  pcl::RandomSampleConsensus<PointXYZRGBNormal> ransac(model_p);
  ransac.setDistanceThreshold(0.02);
  ransac.computeModel();
  ransac.getInliers(inliers);

  pcl::PointCloud<PointXYZRGBNormal>::Ptr final(new pcl::PointCloud<PointXYZRGBNormal>);
  final->resize(inliers.size());

  params = Eigen::VectorXf::Zero(4, 1);
  ransac.getModelCoefficients(params);
}