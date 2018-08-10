#include "point_cloud_plane_curves_extract.h"

PointXYZRGBNormalCloud *PointCloudPlaneCurvesExtract::SearchCurves(const PointXYZRGBNormalCloud &PointCloud)
{
  for (uint64 i = 0; i < PointCloud.size(); i++)
  {
    PointXYZRGBNormal point = PointCloud[i];
    float angle = std::atan(point.z * InverseSqrt(point.x * point.x + point.y * point.y)) / (1.0f * M_PI) * 180.0f;
    int64 ringID = GetScanringID(angle);
    if (ringID < _numOfRings && ringID >= 0)
    {
      mCurvesVector[ringID].push_back(point);
      mCurvesId[ringID].push_back(i);
    }
  }

  for (int i = 0; i <= _planeRings; i++)
  {
    mScanringRadius[i] = GetScanringRadius(i);
  }

  // DensityFilter
  for (int i = 0; i <= _planeRings; i++)
  {
    // ROS_INFO("PointCloudPlaneCurvesExtract: mCurvesVector[%d] Size : %ld", i, mCurvesVector[i].size());
    if (mCurvesVector[i].size() > _windowsize)
    {
      mDensityCurvesVector[i] = CurveDensityFilter(mCurvesVector[i], i, mCurvesId[i]);
    }
    // ROS_INFO("PointCloudPlaneCurvesExtract: mDensityCurvesVector[%d] Size : %ld", i, mDensityCurvesVector[i].size());
  }

  // RadiusFilter
  CurvesRadiusFilter(mDensityCurvesVector, mDensityCurvesId);

  // SizeFilter
  for (int i = 0; i <= _planeRings; i++)
  {
    // ROS_INFO("PointCloudPlaneCurvesExtract: mRadiusCurvesVector[%d] Size : %ld", i, mRadiusCurvesVector[i].size());
    if (mRadiusCurvesVector[i].size() > _windowsize)
    {
      mSizeCurvesVector[i] = CurveSizeFilter(mRadiusCurvesVector[i], i, mRadiusCurvesId[i]);
    }
    // ROS_INFO("PointCloudPlaneCurvesExtract: mSizeCurvesVector[%d] Size : %ld", i, mSizeCurvesVector[i].size());
  }
  return mSizeCurvesVector;
}

PointXYZRGBNormalCloud PointCloudPlaneCurvesExtract::CurveDensityFilter(const PointXYZRGBNormalCloud &Curve, int64 ringID,
                                                                 Uint64Vector &curveId)
{
  PointXYZRGBNormalCloud filterCurve;
  float TrueRadius = mScanringRadius[ringID];
  // ROS_INFO("PointCloudPlaneCurvesExtract: id: %ld, TrueRadius: %f", ringID, TrueRadius);
  if (TrueRadius < 0)
  {
    return filterCurve;
  }
  float arcLen = 0;
  float radiusMean = 0;
  float radius = TrueRadius;
  float radius_1 = TrueRadius;
  int64 arcNum = 0;
  for (int i = 1; i < Curve.size(); i += 1)
  {
    float dx = Curve[i].x - Curve[i - 1].x;
    float dy = Curve[i].y - Curve[i - 1].y;
    float dz = Curve[i].z - Curve[i - 1].z;
    arcNum++;
    radius = 1 / InverseSqrt(Curve[i].x * Curve[i].x + Curve[i].y * Curve[i].y);
    radiusMean += radius;
    arcLen += (1 / InverseSqrt(dx * dx + dy * dy + dz * dz)) * (2 * _basicRadius / (radius + radius_1));
    radius_1 = radius;
    if (arcLen >= _srcLenThreshold)
    {
      radiusMean /= arcNum;
      // if (false)
      //   ROS_INFO("PointCloudPlaneCurvesExtract: radiusMean: %f , TrueRadius: %f, arcNum: %ld, arcLen: %f", radiusMean,
      //            TrueRadius, arcNum, arcLen);
      if (arcNum > _arcNumThreshold) // && radiusMean > _radiusScaleThreshold * TrueRadius)
      {
        for (int k = i - arcNum + 1; k <= i; k++)
        {
          mDensityCurvesId[ringID].push_back(curveId[k]);
          filterCurve.push_back(Curve[k]);
        }
      }
      arcLen = 0;
      arcNum = 0;
      radiusMean = 0;
    }
  }
  return filterCurve;
}

PointXYZRGBNormalCloud PointCloudPlaneCurvesExtract::CurveSizeFilter(const PointXYZRGBNormalCloud &Curve, int64 ringID,
                                                              Uint64Vector &curveId)
{
  PointXYZRGBNormalCloud filterCurve;
  float TrueRadius = mScanringRadius[ringID];
  float ratio = _basicRadius / TrueRadius;
  if (TrueRadius < 0 || Curve.size() < 1)
  {
    return filterCurve;
  }
  int LabelArray[Curve.size()];
  int64 begin = 0;
  int64 end = 0;
  bool left = false;
  bool right = false;
  float meanZ = 0;
  // ROS_INFO("PointCloudPlaneCurvesExtract: mDensityCurves[%ld] mean height: %f", ringID, mHeightMean[ringID]);
  for (int i = 1; i < Curve.size(); i += 1)
  {
    float dx = Curve[i].x - Curve[i - 1].x;
    float dy = Curve[i].y - Curve[i - 1].y;
    float dz = Curve[i].z - Curve[i - 1].z;
    meanZ += dz;

    float dis = (1 / InverseSqrt(dx * dx + dy * dy + dz * dz)) * (ratio);

    if (dis > _breakingDistanceThreshold)
    {
      end = i;
      meanZ /= (end - begin);
      // ROS_INFO("PointCloudPlaneCurvesExtract: delete: mean height: %f", mHeightMean[ringID]);
      if (end - begin < _breakingSizeThreshold)
      {
        for (int k = begin; k < end; k++)
        {
          LabelArray[k] = -1;
        }
        // ROS_INFO("PointCloudPlaneCurvesExtract: mSizeCurvesId delete size: %ld, dis: %f", end - begin, dis);
      }
      begin = end;
    }
  }
  for (int i = 0; i < Curve.size(); i += 1)
  {
    if (LabelArray[i] != -1)
    {
      mSizeCurvesId[ringID].push_back(curveId[i]);
      filterCurve.push_back(Curve[i]);
    }
  }
  // ROS_INFO("PointCloudPlaneCurvesExtract: mSizeCurvesId[%ld] size: %ld", ringID, filterCurve.size());
  return filterCurve;
}

PointXYZRGBNormalCloud *PointCloudPlaneCurvesExtract::CurvesRadiusFilter(const PointXYZRGBNormalCloud *CurvesVector,
                                                                  Uint64Vector *CurvesId)
{
  for (int i = 0; i <= _planeRings; i++)
  {
    for (int j = 0; j < CurvesVector[i].size(); j++)
    {
      float x = CurvesVector[i][j].x;
      float y = CurvesVector[i][j].y;
      int atanAngle = (int)(((atan2f(y, x) / M_PI * 180 + 180) / _horizontalAngleResolution));
      if (atanAngle < 0 || atanAngle >= _numOfAngleGrid)
      {
        continue;
      }
      mAnglePointId[i][atanAngle].push_back(CurvesId[i][j]);
      mAnglePointVector[i][atanAngle].push_back(CurvesVector[i][j]);

      float radius = 1 / InverseSqrt(x * x + y * y);
      mAngleMean[i][atanAngle] = (mAngleMean[i][atanAngle] * (mAnglePointId[i][atanAngle].size() - 1) + radius) /
                                 mAnglePointId[i][atanAngle].size();
    }
  }

  for (int j = 0; j < _planeRings; j++)
  {
    for (int i = 0; i < _numOfAngleGrid; i++)
    {
      if (fabs(mAngleMean[j][i] - mAngleMean[j + 1][i]) > 0.5 * fabs(mScanringRadius[j + 1] - mScanringRadius[j]))
      {
        mRadiusCurvesId[j + 1].insert(mRadiusCurvesId[j + 1].end(), mAnglePointId[j + 1][i].begin(),
                                      mAnglePointId[j + 1][i].end());
        mRadiusCurvesVector[j + 1].insert(mRadiusCurvesVector[j + 1].end(), mAnglePointVector[j + 1][i].begin(),
                                          mAnglePointVector[j + 1][i].end());
        if (j == 0)
        {
          mRadiusCurvesId[0].insert(mRadiusCurvesId[0].end(), mAnglePointId[0][i].begin(), mAnglePointId[0][i].end());
          mRadiusCurvesVector[0].insert(mRadiusCurvesVector[0].end(), mAnglePointVector[0][i].begin(),
                                        mAnglePointVector[0][i].end());
        }
      }
    }
  }
}

int64 PointCloudPlaneCurvesExtract::GetScanringID(const float &angle)
{
  return static_cast<int64>((angle - _lowerBound) / (1.0f * (_upperBound - _lowerBound)) * (_numOfRings - 1) + 0.5);
}

float PointCloudPlaneCurvesExtract::GetScanringRadius(const int64 ID)
{
  float TrueRadius = -1;
  float angle = (_upperBound - _lowerBound) * 1.0 / _numOfRings * ID + _lowerBound;
  if (angle < -2)
  {
    TrueRadius = _basicRadius * fabs(tan(_lowerBound * 1.0 / 180 * M_PI)) * fabs(tan((90 + angle) * 1.0 / 180 * M_PI));
  }
  return TrueRadius;
}