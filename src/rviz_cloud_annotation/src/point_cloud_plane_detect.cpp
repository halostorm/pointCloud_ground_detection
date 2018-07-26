#include "point_cloud_plane_detect.h"

PointXYZRGBNormalCloud* PointCloudPlaneDetect::SearchCurves(PointXYZRGBNormalCloud& PointCloud)
{
  PointXYZRGBNormalCloud curves[_numofrings];
  Uint64Vector curveids[_numofrings];
  for (uint64 i = 0; i < PointCloud.size(); i++)
  {
    PointXYZRGBNormal point = PointCloud[i];
    float angle = std::atan(point.z * InverseSqrt(point.x * point.x + point.y * point.y)) / (1.0f * M_PI) * 180.0f;
    //ROS_INFO("PointCloudPlaneDetect: angle: %f", angle);
    int64 ringID = GetScanringID(angle);
    //ROS_INFO("PointCloudPlaneDetect: ringID: %ld", ringID);
    if (ringID < _numofrings && ringID >= 0)
    {
      curves[ringID].push_back(point);
      curveids[ringID].push_back(i);
    }
    else
    {
       ROS_INFO("PointCloudPlaneDetect: ringID: %ld", ringID);
    }
  }
  ROS_INFO("PointCloudPlaneDetect: OK1");
  for (int i = 0; i < _numofrings; i++)
  {
    ROS_INFO("PointCloudPlaneDetect: curves[%d] size: %ld", i, curves[i].size());
    if (curves[i].size() > _windowsize)
    {
      Curce2Plane(curves[i], i, curveids[i]);
    }
    ROS_INFO("PointCloudPlaneDetect: mCurvesId[%d] size: %ld", i, mCurvesId[i].size());
    ROS_INFO("PointCloudPlaneDetect: OK2");
  }

  return curves;
}

PointXYZRGBNormalCloud* PointCloudPlaneDetect::Curce2Plane(PointXYZRGBNormalCloud& Curve, int64 ringID,
                                                           Uint64Vector& curveid)
{
  float radius = GetScanringRadius(ringID);
  if (radius < 0)
  {
    return NULL;
  }
  float arcLen = 0;
  float disMean = 0;
  int64 arcNum = 0;
  for (int i = 1; i < Curve.size(); i += 1)
  {
    float dx = Curve[i].x - Curve[i - 1].x;
    float dy = Curve[i].y - Curve[i - 1].y;
    float dz = Curve[i].z - Curve[i - 1].z;

    arcNum++;
    float dis = 1 / InverseSqrt(Curve[i].x * Curve[i].x + Curve[i].y * Curve[i].y + Curve[i].z * Curve[i].z);
    disMean += dis;
    arcLen += (1 / InverseSqrt(dx * dx + dy * dy + dz * dz)) * (_radiusRadio / dis);

    // ROS_INFO("PointCloudPlaneDetect: disMean: %f , radius: %f, arcNum: %d, arcLen: %f", disMean, radius, arcNum,
    //         arcLen);

    if (arcLen >= _srcLenThreshold)
    {
      disMean /= arcNum;
      if (arcNum > 7 && disMean > 0.8 * radius)
      {
        for (int k = i - arcNum + 1; k <= i; k++)
        {
          mCurvesId[ringID].push_back(curveid[k]);
        }
      }
      arcLen = 0;
      arcNum = 0;
      disMean = 0;
    }
    // float varDis = 0;
    // float varZ = 0;
    // float listDis[_windowsize];
    // float listZ[_windowsize];
    // for (int j = i; j < i + _windowsize; j++)
    // {
    //   listDis[j - i] = 1 / InverseSqrt(Curve[j].x * Curve[j].x + Curve[j].y * Curve[j].y);
    //   listZ[j - i] = Curve[j].z;
    // }
    // varDis = getVar(listDis);
    // varZ = getVar(listZ);
    // if (varDis < 10E-8 && varZ < 10E-8)
    // {
    //   mCurvesId[ringID].push_back(curveid[i]);
    // }
  }
}

// PointXYZRGBNormalCloud* PointCloudPlaneDetect::Curce2Plane(PointXYZRGBNormalCloud& Curve, int64 ringID,
//                                                            Uint64Vector& curveid)
// {
//   for (int i = 0; i < Curve.size() - 1; i++)
//   {

//     mCurvesId[ringID].push_back(curveid[i]);
//   }
// }

int64 PointCloudPlaneDetect::GetScanringID(const float& angle)
{
  return static_cast<int64>((angle - _lowerBound) / (1.0f * (_upperBound - _lowerBound)) * (_numofrings - 1) + 0.5);
}

/*
typedef complex<int> POINT;
bool circleLeastFit(const std::vector<POINT>& points, double& center_x, double& center_y, double& radius)
{
  center_x = 0.0f;
  center_y = 0.0f;
  radius = 0.0f;
  if (points.size() < 3)
  {
    return false;
  }

  double sum_x = 0.0f, sum_y = 0.0f;
  double sum_x2 = 0.0f, sum_y2 = 0.0f;
  double sum_x3 = 0.0f, sum_y3 = 0.0f;
  double sum_xy = 0.0f, sum_x1y2 = 0.0f, sum_x2y1 = 0.0f;

  int N = points.size();
  for (int i = 0; i < N; i++)
  {
    double x = points[i].real();
    double y = points[i].imag();
    double x2 = x * x;
    double y2 = y * y;
    sum_x += x;
    sum_y += y;
    sum_x2 += x2;
    sum_y2 += y2;
    sum_x3 += x2 * x;
    sum_y3 += y2 * y;
    sum_xy += x * y;
    sum_x1y2 += x * y2;
    sum_x2y1 += x2 * y;
  }

  double C, D, E, G, H;
  double a, b, c;

  C = N * sum_x2 - sum_x * sum_x;
  D = N * sum_xy - sum_x * sum_y;
  E = N * sum_x3 + N * sum_x1y2 - (sum_x2 + sum_y2) * sum_x;
  G = N * sum_y2 - sum_y * sum_y;
  H = N * sum_x2y1 + N * sum_y3 - (sum_x2 + sum_y2) * sum_y;
  a = (H * D - E * G) / (C * G - D * D);
  b = (H * C - E * D) / (D * D - G * C);
  c = -(a * sum_x + b * sum_y + sum_x2 + sum_y2) / N;

  center_x = a / (-2);
  center_y = b / (-2);
  radius = sqrt(a * a + b * b - 4 * c) / 2;
  return true;
}
*/