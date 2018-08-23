#include "point_cloud_plane_segment.h"
#include <boost/date_time/posix_time/posix_time.hpp>

int main(int argc, char **argv)
{
  pcl::PCLPointCloud2 cloud2;
  PointXYZRGBNormalCloud cloud;
  std::string filename = "../16line.pcd";

  pcl::io::loadPCDFile(filename, cloud2);
  pcl::fromPCLPointCloud2(cloud2, cloud);

  clock_t start, finish;
  start = clock();

  PointCloudPlaneCurvesExtract *pcpce = new PointCloudPlaneCurvesExtract();
  pcpce->SearchCurves(cloud);
  PointCloudPlaneSegment *pcps = new PointCloudPlaneSegment();
  pcps->PlaneSegment(pcpce);

  finish = clock();
  float deltTime = (finish - start);
  deltTime /= CLOCKS_PER_SEC;
  deltTime *=1000;

  printf("exc time %f ms\n",deltTime);
}
