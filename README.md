基于线束的地面检测算法(A very efficient ground detection algorithm in 16 lines pointcloud)
---------------------
code文件共4个：

```
地面检测算法流程：
１．　点云预处理，提取候选线束
２．　"３步"线束滤波，提取地面线束
３．　地面分割并计算描述子
```
## **I. 点云预处理**

１．点云ringID计算及地面线束截取
```
参数：
#define _lowerBound -15　　
#define _upperBound 15　　//Lidar垂直方向张角　(-15° ~15 °)
#define _numOfRings 16　　//基于Lidar线数
#define _horizontalAngleResolution 0.4 　//水平角分辨率

计算点云ringID：
float angle = std::atan(point.z / sqrt(point.x * point.x + point.y * point.y)) / (1.0f * M_PI) * 180.0f;
int64 GetScanringID(const float &angle)
{
  return static_cast<int64>((angle - _lowerBound)
　　　 /(1.0f * (_upperBound - _lowerBound)) * (_numOfRings - 1) + 0.5);
}

设定地面范围为半径30m的圆，对应选择线束0~5作为地面线束。
```
２．基准圆设定
```
参数：
#define _basicRadius 6.9 　//基准圆半径
#define _defaultCurveSize 2000　//每条线束默认点数，超过会自动扩张；设置该参数仅仅为了遍历效率
#define _curveSizeThreshold 50   　// Curve点数阈值：大于阈值认为是待选曲线

基准圆的半径设定为：第一条线束(半径最小)的半径，根据目前激光的安装方式，半径为6.9ｍ
```
原始点云：
![图片](https://github.com/halostorm/Ground_Detection/blob/master/images/s1.jpg)
## **II. 第一步：密度滤波**

```
参数：
// Density Filter Params
//密度滤波滤波的作用是去除稀疏的障碍物和线束跳变区域
#define _srcLenThreshold 0.2　//设定的弧长阈值
#define _arcNumThreshold 7　//在设定弧长下遍历过的点数　的阈值
```
```
算法细节：
１．对每一条线束，从头遍历并累计弧长，每当弧长达到设定的弧长阈值_srcLenThreshold
　（每新加入一个点ｉ，计算弧长时，真实的弧长增量需要通过该点的radius缩放到基准圆，成为等尺度的弧长）
２．计算遍历过的点数Ｎ，Ｎ>=_arcNumThreshold，则该部分弧保留；反之，滤除点数少的弧.

```

```
函数：
void PointCloudPlaneCurvesExtract::CurveDensityFilter(
    const PointXYZRGBNormalCloud &Curve, 
    const int64 ringID, 
    const Uint64Vector &curveId,
    PointXYZRGBNormalCloud &outCurve
);
```

## **II. 第二步：相对半径滤波**
```
参数：
// Radius Filter Params
//相对半径滤波的作用是去除致密的障碍物，需要讲每条线束划分为Ｎ和扇形区域
#define _AngleGridResolution 2.0　//　扇形角分辨率
#define _numOfAngleGrid 180  //   每条弧扇形区域个数　＝　360 / _AngleGridResolution
#define _radiusScaleThreshold 0.2　//相对半径阈值
```
```
算法细节：
１．对每一条线束，
　　(1)从头遍历记录： 
　　　　每个Grid的点
　　　　每个点对应的角度索引（保存在point.rgba 中）
　　　　每个点到水平圆点的距离（保存在point.curvature 中）
　　//注意：这里采用点和Grid双相索引的策略，避免重复查找遍历
　　(2) 计算每个Grid的半径　＝　所有点到水平原点的距离的均值，若没有点，Grid的半径记为下一条线相同角度Grid的半径
２．对一条线束m的一个Gridmi 半径为Rmi, 该线束的理想半径为Rm,如果：
　　      Rmi - Rm-1i >= (Rm - Rm-1)*_radiusScaleThreshold
         则保留Gridmi,　反之清除Grimi的点。
 //思想：致密障碍物上的线束水平半径相距较近，或部分消失，可以据此滤除这些障碍物
```
```
函数：
void *CurvesRadiusFilter(
    PointXYZRGBNormalCloud *CurvesVector, 
    const Uint64Vector *CurvesId
);

```
密度滤波及相对半径滤波之后：
![图片](https://github.com/halostorm/Ground_Detection/blob/master/images/s2.jpg)

## **II. 第三步：尺度滤波**
```
参数：
// Size Filter Params
//找到线束的"缺口"作为分割，滤除过短的线段
#define _breakingDistanceThreshold 0.2　//缺口阈值
#define _breakingSizeThreshold 30　//线段长度阈值（点数）
```
```
算法细节：
１．对每一条线束，
　　(1)从头遍历，查找缺口
　　(2)计算两个缺口之间的点数，太短则滤除
```
```
函数：
void CurveSizeFilter(
    const PointXYZRGBNormalCloud &Curve, 
    const int64 ringID, 
    const Uint64Vector &curveId,
    PointXYZRGBNormalCloud &outCurve
);
```
尺度滤波之后：
![图片](https://github.com/halostorm/Ground_Detection/blob/master/images/s3.jpg)

## **III. 地面分割/计算描述子**

```
思想：
　将地面以2.0°的角分辨率，共６条线束，分为6*180块扇形区域，每块区域根据点数判定是否是地面，是地面则计算描述子
　//注意：这里地面分割方式与密度滤波中的相同，可以省略大量的重复计算，分割地面时进行Grid索引即可
分割如下图：
```

![图片](https://github.com/halostorm/Ground_Detection/blob/master/images/s4.jpg)
```
参数：
// Plane Segment Params
#define _isGroundPointNumThreshold 3 //扇形区域的点数阈值：需要扇形的两端边缘曲线的点数均满足阈值条件
#define _ransacDistanceThreshold 0.03　//扇形区域平面拟合　RANSAC算法距离误差阈值
```

```
描述子：
struct Sentor
{
  bool isGround;　//是否地面
  int conf[2]; 　// conf[0] ：后边缘曲线的点数　；　conf[1] ：前边缘曲线的点数　
  float smooth; //平滑度
  float radiusEdge[2];　//radiusEdge[0] ：后边缘曲线的半径　；radiusEdge[1] ：前边缘曲线的半径　
  Eigen::VectorXf planeParams;　//该扇形区域的平面参数
  PointXYZRGBNormalCloud oneLinePoints;　//后边缘曲线的点集
  PointXYZRGBNormalCloud twoLinePoints;　//前后边缘曲线的点集和
};
```

```
具体描述子的计算：
函数：
void PlaneSegment(
    const PointCloudPlaneCurvesExtract *pcpce
);
```





