# ​摘要总结：

## 现存的视觉雷达融合框架中存在两个问题：

1. **3D雷达点和2D稀疏特征点的数据关联问题**(例如稀疏2D点的深度值)，因为雷达点通过标定的外参投影到像素平面上时，不一定会投影到特征点的位置，又由于特征点稀疏，投影到的概率就更低啦，由于特征点大部分是角点和边缘点（也就是说这些点在路标中位于其边界位置），很容易因为几个像素位置的偏差使得雷达点和特征点不在同一个平面上，因此不能轻易用雷达点的三维坐标取代其投影后附近特征点的三维坐标。

2. **通过sweep（scan）-to-map的优化方式导致垂直方向出现明显的漂移**，大部分雷达里程计的通病，对Z轴约束较差，关于scan和sweep的定义看个人习惯，这里就是当频率固定时，每一时刻的点云。

## 本文的解决思路：

**提出了一个adaptive（自适应）的sweep-to-map的雷达里程计，并将其和半直接法视觉里程计进行融合，本文的一些trick：**

- 数据关联的trick:

  - 将雷达点通过外参投影到像素平面上，然后挑选那些灰度变化比较大的雷达点作为候选光流点。
  原文如下：\
  In this paper, we present SDV-LOAM which incorporates a semi-direct visual odometry and an adaptive sweep-to-map LiDAR odometry to effectively avoid the above mentioned errors and in turn achieve high tracking accuracy.

   - 由于光流法的弊端（相邻两帧的运动不能太大），本文在进行帧间匹配光流的时候，将host帧（光流点的起始帧）的光流传播到离当前帧最近的关键帧以减小尺度差异，这里刚开始还很懵，为什么host frame和current frame之间还有一个intermediate keyframe，后来想了一下先骗一下自己，光流传播过程中从某一帧开始一直传播那一帧（host frame）的光流，而不是两帧之间重新计算光流，所以就有可能存在host frame和current frame之间存在keyframe，原文说是intermediate，难道还能产生好几个关键帧吗，不理解.原文如下：\
    To avoid the problem of large scale difference between matching frames in the VO, we design a novel point matching with propagation method to propagate points of a host frame to an intermediate keyframe which is closer to the current frame to reduce scale differences.
- 垂直方向尺度漂移的trick

   - 在雷达里程计中一个根据垂直方向几何约束的约束程度自适应的优化位姿自由度的方法，约束不足的时候只更新水平的三个自由度（yaw，x，y），约束够的时候更新6DoF，这点和Lego_LOAM有共同点，但legoloam是分开约束，如果提取不到地面点，legoloam不知道优不优化剩余三个自由度。原文如下：\
    To reduce the pose estimation drifts in the vertical direction, our LiDAR module employs an adaptive sweep-to-map optimization method which automatically choose to optimize 3 horizontal DOF or 6 full DOF pose according to the richness of geometric constraints
in the vertical direction.
- 雷达和相机频率不一致的trick

  - 这个可以关注一下，如何实现不同频率的传感器在软件上实现相同的频率，不知道算不算时间同步的一种方法,原文如下：\
  In addition, we propose a novel sweep reconstruction method which can increase the input frequency of LiDAR point clouds to the same frequency as the camera images, and in turn yield a high frequency output of the LiDAR odometry in theory.
## 引言部分总结

1. 现存的松耦合LV（Lidar-Visual）里程计如DEMO、LIMO、DVL-SLAM只利用雷达点为视觉里程计提供深度值，却忽略了雷达里程计的位姿估计和三维重建的高精确性。

2. 紧耦合只谈到Zhangji大佬的V-Loam算法，概括为高频的视觉里程计拥有雷达深度值（因为单目没有尺度信息），低频的雷达里程计拥有视觉的位姿进行运动畸变的修正。因为V-Loam的视觉里程计采用特征点法，因此视觉特征点的深度恢复并不是准确的，存在插值误差（也就是说雷达点和视觉特征点真实的三维路标之间不一定能通过插值解决，具体还不清楚）。作者认为利用直接法可以避免这个问题，但是直接法受光照变化和相机内参的误差影响较大，相对于特征点法容易陷入局部极小值。

**作者的思路（基于SVO）采用半直接法，利用直接法进行相邻两帧的数据关联，然后最小化重投影误差**

Different from V-LOAM, our visual module employs a semi-direct method to combine the success-factors of the feature-based method (re-projection error minimization for accurate pose estimation) with the direct method (eliminating 3D-2D depth association errors).

**作者对半直接法的总结：**

1. 将雷达点通过外参映射到像素平面上之后，每个雷达点就获得了像素坐标，然后该像素坐标进行分析，如果该像素坐标的梯度较大（这里应该是u和方向均梯度较大），则跟踪该像素位置，然后可能利用四叉树的方法均为分布光流点。

2. 由于雷达里程计频率较低，光流跟踪的频率较小（对直接法非常不利），作者在上一个最近的关键帧中寻找当前帧对应的光流点，这里对a host frame不是很理解（难道是上一帧吗），将host frame的光流点传播到与当前帧最近的关键帧上（也就是说关键帧上的光流点是host frame的光流点，而不是雷达点投影到关键帧上的光流点），通过发现当前帧和相邻关键帧之间的光流关系得到当前帧和host frame的光流关系。（这里很显然想求出当前帧和host frame的位姿变化）\
which propagates points of a host frame to an intermediate keyframe which is closer to the current frame and then obtain the matching points betweencurrent frame and host frame by finding the corresponding pixels of the intermediate keyframe on current frame

3. 当当前帧跟踪到的光流点较少时，作者额外添加了一些非雷达投影的高像素梯度的点。（可能因为此刻相机帧没有对应的雷达帧，如果不添加的话，位姿求解可能有问题，如果有对应的雷达点，重新投影再筛选不就行了。）

**作者指出室外大多数平面的法向量都是垂直于地面，因此雷达点在垂直方向提供了一个较弱的约束。**

LiDAR points tend to provide weak constraints in the vertical direction as majority of
surfaces whose normal vectors pointing vertically in the outdoor scenes are the ground.

**在CT-ICP中提出了一种方法改善这个问题，by introducing a weighting strategy to favor planar neighborhoods（该方法最早在IMLS-SLAM中提出），但是对于垂直方向约束不足的场景，该方法的效果并不显著，此外在此场景中由于（roll pitch z）的约束不够对其优化还会导致其他三个自由度退化，因此作者直接不优化垂直方向相关的三个自由度，根据场景结构选择性优化。**

到这里我还是很好奇作者的sweep reconstruction module，如何提高雷达点的频率

>相关文献的部分跳过，感兴趣的可以看看作者对相关文献的点评（Lego-Loam排除了大量不能提供足够几何约束的点，如何针对性的约束位姿，找到那些信息约束了哪些位姿（反过来亦如此），还有一些outliers（如动态物体的点和测量值有误的点（测量误差）），一些粗暴的方式容易将能提供约束的点移除掉导致鲁棒性较差，本人用legoloam跑kitti的时候感觉legoloam还是很强的\
>SuMa虽然融合了局部点云生成了基于面元的地图实现全局scan2model的匹配，但这种优化方式的精度不一定比Loam系列好（基于点线约束和基于点面约束）\
>Floam直接去除sweep2map的优化方式，仅采用帧间约束
>ISC在Floam的基础山利用点云强度信息的intensity scan context方法实现闭环检测\
>MULLS利用了点云更多的特征(e.g., facade, beam, pillar and ground)但是不好设置不同特征引起残差的权重，我觉得这个问题多传感器融合都应该考虑\
>这些基于各种ICP的方法都容易有一个问题，垂直方向的约束不够\
>IMLS-SLAM在帧内进行连续的位姿估计，在帧间进行不连续的位姿估计（这就能增强快速运动的鲁棒性吗，原因是什么）\
>LOL利用Segment Matching algorithm（需要GPU）在生成的3D点云和离线点云地图（实际中不易获取）上进行位置时别(place recognition)(这个东西是不是也可以称为重定位)\
LIO-SAM没有在公开数据集上进行比较，并且作者表示该算法不易加入其他传感器（说是受computational burden的限制，不太懂）\
DEMO利用3D点云为单目2D图像特征点提供深度信息（深度关联技术（这也是个不错的研究方向））。LIMO在DEMO的基础上利用语义信息识别并去除动态物体，同时添加了闭环检测模块。\
有些算法尝试利用3D点去增强2D的线特征。（作者表明3D点和2D点和线难以一一对应），因此引入了插值误差。\
DVL-SLAM利用雷达点投影后的直接法，但光度误差的效果一般不如重投影误差。\
V-Loam由于相机和雷达的频率不一致
）
## 方法探讨

现在我们对作者提出的方法进行探讨：
首先是一些数据的备注,作者雷达和相机的标定是离线的，在算法中保持不变，并对其了两个传感器的时间戳（时空标定）\
**设备：Velodyne VLP-16（10Hz）、FL3-FW-14S3M-C（60Hz）**\
在评估时，相机和雷达均为10Hz，并假设雷达和相机做好时空标定，雷达点云以完成运动畸变补偿。\
相机在t时刻获得一张图片$C_i$，在t-0.1s到t获得一帧点云（a sweep）并记为S\
在视觉模块，定义一个frame包含一张图片和相关的点云。由于相机的频率高于雷达，大多数frame只包含一张图片（空点云），只有那些包含点云的frame才会被选为keyframe（关键帧）\
将图片的像素坐标（像素点）分为深度点和非深度点，深度值通过雷达点获得
$$d_u = [T^c_lp^l]_z ,\; where \; [·]_z \; is \;the\; z\; coordinate\; of \;the\; element$$
用匀速运动模型补偿帧内点云运动畸变\
作者的视觉里程计是基于DSO进行修改的，雷达里程计是基于CT-ICP修改的，算法流程如下：
![pipeline](https://i-blog.csdnimg.cn/blog_migrate/4cb10ea93bfdb7933b6b144c430c25b6.png)​ 
### 整个算法分为视觉里程计和雷达里程计两部分:

>对于视觉里程计：首先对输入的点云进行预处理，利用地面分割算法获得粗糙的地面点云，然后用RANSAC算法获得较为精确的地面点云，如果地面点云的数量在当前帧点云总数量的占比大于一个阈值，如果地面点云占比低于阈值，再对非地面点云进行快速聚类判断不规则点云的占比数量，如果不规则点云所占整体点云的比例超过一定阈值，就选用雷达投影点和视觉特征点一起估计位姿，否则就只采用雷达投影点估计当前帧的位姿。（该过程只对带有雷达点云的一帧数据进行，对于不带有雷达点云的一帧数据，只用特征点估计位姿）。

>SVO在稀疏雷达点云中存在的问题：host frame的像素点和当前帧的像素点之间局部差异较大，因此采用最近关键帧和当前帧进行SVO的匹配模式，首先先利用host frame和当前帧进行光流匹配得到粗糙的匹配关系，然后再利用当前帧和最近关键帧之间的匹配关系精细化host frame对应路标点在当前帧的投影位置。

​
