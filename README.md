# SP-SLAM
**Authors:** 
Zhang, Xiaoyu; Wang, Wei; Qi, Xianyu; Liao, Ziwei; Wei, Ran. 

For indoor environments, planes are predominant features that are less affected by measurement noise. We propose a novel point-plane SLAM system using RGB-D cameras. First, We extract feature points from RGB images and planes from depth images. Then plane correspondences in the global map can be found using their contours. Considering the limited size of real planes, we exploit constraints of plane edges. In general, a plane edge is an intersecting line of two perpendicular planes. Therefore, instead of line-based constraints, we calculate and generate supposed perpendicular planes from edge lines, resulting in more plane observations and constraints to reduce estimation errors. To exploit the orthogonal structure in indoor environments, we also add structural (parallel or perpendicular) constraints of planes. Finally, we construct a factor graph using all of these features. The cost functions are minimized to estimate camera poses and global map.

### Related Publications:
For more details, please read our paper [Point-Plane SLAM Using Supposed Planes for Indoor Environments](https://www.mdpi.com/1424-8220/19/17/3795)

A Chinese article introducing this work: https://zhuanlan.zhihu.com/p/143204294

If you have some questions to discuss, you could contact me by e-mail (zhang_xy@buaa.edu.cn) or zhihu (https://www.zhihu.com/people/fishmarch) for fast reply.


If you use SP-SLAM in an academic work, please cite:

     @article{Zhang2019,        
        title = {{Point-Plane SLAM Using Supposed Planes for Indoor Environments}},
        author = {Zhang, Xiaoyu and Wang, Wei and Qi, Xianyu and Liao, Ziwei and Wei, Ran},
        journal = {Sensors},
        number = {17},
        pages = {3795},
        volume = {19},
        year = {2019}
      }

# SP-SLAM System
We build our SLAM system based on ORB-SLAM2 RGBD version. For some prerequisites, you could read their page, [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)

We only add PCL library to deal with point cloud, we tested on PCL-1.80. SP-SLAM only supports RGB-D data. We have not removed those unrelevent components coming from ORB-SLAM2

     

