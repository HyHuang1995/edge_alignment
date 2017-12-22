## Distance Transform Based Image Alignment
Implementation of edge alignment for camera motion estimation, mainly based on the following paper.
Tested on **Ubuntu 16.04 Desktop** with **OpenCV 3.3**, **Eigen3**, **Ceres** and **Sophus(a621ff2)**
```
@inproceedings{kuse2016robust,
  title={Robust camera motion estimation using direct edge alignment and sub-gradient method},
  author={Kuse, Manohar and Shen, Shaojie},
  booktitle={Robotics and Automation (ICRA), 2016 IEEE International Conference on},
  pages={573--579},
  year={2016},
  organization={IEEE}
}
```

## Result
![](https://photos.app.goo.gl/GCFOCtSNojESh2qY2)
1.edge from source image. 2.distance transform of target image
3.edge reprojection to target image 4.optimization result