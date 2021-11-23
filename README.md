## AutoL2LCalib: an Automated LiDAR-to-LiDAR Extrinsic Pose Calibrator
<p align = "center">
<img src= "https://github.com/JunhaAgu/AutoL2LCalib/blob/main/imgs/aligned_four_lidars.png" alt="aligned four lidars via the AutoL2LCalib" width="450" height="470">
</p> 

## 1. Descriptions
**Note:** This program is for a paper submission under review. After acceptance notifications, the paper information will be updated.

The **AutoL2LCalib** is a program to estimate an extrinsic relative pose of two 3D LiDARs by only using point clouds from the LiDARs without any aid of other sensors, such as cameras, inertial measurement units (IMUs), global positioning systems (GPS). By compensating range offset errors for each channel of LiDARs, accuracy of the relative pose estimation is improved.

The source code is written in two languages: MATLAB and C++.

Four main features of the **AutoL2LCalib** are like;
- The algorithm needs **no additional sensor** except for LiDARs to estimate relative poses of LiDARs.
- An **arbitrarily-shaped planar board without dimensions** is an only additional requirement to tackle the algorithm. (i.e. **no need** of chessboard!)
- **No assumption for operating environments** is needed. (e.g. three orthogonal planes, horizontally-attached sensor configuration, specific environmental settings with known dimensions...)
- The planar board regions in 3D point clouds are **automatically extracted** by the proposed planar board extraction and completion method. (**no need** of exhaustive user-intervention to specify target regions!) 

The estimation accuracy of **AutoL2LCalib** is extensively evaluated by using four Velodyne VLP-16 LiDARs with various configurations and various planar boards (long board, broken board, and chess board).

*The C++ version of the code will be uploaded soon.*

- Maintainers: Changhyeon Kim (rlackd93@snu.ac.kr), and Junha Kim (wnsgk02@snu.ac.kr)

### Datasets used in the paper
The datasets used in our submission are available from a bottom URL. All data is obtained in a gym with a monocular camera and two Velodyne VLP-16 Puck LiDARs.
- Camera: mvBlueCOUGAR-X104iG, Matrix Vision GmbH. ([Provider](https://www.matrix-vision.com/GigE-Vision-camera-mvbluecougar-x.html))
- LiDARs: Velodyne Puck LiDAR VLP-16, Velodyne. ([Provider](https://velodynelidar.com/products/puck))

Especially, a dataset (referred as 'four_lidars') is obtained by using four Velodyne VLP-16 LiDARs. All extrinsic relative poses among LiDARs estimated by the proposed algorithm are included in each folder of datasets. 

*The detailed descriptions (data structure, intrinsic parameters, and etc..) can be seen a* **"READ_ME.txt"** *file in the dataset zip file.*

- [Download link](https://larr.snu.ac.kr/junha/submission2021/datasets.zip) (767 Mb)



## 2. How to use?
We provides two versions: MATLAB and C++ (with ROS).

### 1) Inputs

* *Common for both MATLAB and C++ versions.*

Our program uses 3D point clouds from two LiDARs as forms of **" *.pcd "** files. 
The data fields of the pcd files could be like below,
<p align = "left">
<img src= "https://github.com/JunhaAgu/AutoL2LCalib/blob/main/imgs/pcd_structure.png" alt="pcd file inside" width="250" height="270">
</p> 


**The order and types** of data fields should obey below,
- data fields: {x y z intensity ring time}
- types: {float(4 Bytes) / float(4 Bytes) / float(4 Bytes) / unsigned long(2 Bytes) / float(4 Bytes)}

Each field means like, 
- x : x-coordinate of a 3D point
- y : y-coordinate of a 3D point
- z : z-coordinate of a 3D point
- intensity : laser reflection intensity (0~255) of a 3D point
- ring : # of a channel which a 3D point belongs
- time : an acquisition timestamp of a 3D point

*For more convenient use, we are working on **ROS-based data streaming (from real-time acquisitions or rosbag) parser**.*

*We are also working on new data parsers for MATLAB and C++ versions not to require the specific order of data fields.* 

### 2) MATLAB version
#### (a) Dependencies
Recommend: MATLAB version >= 2018b with Windows 10.

In versions under 2018a, some functions in the code could not be supported. Please notify us if you have problems when using the program.

#### (2) Installation
Just download the folder **"MATLAB_VERSION"** in this repository.

#### (3) Run
By starting the m-file **" mainscript.m "** in the folder, the AutoL2LCalib works automatically, and the mainscript shows aligned point clouds with the estimated relative pose of LiDARs.
Default settings is for two Velodyne VLP-16 Pucks with 10 Hz (1,800 horizontal resoultion steps).
You can modify the settings at the top of the mainscript.m to fit your own 3D LiDARs.


### 3) C++ version
TBD ASAP.



## 3. Cite AutoL2LCalib
Thank you for citing our *AutoL2LCalib* paper if you use any of this code:
```
  @inproceedings{
  title={Automated Extrinsic Calibration for 3D LiDARs with Range Offset Correction using an Arbitrary Planar Board},
  author={J. Kim, C. Kim, Y. Han, and HJ Kim},
  booktitle={IEEE International Conference on Robotics and Automation (ICRA)},
  pages={5082-5088},
  year={2021},
  organization={IEEE}
  }
```
