#pragma once
#include <pcl/keypoints/iss_3d.h>
#include <pcl/point_types.h> 
#include <pcl/point_cloud.h>
#include <boost/random.hpp> //随机数
#include <pcl/io/ply_io.h>
#include <direct.h>
#include <filesystem>
#include <cmath>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/common/transforms.h> 
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh_omp.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/keypoints/iss_3d.h>  
#include <pcl/features/fpfh.h>  
#include <pcl/registration/sample_consensus_prerejective.h>  
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_fpcs.h>
#include <iostream>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/icp_nl.h>
#include <flann/flann.hpp>
#include <pcl/features/vfh.h>
using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

////精配准
Eigen::Matrix4f gicpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool &flag);
Eigen::Matrix4f icpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool&flag);
Eigen::Matrix4f multi_scaling_gicp(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool &flag);
//Eigen::Matrix4f multi_scaling_gicp_2(PointCloudT::Ptr src, PointCloudT::Ptr tar);
Eigen::Matrix4f normalIcpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar, bool& flag);
Eigen::Matrix4f nlIcpReg(pcl::PointCloud<PointT>::Ptr src, pcl::PointCloud<PointT>::Ptr tar, bool& flag);
//特征点计算
fpfhFeature::Ptr computeFpfhFeature(PointCloudT::Ptr input_cloud, pcl::search::KdTree<PointT>::Ptr tree);
//void computeIssKeyPoint(const PointCloudT::Ptr& cloud, PointCloudT::Ptr& keyPoints);
//
//粗配准
Eigen::Matrix4f fpfhReg(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
Eigen::Matrix4f FpcsReg(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
//Eigen::Matrix4f fpfhReg_downSample(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
Eigen::Matrix4f NDT(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
Eigen::Matrix4f kfpcs(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
//降采样
void downSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, float voxel);
void downSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out);

////自定义ICP框架
//Eigen::Matrix4f selfReg(PointCloudT::Ptr src, PointCloudT::Ptr tar);