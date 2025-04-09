#pragma once
#include "pclHeaders.h"
#include <boost/random.hpp> //�����
#include <direct.h>
#include <filesystem>
#include <cmath>

#include <iostream>

#include <flann/flann.hpp>
using namespace std;
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;

////����׼
Eigen::Matrix4f gicpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool &flag);
Eigen::Matrix4f icpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool&flag);
Eigen::Matrix4f multi_scaling_gicp(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool &flag);
//Eigen::Matrix4f multi_scaling_gicp_2(PointCloudT::Ptr src, PointCloudT::Ptr tar);
Eigen::Matrix4f normalIcpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar, bool& flag);
Eigen::Matrix4f nlIcpReg(pcl::PointCloud<PointT>::Ptr src, pcl::PointCloud<PointT>::Ptr tar, bool& flag);
//���������
fpfhFeature::Ptr computeFpfhFeature(PointCloudT::Ptr input_cloud, pcl::search::KdTree<PointT>::Ptr tree);
//void computeIssKeyPoint(const PointCloudT::Ptr& cloud, PointCloudT::Ptr& keyPoints);
//
//����׼
Eigen::Matrix4f fpfhReg(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
Eigen::Matrix4f FpcsReg(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
//Eigen::Matrix4f fpfhReg_downSample(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
Eigen::Matrix4f NDT(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
Eigen::Matrix4f kfpcs(PointCloudT::Ptr& src, PointCloudT::Ptr& tar);
//������
void downSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, float voxel);
void downSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out);

////�Զ���ICP���
//Eigen::Matrix4f selfReg(PointCloudT::Ptr src, PointCloudT::Ptr tar);