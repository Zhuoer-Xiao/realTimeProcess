#pragma once
#include <iostream>
#include <string>
#include "pclHeaders.h"
#include <unordered_set>
#include <vector>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

void randomSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, int num = 20000);
void uniformSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out);
void MLSSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out);
void newSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out);
void curveSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out);
void voxelSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, float voxel);