#pragma once
#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/surface/mls.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/voxel_grid.h>
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