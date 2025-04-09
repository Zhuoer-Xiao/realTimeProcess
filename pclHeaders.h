#pragma once
#ifndef pclHeaders_h
#define pclHeaders_h
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
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>s
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/point_types.h> 
#include <pcl/point_cloud.h>
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
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ia_kfpcs.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/features/vfh.h>
#include <pcl/io/ply_io.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

inline void toXYZ(pcl::PointCloud<PointT>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out) {
	out->points.resize(in->points.size());
	for (int i = 0; i < in->points.size(); i++) {
		out->points[i].x = in->points[i].x;
		out->points[i].y = in->points[i].y;
		out->points[i].z = in->points[i].z;
	}
}
#endif // !