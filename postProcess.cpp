//#include "postProcess.h"
//
//void randomSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, int num = 20000) {
//	pcl::RandomSample<PointT> rs;
//	rs.setInputCloud(in);				//设置待滤波点云
//	rs.setSample(num);					//设置下采样点云的点数
//	//rs.setSeed(1);						//设置随机函数种子点
//	rs.filter(*out);
//}
//
//void uniformSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
//	pcl::UniformSampling<PointT> us;
//	us.setInputCloud(in);
//	us.setRadiusSearch(0.01);
//	us.filter(*out);
//}
//
//void MLSSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
//	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);   //定义法线
//	pcl::MovingLeastSquares<PointT, pcl::PointNormal> filter;
//	pcl::search::KdTree<PointT>::Ptr kdtree;  //定义搜索方法
//	filter.setInputCloud(in);    //设置输入点云
//	filter.setSearchRadius(10);// 用于拟合的K近邻半径。在这个半径里进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。
//	filter.setComputeNormals(true);  // 是否存储点云的法向量，true 为存储，false 不存储
//	filter.setSearchMethod(kdtree); //设置搜索方法
//	filter.process(*smoothedCloud); //处理点云并输出
//}
//
//void newSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
//	// 提取ISS特征点
//	pcl::PointCloud<PointT>::Ptr iss_keypoints(new pcl::PointCloud<PointT>);
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//
//	pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
//	iss_detector.setSearchMethod(tree);
//	iss_detector.setSalientRadius(0.17f);
//	iss_detector.setNonMaxRadius(0.2f);
//	iss_detector.setNormalRadius(0.05f);
//	iss_detector.setBorderRadius(0.05f);
//	iss_detector.setThreshold21(0.975f);
//	iss_detector.setThreshold32(0.975f);
//	iss_detector.setMinNeighbors(5);
//	iss_detector.setNumberOfThreads(4);
//	iss_detector.setInputCloud(in);
//	iss_detector.compute(*iss_keypoints);
//	/*iss_detector.setSearchMethod(tree);
//	iss_detector.setSalientRadius(0.1f);
//	iss_detector.setNonMaxRadius(0.15f);
//	iss_detector.setNormalRadius(0.05f);
//	iss_detector.setBorderRadius(0.05f);
//	iss_detector.setThreshold21(0.99f);
//	iss_detector.setThreshold32(0.99f);
//	iss_detector.setMinNeighbors(5);
//	iss_detector.setNumberOfThreads(8);
//	iss_detector.setInputCloud(in);
//	iss_detector.compute(*iss_keypoints);*/
//
//	// 随机采样
//	pcl::PointCloud<PointT>::Ptr random_sampled_points(new pcl::PointCloud<PointT>);
//	randomSample(in, random_sampled_points, 17000);
//
//	// 合并结果
//	pcl::PointCloud<PointT>::Ptr combined_cloud(new pcl::PointCloud<PointT>);
//	*combined_cloud += *iss_keypoints;
//	*combined_cloud += *random_sampled_points;
//}
//#include <pcl/filters/filter.h>
//#include <pcl/filters/extract_indices.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//int curveSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out)
//{
//	PointCloudT::Ptr cloud_filtered(new PointCloudT);
//	std::vector<int> indices_NaN;
//	pcl::removeNaNFromPointCloud(*in, *cloud_filtered, indices_NaN);
//
//	// 计算法线（包含曲率）
//	pcl::NormalEstimation<PointT, pcl::Normal> ne;
//	ne.setInputCloud(cloud_filtered);
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//	ne.setSearchMethod(tree);
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	ne.setKSearch(30); // 根据实际情况调整K值
//	ne.compute(*normals);
//
//	// 收集所有点的曲率
//	std::vector<float> curvatures;
//	for (size_t i = 0; i < normals->size(); ++i)
//	{
//		curvatures.push_back(normals->points[i].curvature);
//	}
//
//	// 将曲率分为10个区间
//	std::vector<std::vector<int>> curvature_intervals(10);
//	float min_curvature = *std::min_element(curvatures.begin(), curvatures.end());
//	float max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
//	float interval_size = (max_curvature - min_curvature) / 10.0;
//
//	for (size_t i = 0; i < curvatures.size(); ++i)
//	{
//		int index = std::min(int((curvatures[i] - min_curvature) / interval_size), 9); // 确保最大值落在最后一个区间
//		curvature_intervals[index].push_back(i);
//	}
//
//	// 根据区间计算采样率，越大的区间采样率越高
//	PointCloudT::Ptr sampled_cloud(new PointCloudT);
//	for (int i = 0; i < 10; ++i)
//	{
//		float sample_rate = static_cast<float>(i + 1) / 10.0; // 增加采样率
//		for (auto idx : curvature_intervals[i])
//		{
//			if ((rand() % 1000) / 1000.0 < sample_rate) // 随机判断是否采样
//			{
//				sampled_cloud->push_back(cloud_filtered->points[idx]);
//			}
//		}
//	}
//
//	// 如果采样点不足20000，可以通过随机采样补充
//	while (sampled_cloud->size() < 3000)
//	{
//		pcl::PointCloud<PointT>::Ptr supplement_points(new pcl::PointCloud<PointT>);
//		randomSample(in, supplement_points, 20000 - sampled_cloud->size());
//		*sampled_cloud += *supplement_points;
//	}
//	pcl::copyPointCloud(*sampled_cloud, *out);
//	return 0;
//}
///*
// * @File: normal_space_sample.cpp
// * @Brief: pcl course
// * @Description: 展示NSS法向空间采样效果
// * @Version: 0.0.1
// * @Author: MuYv
// */
//#include <iostream>
//#include <string>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/filters/normal_space.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/normal_3d_omp.h>
//
//
//
//int normalSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
//
//
//	// 定义变量
//	pcl::PointCloud<pcl::Normal>::Ptr \
//		cloud_normals(new pcl::PointCloud<pcl::Normal>());
//
//	pcl::NormalEstimation<PointT, pcl::Normal> ne;
//	// 创建一个空的kdtree对象，并把它传递给法线估计对象，
//	// 用于创建基于输入点云数据的邻域搜索kdtree
//	pcl::search::KdTree<PointT>::Ptr \
//		tree(new pcl::search::KdTree<PointT>());
//	// 传入待估计法线的点云数据，智能指针
//	ne.setInputCloud(in);
//	// 传入kdtree对象，智能指针
//	ne.setSearchMethod(tree);
//	ne.setKSearch(25);
//	// 设置视点源点，用于调整点云法向（指向视点），默认（0，0，0）
//	ne.setViewPoint(0, 0, 0);
//	// 计算法线数据
//	ne.compute(*cloud_normals);
//
//	// 通过concatenateFields函数将point和normal组合起来形成PointNormal点云数据
//	pcl::PointCloud<pcl::PointNormal>::Ptr \
//		cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>());
//	pcl::PointCloud<pcl::PointNormal>::Ptr \
//		cloud_with_normal_sampled(new pcl::PointCloud<pcl::PointNormal>());
//	pcl::concatenateFields(*in, *cloud_normals, *cloud_with_normal);
//
//	// 创建法向空间采样（模板）类对象
//	pcl::NormalSpaceSampling<pcl::PointNormal, pcl::Normal> nss;
//	// 设置xyz三个法向空间的分类组数，此处设置为一致，根据具体场景可以调整
//	const int kBinNum = 8;
//	nss.setBins(kBinNum, kBinNum, kBinNum);
//	// 如果传入的是有序点云，此处可以尝试设置为true
//	nss.setKeepOrganized(false);
//	// 设置随机种子，这样可以保证同样的输入可以得到同样的结果，便于debug分析
//	nss.setSeed(200);   // random seed
//	// 传入待采样的点云数据
//	nss.setInputCloud(cloud_with_normal);
//	// 传入用于采样分析的法线数据，需与传入点云数据一一对应
//	nss.setNormals(cloud_normals);
//	// 设置采样总数，即目标点云的总数据量
//	const float kSampleRatio = 0.2f;
//	nss.setSample(20000);
//	// 执行采样并带出采样结果
//	nss.filter(*cloud_with_normal_sampled);
//	return 0;
//}