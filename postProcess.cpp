//#include "postProcess.h"
//
//void randomSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, int num = 20000) {
//	pcl::RandomSample<PointT> rs;
//	rs.setInputCloud(in);				//���ô��˲�����
//	rs.setSample(num);					//�����²������Ƶĵ���
//	//rs.setSeed(1);						//��������������ӵ�
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
//	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);   //���巨��
//	pcl::MovingLeastSquares<PointT, pcl::PointNormal> filter;
//	pcl::search::KdTree<PointT>::Ptr kdtree;  //������������
//	filter.setInputCloud(in);    //�����������
//	filter.setSearchRadius(10);// ������ϵ�K���ڰ뾶��������뾶����б���ӳ���������ϡ��뾶ԽС��Ϻ������ʧ���ԽС����֮�п��ܳ��ֹ���ϵ�����
//	filter.setComputeNormals(true);  // �Ƿ�洢���Ƶķ�������true Ϊ�洢��false ���洢
//	filter.setSearchMethod(kdtree); //������������
//	filter.process(*smoothedCloud); //������Ʋ����
//}
//
//void newSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
//	// ��ȡISS������
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
//	// �������
//	pcl::PointCloud<PointT>::Ptr random_sampled_points(new pcl::PointCloud<PointT>);
//	randomSample(in, random_sampled_points, 17000);
//
//	// �ϲ����
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
//	// ���㷨�ߣ��������ʣ�
//	pcl::NormalEstimation<PointT, pcl::Normal> ne;
//	ne.setInputCloud(cloud_filtered);
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//	ne.setSearchMethod(tree);
//	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//	ne.setKSearch(30); // ����ʵ���������Kֵ
//	ne.compute(*normals);
//
//	// �ռ����е������
//	std::vector<float> curvatures;
//	for (size_t i = 0; i < normals->size(); ++i)
//	{
//		curvatures.push_back(normals->points[i].curvature);
//	}
//
//	// �����ʷ�Ϊ10������
//	std::vector<std::vector<int>> curvature_intervals(10);
//	float min_curvature = *std::min_element(curvatures.begin(), curvatures.end());
//	float max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
//	float interval_size = (max_curvature - min_curvature) / 10.0;
//
//	for (size_t i = 0; i < curvatures.size(); ++i)
//	{
//		int index = std::min(int((curvatures[i] - min_curvature) / interval_size), 9); // ȷ�����ֵ�������һ������
//		curvature_intervals[index].push_back(i);
//	}
//
//	// ���������������ʣ�Խ������������Խ��
//	PointCloudT::Ptr sampled_cloud(new PointCloudT);
//	for (int i = 0; i < 10; ++i)
//	{
//		float sample_rate = static_cast<float>(i + 1) / 10.0; // ���Ӳ�����
//		for (auto idx : curvature_intervals[i])
//		{
//			if ((rand() % 1000) / 1000.0 < sample_rate) // ����ж��Ƿ����
//			{
//				sampled_cloud->push_back(cloud_filtered->points[idx]);
//			}
//		}
//	}
//
//	// ��������㲻��20000������ͨ�������������
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
// * @Description: չʾNSS����ռ����Ч��
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
//	// �������
//	pcl::PointCloud<pcl::Normal>::Ptr \
//		cloud_normals(new pcl::PointCloud<pcl::Normal>());
//
//	pcl::NormalEstimation<PointT, pcl::Normal> ne;
//	// ����һ���յ�kdtree���󣬲��������ݸ����߹��ƶ���
//	// ���ڴ�����������������ݵ���������kdtree
//	pcl::search::KdTree<PointT>::Ptr \
//		tree(new pcl::search::KdTree<PointT>());
//	// ��������Ʒ��ߵĵ������ݣ�����ָ��
//	ne.setInputCloud(in);
//	// ����kdtree��������ָ��
//	ne.setSearchMethod(tree);
//	ne.setKSearch(25);
//	// �����ӵ�Դ�㣬���ڵ������Ʒ���ָ���ӵ㣩��Ĭ�ϣ�0��0��0��
//	ne.setViewPoint(0, 0, 0);
//	// ���㷨������
//	ne.compute(*cloud_normals);
//
//	// ͨ��concatenateFields������point��normal��������γ�PointNormal��������
//	pcl::PointCloud<pcl::PointNormal>::Ptr \
//		cloud_with_normal(new pcl::PointCloud<pcl::PointNormal>());
//	pcl::PointCloud<pcl::PointNormal>::Ptr \
//		cloud_with_normal_sampled(new pcl::PointCloud<pcl::PointNormal>());
//	pcl::concatenateFields(*in, *cloud_normals, *cloud_with_normal);
//
//	// ��������ռ������ģ�壩�����
//	pcl::NormalSpaceSampling<pcl::PointNormal, pcl::Normal> nss;
//	// ����xyz��������ռ�ķ����������˴�����Ϊһ�£����ݾ��峡�����Ե���
//	const int kBinNum = 8;
//	nss.setBins(kBinNum, kBinNum, kBinNum);
//	// ����������������ƣ��˴����Գ�������Ϊtrue
//	nss.setKeepOrganized(false);
//	// ����������ӣ��������Ա�֤ͬ����������Եõ�ͬ���Ľ��������debug����
//	nss.setSeed(200);   // random seed
//	// ����������ĵ�������
//	nss.setInputCloud(cloud_with_normal);
//	// �������ڲ��������ķ������ݣ����봫���������һһ��Ӧ
//	nss.setNormals(cloud_normals);
//	// ���ò�����������Ŀ����Ƶ���������
//	const float kSampleRatio = 0.2f;
//	nss.setSample(20000);
//	// ִ�в����������������
//	nss.filter(*cloud_with_normal_sampled);
//	return 0;
//}