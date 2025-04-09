#include "registrations.h"


// 类型定义  
typedef pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> NormalEstimationT;
typedef pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> FPFHEstimationT;
typedef pcl::SampleConsensusPrerejective<PointT, PointT, pcl::Normal> RegistrationT;


Eigen::Matrix4f icpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool&flag)
{

	pcl::IterativeClosestPoint<PointT, PointT> icp;
	PointCloudT::Ptr cloudRes(new PointCloudT);

	//----------------------icp核心代码--------------------
	icp.setInputSource(src);            // 源点云
	icp.setInputTarget(tar);            // 目标点云
	icp.setTransformationEpsilon(1e-10);   // 为终止条件设置最小转换差异
	icp.setMaxCorrespondenceDistance(10);  // 设置对应点对之间的最大距离（此值对配准结果影响较大）。
	icp.setEuclideanFitnessEpsilon(0.001);  // 设置收敛条件是均方误差和小于阈值， 停止迭代；
	//icp.setMaximumIterations(35);           // 最大迭代次数
	//icp.setUseReciprocalCorrespondences(true);//设置为true,则使用相互对应关系
	// 计算需要的刚体变换以便将输入的源点云匹配到目标点云
	icp.align(*cloudRes);
	return icp.getFinalTransformation();
}

Eigen::Matrix4f gicpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool &flag)
{

	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
	PointCloudT::Ptr cloudRes(new PointCloudT);
	gicp.setMaximumIterations(20);
	gicp.setInputSource(src);
	gicp.setInputTarget(tar);
	gicp.setMaxCorrespondenceDistance(10);
	gicp.setUseReciprocalCorrespondences(true);

	//gicp.setMaximumIterations(max_iter);
	gicp.align(*cloudRes);
	if (!gicp.hasConverged()) {
		flag= false;
	}
	else {
		flag = true;
	}
	return gicp.getFinalTransformation();
}

Eigen::Matrix4f gicpReg2(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool &flag)
{

	pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
	PointCloudT::Ptr cloudRes(new PointCloudT);
	gicp.setMaximumIterations(5);
	gicp.setInputSource(src);
	gicp.setInputTarget(tar);
	gicp.setMaxCorrespondenceDistance(10);
	gicp.setUseReciprocalCorrespondences(true);

	//gicp.setMaximumIterations(max_iter);
	gicp.align(*cloudRes);
	
	if (!gicp.hasConverged()) {
		flag = false;
	}
	else {
		flag = true;
	}
	return gicp.getFinalTransformation();
}

Eigen::Matrix4f multi_scaling_gicp(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool&flag) {

	PointCloudT::Ptr src_d1(new PointCloudT);
	PointCloudT::Ptr tar_d1(new PointCloudT);
	PointCloudT::Ptr src_d2(new PointCloudT);
	PointCloudT::Ptr tar_d2(new PointCloudT);
	PointCloudT::Ptr src_d3(new PointCloudT);
	PointCloudT::Ptr tar_d3(new PointCloudT);

	downSample(src, src_d1, 0.5);
	downSample(tar, tar_d1, 0.5);
	downSample(src, src_d2, 2.0);
	downSample(tar, tar_d2, 2.0);
	downSample(src, src_d3, 3.0);
	downSample(tar, tar_d3, 3.0);
	auto res1 = gicpReg2(src_d3, tar_d3,flag);
	transformPointCloud(*src_d2, *src_d2, res1);

	auto res2 = gicpReg2(src_d2, tar_d2,flag);
	transformPointCloud(*src_d1, *src_d1, res2 * res1);

	auto res3 = gicpReg(src_d1, tar_d1, flag);
	auto res = res3 * res2 * res1;
	return res;
}
//
//Eigen::Matrix4f multi_scaling_gicp_2(PointCloudT::Ptr src, PointCloudT::Ptr tar) {
//
//	PointCloudT::Ptr src_d1(new PointCloudT);
//	PointCloudT::Ptr tar_d1(new PointCloudT);
//	PointCloudT::Ptr src_d2(new PointCloudT);
//	PointCloudT::Ptr tar_d2(new PointCloudT);
//
//
//	downSample(src, src_d1, 0.5);
//	downSample(tar, tar_d1, 0.5);
//	downSample(src_d1, src_d2, 4.0);
//	downSample(tar_d1, tar_d2, 4.0);
//	auto start = std::chrono::high_resolution_clock::now();
//
//	auto res2 = gicpReg2(src_d2, tar_d2);
//	transformPointCloud(*src_d1, *src_d1, res2);
//
//	auto res3 = gicpReg2(src_d1, tar_d1);
//	auto res = res2 * res3;
//
//	return res;
//}

fpfhFeature::Ptr computeFpfhFeature(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr tree)
{
	//法向量
	pointnormal::Ptr point_normal(new pointnormal);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> est_normal;
	est_normal.setInputCloud(input_cloud);
	est_normal.setSearchMethod(tree);
	est_normal.setKSearch(10);
	est_normal.compute(*point_normal);
	//fpfh 估计
	fpfhFeature::Ptr fpfh(new fpfhFeature);
	pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> est_fpfh;
	est_fpfh.setNumberOfThreads(8); //指定核计算

	est_fpfh.setInputCloud(input_cloud);
	est_fpfh.setInputNormals(point_normal);
	est_fpfh.setSearchMethod(tree);
	est_fpfh.setKSearch(10);
	est_fpfh.compute(*fpfh);

	return fpfh;

}

Eigen::Matrix4f fpfhReg(PointCloudT::Ptr& src, PointCloudT::Ptr& tar)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    toXYZ(src, cloud_source);
    toXYZ(tar, cloud_target);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh = computeFpfhFeature(cloud_source, tree);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_fpfh = computeFpfhFeature(cloud_target, tree);
	std::cout << "对齐开始" << std::endl << std::endl;
	//对齐(占用了大部分运行时间)
	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
	sac_ia.setInputSource(src);
	sac_ia.setSourceFeatures(source_fpfh);
	sac_ia.setInputTarget(tar);
	sac_ia.setTargetFeatures(cloud_fpfh);
	PointCloudT::Ptr align(new PointCloudT);
	sac_ia.setNumberOfSamples(21);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
	//sac_ia.setCorrespondenceRandomness(10); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
	sac_ia.align(*align);


	std::cout << "计算结束" << std::endl << std::endl;
	return sac_ia.getFinalTransformation();
}

Eigen::Matrix4f FpcsReg(PointCloudT::Ptr& src, PointCloudT::Ptr& tar) {
	pcl::VoxelGrid<PointT> approximate_voxel_grid;
	approximate_voxel_grid.setLeafSize(2, 2, 2); //网格边长.这里的数值越大，则精简的越厉害（剩下的数据少）

	//1
	PointCloudT::Ptr sample_src(new PointCloudT);
	PointCloudT::Ptr sample_tar(new PointCloudT);
	approximate_voxel_grid.setInputCloud(src);
	approximate_voxel_grid.filter(*sample_src);
	//2
	approximate_voxel_grid.setInputCloud(tar);
	approximate_voxel_grid.filter(*sample_tar);

	pcl::registration::FPCSInitialAlignment<PointT, PointT> registration;
	PointCloudT::Ptr tmp(new PointCloudT);
	registration.setMaximumIterations(20);
	registration.setInputSource(sample_src);				// 源点云
	registration.setInputTarget(sample_tar);				// 目标点云
	registration.setApproxOverlap(0.9);					// 设置源和目标之间的近似重叠度。
	registration.setDelta(10);						// 设置常数因子delta，用于对内部计算的参数进行加权
	registration.setNumberOfSamples(20);				// 设置验证配准效果时要使用的采样点数量
	registration.align(*tmp);
	auto res = registration.getFinalTransformation();
	//cout << res << endl;
	return res;
}
//
//Eigen::Matrix4f fpfhReg_downSample(PointCloudT::Ptr& src, PointCloudT::Ptr& tar)
//{
//	//体素化
//	pcl::VoxelGrid<PointT> approximate_voxel_grid;
//	approximate_voxel_grid.setLeafSize(2, 2, 2); //网格边长.这里的数值越大，则精简的越厉害（剩下的数据少）
//
//	//1
//	PointCloudT::Ptr sample_src(new PointCloudT);
//	PointCloudT::Ptr sample_tar(new PointCloudT);
//	approximate_voxel_grid.setInputCloud(src);
//	approximate_voxel_grid.filter(*sample_src);
//	//2
//	approximate_voxel_grid.setInputCloud(tar);
//	approximate_voxel_grid.filter(*sample_tar);
//	pcl::PLYWriter writer;
//	//writer.write("./sample_tar.ply", *sample_tar);
//	//writer.write("./sample_src.ply", *sample_src);
//
//	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
//
//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_fpfh = computeFpfhFeature(sample_src, tree);
//	pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_fpfh = computeFpfhFeature(sample_tar, tree);
//	//std::cout << "对齐开始" << std::endl << std::endl;
//	//对齐(占用了大部分运行时间)
//	pcl::SampleConsensusInitialAlignment<PointT, PointT, pcl::FPFHSignature33> sac_ia;
//	sac_ia.setInputSource(sample_src);
//	sac_ia.setSourceFeatures(source_fpfh);
//	sac_ia.setInputTarget(sample_tar);
//	sac_ia.setTargetFeatures(cloud_fpfh);
//	PointCloudT::Ptr align(new PointCloudT);
//	sac_ia.setNumberOfSamples(21);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
//	sac_ia.setCorrespondenceRandomness(10);
//	sac_ia.align(*align);
//
//
//	//std::cout << "计算结束" << std::endl << std::endl;
//	return sac_ia.getFinalTransformation();
//}

void downSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, float voxel) {
	pcl::VoxelGrid<PointT> approximate_voxel_grid;
	approximate_voxel_grid.setLeafSize(voxel, voxel, voxel);
	approximate_voxel_grid.setInputCloud(in);
	approximate_voxel_grid.filter(*out);
}

void downSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
	pcl::VoxelGrid<PointT> approximate_voxel_grid;
	approximate_voxel_grid.setLeafSize(2, 2, 2);
	approximate_voxel_grid.setInputCloud(in);
	approximate_voxel_grid.filter(*out);
}


Eigen::Matrix4f normalIcpReg(PointCloudT::Ptr src, PointCloudT::Ptr tar,bool &flag)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr srcTmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tarTmp(new pcl::PointCloud<pcl::PointXYZ>);
	toXYZ(src, srcTmp);
	toXYZ(tar, tarTmp);
	// 创建法线估计对象，并为源点云和目标点云计算法线
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	pcl::PointCloud<pcl::Normal>::Ptr normals_source(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_target(new pcl::PointCloud<pcl::Normal>);

	ne.setInputCloud(srcTmp);
	ne.setSearchMethod(tree);
	ne.setKSearch(20); // 设置用于估计法线的邻居数量
	ne.compute(*normals_source);

	ne.setInputCloud(tarTmp);
	ne.compute(*normals_target);

	// 将点和法线信息合并到pcl::PointNormal类型中
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	pcl::concatenateFields(*srcTmp, *normals_source, *source_with_normals);
	pcl::concatenateFields(*tarTmp, *normals_target, *target_with_normals);

	// 创建 ICPWN 对象
	pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
	icp.setInputSource(source_with_normals);
	icp.setInputTarget(target_with_normals);

	// 可选：设置ICP参数
	icp.setMaximumIterations(50); // 最大迭代次数
	icp.setMaxCorrespondenceDistance(0.05); // 对应点的最大距离

	// 执行配准
	pcl::PointCloud<pcl::PointNormal> final;
	icp.align(final);


	return icp.getFinalTransformation();
}


Eigen::Matrix4f NDT(PointCloudT::Ptr& src, PointCloudT::Ptr& tar) {
	pcl::NormalDistributionsTransform<PointT, PointT> ndt;
	ndt.setInputSource(src);
	PointCloudT::Ptr cloud_ndt(new PointCloudT);
	ndt.setTransformationEpsilon(0.01);
	ndt.setStepSize(1);
	ndt.setResolution(2);//网格分辨率
	ndt.setMaximumIterations(10);

	//载入点云
	ndt.setInputSource(src);
	ndt.setInputTarget(tar);
	ndt.align(*cloud_ndt);
	return ndt.getFinalTransformation();
}

Eigen::Matrix4f kfpcs(PointCloudT::Ptr& src, PointCloudT::Ptr& tar) {
	pcl::registration::KFPCSInitialAlignment<PointT, PointT> k4pcs;

	pcl::VoxelGrid<PointT> approximate_voxel_grid;
	PointCloudT::Ptr sample_src(new PointCloudT);
	PointCloudT::Ptr sample_tar(new PointCloudT);
	approximate_voxel_grid.setInputCloud(src);
	approximate_voxel_grid.filter(*sample_src);
	//2
	approximate_voxel_grid.setInputCloud(tar);
	approximate_voxel_grid.filter(*sample_tar);

	k4pcs.setMaxCorrespondenceDistance(0.05); // 设置最大对应距离
	k4pcs.setMaximumIterations(10);
	k4pcs.setInputSource(sample_src);
	k4pcs.setInputTarget(sample_tar);
	k4pcs.align(*src);
	return k4pcs.getFinalTransformation();

}

Eigen::Matrix4f nlIcpReg(pcl::PointCloud<PointT>::Ptr src, pcl::PointCloud<PointT>::Ptr tar,bool&flag) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr srcTmp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tarTmp(new pcl::PointCloud<pcl::PointXYZ>);
	toXYZ(src, srcTmp);
    toXYZ(tar, tarTmp);
	// 创建法线估计对象，并为源点云和目标点云计算法线
	pcl::IterativeClosestPointNonLinear<pcl::PointNormal, pcl::PointNormal> nlicp;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	pcl::PointCloud<pcl::Normal>::Ptr normals_source(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normals_target(new pcl::PointCloud<pcl::Normal>);

	ne.setInputCloud(srcTmp);
	ne.setSearchMethod(tree);
	ne.setKSearch(20); // 设置用于估计法线的邻居数量
	ne.compute(*normals_source);

	ne.setInputCloud(tarTmp);
	ne.compute(*normals_target);

	// 将点和法线信息合并到pcl::PointNormal类型中
	pcl::PointCloud<pcl::PointNormal>::Ptr source_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	pcl::concatenateFields(*srcTmp, *normals_source, *source_with_normals);
	pcl::concatenateFields(*tarTmp, *normals_target, *target_with_normals);


	nlicp.setInputSource(source_with_normals);
	nlicp.setInputTarget(target_with_normals);

	// 可选：设置ICP参数
	nlicp.setMaximumIterations(50); // 最大迭代次数
	nlicp.setMaxCorrespondenceDistance(0.05); // 对应点的最大距离

	// 执行配准
	pcl::PointCloud<pcl::PointNormal> final;
	nlicp.align(final);


	return nlicp.getFinalTransformation();
}