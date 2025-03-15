#include "postProcess.h"
void randomSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, int num) {
	pcl::RandomSample<PointT> rs;
	rs.setInputCloud(in);				//设置待滤波点云
	rs.setSample(num);					//设置下采样点云的点数
	//rs.setSeed(1);						//设置随机函数种子点
	rs.filter(*out);
}

void uniformSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
	pcl::UniformSampling<PointT> us;
	us.setInputCloud(in);
	us.setRadiusSearch(0.01);
	us.filter(*out);
}

void MLSSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);   //定义法线
	pcl::MovingLeastSquares<PointT, pcl::PointNormal> filter;
	pcl::search::KdTree<PointT>::Ptr kdtree;  //定义搜索方法
	filter.setInputCloud(in);    //设置输入点云
	filter.setSearchRadius(10);// 用于拟合的K近邻半径。在这个半径里进行表面映射和曲面拟合。半径越小拟合后曲面的失真度越小，反之有可能出现过拟合的现象。
	filter.setComputeNormals(true);  // 是否存储点云的法向量，true 为存储，false 不存储
	filter.setSearchMethod(kdtree); //设置搜索方法
	filter.process(*smoothedCloud); //处理点云并输出
}

void newSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
	// 提取ISS特征点
    // --------------------------- 1. 读取输入点云 ---------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(in->points.size());
    for (int i = 0; i < in->points.size(); i++) {
        out->points[i].x = in->points[i].x;
        out->points[i].y = in->points[i].y;
        out->points[i].z = in->points[i].z;
    }


    // --------------------------- 2. ISS关键点检测（含响应值） ---------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr iss_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
    float radius = 0.17f;
    //cin >> radius;
    iss_detector.setSearchMethod(tree);
    iss_detector.setSalientRadius(radius);
    iss_detector.setNonMaxRadius(0.2f);
    iss_detector.setNormalRadius(0.05f);
    iss_detector.setBorderRadius(0.05f);
    iss_detector.setThreshold21(0.975f);
    iss_detector.setThreshold32(0.975f);
    iss_detector.setMinNeighbors(5);
    iss_detector.setNumberOfThreads(12);
    iss_detector.setInputCloud(cloud);
    iss_detector.compute(*iss_keypoints);
    // --------------------------- 3. 自适应邻域半径标记关键区域 ---------------------------
    std::unordered_set<int> key_region_indices;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    float r_base = 0.05;
    float alpha = 0.5;

    for (const auto& key : iss_keypoints->points) {
        float dynamic_radius = r_base * (1.0);
        dynamic_radius = std::max(0.03f, std::min(dynamic_radius, 0.2f));

        std::vector<int> indices;
        std::vector<float> distances;
        kdtree.radiusSearch(pcl::PointXYZ(key.x, key.y, key.z), dynamic_radius, indices, distances);
        key_region_indices.insert(indices.begin(), indices.end());
    }

    // --------------------------- 4. 分区处理 ---------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr key_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_key_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> key_indices;
    std::vector<int> non_key_indices;
    std::vector<int> total;
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (key_region_indices.count(i)) {
            key_indices.push_back(i);
        }
        else {
            non_key_indices.push_back(i);
        }
    }
    float p_key = 0.5f;
    // --------------------------- 5. 关键区域高密度采样（保留80%） ---------------------------
    pcl::RandomSample<pcl::PointXYZ> key_sampler;
    key_sampler.setInputCloud(key_cloud);
    key_sampler.setSample(key_cloud->size() * p_key);
    key_sampler.filter(*key_cloud);

    // --------------------------- 6. 非关键区域概率渐变采样 ---------------------------
    pcl::KdTreeFLANN<pcl::PointXYZ> key_kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*iss_keypoints, *keypoints_xyz);
    key_kdtree.setInputCloud(keypoints_xyz);

    float p_nonkey = 0.10;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr non_key_sampled(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < non_key_indices.size();++i) {
        pcl::PointXYZ point = cloud->points[non_key_indices[i]];
        std::vector<int> idx(1);
        std::vector<float> dist(1);
        key_kdtree.nearestKSearch(point, 1, idx, dist);

        float d = std::sqrt(dist[0]);
        float prob = p_nonkey;
        if (d <= 0.7) {
            prob += p_key * 1.0 / (1.0 + 20 * d);
        }
        if (dis(gen) < prob) total.push_back(i);
    }
    // --------------------------- 7. 合并与保存 ---------------------------
    for (int i = 0; i < total.size(); ++i) {
        out->push_back(in->points[total[i]]);
    }
    for (int i = 0; i < key_indices.size(); ++i) {
        out->push_back(in->points[key_indices[i]]);
    }
}

void curveSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out)
{
	PointCloudT::Ptr cloud_filtered(new PointCloudT);
	std::vector<int> indices_NaN;
	pcl::removeNaNFromPointCloud(*in, *cloud_filtered, indices_NaN);

	// 计算法线（包含曲率）
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud_filtered);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(30); // 根据实际情况调整K值
	ne.compute(*normals);

	// 收集所有点的曲率
	std::vector<float> curvatures;
	for (size_t i = 0; i < normals->size(); ++i)
	{
		curvatures.push_back(normals->points[i].curvature);
	}

	// 将曲率分为10个区间
	std::vector<std::vector<int>> curvature_intervals(10);
	float min_curvature = *std::min_element(curvatures.begin(), curvatures.end());
	float max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
	float interval_size = (max_curvature - min_curvature) / 10.0;

	for (size_t i = 0; i < curvatures.size(); ++i)
	{
		int index = std::min(int((curvatures[i] - min_curvature) / interval_size), 9); // 确保最大值落在最后一个区间
		curvature_intervals[index].push_back(i);
	}

	// 根据区间计算采样率，越大的区间采样率越高
	PointCloudT::Ptr sampled_cloud(new PointCloudT);
	for (int i = 0; i < 10; ++i)
	{
		float sample_rate = static_cast<float>(i + 1) / 10.0; // 增加采样率
		for (auto idx : curvature_intervals[i])
		{
			if ((rand() % 1000) / 1000.0 < sample_rate) // 随机判断是否采样
			{
				sampled_cloud->push_back(cloud_filtered->points[idx]);
			}
		}
	}

	// 如果采样点不足20000，可以通过随机采样补充
	while (sampled_cloud->size() < 3000)
	{
		pcl::PointCloud<PointT>::Ptr supplement_points(new pcl::PointCloud<PointT>);
		randomSample(in, supplement_points, 20000 - sampled_cloud->size());
		*sampled_cloud += *supplement_points;
	}
	pcl::copyPointCloud(*sampled_cloud, *out);
}

void voxelSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, float voxel) {
	pcl::VoxelGrid<PointT> approximate_voxel_grid;
	approximate_voxel_grid.setLeafSize(voxel, voxel, voxel);
	approximate_voxel_grid.setInputCloud(in);
	approximate_voxel_grid.filter(*out);
}