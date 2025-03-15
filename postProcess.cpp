#include "postProcess.h"
void randomSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out, int num) {
	pcl::RandomSample<PointT> rs;
	rs.setInputCloud(in);				//���ô��˲�����
	rs.setSample(num);					//�����²������Ƶĵ���
	//rs.setSeed(1);						//��������������ӵ�
	rs.filter(*out);
}

void uniformSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
	pcl::UniformSampling<PointT> us;
	us.setInputCloud(in);
	us.setRadiusSearch(0.01);
	us.filter(*out);
}

void MLSSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
	pcl::PointCloud<pcl::PointNormal>::Ptr smoothedCloud(new pcl::PointCloud<pcl::PointNormal>);   //���巨��
	pcl::MovingLeastSquares<PointT, pcl::PointNormal> filter;
	pcl::search::KdTree<PointT>::Ptr kdtree;  //������������
	filter.setInputCloud(in);    //�����������
	filter.setSearchRadius(10);// ������ϵ�K���ڰ뾶��������뾶����б���ӳ���������ϡ��뾶ԽС��Ϻ������ʧ���ԽС����֮�п��ܳ��ֹ���ϵ�����
	filter.setComputeNormals(true);  // �Ƿ�洢���Ƶķ�������true Ϊ�洢��false ���洢
	filter.setSearchMethod(kdtree); //������������
	filter.process(*smoothedCloud); //������Ʋ����
}

void newSample(PointCloudT::Ptr& in, PointCloudT::Ptr& out) {
	// ��ȡISS������
    // --------------------------- 1. ��ȡ������� ---------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->points.resize(in->points.size());
    for (int i = 0; i < in->points.size(); i++) {
        out->points[i].x = in->points[i].x;
        out->points[i].y = in->points[i].y;
        out->points[i].z = in->points[i].z;
    }


    // --------------------------- 2. ISS�ؼ����⣨����Ӧֵ�� ---------------------------
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
    // --------------------------- 3. ����Ӧ����뾶��ǹؼ����� ---------------------------
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

    // --------------------------- 4. �������� ---------------------------
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
    // --------------------------- 5. �ؼ�������ܶȲ���������80%�� ---------------------------
    pcl::RandomSample<pcl::PointXYZ> key_sampler;
    key_sampler.setInputCloud(key_cloud);
    key_sampler.setSample(key_cloud->size() * p_key);
    key_sampler.filter(*key_cloud);

    // --------------------------- 6. �ǹؼ�������ʽ������ ---------------------------
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
    // --------------------------- 7. �ϲ��뱣�� ---------------------------
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

	// ���㷨�ߣ��������ʣ�
	pcl::NormalEstimation<PointT, pcl::Normal> ne;
	ne.setInputCloud(cloud_filtered);
	pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	ne.setKSearch(30); // ����ʵ���������Kֵ
	ne.compute(*normals);

	// �ռ����е������
	std::vector<float> curvatures;
	for (size_t i = 0; i < normals->size(); ++i)
	{
		curvatures.push_back(normals->points[i].curvature);
	}

	// �����ʷ�Ϊ10������
	std::vector<std::vector<int>> curvature_intervals(10);
	float min_curvature = *std::min_element(curvatures.begin(), curvatures.end());
	float max_curvature = *std::max_element(curvatures.begin(), curvatures.end());
	float interval_size = (max_curvature - min_curvature) / 10.0;

	for (size_t i = 0; i < curvatures.size(); ++i)
	{
		int index = std::min(int((curvatures[i] - min_curvature) / interval_size), 9); // ȷ�����ֵ�������һ������
		curvature_intervals[index].push_back(i);
	}

	// ���������������ʣ�Խ������������Խ��
	PointCloudT::Ptr sampled_cloud(new PointCloudT);
	for (int i = 0; i < 10; ++i)
	{
		float sample_rate = static_cast<float>(i + 1) / 10.0; // ���Ӳ�����
		for (auto idx : curvature_intervals[i])
		{
			if ((rand() % 1000) / 1000.0 < sample_rate) // ����ж��Ƿ����
			{
				sampled_cloud->push_back(cloud_filtered->points[idx]);
			}
		}
	}

	// ��������㲻��20000������ͨ�������������
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