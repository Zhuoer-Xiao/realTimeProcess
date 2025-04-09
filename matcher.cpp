#include "matcher.h"
int Matcher::addNewFeature(PointCloudT::Ptr& cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    toXYZ(cloud, cloud_xyz);
    
    // Compute the feature
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
    esf.setInputCloud(cloud_xyz);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    esf.setSearchMethod(tree);
    pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_features(new pcl::PointCloud<pcl::ESFSignature640>);
    esf.compute(*esf_features);
    // Add the feature to the feature list
    std::unique_lock<std::mutex> lck(listMutex);
    featureList.push_back(esf_features->points[0]);
}

double Matcher::computeEuclideanDistance(pcl::ESFSignature640 hist1, pcl::ESFSignature640 hist2) {
    double dist = 0.0;
    for (int i = 0; i < 640; ++i) { // ESF特征大小是153
        dist += std::pow(hist1.histogram[i] - hist2.histogram[i], 2);
    }
    return std::sqrt(dist);
}

int Matcher::match(PointCloudT::Ptr& cloud) {
    if (featureList.size() == 0) {
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    toXYZ(cloud, cloud_xyz);
   
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
    esf.setInputCloud(cloud_xyz);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    esf.setSearchMethod(tree);
    pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_features(new pcl::PointCloud<pcl::ESFSignature640>);
    esf.compute(*esf_features);
    float distance_res = 10.0f;
    int match_index = -1;
	for (size_t j = 0; j < featureList.size(); ++j)
	{
		// 比较与其他测试点云的相似度
		float distance = computeEuclideanDistance(esf_features->points[0], featureList[j]);
        if (distance < distance_res) {
            distance_res = distance;
            match_index = j;
        }
	}
    return match_index;
}