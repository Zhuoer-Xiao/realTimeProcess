#pragma once
#include "pclHeaders.h"
#include <pcl/features/esf.h>
#include <mutex>
class Matcher {
public:
	int addNewFeature(PointCloudT::Ptr& cloud);
	int match(PointCloudT::Ptr& cloud);
private:
	std::vector<pcl::ESFSignature640> featureList;
	std::mutex listMutex;
	double computeEuclideanDistance(pcl::ESFSignature640 hist1, pcl::ESFSignature640 hist2);
};