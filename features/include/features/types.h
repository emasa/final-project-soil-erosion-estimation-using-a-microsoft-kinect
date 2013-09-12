#ifndef TYPES_H
#define TYPES_H

#include <memory>

#include <pcl/correspondence.h>

struct MatchesInfo
{
	typedef std::shared_ptr<MatchesInfo> Ptr;
	typedef std::shared_ptr<const MatchesInfo> ConstPtr;

	MatchesInfo() : matches(new pcl::Correspondences) {}

	int src_idx, tgt_idx;
    pcl::CorrespondencesPtr matches;
    Eigen::Matrix4f transformation;
};

#endif // TYPES_H