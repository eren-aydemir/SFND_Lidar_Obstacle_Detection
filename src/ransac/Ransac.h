#ifndef RANSAC_H
#define RANSAC_H

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>

template<typename PointT>
pcl::PointIndices::Ptr Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

#endif /* RANSAC_H */
