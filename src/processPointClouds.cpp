// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT> ());

    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter(*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr crop_filtered (new pcl::PointCloud<PointT> ());

    typename pcl::CropBox< PointT > crp;
    crp.setInputCloud (cloud_filtered);
    crp.setMin (minPoint);
    crp.setMax (maxPoint);
    crp.filter(*crop_filtered);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return crop_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT> ());

    // Extract the inliers
    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    
    extract.setNegative (false);
    extract.filter (*planeCloud);
    
    //for(int index: inliers->indices){
    //    planeCloud->points.push_back(cloud->points(index));
    //}

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> 
ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    typename pcl::SACSegmentation<PointT> seg;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}

template<typename PointT>
pcl::PointIndices::Ptr ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    pcl::PointIndices::Ptr inliersResult(new pcl::PointIndices());
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while(maxIterations--)
	{
		int idx1 = rand() % cloud->points.size();
		int idx2 = rand() % cloud->points.size();
		int idx3 = rand() % cloud->points.size();

		PointT point1 = cloud->points[idx1];
		PointT point2 = cloud->points[idx2];
		PointT point3 = cloud->points[idx3];

		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		float x1 = point1.x;
		float y1 = point1.y;
		float z1 = point1.z;

		float x2 = point2.x;
		float y2 = point2.y;
		float z2 = point2.z;

		float x3 = point3.x;
		float y3 = point3.y;
		float z3 = point3.z;

		float aa = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		float bb = (z2 - z1)*(x3-x1) - (x2-x1)*(z3-z1);
		float cc = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		float dd = -(aa*x1 + bb*y1 + cc*z1);

		for (int i = 0; i < cloud->points.size(); i++)
		{
			PointT point = cloud->points[i];
			float d = fabs(aa*point.x + bb*point.y + cc*point.z + dd) / sqrt(aa*aa + bb*bb + cc*cc);
			if (d < distanceTol)
			{
				//inliers.insert(i);
                inliers->indices.push_back(i);
			}
		}
		//if (inliers.size()>inliersResult.size()){
        if (inliers->indices.size()>inliersResult->indices.size()){
			inliersResult = inliers;
		}
	}
	return inliersResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>
 ProcessPointClouds<PointT>::mySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                            int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
    
    auto inliers = ProcessPointClouds<PointT>::Ransac(cloud, maxIterations, distanceThreshold);
    
    if (inliers->indices.size() == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index: getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
template <typename PointT>
void ProcessPointClouds<PointT>::proximity(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, std::vector<int> &processed, int i, std::vector<int> &cluster, float distanceTol)
{
	processed.push_back(i);
	cluster.push_back(i);
	std::vector<int> nearby = tree->search({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, distanceTol);
	for (int index : nearby)
	{
		std::vector<int>::iterator it = std::find(processed.begin(), processed.end(), index);
		if (it == processed.end())
		{
			ProcessPointClouds<PointT>::proximity(cloud, tree, processed, index, cluster, distanceTol);
		}
	}
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree *tree, float distanceTol)
{
	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<int> processed;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		std::vector<int>::iterator it = std::find(processed.begin(), processed.end(), i);
		if (it == processed.end())
		{
			std::vector<int> cluster;
			ProcessPointClouds<PointT>::proximity(cloud, tree, processed, i, cluster, distanceTol);
			clusters.push_back(cluster);
		}
	}

	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    KdTree* tree = new KdTree;

    for (int i=0; i<cloud->points.size(); i++)
    	tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i); 

    std::vector<std::vector<int>> clustersID = ProcessPointClouds<PointT>::euclideanCluster(cloud, tree, clusterTolerance);

    for(std::vector<int> clusterID: clustersID)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice: clusterID)
  			clusterCloud->points.push_back(cloud->points[indice]);
        
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        if ((clusterCloud->points.size() < maxSize) && (clusterCloud->points.size() > minSize))
        {
            clusters.push_back(clusterCloud);
        }

  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
 