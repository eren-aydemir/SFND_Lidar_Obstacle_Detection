
#include "Ransac.h"

template<typename PointT>
pcl::PointIndices::Ptr Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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

		pcl::PointXYZ point1 = cloud->points[idx1];
		pcl::PointXYZ point2 = cloud->points[idx2];
		pcl::PointXYZ point3 = cloud->points[idx3];

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
			pcl::PointXYZ point = cloud->points[i];
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
