/* \author Aaron Brown, Benjamin Söllner */
// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(
        typename pcl::PointCloud<PointT>::Ptr cloud
    )
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float filterRes,
        Eigen::Vector4f minPoint,
        Eigen::Vector4f maxPoint
    )
{
    // voxel grid point reduction: downsample the datadset using a leaf size of .2m
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes);
    voxelGrid.filter(*filteredCloud);

    // region based filtering
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropBox(true);
    cropBox.setInputCloud(filteredCloud);
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.filter(*croppedCloud);

    // substract roof
    typename pcl::PointCloud<PointT>::Ptr extractedCloud(new pcl::PointCloud<PointT>);
    std::vector<int> roofIndices;
    pcl::CropBox<PointT> cropRoof(true);
    cropRoof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    cropRoof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    cropRoof.setInputCloud(croppedCloud);
    cropRoof.filter(roofIndices);
    pcl::PointIndices::Ptr roofPointIndices(new pcl::PointIndices);
    for (int roofIndex: roofIndices)
    {
        roofPointIndices->indices.push_back(roofIndex);
    }
    pcl::ExtractIndices<PointT> extractRoof;
    extractRoof.setInputCloud(croppedCloud);
    extractRoof.setIndices(roofPointIndices);
    extractRoof.setNegative(true);
    extractRoof.filter(*extractedCloud);
    return extractedCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers,
        typename pcl::PointCloud<PointT>::Ptr cloud
    ) 
{
    // create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);
    
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.filter(*planeCloud);
    extract.setNegative(true);
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::vector<int> ProcessPointClouds<PointT>::Ransac3D(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        int maxIterations,
        float distanceTol
    )
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// For max iterations 
	while (maxIterations--)
	{

		// Randomly sample subset
		std::unordered_set<int> inliers;
		while (inliers.size() < 3) {
			inliers.insert(rand()%(cloud->points.size()));
        }
		PointT p1, p2, p3, v1, v2;
		auto itr = inliers.begin();
		p1 = cloud->points[*itr];
		itr++;
		p2 = cloud->points[*itr];
		itr++;
		p3 = cloud->points[*itr];

		// fit plane
		float a = (p2.y-p1.y)*(p3.z-p1.z)-(p2.z-p1.z)*(p3.y-p1.y);
		float b = (p2.z-p1.z)*(p3.x-p1.x)-(p2.x-p1.x)*(p3.z-p1.z);
		float c = (p2.x-p1.x)*(p3.y-p1.y)-(p2.y-p1.y)*(p3.x-p1.x);
		float d = -(a*p1.x+b*p1.y+c*p1.z);

		// Measure distance between every point and fitted line
		for (int index = 0; index < cloud->points.size(); index++) 
		{
			if (inliers.count(index)>0)
            {
				continue;
            }
			PointT point = cloud->points[index];
			float x = point.x;
			float y = point.y;
			float z = point.z;
			float dist = fabs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);
			// If distance is smaller than threshold count it as inlier
			if (dist <= distanceTol)
            {
				inliers.insert(index);
            }
		}
		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
		}

	}

	// Return indicies of inliers from fitted line with most inliers
    std::vector<int> inliersResultAsVector;
	inliersResultAsVector.assign(inliersResult.begin(), inliersResult.end());
	return inliersResultAsVector;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        int maxIterations,
        float distanceThreshold
    )
{
    // find inliers for the cloud.
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());;

    // segment the largest planar component from the input cloud

    // // PointCloud Library Way of doing things
    // pcl::SACSegmentation<PointT> seg;
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // seg.setOptimizeCoefficients (true);
    // seg.setModelType(pcl::SACMODEL_PLANE);
    // seg.setMethodType(pcl::SAC_RANSAC);
    // seg.setMaxIterations(maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);
    // seg.setInputCloud(cloud);
    // seg.segment(*inliers, *coefficients);

    // use own implemented RANSAC since project rubric requires it
    inliers->indices = Ransac3D(cloud, maxIterations, distanceThreshold);

    if (inliers->indices.size() == 0) 
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers, cloud);
    return segResult;
}


template<typename PointT>
void ProcessPointClouds<PointT>::EuclideanClusterProximity(
        typename pcl::PointCloud<PointT>::Ptr cloud,
		int pointIndex, 
		pcl::PointIndices& cluster,
		std::vector<bool>& wasPointProcessed, 
		KdTree<PointT>* tree, 
		float distanceTol
	)
{
	// mark point as processed
	wasPointProcessed[pointIndex] = true;
	// add point to cluster
	cluster.indices.push_back(pointIndex);
	// nearby points = tree(point);
	std::vector<int> nearbyIndices = tree->search(cloud->points[pointIndex], distanceTol);
	// iterate through each nearby point
	for (int nearbyIndex: nearbyIndices) 
	{
		// if point has not been processed
		if (!wasPointProcessed[nearbyIndex])
		{
			// Proximity(cluster)
			EuclideanClusterProximity(cloud, nearbyIndex, cluster, wasPointProcessed, tree, distanceTol);
		}
	}
}



template<typename PointT>
std::vector<pcl::PointIndices> ProcessPointClouds<PointT>::EuclideanCluster(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        KdTree<PointT>* tree,
        float distanceTol
    )
{
	// list of clusters
	std::vector<pcl::PointIndices> clusters;
	// Iterate through each point
	std::vector<bool> wasPointProcessed(cloud->points.size(), false);
	int pointIndex = 0;
	while (pointIndex < cloud->points.size())
	{
		// if point has not been processed
		if (wasPointProcessed[pointIndex])
		{
			pointIndex++;
			continue;
		}
		else
		{
			// create cluster
			pcl::PointIndices cluster;
			// Proximity(point, cluster)
			EuclideanClusterProximity(cloud, pointIndex, cluster, wasPointProcessed, tree, distanceTol);
			// add cluster to clusters
			clusters.push_back(cluster);
		}
	}
	return clusters;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        float clusterTolerance
    )
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;


    // // PointCloud Library Way of doing things
    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud (cloud);
    // std::vector<pcl::PointIndices> clustersIndices;
    // pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance(clusterTolerance);
    // ec.setMinClusterSize(50);
    // ec.setMaxClusterSize(1000);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud);
    // ec.extract(clustersIndices);
    
    // use own implemented euclidean clustering since project rubric requires it
    KdTree<PointT>* tree = new KdTree<PointT>;
    for (int i = 0; i < cloud->points.size(); i++)
    {
    	tree->insert(cloud->points[i], i); 
    }
    std::vector<pcl::PointIndices> clustersIndices = EuclideanCluster(cloud, tree, clusterTolerance);

    for (pcl::PointIndices clusterIndices : clustersIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
        for (int index : clusterIndices.indices)
            cluster->points.push_back(cloud->points[index]);
        cluster->width = cluster->points.size();
        cluster->height = 1;
        clusters.push_back(cluster);
    }

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
        typename pcl::PointCloud<PointT>::Ptr cluster
    )
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
void ProcessPointClouds<PointT>::savePcd(
        typename pcl::PointCloud<PointT>::Ptr cloud,
        std::string file
    )
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(
        std::string file
    )
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(
        std::string dataPath
    )
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}