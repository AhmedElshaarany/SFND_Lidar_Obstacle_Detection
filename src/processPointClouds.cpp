// PCL lib Functions for processing point clouds 
#include "processPointClouds.h"
#include "KdTree.h"

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
float ProcessPointClouds<PointT>::distanceToPlane(float A, float B, float C, float D,float x, float y, float z){
    //cout <<std::abs(A*x+B*y+C)/std::sqrt(A*A+B*B)<<endl; 
    return std::abs(A*x+B*y+C*z+D)/std::sqrt(A*A+B*B+C*C);
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    
    // TODO: Fill in this function
    int max_inliers = 0;
    // For max iterations 
    for(int i = 0; i < maxIterations; i++){
        // create temp inliers
        std::unordered_set<int> tempInliersResult;

        // two sampled points
        int cloud_size = cloud->size();
        int rand_idx1 = rand()%cloud_size;
        int rand_idx2 = rand()%cloud_size;
        int rand_idx3 = rand()%cloud_size;
        while(rand_idx2 == rand_idx1){
            rand_idx2 = rand()%cloud_size;
            
        }

        while(rand_idx1 == rand_idx3 || rand_idx2 == rand_idx3){
            rand_idx3 = rand()%cloud_size;
        }
        
        PointT sampled_point1 = cloud->points[rand_idx1]; 
        PointT sampled_point2 = cloud->points[rand_idx2];
        PointT sampled_point3 = cloud->points[rand_idx3]; 
        
        // Randomly sample subset and fit line
        
        float x1 = sampled_point1.x;
        float x2 = sampled_point2.x;
        float x3 = sampled_point3.x;
        float y1 = sampled_point1.y;
        float y2 = sampled_point2.y;
        float y3 = sampled_point3.y;
        float z1 = sampled_point1.z;
        float z2 = sampled_point2.z;
        float z3 = sampled_point3.z;

        float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
        float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
        float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
        float D = -(A*x1+B*y1+C*z1);


        // Measure distance between every point and fitted line
        // If distance is smaller than threshold count it as inlier
        for(int j = 0; j < cloud->size(); j++){
            float x = cloud->points[j].x;
            float y = cloud->points[j].y;
            float z = cloud->points[j].z;
            float d = distanceToPlane(A,B,C,D,x,y,z);
            if(d < distanceTol){
                tempInliersResult.insert(j);
            }
        }

        if(max_inliers < tempInliersResult.size()){
            max_inliers = tempInliersResult.size();
            inliersResult = tempInliersResult;
        }
    }
    
    // Return indicies of inliers from fitted line with most inliers
    
    return inliersResult;

}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    
    
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    // Create the filtering object
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    //sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    typename pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>);

    // Create new CropBox object on the heap
    pcl::CropBox<PointT> cropped_box (true);
    cropped_box.setMin(minPoint);
    cropped_box.setMax(maxPoint);
    cropped_box.setInputCloud(cloud_filtered);
    // apply filter
    cropped_box.filter(*cropped_cloud);

    
    std::vector<int> indices;
    pcl::CropBox<PointT> roof (true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cropped_cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point: indices)
        inliers->indices.push_back(point);

    typename pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cropped_cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter(*cropped_cloud);
    

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cropped_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // Create the filtering object
    typename pcl::ExtractIndices<PointT> extract;

    // create filter and plane clouds
    typename pcl::PointCloud<PointT>::Ptr plane_cloud (new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacle_cloud (new pcl::PointCloud<PointT>);

    // filter the plane cloud
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*plane_cloud);

    // Create the filtering object for the obstacle cloud
    extract.setNegative (true);
    extract.filter (*obstacle_cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}

/*
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // TODO:: Fill in this function to find inliers for the cloud.

    // create cofficients and inlier pointers
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}
*/

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
    // TODO:: Fill in this function to find inliers for the cloud.
    // get unordered set of inliers
    std::unordered_set<int> inliers = RansacPlane(cloud,maxIterations,distanceThreshold);

    // create empty vector of inliers and outliers
    typename pcl::PointCloud<PointT>::Ptr  plane_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr  obstacle_cloud(new pcl::PointCloud<PointT>());

    // fill inliers and outliers accordingly
    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            plane_cloud->points.push_back(point);
        else
            obstacle_cloud->points.push_back(point);
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle_cloud, plane_cloud);
    return segResult;
}



/*
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

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->points.push_back (cloud->points[*pit]); //*
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
*/




template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // create empty clusters
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // create new empty tree
    KdTree* tree = new KdTree;

    // create empty vector of points
    std::vector<std::vector<float>> points;

    for(int index = 0; index < cloud->points.size(); index++){
        std::vector<float> point;
        point.push_back(cloud->points[index].x);
        point.push_back(cloud->points[index].y);
        point.push_back(cloud->points[index].z);
        points.push_back(point);
        tree->insert(points[index],index);
    }

    // get cluster vectors 
    std::vector<std::vector<int>> cluster_vectors = tree->euclideanCluster(points, tree, clusterTolerance);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;


    // create point clouds from clusters vectors
    for(std::vector<int> cluster_vector : cluster_vectors)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int indice: cluster_vector){
            clusterCloud->points.push_back(cloud->points[indice]);
        }
        if(clusterCloud->points.size() >= minSize && clusterCloud->points.size() <= maxSize){
            clusters.push_back(clusterCloud);
        }
    }

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