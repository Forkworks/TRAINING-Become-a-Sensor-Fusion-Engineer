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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::Tracking(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, Eigen::Vector4f minPoint_2, Eigen::Vector4f maxPoint_2)
{
    pcl::tracking::KLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA,pcl::tracking::ParticleXYZRPY>::Ptr tracker
            (new pcl::tracking::KLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::tracking::ParticleXYZRPY> (8));

    //Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
    tracker->setMaximumParticleNum (1000);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilter
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);



}









template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, Eigen::Vector4f minPoint_2, Eigen::Vector4f maxPoint_2)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    //pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);

    //typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    //Create Region Cloud
    pcl::CropBox<PointT> crop_box;
    crop_box.setMax(maxPoint);
    crop_box.setMin(minPoint);
    crop_box.setInputCloud(cloud_filtered);
    crop_box.filter(*cloud_filtered);


    //Filter out points close to sensor
    pcl::CropBox<PointT> crop_box2;
    crop_box2.setNegative(true);
    crop_box2.setMin(minPoint_2);
    crop_box2.setMax(maxPoint_2);
    crop_box2.setInputCloud(cloud_filtered);
    crop_box2.filter(*cloud_filtered);



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << " Filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane


    typename pcl::PointCloud<PointT>::Ptr road ( new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacles ( new pcl::PointCloud<PointT>);

    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*road);
    //std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
    extract.setNegative(true);
    extract.filter(*obstacles);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::myRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    // TODO: Fill in this function

    std::unordered_set<int> inliersResult;
    std::unordered_set<int> inliersTemp;

    float A;
    float B;
    float C;
    float D;
    float distance;

    int countMax = 0;

    srand (time(NULL));


    int size_pcl = cloud->points.size();

    // For max iterations
    for (int i=0; i<maxIterations; i++)
    {
        int count = 0;
        inliersTemp.clear();
        // Randomly sample subset and fit line
        int point1 = rand() % size_pcl;
        int point2 = rand() % size_pcl;
        int point3 = rand() % size_pcl;

        float x1 = cloud->points[point1].x;
        float y1 = cloud->points[point1].y;
        float z1 = cloud->points[point1].z;

        float x2 = cloud->points[point2].x;
        float y2 = cloud->points[point2].y;
        float z2 = cloud->points[point2].z;

        float x3 = cloud->points[point3].x;
        float y3 = cloud->points[point3].y;
        float z3 = cloud->points[point3].z;

        //std::vector<float> v1 = cloud->points[point2] - cloud->points[point1] ;
        //std::vector<float> v2 = cloud->points[point3] - cloud->points[point1] ;

        A = (y2 - y1)*(z3 - z1) - (z2-z1)*(y3-y1);
        B = (z2 - z1)*(x3 - x1) - (x2-x1)*(z3-z1);
        C = (x2 - x1)*(y3 - y1) - (y2-y1)*(x3-x1);
        D = -1*(A*x1 + B*y1 + C*z1);

        // Measure distance between every point and fitted line (plane)
        for (int j=0; j<size_pcl; j++)
        {
            // If distance is smaller than threshold count it as inlier
            PointT point = cloud->points[j];

            distance = fabs(A*point.x + B*point.y + C*point.z + D)/ sqrt(pow(A,2) + pow(B,2) + pow(C, 2));

            if (distance < distanceTol)
            {
                inliersTemp.insert(j);
                count++;
            }

        }

        if (count > countMax)
        {
            countMax = count;
            inliersResult = inliersTemp;
        }
    }


    // Return indicies of inliers from fitted plane with most inliers

    return inliersResult;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::mySegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO: Change the max iteration and distance tolerance arguments for Ransac function
    std::unordered_set<int> inliers = myRansac(cloud, maxIterations, distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());


    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if(inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }


    if (inliers.empty())
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << " My plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult.first = cloudOutliers;
    segResult.second = cloudInliers;
    return segResult;
}



template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients (true);

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

template<typename PointT>
void ProcessPointClouds<PointT>::Proximity(const std::vector<std::vector<float>> &points, std::vector<int> &cluster, std::vector<int> &mark_vec, int i, KdTree* tree, float distanceTol)
{
    mark_vec[i] = 1;
    cluster.push_back(i);

    std::vector<int> nearby = tree->search(points[i],distanceTol);

    for (int j=0; j<nearby.size(); j++)
    {
        if (mark_vec[nearby[j]] == 0)
        {
            Proximity( points, cluster, mark_vec, nearby[j], tree , distanceTol);
        }
    }


}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::vector<int> mark_vec(points.size(), 0.0);

    for (int i=0; i<points.size(); i++)
    {

        if (mark_vec[i] == 0)
        {
            std::vector<int> cluster;
            Proximity(points, cluster, mark_vec, i, tree, distanceTol);
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

    //typename pcl::PointCloud<PointT>::Ptr clust_pcl ( new pcl::PointCloud<PointT>);


    KdTree* tree = new KdTree;

    std::vector<std::vector<float>> vec_points;

    for (int i=0; i<cloud->points.size(); i++)
    {
        std::vector<float> vecf_point = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
        tree->insert(vecf_point ,i);
        vec_points.push_back(vecf_point);

    }


    std::vector<std::vector<int>> clusters_indices = euclideanCluster(vec_points, tree, clusterTolerance);



    for(std::vector<int> cluster : clusters_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int indice: cluster)
            clusterCloud->points.push_back(cloud->points[indice]);

        if (clusterCloud->size() > minSize and clusterCloud->size() < maxSize)
            clusters.push_back(clusterCloud);

    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << " My clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}








template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    //typename pcl::PointCloud<PointT>::Ptr clust_pcl ( new pcl::PointCloud<PointT>);


    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;



    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);


    for (int i = 0; i < cluster_indices.size();i++)   //for (auto & index : cluster_indices)
    {
        //clust_pcl->clear();
        typename pcl::PointCloud<PointT>::Ptr clust_pcl ( new pcl::PointCloud<PointT>);
        //clust_pcl.reset();

        for (int j=0; j < cluster_indices[i].indices.size();j++)
        {
            clust_pcl->points.push_back(cloud->points[cluster_indices[i].indices[j]]);
        }

        clust_pcl->width = clust_pcl->size ();
        clust_pcl->height = 1;
        clust_pcl->is_dense = true;
        /*
        pointer_indices = &cluster_indices[i];
        extract.setInputCloud (cloud);

        extract.setIndices (&cluster_indices[i]);
        extract.setNegative (false);
        extract.filter (*clust_pcl);
        */

        clusters.push_back(clust_pcl);

        //clusters.push_back(cloud->points[cluster_indices[i]]);
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
Box ProcessPointClouds<PointT>::CreateBox(Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{


    Box box;
    box.x_min = minPoint.x();
    box.y_min = minPoint.y();
    box.z_min = minPoint.z();
    box.x_max = maxPoint.x();
    box.y_max = maxPoint.y();
    box.z_max = maxPoint.z();

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