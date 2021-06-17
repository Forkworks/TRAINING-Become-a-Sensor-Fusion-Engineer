/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{

    auto startTime = std::chrono::steady_clock::now();


	std::unordered_set<int> inliersResult;
    std::unordered_set<int> inliersTemp;

    srand(time(NULL));
	
	// TODO: Fill in this function


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

        /*   2D
        A = y1 - y2;
        B = x2 - x1;
        C = x1*y2 - x2*y1;
        */


        // Measure distance between every point and fitted line (plane)
        for (int j=0; j<size_pcl; j++)
        {
            // If distance is smaller than threshold count it as inlier
            pcl::PointXYZ point = cloud->points[j];


            //------  3D  -------
            //------------------
            distance = fabs(A*point.x + B*point.y + C*point.z + D)/ sqrt(pow(A,2) + pow(B,2) + pow(C, 2));


            //------  2D  -------
            //------------------
            //distance = fabs(A*point.x + B*point.y + C)/ sqrt(pow(A,2) + pow(B,2));


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



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "My Ransac took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
