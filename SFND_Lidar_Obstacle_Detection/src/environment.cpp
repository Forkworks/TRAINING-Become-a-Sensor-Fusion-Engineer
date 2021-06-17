/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar * lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position, inputCloud);
    // renderPointCloud(viewer,inputCloud, "hello");
    // TODO:: Create point processor

    //ProcessPointClouds<pcl::PointXYZ> * pointProcessor = new ProcessPointClouds<pcl::PointXYZ>(); //HEAP
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;//Stack


    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first, "Obstcloud", Color(1,0,0));
    //renderPointCloud(viewer,segmentCloud.second, "planeCloud", Color(0,1,0));


    //Clustering

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }





}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    //Create Point Processor object
    //ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();  //Initiated on the Heap



    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud");


    // Show cropping Boxes
    Eigen::Vector4f minPoint(-15.0, -4.0, -3.0, 1.0) ;
    Eigen::Vector4f maxPoint( 20.0, 8.0, 1.0, 1 );
    Box box1 = pointProcessorI->CreateBox(minPoint,maxPoint);
    renderBox(viewer,box1,0,Color(0.2,0.2,0.2), 0.5);

    Eigen::Vector4f minPoint_2( -2, -2.5, -1.0, 1.0);
    Eigen::Vector4f maxPoint_2( 3, 2.5, 0.0, 1.0);
    Box box2 = pointProcessorI->CreateBox(minPoint_2,maxPoint_2);
    renderBox(viewer,box2,1,Color(0,0,1), 0.5);



    //---------- STEP 1: Filter Cloud (Cropping)---------
    //---------------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1 , minPoint, maxPoint, minPoint_2, maxPoint_2);
    //renderPointCloud(viewer,filterCloud,"filterCloud");


    //---------- STEP 2: Segment Cloud (Road and Obstacles)-----
    //----------------------------------------------------------
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.2);
    //renderPointCloud(viewer,segmentCloud.first, "Obstcloud", Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second, "planeCloud", Color(0,1,0));


    //--------- STEP 3: Clustering -----------------------------
    //----------------------------------------------------------
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 0.5, 15, 2000);


    //--------- STEP 4: Bounding Boxes -----------------------------
    //----------------------------------------------------------
    int clusterId = 2;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    srand (time(NULL));

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        int color_sel = rand() % colors.size();
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[color_sel]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId, colors[color_sel]);
        ++clusterId;
    }


}


void mycityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

    bool showcropBoxes = true;


    Eigen::Vector4f minPoint(-15.0, -5.0, -3.0, 1.0);
    Eigen::Vector4f maxPoint(20.0, 8.0, 1.0, 1);
    Eigen::Vector4f minPoint_2(-2, -2.0, -0.5, 1.0);
    Eigen::Vector4f maxPoint_2(3, 2.0, 0.0, 1.0);

    // Show cropping Boxes
    if (showcropBoxes)
    {

        Box box1 = pointProcessorI->CreateBox(minPoint, maxPoint);
        renderBox(viewer, box1, 0, Color(0.2, 0.2, 0.2), 0.5);


        Box box2 = pointProcessorI->CreateBox(minPoint_2, maxPoint_2);
        renderBox(viewer, box2, 1, Color(0, 0, 1), 0.5);
    }


    //---------- STEP 1: Filter Cloud (Cropping)---------
    //---------------------------------------------------
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud;
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1 , minPoint, maxPoint, minPoint_2, maxPoint_2);
    //renderPointCloud(viewer,filterCloud,"filterCloud");


    //---------- STEP 2: Segment Cloud (Road and Obstacles)-----
    //----------------------------------------------------------
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->mySegmentPlane(filterCloud, 100, 0.15);
    //renderPointCloud(viewer,segmentCloud.first, "Obstcloud", Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second, "planeCloud", Color(0,1,0));


    //--------- STEP 3: Clustering -----------------------------
    //----------------------------------------------------------
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->myClustering(segmentCloud.first, 0.5, 15, 2000);


    //--------- STEP 4: Bounding Boxes -----------------------------
    //----------------------------------------------------------
    int clusterId = 2;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};
    srand (time(NULL));

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        int color_sel = rand() % colors.size();
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[color_sel]);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId, colors[color_sel]);
        ++clusterId;
    }


}







//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);


    //Create Point Processor object
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();  //Initiated on the Heap

    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    //pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer,inputCloud,"inputCloud");


    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        mycityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    } 
}