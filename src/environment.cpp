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
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,ProcessPointClouds<pcl::PointXYZI>* pointprocessor ,pcl::PointCloud<pcl::PointXYZI>::Ptr Filter)
{
    //pcl::PointCloud<pcl::PointXYZI>::Ptr Filter=pointprocessor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    float filterRes=0.3;
    Eigen::Vector4f minpoint(-22,-6,-3,1);
    Eigen::Vector4f maxpoint(28,6,4,1);

    auto filteredCloud =pointprocessor->FilterCloud(Filter,filterRes,minpoint,maxpoint);

    int maxIter=100;
    float distanceTolerance=0.2;
  //pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud= pointprocessor->FilterCloud(Filter,0.3,Eigen::Vector4f(-10,-5,-2,1),Eigen::Vector4f(30,8,1,1));
  //renderPointCloud(viewer,inputcloud,"cloud");
    //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentcloud=pointprocessor->SegmentPlane(filteredCloud,25,0.3);
     std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentcloud=pointprocessor->RansacPlane(filteredCloud,maxIter,distanceTolerance);

renderPointCloud(viewer,segmentcloud.second,"filteredPlanedcloud",Color(0,1,0));
//std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters=pointprocessor->Clustering(segmentcloud.first,0.4,30,5000);
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters=pointprocessor->EculdianClustering(segmentcloud.first,0.3,10,1000);


int clusterId=0;
std::vector<Color> colors={Color(1,0,0),Color(1,1,0),Color(0,0,1)};
for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster:cloudClusters)
{
    //std::cout<<"Cluster size"<<std::endl;
    //pointprocessor->numPoints(cluster);
    renderPointCloud(viewer,cluster,"obstcloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
    Box box=pointprocessor->BoundingBox(cluster);
    renderBox(viewer,box,clusterId);
   ++clusterId;
}
  
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
    Lidar *lidar=new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud=lidar->scan();
      //renderRays(viewer,lidar->position,input_cloud);
    renderPointCloud(viewer,input_cloud,"cloud");    
   
    // TODO:: Create point processor
  ProcessPointClouds<pcl::PointXYZ> pointprocessor;
     std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,  pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentcloud=pointprocessor.SegmentPlane(input_cloud,100,0.2);
    //renderPointCloud(viewer,segmentcloud.first,"obstcloud",Color(1,0,0));   
    //renderPointCloud(viewer,segmentcloud.second,"Planecloud",Color(0,1,0)); 
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters=pointprocessor.Clustering(segmentcloud.first,1.0,3,30);
int clusterId=0;
std::vector<Color> colors={Color(1,0,0),Color(1,1,0),Color(0,0,1)};
for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster:cloudClusters)
{
    std::cout<<"Cluster size"<<std::endl;
    pointprocessor.numPoints(cluster);
    renderPointCloud(viewer,cluster,"obstcloud"+std::to_string(clusterId),colors[clusterId%colors.size()]);
        Box box=pointprocessor.BoundingBox(cluster);
    renderBox(viewer,box,clusterId);
    ++clusterId;
}
    renderPointCloud(viewer,segmentcloud.second,"Planecloud",Color(0,1,0));
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
     ProcessPointClouds<pcl::PointXYZI>* pointprocessor=new   ProcessPointClouds<pcl::PointXYZI>() ;
    std::vector<boost::filesystem::path> stream =pointprocessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamiterator=stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    
    //  simpleHighway(viewer);
    //cityBlock(viewer,pointprocessor,inputCloudI);

    while (!viewer->wasStopped ())
    {
      //  viewer->spinOnce ();
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        inputCloudI=pointprocessor->loadPcd((*streamiterator).string());
 cityBlock(viewer,pointprocessor,inputCloudI);
        streamiterator++;
        if (streamiterator==stream.end())
        {
            streamiterator=stream.begin();
        }
        viewer->spinOnce();
    } 
}