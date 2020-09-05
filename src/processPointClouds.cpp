// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

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
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>());
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);
    


    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int point:indices)
    inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstcloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planecloud (new pcl::PointCloud<PointT>());

    for (int index:inliers->indices)
     planecloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract ;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstcloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstcloud, planecloud);
    return segResult;
}
template<typename PointT>
void ProcessPointClouds<PointT>::Proximity (typename pcl::PointCloud<PointT>::Ptr cloud,std::vector<int> &cluster,std::vector<bool> &processed_f,
    int idx,typename KdTree_eculdian<PointT>::KdTree_eculdian *tree,float distanceTol,int maxSize)
{
  if ((processed_f[idx]==false)&&(cluster.size()<maxSize))
  {
      processed_f[idx]=true;
      cluster.push_back(idx);
      std::vector<int> nearby=tree->search(cloud->points[idx],distanceTol);
      for (int index:nearby)
      {
          if (processed_f[index]==false)
          {
              Proximity(cloud,cluster,processed_f,index,tree,distanceTol,maxSize);
          }
      }

  }
}
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr cofficient {new pcl::ModelCoefficients};

     seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    // TODO:: Fill in this function to find inliers for the cloud.
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*cofficient);
    if(inliers->indices.size()==0)
    {
        std::cout<<"could not estimate a planner model from data set "<<std::endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}



template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);
    for (pcl::PointIndices getIndices:clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr CloudCluster(new pcl::PointCloud<PointT>);
        for (int index:getIndices.indices)
        CloudCluster->points.push_back(cloud->points[index]);
        CloudCluster->width=CloudCluster->points.size();
        CloudCluster->height=1;
        CloudCluster->is_dense=true;
        clusters.push_back(CloudCluster);
        
    }
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}
  
  template<typename PointT>
  std::vector<std::vector<int>> ProcessPointClouds<PointT>::eculdianCluster(typename pcl::PointCloud<PointT>::Ptr cloud,typename KdTree_eculdian<PointT>::KdTree_eculdian *tree,float distanceTol,int minSize,int maxSize)
  {
std::vector<std::vector<int>> clusters;

std::vector<bool> processed_flag(cloud->points.size(),false);

for (int idx=0;idx<cloud->points.size();idx++)
{

    if (processed_flag[idx]==false)
    {
        std::vector<int> cluster;
        Proximity(cloud,cluster,processed_flag,idx,tree,distanceTol,maxSize);
        if ((cluster.size()>=minSize)&&(cluster.size()<=maxSize))
        clusters.push_back(cluster);
    }
}
return clusters;
  }
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::EculdianClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
  auto startTime = std::chrono::steady_clock::now();
  //ClusterPts<PointT> clusterPoints(cloud->points.size(),clusterTolerance,minSize,maxSize);
  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  typename KdTree_eculdian<PointT>::KdTree_eculdian *tree=new KdTree_eculdian<PointT>;
  tree->insert_cloud(cloud);

  std::vector<std::vector<int>> cluster_indicies=eculdianCluster(cloud,tree,clusterTolerance,minSize,maxSize);

  for (std::vector<std::vector<int>>::const_iterator it=cluster_indicies.begin();it!=cluster_indicies.end();++it)
  {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      for (std::vector<int>::const_iterator pit=it->begin();pit!=it->end();++pit)
      cloud_cluster->points.push_back(cloud->points[*pit]);

      cloud_cluster->width=cloud_cluster->points.size();
      cloud_cluster->height=1;
      cloud_cluster->is_dense=true;

      clusters.push_back(cloud_cluster);
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

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
 {
   auto startTime = std::chrono::steady_clock::now();

   std::unordered_set<int> inliersResult;
   srand(time(NULL));

   for (int t=0;t<maxIterations;t++)
   {
      std::unordered_set<int> inliers; 
      while(inliers.size() <3)
      {
          inliers.insert(rand()%cloud->points.size());
      }
      float x1,y1,z1,x2,y2,z2,x3,y3,z3;
      auto itr=inliers.begin();
      x1=cloud->points[*itr].x;
      y1=cloud->points[*itr].y;
      z1=cloud->points[*itr].z;
      itr++;
      x2=cloud->points[*itr].x;
      y2=cloud->points[*itr].y;
      z2=cloud->points[*itr].z;
      itr++;
      x3=cloud->points[*itr].x;
      y3=cloud->points[*itr].y;
      z3=cloud->points[*itr].z;

      float v11=x2-x1;
      float v12=y2-y1;
      float v13=z2-z1;
      float v21=x3-x1;
      float v22=y3-y1;
      float v23=z3-z1;

      float i,j,k;
      i=(v12*v23)-(v13*v22);
      j=(v13*v21)-(v11*v23);
      k=(v11*v22)-(v12*v21);

      float D=-1*(i*x1+j*y1+k*z1);

      for (int index=0;index <cloud->points.size();index++)
      {
          if (inliers.count(index)>0)
          continue;

          PointT point=cloud->points[index];
          float x4=point.x;
          float y4=point.y;
          float z4=point.z;

          float d=fabs(i*x4+j*y4+k*z4+D)/sqrt(i*i+j*j+k*k);

          if (d<=distanceThreshold)
          {
              inliers.insert(index);
          }



      }
      if (inliers.size() >inliersResult.size())
      {
          inliersResult=inliers;
      }
   }

   typename pcl::PointCloud<PointT>::Ptr inliers_cloud(new pcl::PointCloud<PointT>());
   typename pcl::PointCloud<PointT>::Ptr  outliers_cloud(new pcl::PointCloud<PointT>());

   for (int i=0;i<cloud->points.size();i++)
   {
       PointT point=cloud->points[i];
       if(inliersResult.count(i))
       {
           inliers_cloud->points.push_back(point);
       }
       else
       {
           outliers_cloud->points.push_back(point);
           
       }
       
   }

   auto endTime=std::chrono::steady_clock::now();

    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    return std::pair<typename pcl::PointCloud<PointT>::Ptr,typename pcl::PointCloud<PointT>::Ptr> (outliers_cloud,inliers_cloud);
 }