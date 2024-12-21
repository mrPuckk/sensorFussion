// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <algorithm>
#include <iterator>
#include <memory>
#include <utility>

// constructor:
template <typename PointT> ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template <typename PointT> ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud) {
  std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
    Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO: Fill in the function to do voxel grid point reduction and region
  // based filtering

  // Voxel grid point reduction
  typename pcl::PointCloud<PointT>::Ptr cloudVGFiltered{
      new pcl::PointCloud<PointT>};
  pcl::VoxelGrid<PointT> voxelGridFilter;
  voxelGridFilter.setInputCloud(cloud);
  voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);
  voxelGridFilter.filter(*cloudVGFiltered);

  // Region of Interest (ROI) based filtering
  typename pcl::PointCloud<PointT>::Ptr cloudBoxFiltered{
      new pcl::PointCloud<PointT>};
  pcl::CropBox<PointT> cropBoxFilter(true);
  cropBoxFilter.setMin(minPoint);
  cropBoxFilter.setMax(maxPoint);
  cropBoxFilter.setInputCloud(cloudVGFiltered);
  cropBoxFilter.filter(*cloudBoxFiltered);

  // Get the indices of rooftop points
  std::vector<int> indices;
  pcl::CropBox<PointT> roofFilter(true);
  roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
  roofFilter.setInputCloud(cloudBoxFiltered);
  roofFilter.filter(indices);

  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (int point : indices)
    inliers->indices.push_back(point);

  // Remove the rooftop indices
  typename pcl::PointCloud<PointT>::Ptr cloudFiltered{
      new pcl::PointCloud<PointT>};
  pcl::ExtractIndices<PointT> extract;
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.setInputCloud(cloudBoxFiltered);
  extract.filter(*cloudFiltered);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
            << std::endl;

  return cloudFiltered;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers,
    typename pcl::PointCloud<PointT>::Ptr cloud) {

  // TODO: Create two new point clouds, one cloud with obstacles and other with
  // segmented plane

  typename pcl::PointCloud<PointT>::Ptr cloud_obs(
      new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr cloud_pln(
      new pcl::PointCloud<PointT>());

  // Iterate through the inliers and populate the inlier point cloud
  for (const auto &index : inliers->indices) {
    cloud_pln->points.push_back(cloud->points[index]);
  }

  pcl::ExtractIndices<PointT> extractMod;
  extractMod.setInputCloud(cloud);
  extractMod.setIndices(inliers);
  extractMod.setNegative(true);
  extractMod.filter(*cloud_obs);

  return std::make_pair(cloud_obs, cloud_pln);
}

/**
 *@brief find inliers from the cloud
 *
 *@tparam PointT
 *@param cloud
 *@param maxIterations
 *@param distanceThreshold
 *@return pair< , >segResult = SeperateClouds(inliers, cloud)
 */
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
          typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
    float distanceThreshold) {
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in this function to find inliers for the cloud.

  // Create the segmentation object
  pcl::SACSegmentation<PointT> get_inliers;
  // Define inliers var
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices()};
  // Optional
  get_inliers.setOptimizeCoefficients(true);
  // Mandatory
  get_inliers.setModelType(pcl::SACMODEL_PLANE);
  get_inliers.setMethodType(pcl::SAC_RANSAC);
  get_inliers.setDistanceThreshold(distanceThreshold);
  // Create cofficients object
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  // set input to model
  get_inliers.setInputCloud(cloud);
  // set output {inliers}
  get_inliers.segment(*inliers, *coefficients);

  // Handle the case where the plane model cannot be estimated
  if (inliers->indices.size() == 0) {
    throw std::runtime_error(
        "Failed to estimate a planar model for the given dataset.");
  }

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count()
            << " milliseconds" << std::endl;

  return SeparateClouds(inliers, cloud);
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
    int minSize, int maxSize) {

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to perform euclidean clustering to group
  // detected obstacles

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::vector<pcl::PointIndices> clusterIndices;
  // Create a KdTree object for the search method of the extraction
  typename pcl::search::KdTree<PointT>::Ptr tree(
      new pcl::search::KdTree<PointT>);
  tree->setInputCloud(cloud);

  typename pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance(clusterTolerance);
  ec.setMinClusterSize(minSize);
  ec.setMaxClusterSize(maxSize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusterIndices);

  // Use std::transform to convert pcl::PointIndices to pcl::PointCloud
  std::transform(clusterIndices.begin(), clusterIndices.end(),
                 std::back_inserter(clusters),
                 // lambda func
                 [&cloud](const pcl::PointIndices &indices) {
                   typename pcl::PointCloud<PointT>::Ptr cluster(
                       new pcl::PointCloud<PointT>);
                   pcl::copyPointCloud(*cloud, indices.indices, *cluster);
                   return cluster;
                 });

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
      endTime - startTime);
  std::cout << "clustering took " << elapsedTime.count()
            << " milliseconds and found " << clusters.size() << " clusters"
            << std::endl;

  return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
    typename pcl::PointCloud<PointT>::Ptr cluster) {

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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(
    typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
  pcl::io::savePCDFileASCII(file, *cloud);
  std::cerr << "Saved " << cloud->points.size() << " data points to " + file
            << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

  typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

  if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
  {
    PCL_ERROR("Couldn't read file \n");
  }
  std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
            << std::endl;

  return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

  std::vector<boost::filesystem::path> paths(
      boost::filesystem::directory_iterator{dataPath},
      boost::filesystem::directory_iterator{});

  // sort files in accending order so playback is chronological
  sort(paths.begin(), paths.end());

  return paths;
}
