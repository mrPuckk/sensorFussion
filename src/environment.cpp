/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
#include <algorithm>
#include <memory>
#include <utility>
#include <vector>
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer) {

  ProcessPointClouds<pcl::PointXYZI> *pointProcessor =
      new ProcessPointClouds<pcl::PointXYZI>();

  typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputPoint =
      pointProcessor->loadPcd("./src/sensors/data/pcd/data_1/0000000000.pcd");
  // renderPointCloud(viewer, inputPoint, "real_pcd");

  // Set the min/max Eigen::Vector4f(x, y, z, 1.0) values
  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud =
      pointProcessor->FilterCloud(inputPoint, 0.2,
                                  Eigen::Vector4f(-15, -6.0, -3, 1),
                                  Eigen::Vector4f(30, 6.0, 10, 1));
  renderPointCloud(viewer, filteredCloud, "filteredCloud");
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------

  // RENDER OPTIONS
  bool renderScene = false; // Toggle from True
  std::vector<Car> cars = initHighway(renderScene, viewer);

  // // TODO: ✔️ Create lidar sensor
  // const int gndSlope = 0;

  // std::unique_ptr<Lidar> lidar = std::make_unique<Lidar>(cars, gndSlope);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud = lidar->scan();

  // /**
  //  * Toggle to see the lidar rays
  //  */
  // // renderRays(viewer, lidar->position, ptrCloud);

  // /**
  //  * Toggle to see Pointclouds
  //  */
  // renderPointCloud(viewer, ptrCloud, "pointCloud");

  // std::unique_ptr<ProcessPointClouds<pcl::PointXYZ>> pointProcessor =
  //     std::make_unique<ProcessPointClouds<pcl::PointXYZ>>();

  // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr,
  //           pcl::PointCloud<pcl::PointXYZ>::Ptr>
  //     segmentCloud = pointProcessor->SegmentPlane(ptrCloud, 100, 0.2);

  // renderPointCloud(viewer, segmentCloud.first, "inliers", Color(1, 0, 0));
  // renderPointCloud(viewer, segmentCloud.second, "outliers", Color(0, 1, 0));

  // // Cluster the obstacles
  // // TODO:: Create point processor
  // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters =
  //     pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

  // int clusterId = 0;
  // std::vector<Color> colors = {Color(1, 1, 0), Color(1, 0, 0.5),
  //                              Color(1, 0.5, 0), Color(0.7, 0.4, 1)};

  // for (const auto &cluster : cloudClusters) {
  //   std::cout << "cluster size: ";
  //   pointProcessor->numPoints(cluster);
  //   renderPointCloud(viewer, cluster,
  //                    "obstacle cloud" + std::to_string(clusterId),
  //                    colors[clusterId]);
  //   renderBox(viewer, pointProcessor->BoundingBox(cluster), clusterId);
  //   ++clusterId;
  // }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 16;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));

  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
  // simpleHighway(viewer);
  cityBlock(viewer);

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
