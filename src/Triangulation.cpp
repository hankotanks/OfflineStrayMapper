#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h> // for KdTree
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char* argv[]) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPCDFile(argv[1], cloud_blob);
  pcl::fromPCLPointCloud2(cloud_blob, *cloud);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(0.025);

  // Set typical values for the parameters
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  pcl::visualization::PCLVisualizer viewer("3D Viewer");

  // Add the mesh to the viewer
  viewer.addPolygonMesh(triangles, "mesh");

  // Set the background color
  viewer.setBackgroundColor(0.0, 0.0, 0.0); // Black background

  // Set the viewer properties
  viewer.setRepresentationToSurfaceForAllActors(); // Set surface representation
  viewer.addCoordinateSystem(1.0); // Add a coordinate system

  // Start the visualization loop
  while (!viewer.wasStopped()) {
      viewer.spinOnce(100); // Spin the viewer
  }

  return (0);
}