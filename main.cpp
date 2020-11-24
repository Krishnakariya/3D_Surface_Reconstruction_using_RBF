#define EPSILON 0.0001f
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/distances.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace pcl;
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Euclidean_matrix;
Eigen::MatrixXf rbf_matrix;
Eigen::VectorXf weights;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr pos_cloud; 
pcl::PointCloud<pcl::PointXYZ>::Ptr nos_cloud;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

void computeEuclideanDistanceMatrix(){
  std::cout << "Computing RBF_MATRIX" << "\n";
  size_t cloud_size = (cloud->size()*3);
  size_t matrix_size = cloud_size;
  std::cout << "Matrix Size: " << matrix_size << "\n";
  Euclidean_matrix.resize(matrix_size, matrix_size);
  std::cout << "" << Euclidean_matrix.rows()<< " " << Euclidean_matrix.cols() << "\n";
  Eigen::VectorXf distance (matrix_size);
  for(int i=0; i<cloud_size; i++){
    pcl::PointXYZ p1;
    int ind1 = i/3;
    if(i%3  == 0){
      p1 = cloud->points[ind1];
      distance(i) = 0;
    }
    else if(i%3 == 1){
      p1 = pos_cloud->points[ind1+1];
      distance(i) = EPSILON;
    }
    else{
      p1 = nos_cloud->points[ind1+2];
      distance(i) = EPSILON;
    }
    for(int j=0; j<cloud_size; j++){
      pcl::PointXYZ p2;
      int ind2 = j/3;
      if(j%3  == 0){
        p2 = cloud->points[ind2];
      }
      else if(j%3 == 1){
        p2 = pos_cloud->points[ind2+1];
      }
      else{
        p2 = nos_cloud->points[ind2+2];
      }
      Euclidean_matrix (i, j) = pcl::euclideanDistance<pcl::PointXYZ, pcl::PointXYZ>(p1, p2);
    }
  }
}


void RbfFunctionComputation(Eigen::VectorXi rbf_center_indices){
  std::cout << "Computing RBF_MATRIX" << "\n";
  // int size = rbf_center_indices.size();
  int size = cloud->size()*3;
  size_t matrix_size = size + 4;
  std::cout << "Matrix Size: " << matrix_size << "\n";
  // Eigen::MatrixXf rbf_matrix( matrix_size, matrix_size);
  rbf_matrix.resize(matrix_size, matrix_size);
  std::cout << "" << rbf_matrix.rows()<< " " << rbf_matrix.cols() << "\n";
  Eigen::VectorXf distance (matrix_size);
  // rbf_matrix.block(0, 0, size, size) = Euclidean_matrix;
  for(int i=0; i<size; i++){
    for(int j=0; j<size; j++){
      rbf_matrix (i, j) = Euclidean_matrix (i,j);
    }
  }

  for(int i=0; i<size; i++){
    pcl::PointXYZ p1;
    // int ind = rbf_center_indices(i);
    int ind = i;
    int ind1 = ind/3;
    if(ind%3  == 0){
      p1 = cloud->points[ind1];
      distance(i) = 0;
    }
    else if(ind%3 == 1){
      p1 = pos_cloud->points[ind1+1];
      distance(i) = EPSILON;
    }
    else{
      p1 = nos_cloud->points[ind1+2];
      distance(i) = EPSILON;
    }
    rbf_matrix (i, size + 0) = 1;
    rbf_matrix (i, size + 1) = p1.x;
    rbf_matrix (i, size + 2) = p1.y;
    rbf_matrix (i, size + 3) = p1.z;

    rbf_matrix (size + 0, i) = 1;
    rbf_matrix (size + 1, i) = p1.x;
    rbf_matrix (size + 2, i) = p1.y;
    rbf_matrix (size + 3, i) = p1.z;
  }

  distance (size + 0) = 0;
  distance (size + 1) = 0;
  distance (size + 2) = 0;
  distance (size + 3) = 0;

  // std::cout << "RBF Matrix: " << rbf_matrix << "\n";
  // // std::cout << "\n\n\n\nDistance Vector: " << distance << "\n";

  weights = rbf_matrix.ldlt().solve(distance);
}

double calculateError(){
  size_t size = cloud->size()*3;
  double error = 0;
  Eigen::MatrixXf euclideanMat = rbf_matrix.block(0, 0, size, size);
  Eigen::VectorXf weightVec = weights.block(0, 0, size, 1);
  Eigen::MatrixXf polynomialMat = rbf_matrix.block(0, size, size, 4);
  Eigen::VectorXf polynomialWeightVec = weights.block(size, 0, 4, 1);
  Eigen::VectorXf RBFErrorVec = euclideanMat*weightVec;
  Eigen::VectorXf polynomialErrorVec = polynomialMat*polynomialWeightVec;
  error = RBFErrorVec.sum() + polynomialErrorVec.sum();
  return error;
}

void visualizeCloud (pcl::PointXYZ centroid)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pos_color(pos_cloud, 0.0, 0.0, 255.0f);
  // viewer->addPointCloud<pcl::PointXYZ> (pos_cloud, pos_color, "positive off surface cloud");
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> neg_color(nos_cloud, 255.0f, 0.0, 0.0);
  // viewer->addPointCloud<pcl::PointXYZ> (nos_cloud, neg_color, "negative off surface cloud");
  
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "positive off surface cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "negative off surface cloud");

  viewer->addCoordinateSystem (1.0);
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, cloud_normals, 10, 0.05, "normals");
  viewer->initCameraParameters ();
  viewer->setCameraPosition(centroid.x, centroid.y, centroid.z, 1, 1, 1, 1);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void loadCloud(){
  cloud  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/krishna/Desktop/CG/project/hand_gestures/hand_0/image_0000.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    exit(0);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points."
            << std::endl;

    // for (const auto& point: *cloud)
  //   std::cout << "    " << point.x
  //             << " "    << point.y
  //             << " "    << point.z << std::endl;
  // return cloud;
}

void estimateNormals(){
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud (cloud);
  // ne.setViewPoint(centroid.x, centroid.y, centroid.z);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setSearchMethod (tree);
  cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

  ne.setRadiusSearch (0.03);

  ne.compute (*cloud_normals);
}

void generateOffSurfacePoints(){
  pos_cloud  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pos_cloud->width = cloud->width;
  pos_cloud->height = cloud->height;
  pos_cloud->points.resize(cloud->width*cloud->height);
  size_t size = pos_cloud->size();
  
  nos_cloud  = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  nos_cloud->width = cloud->width;
  nos_cloud->height = cloud->height;
  nos_cloud->points.resize(cloud->width*cloud->height);  
  
  for(int i=0; i<size; i++){
    float pos_x = cloud->at(i).x + EPSILON*cloud_normals->at(i).normal_x;
    float pos_y = cloud->at(i).y + EPSILON*cloud_normals->at(i).normal_y;
    float pos_z = cloud->at(i).z + EPSILON*cloud_normals->at(i).normal_z;
    // std::cout << pos_x << " " << pos_y << " " << pos_z << "\n";
    pos_cloud->points[i].x = pos_x;
    pos_cloud->points[i].y = pos_y;
    pos_cloud->points[i].z = pos_z;
    // std::cout << positive_off_surface_cloud->points[i] << "\n";

    float neg_x = cloud->at(i).x - EPSILON*cloud_normals->at(i).normal_x;
    float neg_y = cloud->at(i).y - EPSILON*cloud_normals->at(i).normal_y;
    float neg_z = cloud->at(i).z - EPSILON*cloud_normals->at(i).normal_z;
    nos_cloud->points[i].x = neg_x;
    nos_cloud->points[i].y = neg_y;
    nos_cloud->points[i].z = neg_z;
  }
}

Eigen::VectorXi chooseRbfCenters(){
  int size = 3*cloud->size();
  Eigen::VectorXi indices (size);
  for(int i=0; i<size; i++){
    indices[i] = i;
  }
  return indices;
}


int main (int argc, char** argv)
{
  std::cout << "EPSILON: " << EPSILON << "\n";

  loadCloud();

  pcl::PointXYZ centroid;
  pcl::computeCentroid(*cloud, centroid);
  std::cout << centroid << "\n";

  estimateNormals();

  std::cout << "Normals Size " << cloud_normals->size() << " Points Size " << cloud->size() << "\n"; 

  generateOffSurfacePoints();

  computeEuclideanDistanceMatrix();
  Eigen::VectorXi indices = chooseRbfCenters();
  RbfFunctionComputation(indices);

  std::cout << "Error in Fitting: " << calculateError();

  // visualizeCloud(centroid);

  return (0);
}