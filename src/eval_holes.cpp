#include <ros/ros.h>
// #include <std_srvs/Empty.h>

#include <cmath>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>

// #include <voxblox/core/esdf_map.h>
// #include <voxblox/core/occupancy_map.h>
// #include <voxblox/core/tsdf_map.h>
// #include <voxblox/integrator/esdf_integrator.h>
// #include <voxblox/integrator/occupancy_integrator.h>
// #include <voxblox/integrator/tsdf_integrator.h>
// #include <voxblox/io/layer_io.h>
// #include <voxblox/io/mesh_ply.h>
// #include <voxblox/mesh/mesh_integrator.h>
// #include <voxblox_ros/mesh_vis.h>
// #include <voxblox_ros/ptcloud_vis.h>

#include <boost/filesystem.hpp>

// VTK stuff
#include <pcl/visualization/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkActor.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkCellData.h>
#include <vtkCellIterator.h>
#include <vtkConnectivityFilter.h>
#include <vtkPolyDataConnectivityFilter.h>
#include <vtkDataSetMapper.h>
#include <vtkFillHolesFilter.h>
#include <vtkGenericCell.h>
#include <vtkPointData.h>
#include <vtkPolyDataNormals.h>
#include <vtkCamera.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkSmartPointer.h>
#include <vtkAppendFilter.h>

// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
// #include <pcl/common/distances.h>
#include <pcl/octree/octree.h>
#include <pcl/segmentation/segment_differences.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


namespace mav_exploration_eval {

// Ros-wrapper for c++ voxblox code to evaluates voxblox maps upon request from
// the eval_plotting_node. Largely based on the voxblox_ros/voxblox_eval.cc
// code. Pretty ugly and non-general code but just needs to work in this
// specific case atm...
class EvaluationNode {
public:
  EvaluationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool evaluateHoles();
  bool evalHolesVTK();
  bool evalHolesPCL();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::string gt_path_;
  std::string mapped_mesh_path_;
  std::string output_mesh_path_;
  std::string method_;
  double p_metric_holes_factor_scaling_;
  double p_metric_holes_factor_exp_;
  double voxel_resolution_;
  double max_hole_area_vtk_;
  double max_hole_area_pcl_;
  double difference_max_dist_;
  bool save_clouds_;

  // Ground truth pointcloud
  pcl::PointCloud<pcl::PointXYZRGB> gt_ptcloud_;
};



EvaluationNode::EvaluationNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private) {
  bool success = evaluateHoles();
  if (success) {
    ROS_INFO("Evaluation done.");
  } else {
    ROS_ERROR("Evaluation failed!");
  }
  ROS_INFO("Shutting down.");
  nh_private_.shutdown();
}



bool EvaluationNode::evaluateHoles() {
  // ************ Initialize ************ //
  ROS_INFO("Loading parameters...");
  std::string gt_path;
  std::string mapped_mesh_path;
  std::string output_mesh_path;
  std::string method;
  double p_metric_holes_factor_scaling;
  double p_metric_holes_factor_exp;
  double voxel_resolution;
  double max_hole_area_vtk;
  double max_hole_area_pcl;
  double segment_max_dist;
  bool save_clouds;

  nh_private_.getParam("ground_truth_path", gt_path);
  nh_private_.getParam("mapped_mesh_map", mapped_mesh_path);
  nh_private_.getParam("output_mesh_map", output_mesh_path);
  nh_private_.getParam("method", method);
  nh_private_.param("metric_holes_factor_scaling", p_metric_holes_factor_scaling, 1.0);
  nh_private_.param("metric_holes_factor_exp", p_metric_holes_factor_exp, 1.0);
  nh_private_.param("voxel_resolution", voxel_resolution, 0.1);
  nh_private_.param("max_hole_area_vtk", max_hole_area_vtk, 0.1);
  nh_private_.param("max_hole_area_pcl", max_hole_area_pcl, 0.1);
  nh_private_.param("segment_max_dist", segment_max_dist, 0.141);
  nh_private_.param("save_clouds", save_clouds, false);

  gt_path_ = gt_path;
  mapped_mesh_path_ = mapped_mesh_path;
  output_mesh_path_ = output_mesh_path;
  method_ = method;
  p_metric_holes_factor_scaling_ = p_metric_holes_factor_scaling;
  p_metric_holes_factor_exp_ = p_metric_holes_factor_exp;
  voxel_resolution_ = voxel_resolution;
  max_hole_area_vtk_ = max_hole_area_vtk;
  max_hole_area_pcl_ = max_hole_area_pcl;
  difference_max_dist_ = segment_max_dist;
  save_clouds_ = save_clouds;


  if (method == "VTK_mesh") {
    ROS_INFO("Using method: %s", method.c_str());
    return evalHolesVTK();
  } else if (method == "PCL_voxel") {
    ROS_INFO("Using method: %s", method.c_str());
    return evalHolesPCL();
  } else {
    ROS_ERROR("Method not recognised. Use \"PCL_voxel\" or \"VTK_mesh\".");
    return false;
  }

  return false;
}



bool EvaluationNode::evalHolesVTK() {
  // Load mapped pointcloud from generated meshes
  ROS_INFO("Loading and combining mapped point clouds...");
  // PCL
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mapped(new pcl::PointCloud<pcl::PointXYZ>);
  // VTK
  vtkSmartPointer<vtkPolyData> mesh_mapped = vtkSmartPointer<vtkPolyData>::New();
  // Ensure path exists
  ROS_INFO("Path to directory: %s", mapped_mesh_path_.c_str());
  if (!boost::filesystem::exists(mapped_mesh_path_)) {
    ROS_ERROR("Directory does not exist!");
    return false;
  }
  // Iterate over files
  vtkNew<vtkPLYReader> plyReader;
  int n_clouds = 0;
  boost::filesystem::directory_iterator end_itr;  // Default constructor constructs an end iterator object
  for (boost::filesystem::directory_iterator itr(mapped_mesh_path_); itr != end_itr; ++itr) {
    if (boost::filesystem::is_regular_file(itr->status())) {
      if (itr->path().extension() == ".ply") {
        std::string mesh_path = itr->path().c_str();
        ROS_INFO("  * Processing mesh: %s", mesh_path.c_str());
        // PCL
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::PCLPointCloud2 cloud_blob;
        // pcl::io::loadPLYFile(mesh_path, cloud_blob);
        // pcl::fromPCLPointCloud2(cloud_blob, *cloud);
        // *cloud_mapped += *cloud;

        // VTK
        ROS_INFO("    Reading...");
        plyReader->SetFileName(mesh_path.c_str());
        plyReader->Update();
        // Append meshes
        ROS_INFO("    Appending...");
        vtkSmartPointer<vtkAppendPolyData> appendFilter = vtkSmartPointer<vtkAppendPolyData>::New();
        appendFilter->AddInputData(mesh_mapped);
        appendFilter->AddInputData(plyReader->GetOutput());
        appendFilter->Update();
        ROS_INFO("    Saving...");
        mesh_mapped->DeepCopy(appendFilter->GetOutput());
        ++n_clouds;
      }
    }
  }
  if (n_clouds <= 0) {
    ROS_ERROR("Directory does not contain any .ply files!");
    return false;
  } else {
    ROS_INFO("  * Meshes loaded:  %d", n_clouds);
  }


  ROS_INFO("Cleaning combined mesh...");
  // Remove duplicate points
  vtkSmartPointer<vtkCleanPolyData> cleanFilter = vtkSmartPointer<vtkCleanPolyData>::New();
  cleanFilter->SetInputData(mesh_mapped);
  cleanFilter->Update();


  vtkNew<vtkPLYWriter> plyWriter;
  std::string writer_path;
  if (save_clouds_) {
  ROS_INFO("Saving combined mesh...");
    std::string writer_path = output_mesh_path_ + "/cloud_mapped.ply";
    plyWriter->SetFileName(writer_path.c_str());
    plyWriter->SetInputConnection(cleanFilter->GetOutputPort());
    plyWriter->Write();
  }



  // ************ Extract holes using VTK FillHolesFilter ************ //
  ROS_INFO("Extracting holes...");
  ROS_INFO("  * Filling holes...");

  // Fill the holes
  vtkSmartPointer<vtkFillHolesFilter> fillHoles = vtkSmartPointer<vtkFillHolesFilter>::New();
  fillHoles->SetInputConnection(cleanFilter->GetOutputPort());
  fillHoles->SetHoleSize(1000.0);
  fillHoles->Update();



  // // Do we need to estimate normals?
  // vtkNew<vtkSignedDistance> distance;
  // if (polyData->GetPointData()->GetNormals())
  // {
  //     std::cout << "Using normals from input file" << std::endl;
  //     distance->SetInputData(polyData);
  // }
  // else
  // {
  //     std::cout << "Estimating normals using PCANormalEstimation" << std::endl;
  //     vtkNew<vtkPCANormalEstimation> normals;
  //     normals->SetInputData(polyData);
  //     normals->SetSampleSize(sampleSize);
  //     normals->SetNormalOrientationToGraphTraversal();
  //     normals->FlipNormalsOn();
  //     distance->SetInputConnection(normals->GetOutputPort());
  // }
  // std::cout << "Range: " << range[0] << ", " << range[1] << ", " << range[2]
  //             << std::endl;
  // int dimension = 256;
  // double radius;
  // radius = std::max(std::max(range[0], range[1]), range[2]) /
  //     static_cast<double>(dimension) * 4; // ~4 voxels
  // std::cout << "Radius: " << radius << std::endl;

  // distance->SetRadius(radius);
  // distance->SetDimensions(dimension, dimension, dimension);
  // distance->SetBounds(bounds[0] - range[0] * .1, bounds[1] + range[0] * .1,
  //                     bounds[2] - range[1] * .1, bounds[3] + range[1] * .1,
  //                     bounds[4] - range[2] * .1, bounds[5] + range[2] * .1);

  // vtkNew<vtkExtractSurface> surface;
  // surface->SetInputConnection(distance->GetOutputPort());
  // surface->SetRadius(radius * .99);
  // surface->Update();



  // Make the triangle winding order consistent
  ROS_INFO("  * Estimating normals...");
  vtkSmartPointer<vtkPolyDataNormals> normals_vtk = vtkSmartPointer<vtkPolyDataNormals>::New();
  ;
  normals_vtk->SetInputConnection(fillHoles->GetOutputPort());
  normals_vtk->ConsistencyOn();
  normals_vtk->SplittingOff();
  normals_vtk->Update();
  // normals_vtk->GetOutput()->GetPointData()->SetNormals(
  //     cleanFilter->GetOutput()->GetPointData()->GetNormals());

  // How many added cells
  vtkIdType numOriginalCells = cleanFilter->GetOutput()->GetNumberOfCells();
  vtkIdType numNewCells = normals_vtk->GetOutput()->GetNumberOfCells();
  ROS_INFO("  * Original cells: %d", numOriginalCells);
  ROS_INFO("  * New cells:      %d", numNewCells);
  ROS_INFO("  * Cells added:    %d", numNewCells - numOriginalCells);

  // Iterate over the original cells
  ROS_INFO("  * Iterating to new cells...");
  auto it = normals_vtk->GetOutput()->NewCellIterator();
  vtkIdType numCells = 0;
  for (it->InitTraversal(); !it->IsDoneWithTraversal() && numCells < numOriginalCells; it->GoToNextCell(), ++numCells) {
    // Do nothing, just iterate to get to the new cells
  }

  vtkSmartPointer<vtkPolyData> holePolyData = vtkSmartPointer<vtkPolyData>::New();
  holePolyData->Allocate(normals_vtk->GetOutput(), numNewCells - numOriginalCells);
  holePolyData->SetPoints(normals_vtk->GetOutput()->GetPoints());

  vtkSmartPointer<vtkGenericCell> cell = vtkSmartPointer<vtkGenericCell>::New();

  // The remaining cells are the new ones from the hole filler
  ROS_INFO("  * Extracting new cells (hole surfaces)...");
  for (; !it->IsDoneWithTraversal(); it->GoToNextCell()) {
    it->GetCell(cell);
    holePolyData->InsertNextCell(it->GetCellType(), cell->GetPointIds());
  }
  it->Delete();

  // Determine connectivity of mesh regions and extract a list of regions
  ROS_INFO("  * Determining connectivity of holes...");
  vtkSmartPointer<vtkPolyDataConnectivityFilter> connectivity = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
  connectivity->SetInputData(holePolyData);
  connectivity->SetExtractionModeToSpecifiedRegions();

  // std::vector<vtkSmartPointer<vtkPolyData>> regions = std::vector<vtkSmartPointer<vtkPolyData>::New()>;

  // Iterate through regions
  // ************ Compute area of holes (extracted regions) ************ //
  ROS_INFO("  * Extracting hole regions and computing area...");
  int idx = 0;
  int n_holes = 0;
  std::vector<double> areas;
  while (true) {
    // Get region
    connectivity->AddSpecifiedRegion(idx);
    connectivity->Update();

    vtkSmartPointer<vtkPolyData> region = vtkSmartPointer<vtkPolyData>::New();
    region->DeepCopy(connectivity->GetOutput());
    // Make sure we got something
    if (region->GetNumberOfCells() <= 0) {
      break;
    }

    // Compute area and add to list
    double area = 0;
    for (int i = 0; i < region->GetNumberOfCells(); i++) {
      vtkCell* triangle = region->GetCell(i);
      area += vtkTriangle::TriangleArea(triangle->GetPoints()->GetPoint(0), triangle->GetPoints()->GetPoint(1),
                                        triangle->GetPoints()->GetPoint(2));
    }
    areas.push_back(area);

    // Delete region
    connectivity->DeleteSpecifiedRegion(idx);
    ++idx;
    ++n_holes;  // Counts number of holes
  }
  ROS_INFO("  * Holes extracted: %d", n_holes);

  // Save hole mesh
  if (save_clouds_) {
    writer_path = output_mesh_path_ + "/cloud_holes.ply";
    plyWriter->SetFileName(writer_path.c_str());
    plyWriter->SetInputData(holePolyData);
    plyWriter->Write();
  }

  pcl::PolygonMesh holePolygonMesh;
  pcl::VTKUtils::vtk2mesh(holePolyData, holePolygonMesh);
  ROS_INFO("Saving extracted holes...");
  pcl::io::savePLYFile(output_mesh_path_ + "/cloud_holes_pcl.ply", holePolygonMesh);


  // ************ Compute the metric ************ //
  ROS_INFO("Computing metric...");
  double metric = 0;
  double metric_simple = 0;
  for (int i = 0; i < areas.size(); ++i) {
    metric += p_metric_holes_factor_scaling_ * exp(p_metric_holes_factor_exp_ * areas[i]);
    metric_simple += areas[i];
  }
  ROS_INFO("  * Metric:        %16.6f", metric);
  ROS_INFO("  * Metric simple: %16.6f", metric_simple);

  return true;
}



bool EvaluationNode::evalHolesPCL() {
  // ************ Initialise ************ //
  // Load ground truth pointcloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gt(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_gt_blob;
  ROS_INFO("Loading ground truth point cloud...");
  pcl::io::loadPLYFile(gt_path_, cloud_gt_blob);
  pcl::fromPCLPointCloud2(cloud_gt_blob, *cloud_gt);

  // Load mapped pointcloud from generated meshes
  ROS_INFO("Loading and combining mapped point clouds...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mapped(new pcl::PointCloud<pcl::PointXYZ>);
  // Ensure path exists
  ROS_INFO("  * Path to directory: %s", mapped_mesh_path_.c_str());
  if (!boost::filesystem::exists(mapped_mesh_path_)) {
    ROS_ERROR("  * Directory does not exist!");
    return false;
  }
  // Iterate over files
  int n_clouds = 0;
  boost::filesystem::directory_iterator end_itr;  // Default constructor constructs an end iterator object
  for (boost::filesystem::directory_iterator itr(mapped_mesh_path_); itr != end_itr; ++itr) {
    if (boost::filesystem::is_regular_file(itr->status())) {
      if (itr->path().extension() == ".ply") {
        std::string mesh_path = itr->path().c_str();
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloud_blob;
        ROS_INFO("  * Processing mesh: %s", mesh_path.c_str());
        pcl::io::loadPLYFile(mesh_path, cloud_blob);
        pcl::fromPCLPointCloud2(cloud_blob, *cloud);
        *cloud_mapped += *cloud;
        ++n_clouds;
      }
    }
  }
  if (n_clouds <= 0) {
    ROS_ERROR("  * Directory does not contain any .ply files!");
    return false;
  } else {
    ROS_INFO("  * Meshes loaded:  %d", n_clouds);
  }

  // Save pointclouds
  if (save_clouds_) {
    ROS_INFO("Saving ground truth point cloud...");
    pcl::io::savePLYFile(output_mesh_path_ + "/cloud_gt.ply", *cloud_gt);
    ROS_INFO("Saving mapped point clouds...");
    pcl::io::savePLYFile(output_mesh_path_ + "/cloud_mapped.ply", *cloud_mapped);
  }
  

  // ************ Filter ************ //
  ROS_INFO("Downsampling mapped point cloud...");
  // Downsample to voxel grid
  pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree(voxel_resolution_);
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxel_centers;
  octree.setInputCloud(cloud_mapped);
  octree.addPointsFromInputCloud();
  octree.getOccupiedVoxelCenters(voxel_centers);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto it = voxel_centers.begin(); it != voxel_centers.end(); ++it) {
    cloud_filtered->push_back(*it);
  }

  ROS_INFO("Downsampling ground truth point cloud...");

  // Downsample to voxel grid
  pcl::octree::OctreePointCloudPointVector<pcl::PointXYZ> octree_gt(voxel_resolution_);
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> voxel_centers_gt;
  octree_gt.setInputCloud(cloud_gt);
  octree_gt.addPointsFromInputCloud();
  octree_gt.getOccupiedVoxelCenters(voxel_centers_gt);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gt_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto it = voxel_centers_gt.begin(); it != voxel_centers_gt.end(); ++it) {
    cloud_gt_filtered->push_back(*it);
  }

  // TODO: Filter noisy points
  // ROS_INFO("Filtering outliers...");

  // Save pointcloud
  if (save_clouds_) {
    ROS_INFO("Saving filtered point clouds...");
    pcl::io::savePLYFile(output_mesh_path_ + "/cloud_filtered.ply", *cloud_filtered);
    pcl::io::savePLYFile(output_mesh_path_ + "/cloud_gt_filtered.ply", *cloud_gt_filtered);
  }

  
  // ************ Concave hull ************ //
  ROS_INFO("Computing concave hull...");
  // Compute concave hull
  // https://pointclouds.org/documentation/classpcl_1_1_concave_hull.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::Vertices> hull_polygons; 
  pcl::ConcaveHull<pcl::PointXYZ> concave_hull_filter;
  concave_hull_filter.setInputCloud(cloud_filtered);
  concave_hull_filter.setAlpha(max_hole_area_pcl_); // TODO: Set appropriate distance, Limits the size of the resultant hull segments (the smaller the more detailed the hull)
  concave_hull_filter.reconstruct(*cloud_hull, hull_polygons); 
  // cloud_hull contains the points on the boundary of the concave hull
  // hull_polygons contains the hull indices
  
  // Save pointcloud
  if (save_clouds_) {
    ROS_INFO("Saving concave hull point cloud...");
    pcl::io::savePLYFile(output_mesh_path_ + "/cloud_concave_hull.ply", *cloud_hull);
  }


  // ************ Region of interest ************ //
  ROS_INFO("Establishing region of interest... (This might take a while.)");
  // Use concave hull to select Region of Interest from ground truth point cloud
  // Crop everything but concave hull
  // https://pointclouds.org/documentation/classpcl_1_1_crop_hull.html
  pcl::CropHull<pcl::PointXYZ> crop_hull_filter;
  crop_hull_filter.setHullIndices(hull_polygons);
  crop_hull_filter.setHullCloud(cloud_hull);
  crop_hull_filter.setInputCloud(cloud_gt_filtered);
  crop_hull_filter.setDim(3);
  crop_hull_filter.setCropOutside(true); // Keep points inside the hull
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gt_hull(new pcl::PointCloud<pcl::PointXYZ>);
  crop_hull_filter.filter(*cloud_gt_hull);

  // Save pointcloud
  if (save_clouds_) {
    ROS_INFO("Saving concave hull extracted from ground truth point cloud...");
    pcl::io::savePLYFile(output_mesh_path_ + "/cloud_concave_hull_gt.ply", *cloud_gt_hull);
  }


  // ************ Subtract mapped points from ground truth ************ //
  // Remaining points should be the holes
  ROS_INFO("Extracting holes...");
  
  // Set maximum distance to classify points correspondence
  double max_dist = difference_max_dist_;//sqrt(pow(voxel_resolution_, 2) + pow(voxel_resolution_, 2));

  // Create k-d tree to search for nearest point
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>); 
  // Create filter for computing the difference between the two point clouds
  // https://pointclouds.org/documentation/classpcl_1_1_segment_differences.html
  pcl::SegmentDifferences<pcl::PointXYZ> difference_filter;
  difference_filter.setInputCloud(cloud_gt_hull);
  difference_filter.setTargetCloud(cloud_filtered);
  difference_filter.setSearchMethod(kdtree);
  difference_filter.setDistanceThreshold(max_dist);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_holes(new pcl::PointCloud<pcl::PointXYZ>);
  difference_filter.segment(*cloud_holes);

  // Save pointcloud
  if (save_clouds_) {
    ROS_INFO("Saving holes point cloud...");
    pcl::io::savePLYFile(output_mesh_path_ + "/cloud_holes.ply", *cloud_holes);
  }

  // ************ Area/volume ************ //
  // Compute area/volume of holes
  ROS_INFO("Computing size of holes...");

  int n_hole_points = cloud_holes->points.size();
  // TODO: get area from voxblox
  double area = n_hole_points * pow(voxel_resolution_, 2);
  double volume = n_hole_points * pow(voxel_resolution_, 3);

  // ************ Compute the metric ************ //
  ROS_INFO("Computing metric...");
  double metric = p_metric_holes_factor_scaling_ * exp(p_metric_holes_factor_exp_ * area);
  double metric_simple = area;
  ROS_INFO("  * Number of hole voxels: %d", n_hole_points);
  ROS_INFO("  * Metric:        %16.6f", metric);
  ROS_INFO("  * Metric simple: %16.6f", metric_simple);

  return true;
}



}  // namespace mav_exploration_eval



int main(int argc, char** argv) {
  ros::init(argc, argv, "evaluation_node");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  mav_exploration_eval::EvaluationNode eval(nh, nh_private);
  ros::spin();
  return 0;
}