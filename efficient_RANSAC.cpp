#if defined (_MSC_VER) && !defined (_WIN64)
#pragma warning(disable:4244) // boost::number_distance::distance()
                              // converts 64 to 32 bits integers
#endif

#include <fstream>
#include <iostream>

#include <CGAL/property_map.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/optimal_bounding_box.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

// #include <CGAL/Point_set_3.h>
// #include <CGAL/draw_point_set_3.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef Kernel::FT                                            FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3>          Point_with_normal;
typedef std::vector<Point_with_normal>                        Pwn_vector;
typedef Kernel::Point_3                                       Point;
typedef std::array<unsigned char, 3>                          Color;
// typedef CGAL::Point_set_3<Point>                              Point_set;

typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits
<Kernel, Pwn_vector, Point_map, Normal_map>             Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Plane<Traits>            Plane;

typedef CGAL::Shape_detection::Cone<Traits>             Cone;
typedef CGAL::Shape_detection::Cylinder<Traits>         Cylinder;
typedef CGAL::Shape_detection::Plane<Traits>            Plane;
typedef CGAL::Shape_detection::Sphere<Traits>           Sphere;
typedef CGAL::Shape_detection::Torus<Traits>            Torus;

int main (int argc, char** argv) {

  std::cout << "Efficient RANSAC" << std::endl;
  const std::string filename = (argc > 1) ? argv[1] : CGAL::data_file_path("cube.pwn");

  // Points with normals.
  Pwn_vector points;

  // Points.
  std::vector<Point> points_xyz;

  // Load point set from a file.

  if (!CGAL::IO::read_points(
        filename,
        std::back_inserter(points),
        CGAL::parameters::point_map(Point_map()).
        normal_map(Normal_map()))) {

    std::cerr << "Error: cannot read file! Provide a proper path to a point cloud." << std::endl;
    return EXIT_FAILURE;
  }

  CGAL::IO::read_PLY(filename,std::back_inserter(points_xyz));
  // Compute bounding box
  std::array<Point, 8> obb_points;
  Point min_corner;
  Point max_corner;
  CGAL::oriented_bounding_box(points_xyz, obb_points,
                              CGAL::parameters::use_convex_hull(true));
  min_corner = obb_points[0];
  max_corner = obb_points[0];
  for (int i = 1; i < obb_points.size(); i++){
    if (min_corner.x() < obb_points[i].x() &&
        min_corner.y() < obb_points[i].y() &&
        min_corner.z() < obb_points[i].z() 
    ) {
      min_corner = obb_points[i];
    }

  }
  for (int i = 1; i < obb_points.size(); i++){
    if (max_corner.x() > obb_points[i].x() &&
        max_corner.y() > obb_points[i].y() &&
        max_corner.z() > obb_points[i].z() 
    ) {
      max_corner = obb_points[i];
    }

  }
  float scale = std::max(std::max(std::abs(max_corner.x() - min_corner.x()), 
                                std::abs(max_corner.y() - min_corner.y())), 
                                std::abs(max_corner.z() - min_corner.z()));
  std::cout << "Scale factor: " << scale << std::endl;



  // Instantiate shape detection engine.
  Efficient_ransac ransac;

  // Provide input data.
  ransac.set_input(points);

  // Register shapes for detection.
  ransac.add_shape_factory<Plane>();
  ransac.add_shape_factory<Sphere>();
  ransac.add_shape_factory<Cylinder>();
  // ransac.add_shape_factory<Cone>();
  // ransac.add_shape_factory<Torus>();

  // Set parameters for shape detection.
  Efficient_ransac::Parameters parameters;

  std::ifstream params_file("../params");
  std::string dummy_string;
  params_file >> dummy_string >> parameters.probability;
  std::cout << "Probability: " << parameters.probability << std::endl;
  params_file >> dummy_string >> parameters.min_points;
  std::cout << "Min Points: " << parameters.min_points << std::endl;
  parameters.epsilon = 0.005*scale;
  std::cout << "Epsilon: " << parameters.epsilon << std::endl;
  parameters.cluster_epsilon = 0.01*scale;
  std::cout << "Cluster Epsilon: " << parameters.cluster_epsilon << std::endl;
  params_file >> dummy_string >> parameters.normal_threshold;
  std::cout << "Normal Threshold: " << parameters.normal_threshold << std::endl;

  // Detect shapes.
  ransac.detect(parameters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Colored_Cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointXYZRGB pt; 
  Color primitive_color;

  // Compute coverage, i.e. ratio of the points assigned to a shape.
  FT coverage =
  FT(points.size() - ransac.number_of_unassigned_points()) / FT(points.size());

  // Print number of detected shapes and unassigned points.
  std::cout << ransac.shapes().end() - ransac.shapes().begin()
  << " detected shapes, "
  << ransac.number_of_unassigned_points()
  << " unassigned points, " 
  << coverage << " coverage." << std::endl;


  // Efficient_ransac::shapes() provides
  // an iterator range to the detected shapes.
  Efficient_ransac::Shape_range shapes = ransac.shapes();
  Efficient_ransac::Shape_range::iterator it = shapes.begin();

  while (it != shapes.end()) {

    // Get specific parameters depending on the detected shape.
    if (Plane* plane = dynamic_cast<Plane*>(it->get())) {

      Kernel::Vector_3 normal = plane->plane_normal();
      std::cout << "Plane with normal " << normal << std::endl;

      // Plane shape can also be converted to the Kernel::Plane_3.
      std::cout << "Kernel::Plane_3: " <<
      static_cast<Kernel::Plane_3>(*plane) << std::endl;

      // If it is a sphere assign the red color
      primitive_color[0] = 255; primitive_color[1] = 0; primitive_color[2] = 0;

    } else if (Cylinder* cyl = dynamic_cast<Cylinder*>(it->get())) {

      Kernel::Line_3 axis = cyl->axis();
      FT radius = cyl->radius();

      std::cout << "Cylinder with axis "
      << axis << " and radius " << radius << std::endl;
      
      // If it is a cylinder assign the green color
      primitive_color[0] = 0; primitive_color[1] = 255; primitive_color[2] = 0;
    } else {

      // Print the parameters of the detected shape.
      // This function is available for any type of shape.
      std::cout << (*it)->info() << std::endl;
      
      // If it is a sphere assign the blue color
      primitive_color[0] = 0; primitive_color[1] = 0; primitive_color[2] = 255;
    }

    // Iterate through point indices assigned to each detected shape.
    std::vector<std::size_t>::const_iterator index_it = (*it)->indices_of_assigned_points().begin();

    while (index_it != (*it)->indices_of_assigned_points().end()) {
      // Retrieve point.
      const Point_with_normal& p = *(points.begin() + (*index_it));

      pt.x = p.first.x();
      pt.y = p.first.y();
      pt.z = p.first.z();
      pt.r = primitive_color[0];
      pt.g = primitive_color[1];
      pt.b = primitive_color[2];
      Colored_Cloud->points.push_back(pt);

      // Proceed with the next point.
      index_it++;
    }

    // Proceed with the next detected shape.
    it++;
  }

  Efficient_ransac::Point_index_iterator un_it = ransac.indices_of_unassigned_points().begin();
  primitive_color[0] = 220; primitive_color[1] = 220; primitive_color[2] = 220;
  while (un_it != ransac.indices_of_unassigned_points().end()) {
      // Retrieve point.
      const Point_with_normal& p = *(points.begin() + (*un_it));

      pt.x = p.first.x();
      pt.y = p.first.y();
      pt.z = p.first.z();
      pt.r = primitive_color[0];
      pt.g = primitive_color[1];
      pt.b = primitive_color[2];
      Colored_Cloud->points.push_back(pt);

      // Proceed with the next point.
      un_it++;
    }


  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(Colored_Cloud);
  while (!viewer.wasStopped ())
  {
  }
  return EXIT_SUCCESS;
}