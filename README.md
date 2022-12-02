# Efficient_RANSAC_Visualization

Extract geometric primitives (planes, spheres, cylinders) from a point cloud using the Efficient RANSAC method from Schnabel et Al (2007) and visualize the results using the PCL Cloudviewer with each point having a different color based on the type of primitive it belongs to (red for planes, green for cylinders and blue for spheres).

## Parameters

* **epsilon and normal_threshold**: The error between a point-with-normal p and a shape S is defined by its Euclidean distance and normal deviation to S. The normal deviation is computed between the normal at p and the normal of S at the closest projection of p onto S. The parameter epsilon defines the absolute maximum tolerance Euclidean distance between a point and a shape. 

* **cluster_epsilon**: The Efficient RANSAC uses this parameter to cluster the points into connected components covered by a detected shape. For developable shapes that admit a trivial planar parameterization (plane, cylinder, cone), the points covered by a shape are mapped to a 2D parameter space chosen to minimize distortion and best preserve arc-length distances. This 2D parameter space is discretized using a regular grid, and a connected component search is performed to identify the largest cluster. The parameter cluster_epsilon defines the spacing between two cells of the regular grid, so that two points separated by a distance of at most 2√2 cluster_epsilon are considered adjacent. For non-developable shapes, the connected components are identified by computing a neighboring graph in 3D and walking in the graph.

* **min_points**: The minimum number of points controls the termination of the algorithm. The shape search is iterated until no further shapes can be found with a higher support. Note that this parameter is not strict: depending on the chosen probability, shapes may be extracted with a number of points lower than the specified parameter.

* **probability**: This parameter defines the probability to miss the largest candidate shape. A lower probability provides a higher reliability and determinism at the cost of longer running time due to a higher search endurance.

To modify these parameters simply edit the file "param".

### Dependencies 
  
	cgal (built from source)   
	PCL 

### Compilation
  
	$ mkdir build && cd build 
	$ cmake .. 
	$ make
  
### Execution

	$ cd build 
	$ ./efficient_RANSAC /Path/to/your/point_cloud.xxx
  
Supported formats for point clouds: ply, off, las, xyz
