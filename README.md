# Efficient_RANSAC_Visualization

Extract geometric primitives (planes, spheres, cylinders) from a point cloud using the Efficient RANSAC method from Schnabel et Al (2007) and visualize the results using the PCL Cloudviewer with each point having a different color based on the type of primitive it belongs to (red for planes, green for cylinders and blue for spheres).

## Dependencies 
  
	cgal (built from source)   
	PCL 

## Compilation
  
	$ mkdir build && cd build 
	$ cmake .. 
	$ make
  
## Execution

	$ cd build 
	$ ./efficient_RANSAC /Path/to/your/point_cloud.xxx
  
Supported formats for point clouds: ply, off, las, xyz
