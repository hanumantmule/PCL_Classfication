#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/keypoints/sift_keypoint.h>

#include <fstream>

#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <boost/filesystem.hpp>
using namespace boost::system;
namespace filesys = boost::filesystem;
#ifndef USING_BOOST
#define USING_BOOST
#endif

std::vector<std::string> getAllFilesInDir(const std::string& dirPath, const std::vector<std::string> dirSkipList = { })
{
	// Create a vector of string
	std::vector<std::string> listOfFiles;
	try {
		// Check if given path exists and points to a directory
		if (filesys::exists(dirPath) && filesys::is_directory(dirPath))
		{
			// Create a Recursive Directory Iterator object and points to the starting of directory
			filesys::recursive_directory_iterator iter(dirPath);
			// Create a Recursive Directory Iterator object pointing to end.
			filesys::recursive_directory_iterator end;
			// Iterate till end
			while (iter != end)
			{
				// Check if current entry is a directory and if exists in skip list
				if (filesys::is_directory(iter->path()) &&
					(std::find(dirSkipList.begin(), dirSkipList.end(), iter->path().filename()) != dirSkipList.end()))
				{
					// Skip the iteration of current directory pointed by iterator
#ifdef USING_BOOST
// Boost Fileystsem  API to skip current directory iteration
					iter.no_push();
#else
// c++17 Filesystem API to skip current directory iteration
					iter.disable_recursion_pending();
#endif
				}
				else
				{
					// Add the name in vector
					if (!(filesys::is_directory(iter->path())))
						listOfFiles.push_back(iter->path().string());

				}
				error_code ec;
				// Increment the iterator to point to next entry in recursive iteration
				iter.increment(ec);
				if (ec) {
					std::cerr << "Error While Accessing : " << iter->path().string() << " :: " << ec.message() << '\n';
				}
			}
		}
	}
	catch (std::system_error& e)
	{
		std::cerr << "Exception :: " << e.what();
	}
	return listOfFiles;
}


double
computeCloudResolution(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	double resolution = 0.0;
	int numberOfPoints = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> squaredDistances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
			continue;

		// Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
		if (nres == 2)
		{
			resolution += sqrt(squaredDistances[1]);
			++numberOfPoints;
		}
	}
	if (numberOfPoints != 0)
		resolution /= numberOfPoints;

	return resolution;
}

int iss_detector(std::string file_name)
{
	// Objects for storing the point cloud and the keypoints.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) != 0)
	{
		pcl::console::print_error("Couldn't read file %s!\n", file_name);
		return -1;
	}

	// ISS keypoint detector object.
	pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> detector;
	detector.setInputCloud(cloud);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	detector.setSearchMethod(kdtree);
	double resolution = computeCloudResolution(cloud);
	// Set the radius of the spherical neighborhood used to compute the scatter matrix.
	detector.setSalientRadius(2 * resolution);
	// Set the radius for the application of the non maxima supression algorithm.
	detector.setNonMaxRadius(2 * resolution);
	// Set the minimum number of neighbors that has to be found while applying the non maxima suppression algorithm.
	detector.setMinNeighbors(4);
	// Set the upper bound on the ratio between the second and the first eigenvalue.
	detector.setThreshold21(0.975);
	// Set the upper bound on the ratio between the third and the second eigenvalue.
	detector.setThreshold32(0.975);
	// Set the number of prpcessing threads to use. 0 sets it to automatic.
	detector.setNumberOfThreads(4);
	pcl::StopWatch watch;

	detector.compute(*keypoints);

	std::cout << " ISS keypoint estimated";
	std::cout << " with size " << keypoints->size() << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr iss3d(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*keypoints, *iss3d);

	pcl::console::print_highlight("\n No of cloud points:  %zd in %lfs\n", cloud->size(), watch.getTimeSeconds());

	pcl::console::print_highlight("Detected %zd points in %lfs\n", keypoints->size(), watch.getTimeSeconds());


	pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices();
	if (!keypoints_indices->indices.empty())
	{
		pcl::io::savePCDFile("iss_keypoints.pcd", *cloud, keypoints_indices->indices, true);
		pcl::console::print_info("Saved keypoints to iss_keypoints.pcd\n");
	}
	else
		pcl::console::print_warn("Keypoints indices are empty!\n");

	
	return 0;
}

int harris_3d_detector(std::string file_name)
{
	pcl::console::print_highlight("Harris 3D Keypoint detector");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) == -1) // load the file
	{
		pcl::console::print_error("Couldn't read file %s!\n", file_name);
		return (-1);
	}

	pcl::HarrisKeypoint3D <pcl::PointXYZ, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>);
	detector.setNonMaxSupression(true);
	detector.setInputCloud(cloud);
	detector.setThreshold(1e-6);
	double resolution = computeCloudResolution(cloud);

	detector.setRadius(resolution * 2);
	pcl::StopWatch watch;
	detector.compute(*keypoints);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*keypoints, *cloud_temp);

	pcl::console::print_highlight("\nNo of cloud points:  %zd in %lfs\n", cloud->size(), watch.getTimeSeconds());

	pcl::console::print_highlight("Detected %zd points in %lfs\n", keypoints->size(), watch.getTimeSeconds());

	
	pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices();
	if (!keypoints_indices->indices.empty())
	{
		pcl::io::savePCDFile("./Dataset/A/3d_haris_keypoints.pcd", *cloud, keypoints_indices->indices, false);
		pcl::console::print_info("Saved keypoints to 3d_haris_keypoints.pcd\n");
	}
	else
		pcl::console::print_warn("Keypoints indices are empty!\n");
	

	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud_temp, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 0, 0);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
	viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}

int sift_3d_detector(std::string file_name)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud_xyz) == -1) // load the file
	{
		PCL_ERROR("Couldn't read file");
		return -1;
	}
	std::cout << "points: " << cloud_xyz->points.size() << std::endl;
	
	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;

	// Estimate the normals of the cloud_xyz
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setInputCloud(cloud_xyz);
	ne.setSearchMethod(tree_n);
	ne.setRadiusSearch(0.2);
	ne.compute(*cloud_normals);

	// Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
	for (size_t i = 0; i < cloud_normals->points.size(); ++i)
	{
		cloud_normals->points[i].x = cloud_xyz->points[i].x;
		cloud_normals->points[i].y = cloud_xyz->points[i].y;
		cloud_normals->points[i].z = cloud_xyz->points[i].z;
	}

	// Estimate the sift interest points using normals values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(result);

	std::cout << "\nNo of SIFT points in the result are " << result.points.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(result, *cloud_temp);
	std::cout << "SIFT points in the cloud_temp are " << cloud_temp->points.size() << std::endl;


	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud_temp, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_xyz, 255, 0, 0);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloud(cloud_xyz, cloud_color_handler, "cloud");
	viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	return 0;
}

int shot_descriptor(std::string file_name)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *cloud) != 0)
	{
		pcl::console::print_error("Couldn't read file %s!\n", file_name);
		return -1;
	}

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(2.50);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// SHOT estimation object.
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(cloud);
	shot.setInputNormals(normals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(10.50);
	shot.setKSearch(0);

	shot.compute(*descriptors);


	std::cout << " SHOT feature estimated";
	std::cout << " with size " << descriptors->size() << std::endl;
	std::cout << "SHOT output points.size (): " << descriptors->points.size() << std::endl;
	std::ofstream myfile;
	std::string csv_file_name = file_name.substr(0, file_name.find_last_of('.')) + "_shot_descriptor.csv";
	myfile.open(csv_file_name);
	std::cout << "CSV file writing started"<< std::endl;
	for (int i = 0; i < descriptors->points.size(); i++)
	{
		for (int j = 0; j < descriptors->points[i].descriptorSize(); j++)
		{
			//std::cout << "SHOT output points[" << i << "].size (): " << descriptors->points[i].descriptor[j] << std::endl;	
			
			myfile << descriptors->points[i].descriptor[j] << ",";
		}
		myfile << "\n";
	}
	myfile.close();
	std::cout << "CSV file writing ended.." << std::endl;

	// Display and retrieve the SHOT descriptor for the first point.
	std::cout << descriptors->points[0] << std::endl;
	
	//std::string output_filename = file_name;
	//output_filename.append("_localdesc.pcd");
	
	pcl::io::savePCDFile("./Dataset/decriptor.pcd", *descriptors);
	
	//cout<<save2pcd("./Dataset/decriptor2.pcd", *descriptors);
	
	pcl::console::print_info("Saved local descriptors as %s\n", "./Dataset/decriptor.pcd");
	return 0;
}


int show_pcd(std::string pcd_file)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) // load the file
	{
		pcl::console::print_error("Couldn't read file %s!\n", pcd_file);
		return (-1);
	}
		
	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(cloud, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud, 255, 0, 0);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	//viewer.addPointCloud(cloud_xyz, cloud_color_handler, "cloud");
	viewer.addPointCloud(cloud, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}


int convert_to_pcd(std::string obj_file, std::string pcd_file)
{
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileOBJ(obj_file, mesh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	pcl::io::savePCDFileASCII(pcd_file, *cloud);
	return 0;
}

int list_files(std::string dirPath)
{
	std::vector<std::string> listOfFiles = getAllFilesInDir(dirPath);
	// Iterate over the vector and print all files
	for (auto str : listOfFiles)
	{
		//std::cout << str << std::endl;
		if (str.substr(str.find_last_of(".") + 1) == "obj") {

			std::string pcd_file_name = str.substr(0, str.find_last_of('.')) + ".pcd";
			//cout << pcd_file_name << endl;
			convert_to_pcd(str, pcd_file_name);
		}
		else {
			std::cout << "Skipping file: " << str << " Extenstion is not obj..." << std::endl;
		}


	}
	std::cout << "**********************" << std::endl;
	return listOfFiles.size();
}


int main(int argc, char** argv)
{
		
	shot_descriptor("./Dataset/A/3d_haris_keypoints.pcd");
	//harris_3d_detector("./Dataset/A/Tobacco_WildType_High-heat_C_D0.pcd");
	//convert_to_pcd("./Dataset/Tobacco_WildType_High-heat_C_D0.obj");
	//iss_detector("./Dataset/objtopcd.pcd");
	//show_pcd("./Dataset/objtopcd.pcd");
	//sift_3d_detector("./Dataset/objtopcd.pcd");

	std::string dirPath = "./Dataset";

	// Get recursive list of files in given directory and its sub directories
	
}