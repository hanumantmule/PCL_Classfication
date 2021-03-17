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

int iss_detector()
{
	// Objects for storing the point cloud and the keypoints.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) != 0)
	{
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

	detector.compute(*keypoints);

	std::cout << " ISS keypoint estimated";
	std::cout << " with size " << keypoints->size() << std::endl;

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(keypoints);
	normalEstimation.setRadiusSearch(0.03);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// SHOT estimation object.
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(keypoints);
	shot.setInputNormals(normals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.50);

	shot.compute(*descriptors);


	std::cout << " SHOT feature estimated";
	std::cout << " with size " << descriptors->size() << std::endl;
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
	detector.setRadius(0.10);
	pcl::StopWatch watch;
	detector.compute(*keypoints);
	pcl::PointCloud<pcl::PointXYZ>::Ptr harris3d(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*keypoints, *harris3d);

	pcl::console::print_highlight("\nNo of cloud points:  %zd in %lfs\n", cloud->size(), watch.getTimeSeconds());

	pcl::console::print_highlight("Detected %zd points in %lfs\n", keypoints->size(), watch.getTimeSeconds());



	/*pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud, 255, 255, 255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(harris3d, 255, 0, 0);


	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, white, "cloud");
	viewer->addPointCloud<pcl::PointXYZ>(harris3d, red, "keypoints2");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "keypoints2");
	viewer->addCoordinateSystem(1.0, "global");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}*/

	pcl::PointIndicesConstPtr keypoints_indices = detector.getKeypointsIndices();
	if (!keypoints_indices->indices.empty())
	{
		pcl::io::savePCDFile("keypoints.pcd", *cloud, keypoints_indices->indices, true);
		pcl::console::print_info("Saved keypoints to keypoints.pcd\n");
	}
	else
		pcl::console::print_warn("Keypoints indices are empty!\n");

	return 0;
}

int shot_descriptor()
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptors(new pcl::PointCloud<pcl::SHOT352>());

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) != 0)
	{
		return -1;
	}

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// Estimate the normals.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	normalEstimation.setRadiusSearch(0.03);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	normalEstimation.compute(*normals);

	// SHOT estimation object.
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud(cloud);
	shot.setInputNormals(normals);
	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.50);

	shot.compute(*descriptors);


	std::cout << " SHOT feature estimated";
	std::cout << " with size " << descriptors->size() << std::endl;

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
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white(cloud, 255, 255, 255);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, white, "cloud");
	viewer->addCoordinateSystem(1.0,"global");
	viewer->initCameraParameters();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return 0;
}


int convert_to_pcd(std::string obj_file)
{
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFileOBJ(obj_file, mesh);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
	pcl::io::savePCDFileASCII("head1PCD.pcd", *cloud);
	return 0;
}
int
main(int argc, char** argv)
{
		
	//iss_detector();
	//shot_descriptor();
	harris_3d_detector("head1PCD.pcd");
	//show_obj("Tomato_WildType_High-heat_A_D4.obj");
	//convert_to_pcd("Tomato_WildType_High-heat_A_D4.obj");
	show_pcd("head1PCD.pcd");

}