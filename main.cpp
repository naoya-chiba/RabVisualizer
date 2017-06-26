#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable: 4819)

#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include "rabv.hpp"

enum Mode { rab, pcd, ply };

int main(int argc, char* argv[])
{
	std::cout
		<< "Rab Visualizer ver.2.90" << std::endl
		<< "Copyright (C) 2015-2017 Chiba Naoya" << std::endl;

	Mode mode = rab;
	// read argument
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".rab");

	if (filenames.size() < 1)
	{
		mode = pcd;
		filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

		if (filenames.size() < 1)
		{
			mode = ply;
			filenames = pcl::console::parse_file_extension_argument(argc, argv, ".ply");

			if (filenames.size() < 1)
			{
				std::cerr
					<< "[Rab Visualizer] File name is not found." << std::endl
					<< "Usage: " << argv[0] << " [loadfile1.rab | loadfile.pcd | loadfile.ply]" << std::endl;
				exit(-1);
			}
		}
	}

	if (mode == rab)
	{
		const rabv::Reader reader(argv[filenames[0]]);
		const rabv::Viewer::Ptr viewer = reader.visualize(boost::filesystem::path(argv[filenames[0]]).stem().string());
		viewer->spinLoop();
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
		rabv::Rab::Ptr rab(new rabv::Rab);

		if (mode == pcd)
		{
			if (pcl::io::loadPCDFile(argv[filenames[0]], *cloud) == -1)
			{
				std::cerr << "[Rab Visualizer] PCD file io error." << std::endl;
			}
		}
		if (mode == ply)
		{
			if (pcl::io::loadPLYFile(argv[filenames[0]], *cloud) == -1)
			{
				std::cerr << "[Rab Visualizer] PLY file io error." << std::endl;
			}
		}

		if (cloud->size() > 0)
		{
			rab->addCloud("cloud", cloud);

		}
		const rabv::Viewer::Ptr viewer = rabv::Viewer::Ptr(new rabv::Viewer("Viewer", rab));
		viewer->spinLoop();
	}
	
	return 0;
}