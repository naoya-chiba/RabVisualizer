#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable: 4819)

#include <vector>
#include <iostream>
#include <random>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>
#include "rabv.hpp"

int main(int argc, char *argv[])
{
	//
	// Generate sample data
	//

	const int sample_point_num = 100;
	const int sample_corr_num = 5;

	std::mt19937 mt(20150403);
	std::normal_distribution<float> nd(0.0, 0.2);
	std::uniform_int_distribution<int> ud(0, sample_point_num);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	for (int i = 0; i < sample_point_num; ++i)
	{
		sample_cloud->push_back(pcl::PointXYZ(nd(mt), nd(mt), nd(mt)));
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr sample_point_normal(new pcl::PointCloud<pcl::PointNormal>());
	pcl::PointCloud<pcl::Normal>::Ptr sample_normal(new pcl::PointCloud<pcl::Normal>());
	for (int i = 0; i < sample_point_num; ++i)
	{
		pcl::PointNormal pn;
		pn.x = nd(mt);
		pn.y = nd(mt);
		pn.z = nd(mt);
		const auto normal = Eigen::Vector3f(nd(mt), nd(mt), nd(mt)).normalized();
		pn.normal_x = normal.x();
		pn.normal_y = normal.y();
		pn.normal_z = normal.z();
		sample_normal->push_back(pcl::Normal(normal.x(), normal.y(), normal.z()));
		sample_point_normal->push_back(pn);
	}

	pcl::Correspondences corrs1;
	pcl::CorrespondencesPtr corrs2(new pcl::Correspondences());
	std::vector<std::pair<int, int>> corrs3;
	for (int i = 0; i < sample_corr_num; i++)
	{
		int idx = ud(mt);
		pcl::Correspondence corr1(idx, idx, 2.0);
		corrs1.push_back(corr1);
		idx = ud(mt);
		pcl::Correspondence corr2(idx, idx, 2.0);
		corrs2->push_back(corr2);
		idx = ud(mt);
		corrs3.push_back(std::make_pair(idx, idx));
	}



	//
	// Rab Class Usage
	//

	// 1. Make an instance of rabv::Rab
	auto rab = rabv::Rab::create();


	// 2.1. Add a pointcloud
	rab->addCloud(
		"sample1",   // Unique name of the pointcloud
		sample_cloud // Pointcloud
	);

	rab->addCloud(
		"sample2",                    // Unique name
		sample_cloud,                 // Pointcloud
		1,                            // Point size
		rabv::Color(255, 255, 128),   // Color (r, g, b)
		rabv::Point(2.0, 0.0, 0.0),   // Offset of tlanslation (x, y, z)
		rabv::Rotation(0.5, 0.0, 0.0) // Rotation (X-axis, Y-axis, Z-axis)
	);

	// 2.2. Add a pointcloud and normal together
	rab->addCloudNormal(
		"sample3",                  // Unique name
		sample_point_normal,        // Pointcloud and normals
		1,                          // Point size
		rabv::Color(255, 0, 128),   // Color (r, g, b)
		rabv::Point(3.0, 0.0, 0.0), // Offset of tlanslation (x, y, z)
		rabv::Rotation(),           // Rotation (none)
		1,                          // Density of normals
		0.05,                       // Length of normals
		1,                          // Width of normals
		rabv::Color(0, 255, 255)    // Color of normals
	);
	
	// 2.3. Add a normal after "addPointcloud"
	rab->addNormal(
		"sample1",                  // Unique name of the corresponding pointcloud
		sample_normal,              // Pointcloud
		1,                          // Density of normals
		0.05,                       // Length of normals
		2,                          // Width of normals
		rabv::Color(255, 0, 128)    // Color of normals
	);

	
	// 3.1. Add a coordinate system as world coordinate system
	rab->addCoordinateSystem(
		"world", // Unique name of the pointcloud or "world"
		0.3      // Scale factor
	);

	// 3.2. Add a coordinate system as pointcloud coordinate system
	rab->addCoordinateSystem(
		"sample2", // Unique name of the pointcloud or "world"
		0.2        // Scale factor
	);


	// 4. Add correspondence lines between two pointclouds
	rab->addCorrespondence(
		"corr1",   // Unique name of the correspondence
		"sample1", // Unique name of the pointcloud (from)
		"sample2", // Unique name of the pointcloud (to)
		corrs1     // Correspondence (pcl::Correspondences)
	);

	rab->addCorrespondence(
		"corr2",                   // Unique name of the correspondence
		"sample1",                 // Unique name of the pointcloud (from)
		"sample2",                 // Unique name of the pointcloud (to)
		corrs2,                    // Correspondence (pcl::CorrespondencesPtr)
		rabv::Color(255, 255, 128) // Color (r, g, b)
	);

	rab->addCorrespondence(
		"corr3",                   // Unique name of the correspondence
		"sample1",                 // Unique name of the pointcloud (from)
		"sample2",                 // Unique name of the pointcloud (to)
		corrs3,                    // Correspondence (std::vector<std::pair<int, int>>)
		rabv::Color(255, 128, 128) // Color (r, g, b)
	);


	// 5.1. Add lines (Make rabv::Lines first)
	rabv::Lines line1;

	rabv::Lines line2(
		rabv::Color(128, 128, 255) // Color (r, g, b)
	);

	line1.addLine(
		rabv::Point(1.0, 0.0, 0.0), // Point (x, y, z) for "from"
		rabv::Point(1.0, 0.0, 1.0)  // Point (x, y, z) for "to"
	);
	
	line2.addLine(
		pcl::PointXYZ(1.0, 1.0, 1.0), // Point (x, y, z) for "from"
		pcl::PointXYZ(1.0, 1.0, 0.0)  // Point (x, y, z) for "to"
	);

	line2.addLine(rabv::Line(
		{ 1.0, 1.0, 0.0 },  // Point (x, y, z) for "from"
		{ 1.0, 0.0, 0.0 }   // Point (x, y, z) for "to"
	));

	rab->addLines(
		"line1", // Unique name of the lines
		line1    // rabv::Lines
	);
	rab->addLines(
		"line2", // Unique name of the lines
		line2    // rabv::Lines
	);

	// 5.2 Add lines (into rab, directly)
	rab->addLine(
		"line3",                    // Unique name of the lines
		rabv::Point(0.0, 0.0, 0.0), // Point (x, y, z) for "from"
		rabv::Point(2.0, 0.0, 0.0)  // Point (x, y, z) for "to"
	);

	rab->addLine(
		"line3",                      // Unique name of the lines
		pcl::PointXYZ(2.0, 1.0, 0.0), // Point (x, y, z) for "from"
		pcl::PointXYZ(0.0, 1.0, 0.0)  // Point (x, y, z) for "to"
	);


	// 6.1 Add 2D text
	rab->addText(
		"Text1"	// Text
	);

	rab->addText(
		"Text2",               // Text
		300, 0,                // Position (x, y)
		30,                    // Font size
		rabv::Color(0, 255, 0) // Color (r, g, b)
	);

	// 6.2 Add 3D text (can see them always)
	rab->addText3D(
		"Text3" // Text
	);
	rab->addText3D(
		"Text4",                    // Text
		rabv::Point(2.0, 0.5, 0.1), // Position (x, y, z)
		0.1,                        // Font size
		rabv::Color(0, 255, 0)      // Color (r, g, b)
	);

	// 6.3 Add Flat 3D text (can see them from only several viewpoint)
	rab->addFlatText3D(
		"Text5",                   // Text
		rabv::Point(1.0, 0.5, 0.0) // Position (x, y, z)
	);
	rab->addFlatText3D(
		"Text6",                      // Text
		rabv::Point(1.0, 0.5, 0.1),   // Position (x, y, z)
		0.1,                          // Font size
		rabv::Color(0, 255, 0),       // Color (r, g, b)
		rabv::Rotation(0.5, 0.0, 0.0) // Rotation (X-axis, Y-axis, Z-axis)
	);


	// 7. Add a cube
	rab->addCube("cube1", rabv::Point(-1.0, -1.0, -1.0), rabv::Point(-0.5, -0.5, -0.5));

	// 8. Generate good colors to visualize
	const auto& colors = rabv::Color::devideColors(3 /*The number of color*/);

	rab->addText("Color1", 0,   50, 30, colors[0]);
	rab->addText("Color2", 100, 50, 30, colors[1]);
	rab->addText("Color3", 200, 50, 30, colors[2]);



	//
	// Viewer Class Usage
	//

	// 9. Visualze the rab data while the window is closed
	const auto& viewer1 = rabv::Viewer::create(
		"Viewer1",	// Title of viewer
		rab			// Rab data
	);
	viewer1->spinLoop();



	//
	// Writer Class Usage
	//

	// 10.1. Save the rab data (without an instance of rabv::Writer)
	rabv::Writer::saveRabFile(
		"./test1.rab", // Filename
		rab            // Rab data (rabv::Rab::Ptr)
	);

	// 10.2. Save the rab data (by using an instance of rabv::Writer)
	auto writer = rabv::Writer::create("./test2.rab");
	writer->setRab(rab);
	writer->save();
	
	// 11. Visualize from the writer's rab data
	const auto& viewer2 = writer->visualize();
	viewer2->spinLoop();



	//
	// Reader Class Usage
	//

	// 12.1. Make an instance of rabv::Rab
	// If give a path of rab data, it load the data automatically.
	auto reader = rabv::Reader::create();
	
	// 12.2. Set a load path of rab data
	reader->setPath("./test2.rab");
	
	// 12.3. Load the rab data
	reader->load();
	
	// 13. Visualize from the reader's rab data
	const auto& viewer3 = reader->visualize();
	viewer3->spinLoop();

	return 0;
}
