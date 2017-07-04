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

<<<<<<< HEAD
	//サンプルデータ用乱数生成器
=======
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	std::mt19937 mt(20150403);
	std::normal_distribution<float> nd(0.0, 0.2);
	std::uniform_int_distribution<int> ud(0, sample_point_num);

<<<<<<< HEAD
	//サンプル用PointCloudを生成
=======
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
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

<<<<<<< HEAD
	//サンプル用Correspondenceを生成
=======
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
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



<<<<<<< HEAD
	/*** ここからRab使用サンプル ***/

	//Rabのインスタンスを生成
	auto rab = rabv::Rab::create();

	//PointCloudの追加
	rab->addCloud(
		"sample1",		//識別名(それぞれ別名であること)
		sample_cloud	//データ，PointCloud::Ptrで与える
=======
	//
	// Rab Class Usage
	//

	// 1. Make an instance of rabv::Rab
	auto rab = rabv::Rab::create();


	// 2.1. Add a pointcloud
	rab->addCloud(
		"sample1",   // Unique name of the pointcloud
		sample_cloud // Pointcloud
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	);

	rab->addCloud(
<<<<<<< HEAD
		"sample2",						//識別名
		sample_cloud,					//データ
		1,								//点の大きさ
		rabv::Color(255, 255, 128),		//色(R, G, B)
		rabv::Point(2.0, 0.0, 0.0),		//平行移動オフセット(x, y, z)
		rabv::Rotation(0.5, 0.0, 0.0)	//回転(x-axis, y-axis, z-axis)
=======
		"sample2",                    // Unique name
		sample_cloud,                 // Pointcloud
		1,                            // Point size
		rabv::Color(255, 255, 128),   // Color (r, g, b)
		rabv::Point(2.0, 0.0, 0.0),   // Offset of tlanslation (x, y, z)
		rabv::Rotation(0.5, 0.0, 0.0) // Rotation (X-axis, Y-axis, Z-axis)
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	);

	// 2.2. Add a pointcloud and normal together
	rab->addCloudNormal(
<<<<<<< HEAD
		"sample3",						//識別名
		sample_point_normal,			//データ
		1,								//点の大きさ
		rabv::Color(255, 0, 128),		//色(R, G, B)
		rabv::Point(3.0, 0.0, 0.0),		//平行移動オフセット(x, y, z)
		rabv::Rotation(),				//回転なし
		1,								//法線の密度
		0.05,							//法線の長さ
		1,								//法線の太さ
		rabv::Color(0, 255, 255)		//法線の色
=======
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
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	);
	
	// 2.3. Add a normal after "addPointcloud"
	rab->addNormal(
<<<<<<< HEAD
		"sample1",					//識別名(PointCloudとそろえる)
		sample_normal,				//データ
		1,							//法線の密度
		0.05,						//法線の長さ
		2,							//法線の太さ
		rabv::Color(255, 0, 128)	//法線の色
	);



	//CoordinateSystemの追加(絶対座標系)
	rab->addCoordinateSystem(
		0.3	//スケール
	);
	//CoordinateSystemの追加(点群座標系)
	rab->addCoordinateSystem(
		0.2,			//スケール
		"sample2"		//PointCloud識別名
	);

	//Correspondenceの追加
	rab->addCorrespondence(
		"sample1",	//PointCloud識別名(from)
		"sample2",	//PointCloud識別名(to)
		corrs1		//データ(pcl::Correspondences)
	);

	rab->addCorrespondence(
		"sample1",					//PointCloud識別名(from)
		"sample2",					//PointCloud識別名(to)
		corrs2,						//データ(pcl::CorrespondencesPtrもOK)
		rabv::Color(255, 255, 128)	//色(R, G, B)
	);

	rab->addCorrespondence(
		"sample1",					//PointCloud識別名(from)
		"sample2",					//PointCloud識別名(to)
		corrs3,						//データ(std::vector<std::pair<int, int>>もOK)
		rabv::Color(255, 128, 128)	//色(R, G, B)
	);

	//Lineの追加(先にLinesを生成しておく場合)
	rabv::Lines line1(
		"line1"	//識別名(それぞれ別名であること)
	);
	rabv::Lines line2(
		"line2",					//識別名(それぞれ別名であること)
		rabv::Color(128, 128, 255)	//色(R, G, B)
	);

	//LinesにLineを追加
	line1.addLine(
		rabv::Point(1.0, 0.0, 0.0),	//from座標(X, Y, Z)
		rabv::Point(1.0, 0.0, 1.0)	//to座標(X, Y, Z)
=======
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
		2,                         // Line width
		rabv::Color(255, 255, 128) // Color (r, g, b)
	);

	rab->addCorrespondence(
		"corr3",                   // Unique name of the correspondence
		"sample1",                 // Unique name of the pointcloud (from)
		"sample2",                 // Unique name of the pointcloud (to)
		corrs3,                    // Correspondence (std::vector<std::pair<int, int>>)
		3,                         // Line width
		rabv::Color(255, 128, 128) // Color (r, g, b)
	);


	// 5.1. Add lines (Make rabv::Lines first)
	rabv::Lines line1;

	rabv::Lines line2(
		3.0,                       // Line width
		rabv::Color(128, 128, 255) // Color (r, g, b)
	);

	line1.addLine(
		rabv::Point(1.0, 0.0, 0.0), // Point (x, y, z) for "from"
		rabv::Point(1.0, 0.0, 1.0)  // Point (x, y, z) for "to"
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
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

<<<<<<< HEAD
	//Linesをwriterに追加
	rab->addLines(line1);
	rab->addLines(line2);

	//Lineの追加(逐次識別名を指定, ちょっと遅い)
	//第一引数に識別名が増える．以降は先にLinesを生成しておく場合と同様
	rab->addLine("line3", rabv::Point(0.0, 0.0, 0.0), rabv::Point(2.0, 0.0, 0.0));
	rab->addLine("line3", pcl::PointXYZ(2.0, 1.0, 0.0), pcl::PointXYZ(0.0, 1.0, 0.0));
	rab->addLine("line3", rabv::Line({ 0.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0 }));

	//Textの追加
	rab->addText(
		"Text1"		//表示Text
=======
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
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	);

	rab->addText(
<<<<<<< HEAD
		"Text2",				//表示Text
		300, 0,					//表示座標(X, Y)
		30,						//文字サイズ
		rabv::Color(0, 255, 0)	//色(R, G, B)
	);

	//Text3Dの追加
	rab->addText3D(
		"Text3"			//表示Text
	);
	rab->addText3D(
		"Text4",					//表示Text
		rabv::Point(2.0, 0.5, 0.1),	//表示座標(X, Y, Z)
		0.1,						//文字サイズ
		rabv::Color(0, 255, 0)		//色(R, G, B)
	);

	//FlatText3Dの追加
	rab->addFlatText3D(
		"Text5",					//表示Text
		rabv::Point(1.0, 0.5, 0.0)	//表示座標(X, Y, Z)
	);
	rab->addFlatText3D(
		"Text6",						//表示Text
		rabv::Point(1.0, 0.5, 0.1),		//表示座標(X, Y, Z)
		0.1,							//文字サイズ
		rabv::Color(0, 255, 0),			//色(R, G, B)
		rabv::Rotation(0.5, 0.0, 0.0)	//回転(x-axis, y-axis, z-axis)
	);

	//見やすい色の自動生成(by 福地)
	auto colors = rabv::Color::devideColors(3 /*生成する色数*/);
=======
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

>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0

	// 7. Add a cube
	rab->addCube("cube1", rabv::Point(-1.0, -1.0, -1.0), rabv::Point(-0.5, -0.5, -0.5));

	// 8. Generate good colors to visualize
	const auto& colors = rabv::Color::devideColors(3 /*The number of color*/);

	rab->addText("Color1", 0,   50, 30, colors[0]);
	rab->addText("Color2", 100, 50, 30, colors[1]);
	rab->addText("Color3", 200, 50, 30, colors[2]);



<<<<<<< HEAD
	/*** ここからViewer使用サンプル ***/

	//表示(閉じるまでループ, Viewerを使う)
	auto viewer1 = rabv::Viewer::create(
		"Viewer1",	//Viewerのタイトル
		rab			//表示するrabデータ
=======
	//
	// Viewer Class Usage
	//

	// 9. Visualze the rab data while the window is closed
	const auto& viewer1 = rabv::Viewer::create(
		"Viewer1",	// Title of viewer
		rab			// Rab data
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	);
	viewer1->spinLoop();



<<<<<<< HEAD
	/*** ここからWriter使用サンプル ***/

	//保存
	rabv::Writer::saveRabFile(
		"./test1.rab",	//保存ファイルパス(相対パスの場合．カレントディレクトリで展開)
		rab				//保存するrabデータ(rabv::Rab::Ptr)
	);

	//Writerのインスタンスを作って保存
=======
	//
	// Writer Class Usage
	//

	// 10.1. Save the rab data (without an instance of rabv::Writer)
	rabv::Writer::saveRabFile(
		"./test1.rab", // Filename
		rab            // Rab data (rabv::Rab::Ptr)
	);

	// 10.2. Save the rab data (by using an instance of rabv::Writer)
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	auto writer = rabv::Writer::create("./test2.rab");
	writer->setRab(rab);
	writer->save();
	
<<<<<<< HEAD
	//表示(閉じるまでループ, Viewerを使う)
	rabv::Viewer::Ptr viewer2 = writer->visualize();
=======
	// 11. Visualize from the writer's rab data
	const auto& viewer2 = writer->visualize();
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	viewer2->spinLoop();



	//
	// Reader Class Usage
	//

<<<<<<< HEAD
	/*** ここからReader使用サンプル ***/

	//Readerのインスタンスを生成．インスタンス生成時にパスを渡すと読み込みまで自動で行う
	auto reader = rabv::Reader::create();
	
	//パス指定
	reader->setPath("./test2.rab");
	
	//読み込み
	reader->load();
	
	//表示(閉じるまでループ, Viewerを使う)
	rabv::Viewer::Ptr viewer3 = writer->visualize();
=======
	// 12.1. Make an instance of rabv::Rab
	// If give a path of rab data, it load the data automatically.
	auto reader = rabv::Reader::create();
	
	// 12.2. Set a load path of rab data
	reader->setPath("./test2.rab");
	
	// 12.3. Load the rab data
	reader->load();
	
	// 13. Visualize from the reader's rab data
	const auto& viewer3 = reader->visualize();
>>>>>>> 48c899ed15f48337747564afdeacf00b34e4e4f0
	viewer3->spinLoop();

	return 0;
}
