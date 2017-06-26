#define _CRT_SECURE_NO_WARNINGS
#pragma warning(disable: 4819)

#include <iostream>
#include <random>
#include <pcl/point_cloud.h>
#include "rabv.hpp"

int main(int argc, char *argv[])
{
	const int sample_point_num = 100;
	const int sample_corr_num = 5;

	//サンプルデータ用乱数生成器
	std::mt19937 mt(20150403);
	std::normal_distribution<float> nd(0.0, 0.2);
	std::uniform_int_distribution<int> ud(0, sample_point_num);

	//サンプル用PointCloudを生成
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
		const auto n = Eigen::Vector3f(nd(mt), nd(mt), nd(mt)).normalized();
		pn.normal_x = n.x();
		pn.normal_y = n.y();
		pn.normal_z = n.z();
		sample_normal->push_back(pcl::Normal(n.x(), n.y(), n.z()));
		sample_point_normal->push_back(pn);
	}

	//サンプル用Correspondenceを生成
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



	/*** ここからRab使用サンプル ***/

	//Rabのインスタンスを生成
	auto rab = rabv::Rab::create();

	//PointCloudの追加
	rab->addCloud(
		"sample1",		//識別名(それぞれ別名であること)
		sample_cloud	//データ，PointCloud::Ptrで与える
	);
	rab->addCloud(
		"sample2",						//識別名
		sample_cloud,					//データ
		1,								//点の大きさ
		rabv::Color(255, 255, 128),		//色(R, G, B)
		rabv::Point(2.0, 0.0, 0.0),		//平行移動オフセット(x, y, z)
		rabv::Rotation(0.5, 0.0, 0.0)	//回転(x-axis, y-axis, z-axis)
	);

	rab->addCloudNormal(
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
	);
	
	rab->addNormal(
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
	);
	line2.addLine(
		pcl::PointXYZ(1.0, 1.0, 1.0),	//from(PointXYZ)
		pcl::PointXYZ(1.0, 1.0, 0.0)	//to(PointXYZ)
	);
	line2.addLine(
		rabv::Line({ 1.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 })	//rabv::Line
	);

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
	);
	rab->addText(
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

	rab->addText("Color1", 0, 50, 30, colors[0]);
	rab->addText("Color2", 100, 50, 30, colors[1]);
	rab->addText("Color3", 200, 50, 30, colors[2]);



	/*** ここからViewer使用サンプル ***/

	//表示(閉じるまでループ, Viewerを使う)
	auto viewer1 = rabv::Viewer::create(
		"Viewer1",	//Viewerのタイトル
		rab			//表示するrabデータ
	);
	viewer1->spinLoop();



	/*** ここからWriter使用サンプル ***/

	//保存
	rabv::Writer::saveRabFile(
		"./test1.rab",	//保存ファイルパス(相対パスの場合．カレントディレクトリで展開)
		rab				//保存するrabデータ(rabv::Rab::Ptr)
	);

	//Writerのインスタンスを作って保存
	auto writer = rabv::Writer::create("./test2.rab");
	writer->setRab(rab);
	writer->save();
	
	//表示(閉じるまでループ, Viewerを使う)
	rabv::Viewer::Ptr viewer2 = writer->visualize();
	viewer2->spinLoop();




	/*** ここからReader使用サンプル ***/

	//Readerのインスタンスを生成．インスタンス生成時にパスを渡すと読み込みまで自動で行う
	auto reader = rabv::Reader::create();
	
	//パス指定
	reader->setPath("./test2.rab");
	
	//読み込み
	reader->load();
	
	//表示(閉じるまでループ, Viewerを使う)
	rabv::Viewer::Ptr viewer3 = writer->visualize();
	viewer3->spinLoop();

	return 0;
}