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

	//�T���v���f�[�^�p����������
	std::mt19937 mt(20150403);
	std::normal_distribution<float> nd(0.0, 0.2);
	std::uniform_int_distribution<int> ud(0, sample_point_num);

	//�T���v���pPointCloud�𐶐�
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

	//�T���v���pCorrespondence�𐶐�
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



	/*** ��������Rab�g�p�T���v�� ***/

	//Rab�̃C���X�^���X�𐶐�
	auto rab = rabv::Rab::create();

	//PointCloud�̒ǉ�
	rab->addCloud(
		"sample1",		//���ʖ�(���ꂼ��ʖ��ł��邱��)
		sample_cloud	//�f�[�^�CPointCloud::Ptr�ŗ^����
	);
	rab->addCloud(
		"sample2",						//���ʖ�
		sample_cloud,					//�f�[�^
		1,								//�_�̑傫��
		rabv::Color(255, 255, 128),		//�F(R, G, B)
		rabv::Point(2.0, 0.0, 0.0),		//���s�ړ��I�t�Z�b�g(x, y, z)
		rabv::Rotation(0.5, 0.0, 0.0)	//��](x-axis, y-axis, z-axis)
	);

	rab->addCloudNormal(
		"sample3",						//���ʖ�
		sample_point_normal,			//�f�[�^
		1,								//�_�̑傫��
		rabv::Color(255, 0, 128),		//�F(R, G, B)
		rabv::Point(3.0, 0.0, 0.0),		//���s�ړ��I�t�Z�b�g(x, y, z)
		rabv::Rotation(),				//��]�Ȃ�
		1,								//�@���̖��x
		0.05,							//�@���̒���
		1,								//�@���̑���
		rabv::Color(0, 255, 255)		//�@���̐F
	);
	
	rab->addNormal(
		"sample1",					//���ʖ�(PointCloud�Ƃ��낦��)
		sample_normal,				//�f�[�^
		1,							//�@���̖��x
		0.05,						//�@���̒���
		2,							//�@���̑���
		rabv::Color(255, 0, 128)	//�@���̐F
	);



	//CoordinateSystem�̒ǉ�(��΍��W�n)
	rab->addCoordinateSystem(
		0.3	//�X�P�[��
	);
	//CoordinateSystem�̒ǉ�(�_�Q���W�n)
	rab->addCoordinateSystem(
		0.2,			//�X�P�[��
		"sample2"		//PointCloud���ʖ�
	);

	//Correspondence�̒ǉ�
	rab->addCorrespondence(
		"sample1",	//PointCloud���ʖ�(from)
		"sample2",	//PointCloud���ʖ�(to)
		corrs1		//�f�[�^(pcl::Correspondences)
	);

	rab->addCorrespondence(
		"sample1",					//PointCloud���ʖ�(from)
		"sample2",					//PointCloud���ʖ�(to)
		corrs2,						//�f�[�^(pcl::CorrespondencesPtr��OK)
		rabv::Color(255, 255, 128)	//�F(R, G, B)
	);

	rab->addCorrespondence(
		"sample1",					//PointCloud���ʖ�(from)
		"sample2",					//PointCloud���ʖ�(to)
		corrs3,						//�f�[�^(std::vector<std::pair<int, int>>��OK)
		rabv::Color(255, 128, 128)	//�F(R, G, B)
	);

	//Line�̒ǉ�(���Lines�𐶐����Ă����ꍇ)
	rabv::Lines line1(
		"line1"	//���ʖ�(���ꂼ��ʖ��ł��邱��)
	);
	rabv::Lines line2(
		"line2",					//���ʖ�(���ꂼ��ʖ��ł��邱��)
		rabv::Color(128, 128, 255)	//�F(R, G, B)
	);

	//Lines��Line��ǉ�
	line1.addLine(
		rabv::Point(1.0, 0.0, 0.0),	//from���W(X, Y, Z)
		rabv::Point(1.0, 0.0, 1.0)	//to���W(X, Y, Z)
	);
	line2.addLine(
		pcl::PointXYZ(1.0, 1.0, 1.0),	//from(PointXYZ)
		pcl::PointXYZ(1.0, 1.0, 0.0)	//to(PointXYZ)
	);
	line2.addLine(
		rabv::Line({ 1.0, 1.0, 0.0 }, { 1.0, 0.0, 0.0 })	//rabv::Line
	);

	//Lines��writer�ɒǉ�
	rab->addLines(line1);
	rab->addLines(line2);

	//Line�̒ǉ�(�������ʖ����w��, ������ƒx��)
	//�������Ɏ��ʖ���������D�ȍ~�͐��Lines�𐶐����Ă����ꍇ�Ɠ��l
	rab->addLine("line3", rabv::Point(0.0, 0.0, 0.0), rabv::Point(2.0, 0.0, 0.0));
	rab->addLine("line3", pcl::PointXYZ(2.0, 1.0, 0.0), pcl::PointXYZ(0.0, 1.0, 0.0));
	rab->addLine("line3", rabv::Line({ 0.0, 1.0, 0.0 }, { 0.0, 0.0, 0.0 }));

	//Text�̒ǉ�
	rab->addText(
		"Text1"		//�\��Text
	);
	rab->addText(
		"Text2",				//�\��Text
		300, 0,					//�\�����W(X, Y)
		30,						//�����T�C�Y
		rabv::Color(0, 255, 0)	//�F(R, G, B)
	);

	//Text3D�̒ǉ�
	rab->addText3D(
		"Text3"			//�\��Text
	);
	rab->addText3D(
		"Text4",					//�\��Text
		rabv::Point(2.0, 0.5, 0.1),	//�\�����W(X, Y, Z)
		0.1,						//�����T�C�Y
		rabv::Color(0, 255, 0)		//�F(R, G, B)
	);

	//FlatText3D�̒ǉ�
	rab->addFlatText3D(
		"Text5",					//�\��Text
		rabv::Point(1.0, 0.5, 0.0)	//�\�����W(X, Y, Z)
	);
	rab->addFlatText3D(
		"Text6",						//�\��Text
		rabv::Point(1.0, 0.5, 0.1),		//�\�����W(X, Y, Z)
		0.1,							//�����T�C�Y
		rabv::Color(0, 255, 0),			//�F(R, G, B)
		rabv::Rotation(0.5, 0.0, 0.0)	//��](x-axis, y-axis, z-axis)
	);

	//���₷���F�̎�������(by ���n)
	auto colors = rabv::Color::devideColors(3 /*��������F��*/);

	rab->addText("Color1", 0, 50, 30, colors[0]);
	rab->addText("Color2", 100, 50, 30, colors[1]);
	rab->addText("Color3", 200, 50, 30, colors[2]);



	/*** ��������Viewer�g�p�T���v�� ***/

	//�\��(����܂Ń��[�v, Viewer���g��)
	auto viewer1 = rabv::Viewer::create(
		"Viewer1",	//Viewer�̃^�C�g��
		rab			//�\������rab�f�[�^
	);
	viewer1->spinLoop();



	/*** ��������Writer�g�p�T���v�� ***/

	//�ۑ�
	rabv::Writer::saveRabFile(
		"./test1.rab",	//�ۑ��t�@�C���p�X(���΃p�X�̏ꍇ�D�J�����g�f�B���N�g���œW�J)
		rab				//�ۑ�����rab�f�[�^(rabv::Rab::Ptr)
	);

	//Writer�̃C���X�^���X������ĕۑ�
	auto writer = rabv::Writer::create("./test2.rab");
	writer->setRab(rab);
	writer->save();
	
	//�\��(����܂Ń��[�v, Viewer���g��)
	rabv::Viewer::Ptr viewer2 = writer->visualize();
	viewer2->spinLoop();




	/*** ��������Reader�g�p�T���v�� ***/

	//Reader�̃C���X�^���X�𐶐��D�C���X�^���X�������Ƀp�X��n���Ɠǂݍ��݂܂Ŏ����ōs��
	auto reader = rabv::Reader::create();
	
	//�p�X�w��
	reader->setPath("./test2.rab");
	
	//�ǂݍ���
	reader->load();
	
	//�\��(����܂Ń��[�v, Viewer���g��)
	rabv::Viewer::Ptr viewer3 = writer->visualize();
	viewer3->spinLoop();

	return 0;
}