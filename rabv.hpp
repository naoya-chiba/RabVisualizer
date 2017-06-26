// Rab Visualizer Ver.2.90 beta
// 2017/06/14
// Copyright (C) 2015-2017 Chiba Naoya

#pragma once
#pragma warning(disable: 4819)

#include <vector>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCamera.h>
#include <vtkCollectionIterator.h>
#include <vtkAxisFollower.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkPolyData.h>
#include <vtkPolyLine.h>

#ifdef near
#undef near
#endif
#ifdef far
#undef far
#endif

namespace
{
namespace rabv
{
	class Color;
	class Rotation;
	class Line;
	class Lines;
	class Text;
	class Text3D;
	class FlatText3D;
	class Cloud;
	class Normal;
	class Correspondence;
	class CoordinateSystem;
	class Rab;
	class Viewer;
	class Reader;
	class Writer;

	typedef pcl::PointXYZ Point;
	typedef std::vector<rabv::Lines> LinesSet;
	typedef std::vector<rabv::Text> Texts;
	typedef std::vector<rabv::Text3D> Text3Ds;
	typedef std::vector<rabv::FlatText3D> FlatText3Ds;
	typedef std::vector<rabv::Cloud> Clouds;
	typedef std::vector<rabv::Normal> Normals;
	typedef std::vector<rabv::Correspondence> Correspondences;
	typedef std::vector<rabv::CoordinateSystem> CoordinateSystems;

	template<typename T>
	constexpr const T PI()
	{
		return static_cast<T>(std::atan2(0.0, -1.0));
	}

	template<typename T>
	class Creatable
	{
	public:
		template <typename... Args>
		static boost::shared_ptr<T> create(Args... args)
		{
			return boost::shared_ptr<T>(new T(std::forward<Args>(args)...));
		}

		virtual boost::shared_ptr<T> makeShared()
		{
			return boost::shared_ptr<T>(new T(*static_cast<T*>(this)));
		}
	};

	class Color
	{
	public:
		int r;
		int g;
		int b;

		// r, g, b: 0 - 255
		Color(const int r_, const int g_, const int b_) : r(r_), g(g_), b(b_) {}
		Color() : r(255), g(255), b(255) {}

		static rabv::Color getColorFromHSV(const int h, const int s, const int v)
		{
			const int i = static_cast<int>(std::floor(h / 60.0f)) % 6;
			const double f = static_cast<double>((h / 60.0f) - static_cast<double>(std::floor(h / 60.0f)));
			const int p = static_cast<int>(std::round(v * (1.0f - (s / 255.0f))));
			const int q = static_cast<int>(std::round(v * (1.0f - (s / 255.0f) * f)));
			const int t = static_cast<int>(std::round(v * (1.0f - (s / 255.0f) * (1.0f - f))));

			switch (i)
			{
			case 0:
				return rabv::Color(v, t, p);
			case 1:
				return rabv::Color(q, v, p);
			case 2:
				return rabv::Color(p, v, t);
			case 3:
				return rabv::Color(p, q, v);
			case 4:
				return rabv::Color(t, p, v);
			case 5:
				return rabv::Color(v, p, q);
			}
			return rabv::Color();
		}

		static std::vector<rabv::Color> devideColors(const int step)
		{
			std::vector<rabv::Color> color_vec;
			if (step < 1)
			{
				color_vec.push_back(rabv::Color());
			}
			int step_h = static_cast<int>(std::round(360.0 / step));
			for (int i = 0; i < step; ++i)
			{
				color_vec.push_back(rabv::Color::getColorFromHSV(step_h * i, 255, 255));
			}
			return color_vec;
		}
	};

	class Rotation
	{
	public:
		double x, y, z;

		Rotation(const double x_, const double y_, const double z_) : x(x_), y(y_), z(z_) {}
		Rotation() : x(0.0), y(0.0), z(0.0) {}

		template<typename T>
		Rotation(const Eigen::Matrix<T, 3, 3>& rotmat)
		{
			if (rotmat(2, 0) == -1.0)
			{
				x = std::atan2(rotmat(1, 1), rotmat(0, 1));
				y = rabv::pi() / 2;
				z = 0.0;
			}

			if (rotmat(2, 0) == 1.0)
			{
				x = std::atan2(-rotmat(1, 1), rotmat(0, 1));
				y = -rabv::pi<double>() / 2.0;
				z = 0.0;
			}

			x = std::atan2(rotmat(2, 1), rotmat(2, 2));
			y = std::atan(std::sqrt(
				rotmat(2, 0) * rotmat(2, 0) /
				(rotmat(2, 1) * rotmat(2, 1) + rotmat(2, 2) * rotmat(2, 2))
			));
			z = std::atan2(rotmat(1, 0), rotmat(0, 0));
		}

		Eigen::Affine3f affine() const
		{
			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.rotate(Eigen::AngleAxisf(z, Eigen::Vector3f::UnitZ()));
			transform.rotate(Eigen::AngleAxisf(y, Eigen::Vector3f::UnitY()));
			transform.rotate(Eigen::AngleAxisf(x, Eigen::Vector3f::UnitX()));

			return transform;
		}

		Eigen::Matrix3f matrix() const
		{
			return this->affine().matrix().block<3, 3>(0, 0);
		}
	};

	class Line
	{
	public:
		rabv::Point from;
		rabv::Point to;

		Line(const rabv::Point& from_, const rabv::Point& to_)
		{
			from = from_;
			to = to_;
		}
	};

	class Lines : public Creatable<Lines>
	{
	public:
		typedef boost::shared_ptr<rabv::Lines> Ptr;
		typedef boost::shared_ptr<const rabv::Lines> ConstPtr;
		std::string name;
		std::vector<rabv::Line> lines;
		rabv::Color color;
		bool visible;

		Lines(
			const std::string& name_,
			const rabv::Color& color_ = rabv::Color(),
			const std::vector<rabv::Line>& lines_ = std::vector<rabv::Line>(),
			const bool visible_ = true)
			: name(name_), lines(lines_), color(color_), visible(visible_) {}

		void addLine(const rabv::Line& line)
		{
			lines.push_back(line);
		}
		template<typename... Args>
		void addLine(Args... args)
		{
			lines.push_back(rabv::Line(std::forward<Args>(args)...));
		}
	};

	class Text
	{
	public:
		std::string text;
		int x, y;
		int font_size;
		rabv::Color color;
		bool visible;

		Text(
			const std::string& text_,
			const int x_ = 0,
			const int y_ = 0,
			const int font_size_ = 30,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: text(text_),
			  x(x_),
			  y(y_),
			  font_size(font_size_),
			  color(color_),
			  visible(visible_) {}
	};

	class Text3D
	{
	public:
		std::string text;
		double x, y, z;
		double font_size;
		rabv::Color color;
		bool visible;

		Text3D(
			const std::string& text_,
			rabv::Point& point = rabv::Point(),
			const double font_size_ = 0.1,
			const rabv::Color color_ = rabv::Color(),
			const bool visible_ = true)
			: text(text_),
			  x(point.x),
			  y(point.y),
			  z(point.z),
			  font_size(font_size_),
			  color(color_),
			  visible(visible_) {}
	};

	// Fast Text3D by Fukuchi
	class FlatText3D
	{
	public:
		std::string text;
		double x, y, z;
		double font_size;
		rabv::Color color;
		rabv::Rotation rotation;
		bool visible;

		FlatText3D(
			const std::string& text_,
			const rabv::Point& point = rabv::Point(),
			const double font_size_ = 0.1,
			const rabv::Color& color_ = rabv::Color(),
			const rabv::Rotation& rotation_ = rabv::Rotation(),
			const bool visible_ = true)
			: text(text_),
			  x(point.x),
			  y(point.y),
			  z(point.z),
			  font_size(font_size_),
			  color(color_),
			  rotation(rotation_),
			  visible(visible_) {}
	};

	class Cloud
	{
	public:
		std::string name;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		rabv::Color color;
		rabv::Point offset;
		int point_size;
		rabv::Rotation rotation;
		bool visible;

		Cloud(
			const std::string& name_,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ =
				pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()),
			const int point_size_ = 1,
			const rabv::Color& color_ = rabv::Color(),
			const rabv::Point& offset_ = rabv::Point(),
			const rabv::Rotation& rotation_ = rabv::Rotation(),
			const bool visible_ = true)
			: name(name_),
			  offset(offset_),
			  cloud(cloud_),
			  color(color_),
			  point_size(point_size_),
			  rotation(rotation_),
			  visible(visible_) {};

		// copy constructor
		Cloud(const rabv::Cloud& cloud_)
			: name(cloud_.name),
			  offset(cloud_.offset),
			  color(cloud_.color),
			  point_size(cloud_.point_size),
			  rotation(cloud_.rotation),
			  visible(cloud_.visible)
		{
			cloud = cloud_.cloud->makeShared();
		}

		// move constructor
		Cloud(rabv::Cloud&& cloud_)
			: name(cloud_.name),
			  offset(cloud_.offset),
			  color(cloud_.color),
			  point_size(cloud_.point_size),
			  rotation(cloud_.rotation),
			  visible(cloud_.visible)
		{
			cloud = std::move(cloud_.cloud);
		}

		// copy
		Cloud& operator=(const rabv::Cloud& cloud_)
		{
			name = cloud_.name;
			offset = cloud_.offset;
			cloud = cloud_.cloud->makeShared();
			color = cloud_.color;
			point_size = cloud_.point_size;
			rotation = cloud_.rotation;
			visible = cloud_.visible;
			return *this;
		}

		// move
		Cloud& operator=(rabv::Cloud&& cloud_)
		{
			name = cloud_.name;
			offset = cloud_.offset;
			cloud = std::move(cloud_.cloud);
			color = cloud_.color;
			point_size = cloud_.point_size;
			rotation = cloud_.rotation;
			visible = cloud_.visible;
			return *this;
		}
	};

	class Normal
	{
	public:
		std::string name;
		pcl::PointCloud<pcl::Normal>::Ptr normal;
		int level;
		double scale;
		int line_width;
		rabv::Color color;
		bool visible;

		Normal(
			const std::string& name_,
			const pcl::PointCloud<pcl::Normal>::Ptr& normal_ =
				pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>()),
			const int level_ = 100,
			const double scale_ = 0.02,
			const int line_width_ = 1,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: name(name_),
			  normal(normal_),
			  level(level_),
			  scale(scale_),
			  line_width(line_width_),
			  color(color_),
			  visible(visible_) {};

		// copy constructor
		Normal(const rabv::Normal& normal_)
			: name(normal_.name),
			  level(normal_.level),
			  scale(normal_.scale),
			  line_width(normal_.line_width),
			  color(normal_.color),
			  visible(normal_.visible)
		{
			normal = normal_.normal->makeShared();
		}

		// move constructor
		Normal(rabv::Normal&& normal_)
			: name(normal_.name),
			  level(normal_.level),
			  scale(normal_.scale),
			  line_width(normal_.line_width),
			  color(normal_.color),
			  visible(normal_.visible)
		{
			normal = std::move(normal_.normal);
		}

		// copy
		Normal& operator=(const rabv::Normal& normal_)
		{
			name = normal_.name;
			normal = normal_.normal->makeShared();
			level = normal_.level;
			scale = normal_.scale;
			line_width = normal_.line_width;
			color = normal_.color;
			visible = normal_.visible;
			return *this;
		}

		// move
		Normal& operator=(rabv::Normal&& normal_)
		{
			name = normal_.name;
			normal = std::move(normal_.normal);
			level = normal_.level;
			scale = normal_.scale;
			line_width = normal_.line_width;
			color = normal_.color;
			visible = normal_.visible;
			return *this;
		}
	};

	class Correspondence : Creatable<Correspondence>
	{
	public:
		typedef boost::shared_ptr<rabv::Correspondence> Ptr;
		typedef boost::shared_ptr<const rabv::Correspondence> ConstPtr;
		typedef std::pair<int/* from */, int/* to */> IndexPair;
		std::string from;
		std::string to;
		std::string name;
		std::vector<rabv::Correspondence::IndexPair> pairs;
		rabv::Color color;
		bool visible;

		Correspondence(
			const std::string& name_,
			const std::string& from_,
			const std::string& to_,
			const rabv::Color color_ = rabv::Color(),
			const bool visible_ = true)
			: name(name_), from(from_), to(to_), color(color_), visible(visible_) {}
		Correspondence(
			const std::string& from_,
			const std::string& to_,
			const std::vector<std::pair<int, int>>& pairs_,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: name(from_ + "_" + to_), from(from_), to(to_), color(color_), visible(visible_)
		{
			for (const auto& pair : pairs_)
			{
				pairs.push_back(pair);
			}
		}
		Correspondence(
			const std::string& from_,
			const std::string& to_,
			const pcl::Correspondences& correspondences_,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: name(from_ + "_" + to_), from(from_), to(to_), color(color_), visible(visible_)
		{
			for (const auto& corr : correspondences_)
			{
				pairs.push_back(rabv::Correspondence::IndexPair(corr.index_query, corr.index_match));
			}
		}
		Correspondence(
			const std::string& from_,
			const std::string& to_,
			const pcl::CorrespondencesPtr& correspondences_,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: name(from_ + "_" + to_), from(from_), to(to_), color(color_), visible(visible_)
		{
			for (const auto& corr : *correspondences_)
			{
				pairs.push_back(rabv::Correspondence::IndexPair(corr.index_query, corr.index_match));
			}
		}
	};

	class CoordinateSystem
	{
	public:
		std::string name;
		double scale;
		bool visible;

		CoordinateSystem(
			const double& scale_ = 1.0,
			const std::string& cloud_name_ = "world",
			const bool visible_ = true)
			: name(cloud_name_), scale(scale_), visible(visible_) {}
	};

	boost::property_tree::ptree CameraToPree(const pcl::visualization::Camera& cam)
	{
		boost::property_tree::ptree ptree;

		ptree.add("ClippingPlane.Near", cam.clip[0]);
		ptree.add("ClippingPlane.Far", cam.clip[1]);
		ptree.add("FocalPoint.x", cam.focal[0]);
		ptree.add("FocalPoint.y", cam.focal[1]);
		ptree.add("FocalPoint.z", cam.focal[2]);
		ptree.add("Position.x", cam.pos[0]);
		ptree.add("Position.y", cam.pos[1]);
		ptree.add("Position.z", cam.pos[2]);
		ptree.add("ViewUp.x", cam.view[0]);
		ptree.add("ViewUp.y", cam.view[1]);
		ptree.add("ViewUp.z", cam.view[2]);
		ptree.add("Window.Width", cam.window_size[0]);
		ptree.add("Window.Height", cam.window_size[1]);

		return ptree;
	}

	pcl::visualization::Camera PreeToCamera(const boost::property_tree::ptree& ptree)
	{
		pcl::visualization::Camera cam;
		try
		{
			cam.clip[0] = ptree.get<double>("ClippingPlane.Near");
			cam.clip[1] = ptree.get<double>("ClippingPlane.Far");
			cam.focal[0] = ptree.get<double>("FocalPoint.x");
			cam.focal[1] = ptree.get<double>("FocalPoint.y");
			cam.focal[2] = ptree.get<double>("FocalPoint.z");
			cam.pos[0] = ptree.get<double>("Position.x");
			cam.pos[1] = ptree.get<double>("Position.y");
			cam.pos[2] = ptree.get<double>("Position.z");
			cam.view[0] = ptree.get<double>("ViewUp.x");
			cam.view[1] = ptree.get<double>("ViewUp.y");
			cam.view[2] = ptree.get<double>("ViewUp.z");
			cam.window_size[0] = ptree.get<double>("Window.Width");
			cam.window_size[1] = ptree.get<double>("Window.Height");
		}
		catch (const boost::property_tree::xml_parser::xml_parser_error& e)
		{
			std::cerr
				<< "[Rab Visualizer] Can't load camera settings."
				<< std::endl << e.what() << std::endl;
		}
		return cam;
	}

	class Rab : public Creatable<Rab>
	{
	public:
		typedef boost::shared_ptr<rabv::Rab> Ptr;
		typedef boost::shared_ptr<const rabv::Rab> ConstPtr;
		rabv::Clouds clouds;
		rabv::Normals normals;
		rabv::Correspondences correspondences;
		rabv::LinesSet lines_set;
		rabv::Texts texts;
		rabv::Text3Ds text3Ds;
		rabv::FlatText3Ds flat_text3Ds;
		rabv::CoordinateSystems coordinate_systems;
		boost::optional<pcl::visualization::Camera> camera;

		void addCloud(const rabv::Cloud& cloud_)
		{
			clouds.push_back(cloud_);
		}
		template<typename... Args>
		void addCloud(Args... args)
		{
			clouds.push_back(rabv::Cloud(std::forward<Args>(args)...));
		}

		void addNormal(const rabv::Normal& cloud_)
		{
			normals.push_back(cloud_);
		}
		template<typename... Args>
		void addNormal(Args... args)
		{
			normals.push_back(rabv::Normal(std::forward<Args>(args)...));
		}

		void addCloudNormal(
			const std::string& name_,
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_,
			const pcl::PointCloud<pcl::Normal>::Ptr& normal_,
			const int point_size_ = 1,
			const rabv::Color& cloud_color_ = rabv::Color(),
			const rabv::Point& offset_ = rabv::Point(),
			const rabv::Rotation& rotation_ = rabv::Rotation(),
			const int level_ = 100,
			const double scale_ = 0.02,
			const int line_width_ = 1,
			const rabv::Color& normal_color_ = rabv::Color(),
			const bool visible_ = true)
		{
			clouds.push_back(rabv::Cloud(
				name_,
				cloud_,
				point_size_,
				cloud_color_,
				offset_,
				rotation_,
				visible_
			));

			normals.push_back(rabv::Normal(
				name_,
				normal_,
				level_,
				scale_,
				line_width_,
				normal_color_,
				visible_
			));
		}

		void addCloudNormal(
			const std::string& name_,
			const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud_,
			const int point_size_ = 1,
			const rabv::Color& cloud_color_ = rabv::Color(),
			const rabv::Point& offset_ = rabv::Point(),
			const rabv::Rotation& rotation_ = rabv::Rotation(),
			const int level_ = 100,
			const double scale_ = 0.02,
			const int line_width_ = 1,
			const rabv::Color& normal_color_ = rabv::Color(),
			const bool visible_ = true)
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
			pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());
			pcl::copyPointCloud(*cloud_, *cloud);
			pcl::copyPointCloud(*cloud_, *normal);

			clouds.push_back(rabv::Cloud(
				name_,
				cloud,
				point_size_,
				cloud_color_,
				offset_,
				rotation_,
				visible_
			));

			normals.push_back(rabv::Normal(
				name_,
				normal,
				level_,
				scale_,
				line_width_,
				normal_color_,
				visible_
			));
		}

		// parameters will be overwritten
		void addPoint(
			const std::string& name_ = "world",
			const rabv::Point point = rabv::Point(),
			const int point_size_ = 1,
			const rabv::Color& color_ = rabv::Color(),
			const rabv::Point& offset_ = rabv::Point(),
			const rabv::Rotation& rotation_ = rabv::Rotation(),
			const bool visible_ = true)
		{
			bool found_cloud = false;
			rabv::Cloud& target_cloud = rabv::Cloud(name_);
			for (auto& cloud : clouds)
			{
				if (cloud.name == name_)
				{
					target_cloud = cloud;
					found_cloud = true;
					break;
				}
			}

			target_cloud.cloud->push_back(point);
			target_cloud.point_size = point_size_;
			target_cloud.color = color_;
			target_cloud.offset = offset_;
			target_cloud.rotation = rotation_;
			target_cloud.visible = visible_;

			if (!found_cloud)
			{
				clouds.push_back(target_cloud);
			}
		}

		void addCorrespondence(const rabv::Correspondence& corr)
		{
			int i = 0;
			bool unique_flag = false;
			std::string corr_name = corr.name;
			while (!unique_flag)
			{
				unique_flag = true;
				for (const auto c : correspondences)
				{
					if (c.name == corr_name)
					{
						unique_flag = false;
						break;
					}
				}
				if (unique_flag) break;
				corr_name = corr.name + "_" + boost::lexical_cast<std::string>(i);
				++i;
			}

			auto corr_ = corr;
			corr_.name = corr_name;
			correspondences.push_back(corr_);
		}
		template<typename... Args>
		void addCorrespondence(Args... args)
		{
			correspondences.push_back(rabv::Correspondence(std::forward<Args>(args)...));
		}

		void addLines(const rabv::Lines& lines)
		{
			lines_set.push_back(lines);
		}
		template<typename... Args>
		void addLines(Args... args)
		{
			lines_set.push_back(rabv::Lines(std::forward<Args>(args)...));
		}

		template<typename... Args>
		void addLine(const std::string& name, Args... args)
		{
			for (auto& lines : lines_set)
			{
				if (lines.name == name)
				{
					lines.addLine(std::forward<Args>(args)...);
					return;
				}
			}
			rabv::Lines lines(name);
			lines.addLine(std::forward<Args>(args)...);
			lines_set.push_back(lines);
		}

		void rabv::Rab::addCube(
			const std::string& name,
			const rabv::Point& p1,
			const rabv::Point& p2,
			const rabv::Color &color = rabv::Color())
		{
			rabv::Lines lines(name, color);

			lines.addLine(rabv::Point(p1.x, p1.y, p1.z), rabv::Point(p2.x, p1.y, p1.z));
			lines.addLine(rabv::Point(p1.x, p1.y, p1.z), rabv::Point(p1.x, p2.y, p1.z));
			lines.addLine(rabv::Point(p1.x, p1.y, p1.z), rabv::Point(p1.x, p1.y, p2.z));
			lines.addLine(rabv::Point(p1.x, p2.y, p2.z), rabv::Point(p1.x, p1.y, p2.z));
			lines.addLine(rabv::Point(p1.x, p2.y, p2.z), rabv::Point(p1.x, p2.y, p1.z));
			lines.addLine(rabv::Point(p1.x, p2.y, p2.z), rabv::Point(p2.x, p2.y, p2.z));
			lines.addLine(rabv::Point(p2.x, p1.y, p2.z), rabv::Point(p2.x, p1.y, p1.z));
			lines.addLine(rabv::Point(p2.x, p1.y, p2.z), rabv::Point(p1.x, p1.y, p2.z));
			lines.addLine(rabv::Point(p2.x, p1.y, p2.z), rabv::Point(p2.x, p2.y, p2.z));
			lines.addLine(rabv::Point(p2.x, p2.y, p1.z), rabv::Point(p1.x, p2.y, p1.z));
			lines.addLine(rabv::Point(p2.x, p2.y, p1.z), rabv::Point(p2.x, p1.y, p1.z));
			lines.addLine(rabv::Point(p2.x, p2.y, p1.z), rabv::Point(p2.x, p2.y, p2.z));

			lines_set.push_back(lines);
		}

		void addText(const rabv::Text& text)
		{
			texts.push_back(text);
		}
		template<typename... Args>
		void addText(Args... args)
		{
			texts.push_back(rabv::Text(std::forward<Args>(args)...));
		}

		void addText3D(const rabv::Text3D& text)
		{
			text3Ds.push_back(text);
		}
		template<typename... Args>
		void addText3D(Args... args)
		{
			text3Ds.push_back(rabv::Text3D(std::forward<Args>(args)...));
		}

		void addFlatText3D(const rabv::FlatText3D& flat_text3D)
		{
			flat_text3Ds.push_back(flat_text3D);
		}
		template<typename... Args>
		void addFlatText3D(Args... args)
		{
			flat_text3Ds.push_back(rabv::FlatText3D(std::forward<Args>(args)...));
		}

		void addCoordinateSystem(const rabv::CoordinateSystem& coordinate_system)
		{
			coordinate_systems.push_back(coordinate_system);
		}
		template<typename... Args>
		void addCoordinateSystem(Args... args)
		{
			coordinate_systems.push_back(rabv::CoordinateSystem(std::forward<Args>(args)...));
		}

		void setCamera(const pcl::visualization::Camera& cam)
		{
			camera = cam;
		}

		boost::optional<pcl::visualization::Camera> getCamera() const
		{
			return camera;
		}

		boost::shared_ptr<rabv::Viewer> visualize(
			const std::string& viewer_name_,
			const boost::filesystem::path& camfile_path_);
	};

	class Viewer : public Creatable<Viewer>
	{
	public:
		typedef boost::shared_ptr<rabv::Viewer> Ptr;
		typedef boost::shared_ptr<const rabv::Viewer> ConstPtr;

		Viewer(
			const std::string& viewer_name_,
			const boost::filesystem::path& camfile_path_,
			const rabv::Rab::Ptr rab_ = rabv::Rab::create(),
			const bool visualize_ = true)
			: viewer_name(viewer_name_),
			  camfile_path(camfile_path_),
			  pcl_viewer(new pcl::visualization::PCLVisualizer(viewer_name_)),
			  rab(rab_)
		{
			initPclViewer(visualize_);
		}

		Viewer(
			const std::string& viewer_name_ = "Viewer",
			const rabv::Rab::Ptr rab_ = rabv::Rab::create(),
			const bool visualize_ = true)
			: viewer_name(viewer_name_), 
			camfile_path(boost::filesystem::current_path() / (viewer_name_ + "_cam.txt")),
			pcl_viewer(new pcl::visualization::PCLVisualizer(viewer_name_)), rab(rab_)
		{
			initPclViewer(visualize_);
		}

		void visualize();

		void spinLoop(const int time = 10) const
		{
			while (!pcl_viewer->wasStopped())
			{
				pcl_viewer->spinOnce(time);
			}
		}

		void spin(const int time = 10) const
		{
			pcl_viewer->spinOnce(time);
		}

		bool wasStopped() const
		{
			return pcl_viewer->wasStopped();
		}

		void setRab(const rabv::Rab::Ptr& rab_)
		{
			rab = rab_;
		}

		rabv::Rab::Ptr getRab() const
		{
			return rab;
		}

		void saveCameraParams() const
		{
			pcl::visualization::Camera cam;
			pcl_viewer->getCameraParameters(cam);

			if (!rab->camera)
			{
				rab->camera = cam;
			}

			const auto parent_path = boost::filesystem::path(camfile_path).parent_path();
			if (!boost::filesystem::exists(parent_path))
			{
				boost::filesystem::create_directories(parent_path);
			}

			try
			{
				boost::property_tree::xml_parser::write_xml(
					camfile_path.string(),
					rabv::CameraToPree(*(rab->camera)),
					std::locale(),
					boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1)
				);
			}
			catch (const boost::property_tree::xml_parser::xml_parser_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Can't write camera settings."
					<< std::endl << e.what() << std::endl;
			}
		}

		void loadCameraParams()
		{
			boost::property_tree::ptree cam_ptree;
			try
			{
				boost::property_tree::read_xml(camfile_path.string(), cam_ptree);
				rab->camera = rabv::PreeToCamera(cam_ptree);
				const auto& cam = *(rab->camera);
				pcl_viewer->setCameraParameters(cam);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: camera settings file load error."
					<< std::endl << e.what() << std::endl;
			}

			vtkSmartPointer<vtkRenderer> renderer =
				pcl_viewer->getRendererCollection()->GetFirstRenderer();
			renderer->Render();
		}

		static void keyboardEvent(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
		{
			const auto viewer = static_cast<rabv::Viewer *>(viewer_void);
			if (event.getKeySym() == "s" && event.keyDown())
			{
				viewer->saveCameraParams();
				std::cout << "[Rab Visualizer] Camera Parameter is saved." << std::endl;
			}
			else if (event.getKeySym() == "l" && event.keyDown())
			{
				viewer->loadCameraParams();
				std::cout << "[Rab Visualizer] Camera Parameter is loaded." << std::endl;
			}
		}

		pcl::visualization::PCLVisualizer::Ptr getPCLVisualizer() const
		{
			return pcl_viewer;
		}

	private:
		const std::string viewer_name;
		const boost::filesystem::path camfile_path;
		const pcl::visualization::PCLVisualizer::Ptr pcl_viewer;
		rabv::Rab::Ptr rab;

		void initPclViewer(bool visualize_ = true)
		{
			vtkSmartPointer<vtkCamera> cam =
				pcl_viewer->getRendererCollection()->GetFirstRenderer()->GetActiveCamera();
			cam->SetParallelProjection(1);
			pcl_viewer->registerKeyboardCallback(rabv::Viewer::keyboardEvent, (void *)this);
			if (visualize_)
			{
				visualize();
			}
		}
	};

	class Reader : public Creatable<Reader>
	{
	public:
		typedef boost::shared_ptr<rabv::Reader> Ptr;
		typedef boost::shared_ptr<const rabv::Reader> ConstPtr;

		Reader(const std::string& rabfile_path_, const std::string& camfile_path_)
			: rabfile_path(rabfile_path_), camfile_path(camfile_path_)
		{
			load();
		}
		Reader(const std::string& rabfile_path_)
			: rabfile_path(rabfile_path_), camfile_path(getCamfilePath(rabfile_path_))
		{
			load();
		}
		Reader() {};

		void setPath(const std::string& rabfile_path_, const std::string& camfile_path_)
		{
			rabfile_path = rabfile_path_;
			camfile_path = camfile_path_;
		}

		void setPath(const std::string& rabfile_path_)
		{
			rabfile_path = rabfile_path_;
			camfile_path = getCamfilePath(rabfile_path);
		}

		void load();

		void setRab(const rabv::Rab::Ptr& rab_)
		{
			rab = rab_;
		}

		rabv::Rab::Ptr getRab() const
		{
			return rab;
		}

		template<typename ...Args>
		static rabv::Rab::Ptr loadRabFile(Args... args)
		{
			rabv::Reader reader(std::forward<Args...>(args...));
			return reader.getRab();
		}

		static boost::filesystem::path getCamfilePath(const boost::filesystem::path& rabfile_path)
		{
			return (rabfile_path.parent_path() / (rabfile_path.stem().string() + "_cam.txt"));
		}

		rabv::Viewer::Ptr visualize(const std::string& title = "Viewer") const
		{
			return rabv::Viewer::create(title, getRab());
		}

	private:
		boost::filesystem::path rabfile_path;
		boost::filesystem::path camfile_path;
		rabv::Rab::Ptr rab;
	};

	class Writer : public Creatable<Writer>
	{
	public:
		typedef boost::shared_ptr<rabv::Writer> Ptr;
		typedef boost::shared_ptr<const rabv::Writer> ConstPtr;

		Writer(const std::string& rab_path, rabv::Rab::Ptr& rab = rabv::Rab::create())
			: rab(rab)
		{
			setPath(rab_path);
		}

		void setPath(const std::string& rab_path_)
		{
			const auto rab_path = boost::filesystem::absolute(rab_path_);
			rab_name = rab_path.stem().string();
			path = rab_path.parent_path();
		}

		void save() const;

		void setRab(const rabv::Rab::Ptr& rab_)
		{
			rab = rab_;
		}

		rabv::Rab::Ptr getRab() const
		{
			return rab;
		}

		rabv::Viewer::Ptr visualize() const
		{
			return rabv::Viewer::create(rab_name, getRab());
		}

		static void saveRabFile(const std::string& rab_path_, rabv::Rab::Ptr& rab_)
		{
			rabv::Writer writer(rab_path_, rab_);
			writer.save();
		}

	private:
		std::string rab_name;
		boost::filesystem::path path;
		rabv::Rab::Ptr rab;

		static void create_dir(const boost::filesystem::path& path_)
		{
			if (!boost::filesystem::exists(path_))
			{
				boost::filesystem::create_directories(path_);
			}
		}
	};
}

boost::shared_ptr<rabv::Viewer> rabv::Rab::visualize(
	const std::string& viewer_name_,
	const boost::filesystem::path& camfile_path_)
{
	return rabv::Viewer::create(viewer_name_, camfile_path_, this->makeShared());
}

void rabv::Viewer::visualize()
{
	if (pcl_viewer->wasStopped()) return;

	// remove all drawing objects
	pcl_viewer->removeAllPointClouds();
	pcl_viewer->removeAllShapes();
	//pcl_viewer->removeAllCoordinateSystems(); in this version's PCL, this function doesn't work.
	pcl_viewer->removeCoordinateSystem("world");

	// prepare coors and add world coor
	std::vector<std::pair<std::string, double>> coors;
	for (const auto& coor : rab->coordinate_systems)
	{
		if (!coor.visible) continue;

		if (coor.name == "world")
		{
			pcl_viewer->addCoordinateSystem(coor.scale);
		}
		coors.push_back(std::make_pair(coor.name, coor.scale));
	}

	// add point cloud and coor
	std::vector<std::string> used_name;
	std::map<std::string, pcl::PointCloud<pcl::PointXYZ>::Ptr> trans_clouds;
	for (const auto& cloud : rab->clouds)
	{
		if (!cloud.visible)
		{
			continue;
		}

		int i = 0;
		std::string cloud_name = cloud.name;
		while (std::find(used_name.begin(), used_name.end(), cloud_name) != used_name.end())
		{
			cloud_name = cloud.name + "_" + std::to_string(i);
			++i;
		}
		used_name.push_back(cloud_name);

		// this is bad workaround
		pcl_viewer->removeCoordinateSystem(cloud_name);

		// load transform
		Eigen::Vector3f offset;
		offset[0] = cloud.offset.x;
		offset[1] = cloud.offset.y;
		offset[2] = cloud.offset.z;
		Eigen::Affine3f transform = cloud.rotation.affine();
		transform.translate(offset);

		pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*cloud.cloud, *trans_cloud, transform);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(
			trans_cloud, cloud.color.r, cloud.color.g, cloud.color.b);

		pcl_viewer->addPointCloud(trans_cloud, color_handler, cloud_name);
		pcl_viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.point_size, cloud_name);
		trans_clouds.insert(std::make_pair(cloud_name, trans_cloud));

		for (const auto& coor : coors)
		{
			if (coor.first == cloud.name)
			{
				pcl_viewer->removeCoordinateSystem(coor.first);
				pcl_viewer->addCoordinateSystem(coor.second, transform, cloud.name + "_coor");
			}
		}
	}

	for (const auto& normal : rab->normals)
	{
		if (!normal.visible)
		{
			continue;
		}

		int i = 0;
		std::string normal_name = normal.name + "_normal";
		while (std::find(used_name.begin(), used_name.end(), normal_name) != used_name.end())
		{
			normal_name = normal.name + "_" + std::to_string(i);
			++i;
		}
		used_name.push_back(normal_name);

		if (trans_clouds.find(normal.name) == trans_clouds.end())
		{
			std::cerr
				<< "[Rab Visualizer] Error: Normal cannot find base pointcloud named "
				<< normal.name << "." << std::endl;
			continue;
		}

		pcl_viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
			trans_clouds[normal.name],
			normal.normal,
			normal.level,
			normal.scale,
			normal_name);
		pcl_viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			static_cast<double>(normal.color.r) / 255.0,
			static_cast<double>(normal.color.g) / 255.0,
			static_cast<double>(normal.color.b) / 255.0,
			normal_name);
		pcl_viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
			normal.line_width,
			normal_name);
	}

	used_name.clear();
	for (const auto& corr : rab->correspondences)
	{
		if (!corr.visible) continue;

		// if cloud_from and cloud_to is found
		if (trans_clouds.find(corr.from) != trans_clouds.end() ||
			trans_clouds.find(corr.to) != trans_clouds.end())
		{
			int i = 0;
			std::string corr_name = corr.name;
			while (std::find(used_name.begin(), used_name.end(), corr_name) != used_name.end())
			{
				corr_name = corr.name + "_" + std::to_string(i);
				++i;
			}
			used_name.push_back(corr_name);

			const auto& cloud_from = trans_clouds[corr.from];
			const auto& cloud_to = trans_clouds[corr.to];
			for (int i = 0; i < corr.pairs.size(); ++i)
			{
				std::stringstream ss_line;
				ss_line << corr_name << "__" << i;

				const auto& point_from = (*cloud_from)[corr.pairs[i].first];
				const auto& point_to = (*cloud_to)[corr.pairs[i].second];

				pcl_viewer->addLine(
					point_from,
					point_to,
					static_cast<double>(corr.color.r) / 255,
					static_cast<double>(corr.color.g) / 255,
					static_cast<double>(corr.color.b) / 255,
					ss_line.str());
			}
		}
	}

	used_name.clear();
	for (const auto& lines : rab->lines_set)
	{
		if (!lines.visible)
		{
			continue;
		}

		int i = 0;
		std::string lines_name = lines.name;
		while (std::find(used_name.begin(), used_name.end(), lines_name) != used_name.end())
		{
			lines_name = lines.name + "_" + std::to_string(i);
			++i;
		}
		used_name.push_back(lines_name);

		for (int i = 0; i < lines.lines.size(); ++i)
		{
			std::stringstream ss_line;
			ss_line << lines_name << "_" << i;

			pcl_viewer->addLine(
				rabv::Point(lines.lines[i].from.x, lines.lines[i].from.y, lines.lines[i].from.z),
				rabv::Point(lines.lines[i].to.x, lines.lines[i].to.y, lines.lines[i].to.z),
				static_cast<double>(lines.color.r) / 255,
				static_cast<double>(lines.color.g) / 255,
				static_cast<double>(lines.color.b) / 255,
				ss_line.str());
		}
	}

	for (int i = 0; i < rab->texts.size(); ++i)
	{
		const rabv::Text& text = rab->texts[i];
		if (!text.visible)
		{
			continue;
		}

		pcl_viewer->addText(
			text.text,
			text.x,
			text.y,
			text.font_size,
			static_cast<double>(text.color.r) / 255,
			static_cast<double>(text.color.g) / 255,
			static_cast<double>(text.color.b) / 255,
			std::string("text_") + boost::lexical_cast<std::string>(i)
		);
	}

	// text3D
	for (int i = 0; i < rab->text3Ds.size(); ++i)
	{
		const rabv::Text3D& text3D = rab->text3Ds[i];
		if (!text3D.visible)
		{
			continue;
		}

		pcl_viewer->addText3D(
			text3D.text,
			rabv::Point(text3D.x, text3D.y, text3D.z),
			text3D.font_size,
			static_cast<double>(text3D.color.r) / 255,
			static_cast<double>(text3D.color.g) / 255,
			static_cast<double>(text3D.color.b) / 255,
			std::string("text3D_") + boost::lexical_cast<std::string>(i)
		);
	}

	if (rab->flat_text3Ds.size() > 0)
	{
		const vtkSmartPointer<vtkAppendPolyData> append_filter =
			vtkSmartPointer<vtkAppendPolyData>::New();
		for (int j = 0; j < rab->flat_text3Ds.size(); ++j)
		{
			const FlatText3D& flat_text3D = rab->flat_text3Ds[j];
			if (!flat_text3D.visible)
			{
				continue;
			}

			const vtkSmartPointer<vtkVectorText> vec_text =
				vtkSmartPointer<vtkVectorText>::New();
			vec_text->SetText(flat_text3D.text.c_str());
			vec_text->Update();

			vtkPolyData* const polydata = vec_text->GetOutput();

			for (int j = 0; j < polydata->GetNumberOfPoints(); ++j)
			{
				const double* point = polydata->GetPoint(j);
				const Eigen::Matrix3f rmat = flat_text3D.rotation.matrix();
				const Eigen::Vector3f vec(point[0], point[1], point[2]);
				const Eigen::Vector3f t = rmat * vec;

				polydata->GetPoints()->SetPoint(
					j,
					t[0] * flat_text3D.font_size + flat_text3D.x,
					t[1] * flat_text3D.font_size + flat_text3D.y,
					t[2] * flat_text3D.font_size + flat_text3D.z);
			}

			//Set colors
			const unsigned char color[3] = {
				static_cast<unsigned char>(flat_text3D.color.r),
				static_cast<unsigned char>(flat_text3D.color.g),
				static_cast<unsigned char>(flat_text3D.color.b)
			};

			const vtkSmartPointer<vtkUnsignedCharArray> colors =
				vtkSmartPointer<vtkUnsignedCharArray>::New();
			colors->SetNumberOfComponents(3);

			for (int j = 0; j < polydata->GetNumberOfPoints(); ++j)
			{
				colors->InsertNextTypedTuple(color);
			}

			polydata->GetCellData()->SetScalars(colors);

			append_filter->AddInputData(polydata);
		}
		append_filter->Update();

		//Create a mapper and actor
		const vtkSmartPointer<vtkPolyDataMapper> mapper =
			vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(append_filter->GetOutputPort());
		pcl_viewer->addModelFromPolyData(mapper->GetInput(), "3DTextSet");
	}

	vtkSmartPointer<vtkRenderer> renderer =
		pcl_viewer->getRendererCollection()->GetFirstRenderer();
	renderer->ResetCamera();
	if (rab->camera)
	{
		pcl_viewer->setCameraParameters(*(rab->camera));
	}
	renderer->Render();

	pcl_viewer->spinOnce();
};

void rabv::Reader::load()
{
	const boost::filesystem::path data_dir_path =
		rabfile_path.parent_path() / rabfile_path.stem();
	boost::property_tree::ptree ptree_load;
	boost::property_tree::ptree ptree;
	try
	{
		boost::property_tree::read_xml(rabfile_path.string(), ptree_load);
		ptree = ptree_load.get_child("rab");
	}
	catch (boost::property_tree::ptree_error& e)
	{
		std::cerr
			<< "[Rab Visualizer] Error: rab file load error."
			<< std::endl << e.what() << std::endl;
	}

	try
	{
		if (ptree.get<int>("version", 1) != 3)
		{
			std::cerr
				<< "[Rab Visualizer] Error: rab file version error." << std::endl;
		}
	}
	catch (boost::property_tree::ptree_error& e)
	{
		std::cerr
			<< "[Rab Visualizer] Error: rab file load error."
			<< std::endl << e.what() << std::endl;
	}

	rab = Rab::create();
	
	if (ptree.get_child_optional("cloudXYZ"))
	{
		for (const auto& ptree_itr : ptree.get_child("cloudXYZ", boost::property_tree::ptree()))
		{
			const auto& ptree_ = ptree_itr.second;
			try
			{
				rabv::Cloud cloud(ptree_.get<std::string>("name"));
				const auto pcd_filename = ptree_.get<std::string>("name") + ".pcd";
				const auto pcd_path = boost::filesystem::absolute(pcd_filename, data_dir_path);

				if (pcl::io::loadPCDFile(pcd_path.string(), *(cloud.cloud)) == -1)
				{
					std::cerr
						<< "[Rab Visualizer] Error: pcd file load error."
						<< std::endl << " File: " << pcd_filename << std::endl;
					continue;
				}

				cloud.offset.x = ptree_.get<float>("offset.x", 0.0f);
				cloud.offset.y = ptree_.get<float>("offset.y", 0.0f);
				cloud.offset.z = ptree_.get<float>("offset.z", 0.0f);

				cloud.rotation.x = ptree_.get<float>("rotation.x", 1.0f);
				cloud.rotation.y = ptree_.get<float>("rotation.y", 0.0f);
				cloud.rotation.z = ptree_.get<float>("rotation.z", 0.0f);

				cloud.color = rabv::Color(
					ptree_.get<int>("color.r", 255),
					ptree_.get<int>("color.g", 255),
					ptree_.get<int>("color.b", 255)
				);

				cloud.point_size = ptree_.get<int>("point_size", 1);

				cloud.visible = ptree_.get<bool>("visible", true);

				rab->clouds.push_back(cloud);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error."
					<< std::endl << e.what() << std::endl;
				continue;
			}
		}
	}

	// load normal
	if (ptree.get_child_optional("normal"))
	{
		for (const auto& ptree_itr : ptree.get_child("normal", boost::property_tree::ptree()))
		{
			const auto& ptree_ = ptree_itr.second;
			try
			{
				rabv::Normal normal(ptree_.get<std::string>("name"));
				const auto pcd_filename = ptree_.get<std::string>("name") + "_normal.pcd";
				const auto pcd_path = boost::filesystem::absolute(pcd_filename, data_dir_path);

				if (pcl::io::loadPCDFile(pcd_path.string(), *(normal.normal)) == -1)
				{
					std::cerr
						<< "[Rab Visualizer] Error: pcd file load error."
						<< std::endl << " File: " << pcd_filename << std::endl;
					continue;
				}

				normal.color = rabv::Color(
					ptree_.get<int>("color.r", 255),
					ptree_.get<int>("color.g", 255),
					ptree_.get<int>("color.b", 255));

				normal.level = ptree_.get<int>("level", 100);
				normal.scale = ptree_.get<double>("scale", 0.02);
				normal.line_width = ptree_.get<int>("line_width", 1);
				normal.visible = ptree_.get<bool>("visible", true);

				rab->normals.push_back(normal);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error."
					<< std::endl << e.what() << std::endl;
				continue;
			}
		}
	}

	if (ptree.get_child_optional("correspondences"))
	{
		for (const auto& ptree_itr : ptree.get_child("correspondences", boost::property_tree::ptree()))
		{
			const auto&  ptree_ = ptree_itr.second;
			try
			{
				const auto corr_name = ptree_.get<std::string>("name");
				const auto corr_filename = corr_name + "_corr.txt";
				rabv::Color corr_color = rabv::Color(
					ptree_.get<int>("color.r", 255),
					ptree_.get<int>("color.g", 255),
					ptree_.get<int>("color.b", 255)
				);

				rabv::Correspondence corr(
					corr_name,
					ptree_.get<std::string>("from"),
					ptree_.get<std::string>("to"),
					corr_color,
					ptree_.get<bool>("visible", true)
				);

				const auto corr_path = boost::filesystem::absolute(corr_filename, data_dir_path);

				ifstream ifs(corr_path.string());
				std::string str;
				while (std::getline(ifs, str))
				{
					std::vector<std::string> indices(2);
					boost::algorithm::split(indices, str, boost::algorithm::is_space());
					const Correspondence::IndexPair pair = std::make_pair(
						boost::lexical_cast<int>(indices[0]),
						boost::lexical_cast<int>(indices[1])
					);
					corr.pairs.push_back(pair);
				}

				rab->correspondences.push_back(corr);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error." << std::endl
					<< e.what() << std::endl;
				continue;
			}
		}
	}

	if (ptree.get_child_optional("linesSet"))
	{
		for (const auto& ptree_itr : ptree.get_child("linesSet", boost::property_tree::ptree()))
		{
			const auto&  ptree_ = ptree_itr.second;
			try
			{
				const auto lines_name = ptree_.get<std::string>("name");
				rabv::Lines lines(lines_name);

				lines.color = rabv::Color(
					ptree_.get<int>("color.r", 255),
					ptree_.get<int>("color.g", 255),
					ptree_.get<int>("color.b", 255)
				);

				lines.visible = ptree_.get<bool>("visible", true);

				const auto lines_path =
					boost::filesystem::absolute(lines_name + "_line.txt", data_dir_path);

				ifstream ifs(lines_path.string());
				std::string str;
				while (std::getline(ifs, str))
				{
					std::vector<std::string> indices(6);
					boost::algorithm::split(indices, str, boost::algorithm::is_space());
					lines.addLine(
						rabv::Point(
							boost::lexical_cast<float>(indices[0]),
							boost::lexical_cast<float>(indices[1]),
							boost::lexical_cast<float>(indices[2])),
						rabv::Point(
							boost::lexical_cast<float>(indices[3]),
							boost::lexical_cast<float>(indices[4]),
							boost::lexical_cast<float>(indices[5]))
					);
				}

				rab->lines_set.push_back(lines);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error." << std::endl
					<< e.what() << std::endl;
				continue;
			}
		}
	}

	if (ptree.get_child_optional("texts"))
	{
		for (const auto& ptree_itr : ptree.get_child("texts", boost::property_tree::ptree()))
		{
			const auto&  ptree_ = ptree_itr.second;
			try
			{
				rab->texts.push_back(rabv::Text(
					ptree_.get<std::string>("text", ""),
					ptree_.get<int>("x", 0),
					ptree_.get<int>("y", 0),
					ptree_.get<int>("font_size", 30),
					rabv::Color(
						ptree_.get<int>("color.r", 0),
						ptree_.get<int>("color.g", 0),
						ptree_.get<int>("color.b", 0)
					),
					ptree_.get<bool>("visible", true)
				));
			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error."
					<< std::endl << e.what() << std::endl;
				continue;
			}
		}
	}

	if (ptree.get_child_optional("text3Ds"))
	{
		for (const auto& ptree_itr : ptree.get_child("text3Ds", boost::property_tree::ptree()))
		{
			const auto&  ptree_ = ptree_itr.second;
			try
			{
				rab->text3Ds.push_back(rabv::Text3D(
					ptree_.get<std::string>("text", ""),
					rabv::Point(
						ptree_.get<double>("x", 0),
						ptree_.get<double>("y", 0),
						ptree_.get<double>("z", 0)
					),
					ptree_.get<double>("font_size", 0.1),
					rabv::Color(
						ptree_.get<int>("color.r", 0),
						ptree_.get<int>("color.g", 0),
						ptree_.get<int>("color.b", 0)
					),
					ptree_.get<bool>("visible", true))
				);

			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error."
					<< std::endl << e.what() << std::endl;
				continue;
			}
		}
	}

	if (ptree.get_child_optional("flatText3Ds"))
	{
		for (const auto& ptree_itr : ptree.get_child("flatText3Ds", boost::property_tree::ptree()))
		{
			const auto&  ptree_ = ptree_itr.second;
			try
			{
				rab->flat_text3Ds.push_back(FlatText3D(
					ptree_.get<std::string>("text", ""),
					rabv::Point(
						ptree_.get<double>("x", 0),
						ptree_.get<double>("y", 0),
						ptree_.get<double>("z", 0)
					),
					ptree_.get<double>("font_size", 0.1),
					Color(
						ptree_.get<int>("color.r", 0),
						ptree_.get<int>("color.g", 0),
						ptree_.get<int>("color.b", 0)
					),
					Rotation(
						ptree_.get<int>("rotation.x", 0),
						ptree_.get<int>("rotation.y", 0),
						ptree_.get<int>("rotation.z", 0)
					),
					ptree_.get<bool>("visible", true)
				));

			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error."
					<< std::endl << e.what() << std::endl;
				continue;
			}
		}
	}

	if (ptree.get_child_optional("coordinates"))
	{
		for (const auto& ptree_itr : ptree.get_child("coordinates", boost::property_tree::ptree()))
		{
			const auto&  ptree_ = ptree_itr.second;
			try
			{
				rab->coordinate_systems.push_back(rabv::CoordinateSystem(
					ptree_.get<double>("scale", 1),
					ptree_.get<std::string>("name", "world"),
					ptree_.get<bool>("visible", true)
				));
			}
			catch (boost::property_tree::ptree_error& e)
			{
				std::cerr
					<< "[Rab Visualizer] Error: rab file format error."
					<< std::endl << e.what() << std::endl;
				continue;
			}
		}
	}

	if (boost::filesystem::exists(camfile_path))
	{
		ptree.clear(); // to use ptree for camera
		try
		{
			boost::property_tree::read_xml(camfile_path.string(), ptree);
			rab->camera = rabv::PreeToCamera(ptree);
		}
		catch (boost::property_tree::ptree_error& e)
		{
			std::cerr
				<< "[Rab Visualizer] Error: camera file load error."
				<< std::endl << e.what() << std::endl;
		}
	}
	else
	{
		rab->camera = boost::none;
	}
}

void rabv::Writer::save() const
{
	create_dir(path / rab_name);

	boost::property_tree::ptree ptree;

	ptree.add("version", 3);

	for (const auto& cloud : rab->clouds)
	{
		const std::string filename = cloud.name + ".pcd";
		boost::filesystem::path data_path = path / rab_name / filename;
		if (cloud.cloud->size() > 0)
		{
			pcl::io::savePCDFileBinaryCompressed(data_path.string(), *(cloud.cloud));
		}

		boost::property_tree::ptree ptree_cloud;
		ptree_cloud.add("name", cloud.name);
		ptree_cloud.add("offset.x", cloud.offset.x);
		ptree_cloud.add("offset.y", cloud.offset.y);
		ptree_cloud.add("offset.z", cloud.offset.z);
		ptree_cloud.add("point_size", cloud.point_size);
		ptree_cloud.add("color.r", cloud.color.r);
		ptree_cloud.add("color.g", cloud.color.g);
		ptree_cloud.add("color.b", cloud.color.b);
		ptree_cloud.add("rotation.x", cloud.rotation.x);
		ptree_cloud.add("rotation.y", cloud.rotation.y);
		ptree_cloud.add("rotation.z", cloud.rotation.z);
		ptree_cloud.add("visible", cloud.visible);
		ptree.add_child("cloudXYZ.cloud", ptree_cloud);
	}

	for (const auto& normal : rab->normals)
	{
		const std::string filename = normal.name + "_normal.pcd";
		boost::filesystem::path data_path = path / rab_name / filename;
		if (normal.normal->size() > 0)
		{
			pcl::io::savePCDFileBinaryCompressed(data_path.string(), *(normal.normal));
		}

		boost::property_tree::ptree ptree_cloud;
		ptree_cloud.add("name", normal.name);
		ptree_cloud.add("color.r", normal.color.r);
		ptree_cloud.add("color.g", normal.color.g);
		ptree_cloud.add("color.b", normal.color.b);
		ptree_cloud.add("level", normal.level);
		ptree_cloud.add("scale", normal.scale);
		ptree_cloud.add("line_width", normal.line_width);
		ptree_cloud.add("visible", normal.visible);
		ptree.add_child("normal.normal", ptree_cloud);
	}

	for (const auto& corr : rab->correspondences)
	{
		const std::string filename = corr.name + "_corr.txt";
		boost::filesystem::path data_path = path / rab_name / filename;
		std::ofstream ofs(data_path.string());
		for (const auto& line : corr.pairs)
		{
			ofs << line.first << " " << line.second << std::endl;
		}

		boost::property_tree::ptree ptree_corr;
		ptree_corr.add("name", corr.name);
		ptree_corr.add("from", corr.from);
		ptree_corr.add("to", corr.to);
		ptree_corr.add("color.r", corr.color.r);
		ptree_corr.add("color.g", corr.color.g);
		ptree_corr.add("color.b", corr.color.b);
		ptree_corr.add("visible", corr.visible);
		ptree.add_child("correspondences.correspondence", ptree_corr);
	}

	for (const auto& lines : rab->lines_set)
	{
		const std::string filename = lines.name + "_line.txt";
		boost::filesystem::path data_path = path / rab_name / filename;
		std::ofstream ofs(data_path.string());
		for (const auto& line : lines.lines)
		{
			ofs << line.from.x << " " << line.from.y << " " << line.from.z << " "
				<< line.to.x << " " << line.to.y << " " << line.to.z << " " << std::endl;
		}

		boost::property_tree::ptree ptree_lines;
		ptree_lines.add("name", lines.name);
		ptree_lines.add("color.r", lines.color.r);
		ptree_lines.add("color.g", lines.color.g);
		ptree_lines.add("color.b", lines.color.b);
		ptree_lines.add("visible", lines.visible);
		ptree.add_child("linesSet.lines", ptree_lines);
	}

	for (const auto& text : rab->texts)
	{
		boost::property_tree::ptree ptree_text;
		ptree_text.add("text", text.text);
		ptree_text.add("x", text.x);
		ptree_text.add("y", text.y);
		ptree_text.add("font_size", text.font_size);
		ptree_text.add("color.r", text.color.r);
		ptree_text.add("color.g", text.color.g);
		ptree_text.add("color.b", text.color.b);
		ptree_text.add("visible", text.visible);
		ptree.add_child("texts.text", ptree_text);
	}

	for (const auto text3D : rab->text3Ds)
	{
		boost::property_tree::ptree ptree_text;
		ptree_text.add("text", text3D.text);
		ptree_text.add("x", text3D.x);
		ptree_text.add("y", text3D.y);
		ptree_text.add("z", text3D.z);
		ptree_text.add("font_size", text3D.font_size);
		ptree_text.add("color.r", text3D.color.r);
		ptree_text.add("color.g", text3D.color.g);
		ptree_text.add("color.b", text3D.color.b);
		ptree_text.add("visible", text3D.visible);
		ptree.add_child("text3Ds.text3D", ptree_text);
	}

	for (const auto& flat_text3D : rab->flat_text3Ds)
	{
		boost::property_tree::ptree ptree_text;
		ptree_text.add("text", flat_text3D.text);
		ptree_text.add("x", flat_text3D.x);
		ptree_text.add("y", flat_text3D.y);
		ptree_text.add("z", flat_text3D.z);
		ptree_text.add("font_size", flat_text3D.font_size);
		ptree_text.add("color.r", flat_text3D.color.r);
		ptree_text.add("color.g", flat_text3D.color.g);
		ptree_text.add("color.b", flat_text3D.color.b);
		ptree_text.add("rotation.x", flat_text3D.rotation.x);
		ptree_text.add("rotation.y", flat_text3D.rotation.y);
		ptree_text.add("rotation.z", flat_text3D.rotation.z);
		ptree_text.add("visible", flat_text3D.visible);
		ptree.add_child("flatText3Ds.flatText3D", ptree_text);
	}

	for (const auto& coordinate_system : rab->coordinate_systems)
	{
		boost::property_tree::ptree ptree_text;
		ptree_text.add("name", coordinate_system.name);
		ptree_text.add("scale", coordinate_system.scale);
		ptree_text.add("visible", coordinate_system.visible);
		ptree.add_child("coordinates.coordinate", ptree_text);
	}

	const std::string data_path = (path / (rab_name + ".rab")).string();
	try
	{
		auto ptree_save = boost::property_tree::ptree();
		ptree_save.add_child("rab", ptree);

		boost::property_tree::xml_parser::write_xml(
			data_path, ptree_save, std::locale(),
			boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1));
	}
	catch (const boost::property_tree::xml_parser::xml_parser_error& e)
	{
		std::cerr
			<< "[Rab Visualizer] Can't write rab file." << std::endl
			<< e.what() << std::endl;
	}

	if (rab->camera)
	{
		try
		{
			boost::property_tree::xml_parser::write_xml(
				Reader::getCamfilePath(data_path).string(),
				rabv::CameraToPree(*(rab->camera)),
				std::locale(),
				boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1)
			);
		}
		catch (const boost::property_tree::xml_parser::xml_parser_error& e)
		{
			std::cerr << "[Rab Visualizer] Can't write camera settings." << std::endl
				<< e.what() << std::endl;
		}
	}
}
}
