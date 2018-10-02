// Rab Visualizer Ver.3.00-rc1
// 2017/06/14
// Copyright (C) 2015-2017 Chiba Naoya

#pragma once

#pragma warning(disable: 4819)

#include <iostream>
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

	template<typename T>
	class UniqueMap : public std::map<std::string, T>
	{
	public:
		void unique_insert(const std::string& name_, const T& element, const bool quiet = false)
		{
			int i = 0;
			std::string name = name_;
			while (true)
			{
				if (this->find(name) == this->end())
				{
					break;
				}
				else
				{
					name = name_ + "_" + std::to_string(i);
					++i;
				}
			}

			if (name != name_)
			{
				if (!quiet)
				{
					std::stringstream ss;
					ss	<< "[Rab Visualizer] "
						<< name << " already exists. New element is renamed as " << name << "." << std::endl;

					pcl::console::print_warn(ss.str().c_str());
				}
			}

			this->insert(std::make_pair(name, element));
		}

		void check_erase(const std::string& name, const bool quiet = false)
		{
			if (this->find(name) == this->end())
			{
				if (!quiet)
				{
					std::stringstream ss;
					ss	<< "[Rab Visualizer] Connot remove "
						<< name << ". It does not exist." << std::endl;

					pcl::console::print_warn(ss.str().c_str());
				}
			}
			else
			{
				this->erase(name);
			}
		}
	};

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
	typedef UniqueMap<rabv::Lines> LinesSet;
	typedef std::vector<rabv::Text> Texts;
	typedef std::vector<rabv::Text3D> Text3Ds;
	typedef std::vector<rabv::FlatText3D> FlatText3Ds;
	typedef UniqueMap<rabv::Cloud> Clouds;
	typedef UniqueMap<rabv::Normal> Normals;
	typedef UniqueMap<rabv::Correspondence> Correspondences;
	typedef UniqueMap<rabv::CoordinateSystem> CoordinateSystems;

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

		static std::vector<rabv::Color> divideColors(const int step)
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
		double x;
		double y;
		double z;

		Rotation(const double x_, const double y_, const double z_) : x(x_), y(y_), z(z_) {}
		Rotation() : x(0.0), y(0.0), z(0.0) {}

		template<typename T>
		Rotation(const Eigen::Matrix<T, 3, 3>& rotmat)
		{
			if (rotmat(2, 0) == -1.0)
			{
				x = std::atan2(rotmat(1, 1), rotmat(0, 1));
				y = rabv::PI<double>() / 2.0;
				z = 0.0;
			}

			if (rotmat(2, 0) == 1.0)
			{
				x = std::atan2(-rotmat(1, 1), rotmat(0, 1));
				y = -rabv::PI<double>() / 2.0;
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

	class Lines : public Creatable<rabv::Lines>
	{
	public:
		typedef boost::shared_ptr<rabv::Lines> Ptr;
		typedef boost::shared_ptr<const rabv::Lines> ConstPtr;
		double line_width;
		rabv::Color color;
		std::vector<rabv::Line> lines;
		bool visible;

		Lines(
			const double line_width_ = 1.0,
			const rabv::Color& color_ = rabv::Color(),
			const std::vector<rabv::Line>& lines_ = std::vector<rabv::Line>(),
			const bool visible_ = true)
			: line_width(line_width_), color(color_), lines(lines_), visible(visible_) {}
		Lines(const Lines::Ptr& lines_)
		: line_width(lines_->line_width), color(lines_->color), lines(lines_->lines), visible(lines_->visible) {}

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
		int x;
		int y;
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
		double x;
		double y;
		double z;
		double font_size;
		rabv::Color color;
		bool visible;

		Text3D(
			const std::string& text_,
			const rabv::Point& point = rabv::Point(),
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
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		int point_size;
		rabv::Color color;
		rabv::Point offset;
		rabv::Rotation rotation;
		bool visible;

		Cloud(
			const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_ =
				pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>()),
			const int point_size_ = 1,
			const rabv::Color& color_ = rabv::Color(),
			const rabv::Point& offset_ = rabv::Point(),
			const rabv::Rotation& rotation_ = rabv::Rotation(),
			const bool visible_ = true)
			: cloud(cloud_),
			  color(color_),
			  offset(offset_),
			  point_size(point_size_),
			  rotation(rotation_),
			  visible(visible_) {};

		// copy constructor
		Cloud(const rabv::Cloud& cloud_)
			: color(cloud_.color),
			  offset(cloud_.offset),
			  point_size(cloud_.point_size),
			  rotation(cloud_.rotation),
			  visible(cloud_.visible)
		{
			cloud = cloud_.cloud->makeShared();
		}

		// move constructor
		Cloud(rabv::Cloud&& cloud_)
			: color(cloud_.color),
			  offset(cloud_.offset),
			  point_size(cloud_.point_size),
			  rotation(cloud_.rotation),
			  visible(cloud_.visible)
		{
			cloud = std::move(cloud_.cloud);
		}

		// copy
		Cloud& operator=(const rabv::Cloud& cloud_)
		{
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
			offset = cloud_.offset;
			cloud = std::move(cloud_.cloud);
			color = cloud_.color;
			point_size = cloud_.point_size;
			rotation = cloud_.rotation;
			visible = cloud_.visible;
			return *this;
		}

		Eigen::Affine3f affine() const
		{
			const Eigen::Vector3f offset_vec(offset.x, offset.y, offset.z);
			Eigen::Affine3f transform = rotation.affine();
			transform.translate(offset_vec);

			return transform;
		}

		Eigen::Matrix4f transform() const
		{
			return affine().matrix();
		}
	};

	class Normal
	{
	public:
		pcl::PointCloud<pcl::Normal>::Ptr normal;
		int level;
		double scale;
		double line_width;
		rabv::Color color;
		bool visible;

		Normal(
			const pcl::PointCloud<pcl::Normal>::Ptr& normal_ =
				pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>()),
			const int level_ = 100,
			const double scale_ = 0.02,
			const double line_width_ = 1.0,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: normal(normal_),
			  level(level_),
			  scale(scale_),
			  line_width(line_width_),
			  color(color_),
			  visible(visible_) {};

		// copy constructor
		Normal(const rabv::Normal& normal_)
			: level(normal_.level),
			  scale(normal_.scale),
			  line_width(normal_.line_width),
			  color(normal_.color),
			  visible(normal_.visible)
		{
			normal = normal_.normal->makeShared();
		}

		// move constructor
		Normal(rabv::Normal&& normal_)
			: level(normal_.level),
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
			normal = std::move(normal_.normal);
			level = normal_.level;
			scale = normal_.scale;
			line_width = normal_.line_width;
			color = normal_.color;
			visible = normal_.visible;
			return *this;
		}
	};

	class Correspondence : public Creatable<rabv::Correspondence>
	{
	public:
		typedef boost::shared_ptr<rabv::Correspondence> Ptr;
		typedef boost::shared_ptr<const rabv::Correspondence> ConstPtr;
		typedef std::pair<int/* from */, int/* to */> IndexPair;
		std::string from;
		std::string to;
		std::vector<rabv::Correspondence::IndexPair> pairs;
		double line_width;
		rabv::Color color;
		bool visible;

		Correspondence(
			const std::string& from_,
			const std::string& to_,
			const double line_width_ = 1.0,
			const rabv::Color color_ = rabv::Color(),
			const bool visible_ = true)
			: from(from_), to(to_), line_width(line_width_), color(color_), visible(visible_) {}
		Correspondence(
			const std::string& from_,
			const std::string& to_,
			const std::vector<std::pair<int, int>>& pairs_,
			const double line_width_ = 1.0,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: from(from_), to(to_), line_width(line_width_), color(color_), visible(visible_)
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
			const double line_width_ = 1.0,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: from(from_), to(to_), line_width(line_width_), color(color_), visible(visible_)
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
			const double line_width_ = 1.0,
			const rabv::Color& color_ = rabv::Color(),
			const bool visible_ = true)
			: from(from_), to(to_), line_width(line_width_), color(color_), visible(visible_)
		{
			for (const auto& corr : *correspondences_)
			{
				pairs.push_back(rabv::Correspondence::IndexPair(corr.index_query, corr.index_match));
			}
		}
		Correspondence(const Correspondence::Ptr& correspondences_)
			: from(correspondences_->from),
			  to(correspondences_->to),
			  pairs(correspondences_->pairs),
			  line_width(correspondences_->line_width),
			  color(correspondences_->color),
			  visible(correspondences_->visible) {}
	};

	class CoordinateSystem
	{
	public:
		double scale;
		bool visible;

		CoordinateSystem(
			const double& scale_ = 1.0,
			const bool visible_ = true)
			: scale(scale_), visible(visible_) {}
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
			pcl::console::print_error("[Rab Visualizer] Can't load camera settings.\n");
			pcl::console::print_error(e.what());
		}
		return cam;
	}

	class Rab : public Creatable<rabv::Rab>
	{
	private:
		rabv::Clouds clouds;
		rabv::Normals normals;
		rabv::Correspondences correspondences;
		rabv::LinesSet lines_set;
		rabv::Texts texts;
		rabv::Text3Ds text3Ds;
		rabv::FlatText3Ds flat_text3Ds;
		rabv::CoordinateSystems coordinate_systems;
		boost::optional<pcl::visualization::Camera> camera = boost::none;

	public:
		friend rabv::Writer;
		friend rabv::Viewer;
		typedef boost::shared_ptr<rabv::Rab> Ptr;
		typedef boost::shared_ptr<const rabv::Rab> ConstPtr;

		Rab(const Rab::Ptr& rab_)
			: clouds(rab_->clouds),
			  normals(rab_->normals),
			  correspondences(rab_->correspondences),
			  lines_set(rab_->lines_set),
			  texts(rab_->texts),
			  text3Ds(rab_->text3Ds),
			  flat_text3Ds(rab_->flat_text3Ds),
			  coordinate_systems(rab_->coordinate_systems),
			  camera(rab_->camera) {}
		Rab() {}

		void addCloud(const std::string& name_, const rabv::Cloud& cloud_)
		{
			clouds.unique_insert(name_, cloud_);

		}
		template<typename... Args>
		void addCloud(const std::string& name_, Args... args)
		{
			addCloud(name_, rabv::Cloud(std::forward<Args>(args)...));
		}

		void removeCloud(const std::string& name)
		{
			clouds.check_erase(name);
		}

		void addNormal(const std::string& name_, const rabv::Normal& normal_)
		{
			normals.unique_insert(name_, normal_);
		}
		template<typename... Args>
		void addNormal(const std::string& name_, Args... args)
		{
			addNormal(name_, rabv::Normal(std::forward<Args>(args)...));
		}

		void removeNormal(const std::string& name)
		{
			normals.check_erase(name);
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
			addCloud(
				name_,
				cloud_,
				point_size_,
				cloud_color_,
				offset_,
				rotation_,
				visible_
			);

			addNormal(
				name_,
				normal_,
				level_,
				scale_,
				line_width_,
				normal_color_,
				visible_
			);
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

			addCloudNormal(
				name_,
				cloud,
				normal,
				point_size_,
				cloud_color_,
				offset_,
				rotation_,
				level_,
				scale_,
				line_width_,
				normal_color_,
				visible_
			);
		}

		void removeCloudNormal(const std::string& name)
		{
			clouds.check_erase(name);
			normals.check_erase(name);
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
			if (clouds.find(name_) == clouds.end())
			{
				addCloud(name_);
			}

			auto& cloud = clouds.find(name_)->second;

			cloud.cloud->push_back(point);
			cloud.point_size = point_size_;
			cloud.color = color_;
			cloud.offset = offset_;
			cloud.rotation = rotation_;
			cloud.visible = visible_;
		}

		void addCorrespondence(const std::string& name, const rabv::Correspondence& corr)
		{
			correspondences.unique_insert(name, corr);

		}
		template<typename... Args>
		void addCorrespondence(const std::string& name, Args... args)
		{
			addCorrespondence(name, rabv::Correspondence(std::forward<Args>(args)...));
		}

		void removeCorrespondence(const std::string& name)
		{
			correspondences.check_erase(name);
		}

		void addLines(const std::string& name, const rabv::Lines& lines)
		{
			lines_set.unique_insert(name, lines);
		}
		template<typename... Args>
		void addLines(const std::string& name, Args... args)
		{
			addLines(name, rabv::Lines(std::forward<Args>(args)...));
		}

		void removeLines(const std::string& name)
		{
			lines_set.check_erase(name);
		}

		template<typename... Args>
		void addLine(const std::string& name, Args... args)
		{
			if (lines_set.find(name) == lines_set.end())
			{
				addLines(name);
			}

			lines_set.find(name)->second.addLine(std::forward<Args>(args)...);
		}

		void addCube(
			const std::string& name,
			const rabv::Point& p1,
			const rabv::Point& p2,
			const double line_width = 1.0,
			const rabv::Color &color = rabv::Color())
		{
			auto lines = rabv::Lines::create(line_width, color);

			lines->addLine(rabv::Point(p1.x, p1.y, p1.z), rabv::Point(p2.x, p1.y, p1.z));
			lines->addLine(rabv::Point(p1.x, p1.y, p1.z), rabv::Point(p1.x, p2.y, p1.z));
			lines->addLine(rabv::Point(p1.x, p1.y, p1.z), rabv::Point(p1.x, p1.y, p2.z));
			lines->addLine(rabv::Point(p1.x, p2.y, p2.z), rabv::Point(p1.x, p1.y, p2.z));
			lines->addLine(rabv::Point(p1.x, p2.y, p2.z), rabv::Point(p1.x, p2.y, p1.z));
			lines->addLine(rabv::Point(p1.x, p2.y, p2.z), rabv::Point(p2.x, p2.y, p2.z));
			lines->addLine(rabv::Point(p2.x, p1.y, p2.z), rabv::Point(p2.x, p1.y, p1.z));
			lines->addLine(rabv::Point(p2.x, p1.y, p2.z), rabv::Point(p1.x, p1.y, p2.z));
			lines->addLine(rabv::Point(p2.x, p1.y, p2.z), rabv::Point(p2.x, p2.y, p2.z));
			lines->addLine(rabv::Point(p2.x, p2.y, p1.z), rabv::Point(p1.x, p2.y, p1.z));
			lines->addLine(rabv::Point(p2.x, p2.y, p1.z), rabv::Point(p2.x, p1.y, p1.z));
			lines->addLine(rabv::Point(p2.x, p2.y, p1.z), rabv::Point(p2.x, p2.y, p2.z));

			addLines(name, lines);
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

		void addCoordinateSystem(const std::string& name, const rabv::CoordinateSystem& coordinate_system)
		{
			coordinate_systems.unique_insert(name, coordinate_system);
		}
		template<typename... Args>
		void addCoordinateSystem(const std::string& name, Args... args)
		{
			addCoordinateSystem(name, rabv::CoordinateSystem(std::forward<Args>(args)...));
		}

		void removeCoordinateSystem(const std::string& name)
		{
			coordinate_systems.check_erase(name);
		}

		void setCamera(const pcl::visualization::Camera& cam)
		{
			camera = cam;
		}
		void setCamera(const boost::none_t cam)
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

	class Viewer : public Creatable<rabv::Viewer>
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
			  pcl_viewer(new pcl::visualization::PCLVisualizer(viewer_name_)),
			  rab(rab_)
		{
			initPclViewer(visualize_);
		}
		Viewer(const Viewer::Ptr& viewer_)
			: viewer_name(viewer_->viewer_name),
			  camfile_path(viewer_->camfile_path),
			  pcl_viewer(new pcl::visualization::PCLVisualizer(*viewer_->pcl_viewer)),
			  rab(viewer_->rab->makeShared()) {}

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

			if (!rab->getCamera())
			{
				rab->setCamera(cam);
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
					rabv::CameraToPree(*(rab->getCamera())),
					std::locale(),
					boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1)
				);
			}
			catch (const boost::property_tree::xml_parser::xml_parser_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Can't write camera settings.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");
			}
		}

		void loadCameraParams()
		{
			boost::property_tree::ptree cam_ptree;
			try
			{
				boost::property_tree::read_xml(camfile_path.string(), cam_ptree);
				rab->setCamera(rabv::PreeToCamera(cam_ptree));
				const auto& cam = *(rab->getCamera());
				pcl_viewer->setCameraParameters(cam);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: camera settings file load error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");
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
				pcl::console::print_info("[Rab Visualizer] Camera Parameter is saved.");
			}
			else if (event.getKeySym() == "l" && event.keyDown())
			{
				viewer->loadCameraParams();
				pcl::console::print_info("[Rab Visualizer] Camera Parameter is loaded.");
			}
		}

		static void pointPickupEvent(const pcl::visualization::PointPickingEvent& event)
		{
			const int index = event.getPointIndex();
			if (index == -1)
			{
				return;
			}

			static rabv::Point prev_point;
			rabv::Point point;
			event.getPoint(point.x, point.y, point.z);

			const rabv::Point diff(
				point.x - prev_point.x,
				point.y - prev_point.y,
				point.z - prev_point.z
			);

			std::stringstream ss;

			ss	<< "Index: " << index
				<< ", Point: " << point.x << " " << point.y << " " << point.z << std::endl
				<< "Distance from the origin: " << point.getVector3fMap().norm() << std::endl
				<< "Diff (prev point) : " << diff << std::endl << std::endl;

			pcl::console::print_info(ss.str().c_str());

			prev_point = point;
		}

		pcl::visualization::PCLVisualizer::Ptr getPCLVisualizer() const
		{
			return pcl_viewer;
		}
		
		std::string viewerName() const
		{
			return viewer_name;
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
			pcl_viewer->registerPointPickingCallback(rabv::Viewer::pointPickupEvent);
			if (visualize_)
			{
				visualize();
			}
		}
	};

	class Reader : public Creatable<rabv::Reader>
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
		Reader(const rabv::Reader::Ptr& reader_)
			: rabfile_path(reader_->rabfile_path),
			  camfile_path(reader_->camfile_path),
			  rab(reader_->rab->makeShared())
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

	class Writer : public Creatable<rabv::Writer>
	{
	public:
		typedef boost::shared_ptr<rabv::Writer> Ptr;
		typedef boost::shared_ptr<const rabv::Writer> ConstPtr;

		Writer(const std::string& rab_path, const rabv::Rab::Ptr& rab = rabv::Rab::create())
			: rab(rab)
		{
			setPath(rab_path);
		}
		Writer(const rabv::Writer::Ptr& writer_)
			: rab_name(writer_->rab_name), path(writer_->path), rab(writer_->rab->makeShared()) {}

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

		static void saveRabFile(const std::string& rab_path_, const rabv::Rab::Ptr& rab_)
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
	if (pcl_viewer->wasStopped())
	{
		return;
	}

	// remove all drawing objects
	pcl_viewer->removeAllPointClouds();
	pcl_viewer->removeAllShapes();
	pcl_viewer->removeAllCoordinateSystems(); //in this version's PCL, this function doesn't work.
	pcl_viewer->removeCoordinateSystem("world");
	for (const auto& cloud : rab->clouds)
	{
		pcl_viewer->removeCoordinateSystem(cloud.first);
	}

	// add point cloud and coor
	UniqueMap<pcl::PointCloud<pcl::PointXYZ>::Ptr> trans_clouds;
	for (const auto& cloud_pair : rab->clouds)
	{
		const auto& name = cloud_pair.first;
		const auto& cloud = cloud_pair.second;

		pcl::PointCloud<pcl::PointXYZ>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::transformPointCloud(*cloud.cloud, *trans_cloud, cloud.transform());

		trans_clouds.unique_insert(name, trans_cloud);
	}

	for (const auto& cloud_pair : rab->clouds)
	{
		const auto& name = cloud_pair.first;
		const auto& cloud = cloud_pair.second;

		if (!cloud.visible)
		{
			continue;
		}

		const auto& trans_cloud = trans_clouds[name];

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(
			trans_cloud, cloud.color.r, cloud.color.g, cloud.color.b);

		pcl_viewer->addPointCloud(trans_cloud, color_handler, name);
		pcl_viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_POINT_SIZE, cloud.point_size, name);
	}

	for (const auto& coor_pair : rab->coordinate_systems)
	{
		const auto& coor_name = coor_pair.first;
		const auto& coor = coor_pair.second;

		if (!coor.visible)
		{
			continue;
		}

		if (coor_name == "world")
		{
			pcl_viewer->addCoordinateSystem(coor.scale);
		}
		else
		{
			for (const auto& cloud_pair : rab->clouds)
			{
				const auto& cloud_name = cloud_pair.first;
				const auto& cloud = cloud_pair.second;

				pcl_viewer->removeCoordinateSystem(cloud_name);

				if (coor_name == cloud_name)
				{
					pcl_viewer->removeCoordinateSystem(coor_name);
					pcl_viewer->addCoordinateSystem(coor.scale, cloud.affine(), cloud_name + "_coor");
				}
			}
		}
	}

	for (const auto& normal_pair : rab->normals)
	{
		const auto& name = normal_pair.first;
		const auto& normal = normal_pair.second;

		if (!normal.visible)
		{
			continue;
		}

		if (trans_clouds.find(name) == trans_clouds.end())
		{
			std::stringstream ss;
			ss << "Rab Visualizer] Error: Normal cannot find base pointcloud named "
				<< name << "." << std::endl;

			pcl::console::print_error(ss.str().c_str());

			continue;
		}

		pcl_viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
			trans_clouds[name],
			normal.normal,
			normal.level,
			normal.scale,
			name + "_normal");
		pcl_viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_COLOR,
			static_cast<double>(normal.color.r) / 255.0,
			static_cast<double>(normal.color.g) / 255.0,
			static_cast<double>(normal.color.b) / 255.0,
			name + "_normal");
		pcl_viewer->setPointCloudRenderingProperties(
			pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
			normal.line_width,
			name + "_normal");
	}

	for (const auto& corr_pair : rab->correspondences)
	{
		const auto& name = corr_pair.first + "_correspondence";
		const auto& corr = corr_pair.second;

		if (!corr.visible)
		{
			continue;
		}

		if (trans_clouds.find(corr.from) == trans_clouds.end() ||
			trans_clouds.find(corr.to) == trans_clouds.end())
		{
			continue;
		}

		const vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();

		const auto& cloud_from = trans_clouds[corr.from];
		const auto& cloud_to = trans_clouds[corr.to];

		const vtkSmartPointer<vtkCellArray> lines_cellarray = vtkSmartPointer<vtkCellArray>::New();
		const vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

		for (int i = 0; i < corr.pairs.size(); ++i)
		{
			const auto& point_from = (*cloud_from)[corr.pairs[i].first];
			const auto& point_to = (*cloud_to)[corr.pairs[i].second];

			const float from[3] = { point_from.x, point_from.y, point_from.z };
			const float to[3] = { point_to.x, point_to.y, point_to.z };
			pts->InsertNextPoint(from);
			pts->InsertNextPoint(to);
		}

		linesPolyData->SetPoints(pts);

		for (int i = 0; i < corr.pairs.size(); ++i)
		{
			const vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();

			line->GetPointIds()->SetId(0, 2 * i + 0);
			line->GetPointIds()->SetId(1, 2 * i + 1);

			lines_cellarray->InsertNextCell(line);
		}

		linesPolyData->SetLines(lines_cellarray);

		const unsigned char color[3] = {
			static_cast<unsigned char>(corr.color.r),
			static_cast<unsigned char>(corr.color.g),
			static_cast<unsigned char>(corr.color.b)
		};

		const vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();

		colors->SetNumberOfComponents(3);
		for (int i = 0; i < corr.pairs.size(); ++i)
		{
#if (VTK_MAJOR_VERSION == 7 && VTK_MINOR_VERSION >= 1) || (VTK_MAJOR_VERSION > 7)
			colors->InsertNextTypedTuple(color);
#else
			colors->InsertNextTupleValue(color);
#endif
		}

		linesPolyData->GetCellData()->SetScalars(colors);

		const vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(linesPolyData);

		pcl_viewer->addModelFromPolyData(mapper->GetInput(), name);
		pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, corr.line_width, name);
	}

	for (const auto& lines_pair : rab->lines_set)
	{
		const auto& name = lines_pair.first + "_lines";
		const auto& lines = lines_pair.second;

		if (!lines.visible)
		{
			continue;
		}

		const vtkSmartPointer<vtkPolyData> linesPolyData = vtkSmartPointer<vtkPolyData>::New();

		const vtkSmartPointer<vtkCellArray> lines_cellarray = vtkSmartPointer<vtkCellArray>::New();
		const vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

		for (int i = 0; i < lines.lines.size(); ++i)
		{
			const float from[3] = { lines.lines[i].from.x, lines.lines[i].from.y, lines.lines[i].from.z };
			const float to[3] = { lines.lines[i].to.x, lines.lines[i].to.y, lines.lines[i].to.z };
			pts->InsertNextPoint(from);
			pts->InsertNextPoint(to);
		}

		linesPolyData->SetPoints(pts);

		for (int i = 0; i < lines.lines.size(); ++i)
		{
			const vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();

			line->GetPointIds()->SetId(0, 2 * i + 0);
			line->GetPointIds()->SetId(1, 2 * i + 1);

			lines_cellarray->InsertNextCell(line);
		}

		linesPolyData->SetLines(lines_cellarray);

		const unsigned char color[3] = {
			static_cast<unsigned char>(lines.color.r),
			static_cast<unsigned char>(lines.color.g),
			static_cast<unsigned char>(lines.color.b)
		};

		const vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();

		colors->SetNumberOfComponents(3);
		for (int i = 0; i < lines.lines.size(); ++i)
		{
#if (VTK_MAJOR_VERSION == 7 && VTK_MINOR_VERSION >= 1) || (VTK_MAJOR_VERSION > 7)
			colors->InsertNextTypedTuple(color);
#else
			colors->InsertNextTupleValue(color);
#endif
		}

		linesPolyData->GetCellData()->SetScalars(colors);

		const vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputData(linesPolyData);

		pcl_viewer->addModelFromPolyData(mapper->GetInput(), name);
		pcl_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lines.line_width, name);
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
			std::string("text_") + std::to_string(i)
		);
	}

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
			std::string("text3D_") + std::to_string(i)
		);
	}

	if (rab->flat_text3Ds.size() > 0)
	{
		const auto append_filter = vtkSmartPointer<vtkAppendPolyData>::New();
		for (const auto& flat_text3D : rab->flat_text3Ds)
		{
			if (!flat_text3D.visible)
			{
				continue;
			}

			const auto vec_text = vtkSmartPointer<vtkVectorText>::New();
			vec_text->SetText(flat_text3D.text.c_str());
			vec_text->Update();

			const auto polydata = vec_text->GetOutput();

			for (int i = 0; i < polydata->GetNumberOfPoints(); ++i)
			{
				const double* point = polydata->GetPoint(i);
				const Eigen::Matrix3f rmat = flat_text3D.rotation.matrix();
				const Eigen::Vector3f vec(point[0], point[1], point[2]);
				const Eigen::Vector3f t = rmat * vec;

				polydata->GetPoints()->SetPoint(
					i,
					t[0] * flat_text3D.font_size + flat_text3D.x,
					t[1] * flat_text3D.font_size + flat_text3D.y,
					t[2] * flat_text3D.font_size + flat_text3D.z);
			}

			const unsigned char color[3] = {
				static_cast<unsigned char>(flat_text3D.color.r),
				static_cast<unsigned char>(flat_text3D.color.g),
				static_cast<unsigned char>(flat_text3D.color.b)
			};

			const auto colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
			colors->SetNumberOfComponents(3);

			for (int i = 0; i < polydata->GetNumberOfCells(); ++i)
			{
#if (VTK_MAJOR_VERSION == 7 && VTK_MINOR_VERSION >= 1) || (VTK_MAJOR_VERSION > 7)
				colors->InsertNextTypedTuple(color);
#else
				colors->InsertNextTupleValue(color);
#endif
			}

			polydata->GetCellData()->SetScalars(colors);

			append_filter->AddInputData(polydata);
		}
		append_filter->Update();

		const auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		mapper->SetInputConnection(append_filter->GetOutputPort());

		pcl_viewer->addModelFromPolyData(mapper->GetInput(), "3DTextSet");
	}

	const auto& renderer = pcl_viewer->getRendererCollection()->GetFirstRenderer();
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
		pcl::console::print_error("[Rab Visualizer] Error: rab file load error.\n");
		pcl::console::print_error(e.what());
		pcl::console::print_error("\n");
	}

	try
	{
		if (ptree.get<int>("version", 1) != 3)
		{
			pcl::console::print_error("[Rab Visualizer] Error: rab file version error.\n");
		}
	}
	catch (boost::property_tree::ptree_error& e)
	{
		pcl::console::print_error("[Rab Visualizer] Error: rab file load error.\n");
		pcl::console::print_error(e.what());
		pcl::console::print_error("\n");
	}

	rab = rabv::Rab::create();
	
	if (ptree.get_child_optional("cloudXYZ"))
	{
		for (const auto& ptree_itr : ptree.get_child("cloudXYZ", boost::property_tree::ptree()))
		{
			const auto& ptree_ = ptree_itr.second;
			try
			{
				const std::string& name = ptree_.get<std::string>("name");

				rabv::Cloud cloud;
				const auto pcd_filename = ptree_.get<std::string>("name") + ".pcd";
				const auto pcd_path = boost::filesystem::absolute(pcd_filename, data_dir_path);

				if (pcl::io::loadPCDFile(pcd_path.string(), *(cloud.cloud)) == -1)
				{
					pcl::console::print_error("[Rab Visualizer] Error: pcd file load error.\n");
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

				rab->addCloud(name, cloud);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file format error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");

				continue;
			}
		}
	}

	if (ptree.get_child_optional("normal"))
	{
		for (const auto& ptree_itr : ptree.get_child("normal", boost::property_tree::ptree()))
		{
			const auto& ptree_ = ptree_itr.second;
			try
			{
				const std::string& name = ptree_.get<std::string>("name");

				rabv::Normal normal;
				const auto pcd_filename = ptree_.get<std::string>("name") + "_normal.pcd";
				const auto pcd_path = boost::filesystem::absolute(pcd_filename, data_dir_path);

				if (pcl::io::loadPCDFile(pcd_path.string(), *(normal.normal)) == -1)
				{
					pcl::console::print_error("[Rab Visualizer] Error: pcd file load error.\n");
					continue;
				}

				normal.color = rabv::Color(
					ptree_.get<int>("color.r", 255),
					ptree_.get<int>("color.g", 255),
					ptree_.get<int>("color.b", 255));

				normal.level = ptree_.get<int>("level", 100);
				normal.scale = ptree_.get<double>("scale", 0.02);
				normal.line_width = ptree_.get<double>("line_width", 1);
				normal.visible = ptree_.get<bool>("visible", true);

				rab->addNormal(name, normal);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file format error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");
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
					ptree_.get<std::string>("from"),
					ptree_.get<std::string>("to"),
					ptree_.get<double>("line_width", 1.0),
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
						std::stoi(indices[0]),
						std::stoi(indices[1])
					);
					corr.pairs.push_back(pair);
				}

				rab->addCorrespondence(corr_name, corr);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file format error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");

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

				rabv::Lines lines;

				lines.line_width = ptree_.get<double>("line_width", 1.0);

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

				rab->addLines(lines_name, lines);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file firmat error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");

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
				rab->addText(
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
				);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file firmat error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");

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
				rab->addText3D(
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
					ptree_.get<bool>("visible", true)
				);

			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file firmat error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");

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
				rab->addFlatText3D(
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
				);

			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file firmat error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");

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
				rab->addCoordinateSystem(
					ptree_.get<std::string>("name", "world"),
					ptree_.get<double>("scale", 1),
					ptree_.get<bool>("visible", true)
				);
			}
			catch (boost::property_tree::ptree_error& e)
			{
				pcl::console::print_error("[Rab Visualizer] Error: rab file firmat error.\n");
				pcl::console::print_error(e.what());
				pcl::console::print_error("\n");

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
			rab->setCamera(rabv::PreeToCamera(ptree));
		}
		catch (boost::property_tree::ptree_error& e)
		{
			pcl::console::print_error("[Rab Visualizer] Error: camera file load error.\n");
			pcl::console::print_error(e.what());
			pcl::console::print_error("\n");
		}
	}
}

void rabv::Writer::save() const
{
	create_dir(path / rab_name);

	boost::property_tree::ptree ptree;

	ptree.add("version", 3);

	for (const auto& cloud_pair : rab->clouds)
	{
		const auto& name = cloud_pair.first;
		const auto& cloud = cloud_pair.second;

		const std::string filename = name + ".pcd";
		boost::filesystem::path data_path = path / rab_name / filename;
		if (cloud.cloud->size() > 0)
		{
			pcl::io::savePCDFileBinaryCompressed(data_path.string(), *(cloud.cloud));
		}

		boost::property_tree::ptree ptree_cloud;
		ptree_cloud.add("name", name);
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

	for (const auto& normal_pair : rab->normals)
	{
		const auto& name = normal_pair.first;
		const auto& normal = normal_pair.second;

		const std::string filename = name + "_normal.pcd";
		boost::filesystem::path data_path = path / rab_name / filename;
		if (normal.normal->size() > 0)
		{
			pcl::io::savePCDFileBinaryCompressed(data_path.string(), *(normal.normal));
		}

		boost::property_tree::ptree ptree_cloud;
		ptree_cloud.add("name", name);
		ptree_cloud.add("color.r", normal.color.r);
		ptree_cloud.add("color.g", normal.color.g);
		ptree_cloud.add("color.b", normal.color.b);
		ptree_cloud.add("level", normal.level);
		ptree_cloud.add("scale", normal.scale);
		ptree_cloud.add("line_width", normal.line_width);
		ptree_cloud.add("visible", normal.visible);
		ptree.add_child("normal.normal", ptree_cloud);
	}

	for (const auto& corr_pair : rab->correspondences)
	{
		const auto& name = corr_pair.first;
		const auto& corr = corr_pair.second;

		const std::string filename = name + "_corr.txt";
		boost::filesystem::path data_path = path / rab_name / filename;
		std::ofstream ofs(data_path.string());
		for (const auto& line : corr.pairs)
		{
			ofs << line.first << " " << line.second << std::endl;
		}

		boost::property_tree::ptree ptree_corr;
		ptree_corr.add("name", name);
		ptree_corr.add("from", corr.from);
		ptree_corr.add("to", corr.to);
		ptree_corr.add("line_width", corr.line_width);
		ptree_corr.add("color.r", corr.color.r);
		ptree_corr.add("color.g", corr.color.g);
		ptree_corr.add("color.b", corr.color.b);
		ptree_corr.add("visible", corr.visible);
		ptree.add_child("correspondences.correspondence", ptree_corr);
	}

	for (const auto& lines_pair : rab->lines_set)
	{
		const auto& name = lines_pair.first;
		const auto& lines = lines_pair.second;

		const std::string filename = name + "_line.txt";
		boost::filesystem::path data_path = path / rab_name / filename;
		std::ofstream ofs(data_path.string());
		for (const auto& line : lines.lines)
		{
			ofs << line.from.x << " " << line.from.y << " " << line.from.z << " "
				<< line.to.x << " " << line.to.y << " " << line.to.z << " " << std::endl;
		}

		boost::property_tree::ptree ptree_lines;
		ptree_lines.add("name", name);
		ptree_lines.add("line_width", lines.line_width);
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

	for (const auto& coor_pair : rab->coordinate_systems)
	{
		const auto& name = coor_pair.first;
		const auto& coor = coor_pair.second;

		boost::property_tree::ptree ptree_text;
		ptree_text.add("name", name);
		ptree_text.add("scale", coor.scale);
		ptree_text.add("visible", coor.visible);
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
		pcl::console::print_error("[Rab Visualizer] Can't write rab file.\n");
		pcl::console::print_error(e.what());
		pcl::console::print_error("\n");
	}

	if (rab->getCamera())
	{
		try
		{
			boost::property_tree::xml_parser::write_xml(
				Reader::getCamfilePath(data_path).string(),
				rabv::CameraToPree(*(rab->getCamera())),
				std::locale(),
				boost::property_tree::xml_writer_make_settings<boost::property_tree::ptree::key_type>('\t', 1)
			);
		}
		catch (const boost::property_tree::xml_parser::xml_parser_error& e)
		{
			pcl::console::print_error("[Rab Visualizer] Can't write camera settings.\n");
			pcl::console::print_error(e.what());
			pcl::console::print_error("\n");
		}
	}
}
}
