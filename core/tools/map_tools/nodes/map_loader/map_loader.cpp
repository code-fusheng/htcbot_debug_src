/*
 * @Author: code-fusheng
 * @Date: 2024-04-19 13:45:12
 * @LastEditors: code-fusheng 2561035977@qq.com
 * @LastEditTime: 2024-05-14 01:24:37
 * @Description:
 */

#include "map_loader.h"

using namespace MapCommonNS;

namespace MapLoaderNS
{

	MapLoader::MapLoader() : nh_private_("~")
	{
		area_list_filename_ = "pcd_info.csv";
		map_status_.module_type = static_cast<int>(HtcbotCommonNS::MODULE_TYPE::MAP);
		map_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::NONE);
	}

	MapLoader::~MapLoader()
	{
	}

	void MapLoader::init()
	{

		server_.setCallback(boost::bind(&MapLoader::dynamicReconfigureCallback, this, _1, _2));

		nh_private_.param<double>("margin", margin_, 200.0);
		nh_private_.param<double>("update_interval", update_interval_, 1000.0);
		nh_private_.param<double>("update_offset", update_offset_, 10.0);

		nh_private_.param<std::string>("static_topic", static_topic_, "static_map");
		nh_private_.param<std::string>("dynamic_topic", dynamic_topic_, "points_map");

		nh_private_.param<bool>("/map_loader_node/is_pub_pgm", is_pub_pgm_, false);
		nh_private_.param<double>("/map_loader_node/map_resolution", map_resolution_, 0.1);
		nh_private_.param<bool>("/map_loader_node/is_filter_pass_through", is_filter_pass_through_, false);
		nh_private_.param<double>("/map_loader_node/filter_high", filter_high_, 2.0);
		nh_private_.param<double>("/map_loader_node/filter_low", filter_low_, -2.0);
		nh_private_.param<bool>("/map_loader_node/is_filter_radius_outlier", is_filter_radius_outlier_, false);
		nh_private_.param<double>("/map_loader_node/filter_radius", filter_radius_, 1.0);
		nh_private_.param<int>("/map_loader_node/filter_thre_count", filter_thre_count_, 5);
		nh_private_.param<bool>("/map_loader_node/is_filter_voxel_leaf", is_filter_voxel_leaf_, false);
		nh_private_.param<double>("/map_loader_node/filter_leaf_size", filter_leaf_size_, 0.2);

		pub_static_points_map_ = nh_.advertise<sensor_msgs::PointCloud2>(static_topic_, 1, true);
		pub_dynamic_points_map_ = nh_.advertise<sensor_msgs::PointCloud2>(dynamic_topic_, 1, true);
		pub_module_status_ = nh_.advertise<htcbot_msgs::StatusHtcbotModule>("/htcbot/module_status", 10);
		pub_occ_grid_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);

		sub_map_path_conf_ = nh_.subscribe("/htcbot/map_path_conf", 1, &MapLoader::callbackMapPathConf, this);
		sub_initialpose_ = nh_.subscribe("/initialpose", 1, &MapLoader::callbackInitialPose, this);
		sub_current_pose_ = nh_.subscribe("/current_pose", 1, &MapLoader::callbackCurrentPose, this);
	}

	void MapLoader::run()
	{
		init();
		while (ros::ok())
		{
			ros::spin();
		}
	}

	void MapLoader::dynamicReconfigureCallback(map_tools::MapLoaderConfig &config, uint32_t level)
	{
		is_debug_ = config.is_debug;
		margin_ = config.margin;
		is_pub_pgm_ = config.is_pub_pgm;
		map_resolution_ = config.map_resolution;

		is_filter_pass_through_ = config.is_filter_pass_through;
		filter_high_ = config.filter_high;
		filter_low_ = config.filter_low;
		is_filter_radius_outlier_ = config.is_filter_radius_outlier;
		filter_radius_ = config.filter_radius;
		filter_thre_count_ = config.filter_thre_count;
		is_filter_voxel_leaf_ = config.is_filter_voxel_leaf;
		filter_leaf_size_ = config.filter_leaf_size;
	}

	void MapLoader::callbackMapPathConf(const htcbot_msgs::MapPathConf::ConstPtr &msg)
	{
		ROS_INFO("[map_loader] ==> received conf map_static_path: %s, map_dynamic_path: %s", msg->map_static_path.c_str(), msg->map_dynamic_path.c_str());
		dir_static_map_ = msg->map_static_path;
		dir_dynamic_map_ = msg->map_dynamic_path;
		default_area_list_.clear();
		cached_area_list_.clear();
		default_area_list_ = loadAreaList(dir_dynamic_map_ + "/" + area_list_filename_);
		doFilterMap();
		publishStaticPcdMap();
		if (is_pub_pgm_)
		{
			nav_msgs::OccupancyGrid occ_map_msg;
			pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_map_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcd_map_filtered_));
			convertPointCloudToOccupancyGrid(pcd_map_ptr, occ_map_msg);
			pub_occ_grid_.publish(occ_map_msg);
		}
	}

	void MapLoader::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped &input)
	{
		if (dir_dynamic_map_ == "")
		{
			ROS_ERROR("[map_loader] Received /initialpose, but dynamic map dir is empty.");
			return;
		}
		geometry_msgs::PoseStamped cur_pose;
		cur_pose.header = input.header;
		cur_pose.pose = input.pose.pose;
		loadPcdByPoint(input.pose.pose.position);
		publishDynamicPcdMap();
		last_update_time_ = ros::Time::now();
	}

	void MapLoader::callbackCurrentPose(const geometry_msgs::PoseStamped &input)
	{
		if (dir_dynamic_map_ == "")
		{
			// ROS_ERROR("[map_loader] Received /current_pose, but dynamic map dir is empty.");
			return;
		}
		if ((ros::Time::now() - last_update_time_).toSec() * 1000 < update_interval_)
		{
			return;
		}
		loadPcdByPoint(input.pose.position);
		publishDynamicPcdMap();
		last_update_time_ = ros::Time::now();
	}

	void MapLoader::doFilterMap()
	{
		ROS_INFO("[map_loader] >> start to filter map");
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_pass_filtered(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_radius_filtered(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr map_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>());
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_map_ptr(new pcl::PointCloud<pcl::PointXYZI>());
		// 从指定路径加载PCD文件（点云数据），并将其存储到msg_globalmap中
		int res = pcl::io::loadPCDFile(dir_static_map_ + "/origin.pcd", *map_ptr);
		pcd_map_ = *map_ptr;
		// 判断加载是否成功
		if (res != 0)
		{
			ROS_ERROR("[map_loader] Cann't load Static Map");
			return;
		}
		// PassThroughFilter
		if (is_filter_pass_through_)
		{
			pcl::PassThrough<pcl::PointXYZI> pass_through;
			pass_through.setInputCloud(map_ptr);
			pass_through.setFilterFieldName("z");
			pass_through.setFilterLimits(filter_low_, filter_high_);
			pass_through.setFilterLimitsNegative(false);
			pass_through.filter(*map_pass_filtered);
		}
		else
		{
			map_pass_filtered = map_ptr;
		}
		// RadiusOutlierFilter
		if (is_filter_radius_outlier_)
		{
			pcl::RadiusOutlierRemoval<pcl::PointXYZI> radius_outlier;
			radius_outlier.setInputCloud(map_pass_filtered);
			radius_outlier.setRadiusSearch(filter_radius_);
			radius_outlier.setMinNeighborsInRadius(filter_thre_count_);
			radius_outlier.filter(*map_radius_filtered);
		}
		else
		{
			map_radius_filtered = map_pass_filtered;
		}
		//
		if (is_filter_voxel_leaf_)
		{
			pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_save;
			voxel_grid_filter_save.setLeafSize(filter_leaf_size_, filter_leaf_size_, filter_leaf_size_);
			voxel_grid_filter_save.setInputCloud(map_radius_filtered);
			voxel_grid_filter_save.filter(*map_voxel_filtered);
		}
		else
		{
			map_voxel_filtered = map_radius_filtered;
		}
		// final
		pcd_map_filtered_ = *map_voxel_filtered;
	}

	void MapLoader::convertPointCloudToOccupancyGrid(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &in_cloud, nav_msgs::OccupancyGrid &out_occ_msg)
	{
		out_occ_msg.header.stamp = ros::Time::now();
		out_occ_msg.header.frame_id = "map";
		out_occ_msg.info.map_load_time = ros::Time::now();
		// 地图边长
		// map_resolution_ = 0.05;
		out_occ_msg.info.resolution = map_resolution_;
		// 这里是投影到xy平面，如果要投到xz/yz，这里以及后面的xy对应的数据改为你想投影的平面
		double x_min, x_max, y_min, y_max;
		if (in_cloud->points.empty())
		{
			ROS_WARN("[map_loader] ===> In Pcd Cloud Is Empty!\n");
			return;
		}
		for (int i = 0; i < in_cloud->points.size() - 1; i++)
		{
			if (i == 0)
			{
				x_min = x_max = in_cloud->points[i].x;
				y_min = y_max = in_cloud->points[i].y;
			}
			double x = in_cloud->points[i].x;
			double y = in_cloud->points[i].y;
			if (x < x_min)
				x_min = x;
			if (x > x_max)
				x_max = x;
			if (y < y_min)
				y_min = y;
			if (y > y_max)
				y_max = y;
		}
		out_occ_msg.info.origin.position.x = x_min;
		out_occ_msg.info.origin.position.y = y_min;
		out_occ_msg.info.origin.position.z = 0.0;
		out_occ_msg.info.origin.orientation.x = 0.0;
		out_occ_msg.info.origin.orientation.y = 0.0;
		out_occ_msg.info.origin.orientation.z = 0.0;
		out_occ_msg.info.origin.orientation.w = 1.0;

		out_occ_msg.info.width = int((x_max - x_min) / map_resolution_);
		out_occ_msg.info.height = int((y_max - y_min) / map_resolution_);

		out_occ_msg.data.resize(out_occ_msg.info.width * out_occ_msg.info.height);
		out_occ_msg.data.assign(out_occ_msg.info.width * out_occ_msg.info.height, 0);

		for (int iter = 0; iter < in_cloud->points.size(); iter++)
		{
			int i = int((in_cloud->points[iter].x - x_min) / map_resolution_);
			if (i < 0 || i >= out_occ_msg.info.width)
				continue;
			int j = int((in_cloud->points[iter].y - y_min) / map_resolution_);
			if (j < 0 || j >= out_occ_msg.info.height - 1)
				continue;
			out_occ_msg.data[i + j * out_occ_msg.info.width] = 100;
		}
	}

	sensor_msgs::PointCloud2 MapLoader::loadPcdByPoint(const geometry_msgs::Point &point)
	{
		// 根据给定点p创建一个待加载区域列表
		MapCommonNS::AreaList need_area_list = loadNeedAreaList(point);
		// 用于存储加载的区域列表
		MapCommonNS::AreaList tmp_area_list;
		// 标志是否需要更新缓存
		bool cache_update = false;
		// 遍历待加载区域列表
		for (MapCommonNS::Area &area : need_area_list)
		{
			// 检查该区域是否已经在缓存区域列表中
			int index = isInCachedAreaList(area);
			if (index == -1)
			{ // io
				// 如果不在缓存中，进行IO操作加载点云
				MapCommonNS::Area in_area(area);
				pcl::io::loadPCDFile(dir_dynamic_map_ + "/" + area.filename, in_area.points);
				tmp_area_list.push_back(in_area);
				cache_update = true;
			}
			else
			{
				// 如果在缓存中，直接使用缓存中的数据
				MapCommonNS::Area in_area(cached_area_list_[index]);
				tmp_area_list.push_back(in_area);
			}
		}
		// 如果有新的区域加载到缓存中，更新缓存
		if (cache_update)
		{
			cached_area_list_.clear();
			cached_area_list_ = tmp_area_list;

			sensor_msgs::PointCloud2 new_dynamic_pcd_map_msg;
			// 遍历缓存中的区域列表，将每个区域的点云数据合并到 new_dynamic_pcd_map_msg 中
			for (const MapCommonNS::Area &area : cached_area_list_)
			{
				if (new_dynamic_pcd_map_msg.width == 0)
				{
					new_dynamic_pcd_map_msg = area.points;
				}
				else
				{
					// 将当前区域的点云数据的宽度累加到 new_dynamic_pcd_map_msg 中
					new_dynamic_pcd_map_msg.width += area.points.width;
					// 将当前区域的点云数据的行步幅累加到 new_dynamic_pcd_map_msg 中
					new_dynamic_pcd_map_msg.row_step += area.points.row_step;
					// 将当前区域的点云数据的二进制数据插入到 new_dynamic_pcd_map_msg 的二进制数据的末尾，实现点云数据的合并
					new_dynamic_pcd_map_msg.data.insert(new_dynamic_pcd_map_msg.data.end(), area.points.data.begin(), area.points.data.end());
				}
			}
			current_dynamic_pcd_map_msg_ = new_dynamic_pcd_map_msg;
			ROS_INFO("[map_loader] ===>> Created New Pcd Points Map size = %d", current_dynamic_pcd_map_msg_.width);
		}
		else
		{
			ROS_WARN("[map_loader] ===>> Loaded Old Pcd Points Map size = %d", current_dynamic_pcd_map_msg_.width);
		}
		return current_dynamic_pcd_map_msg_;
	}

	MapCommonNS::AreaList MapLoader::loadNeedAreaList(const geometry_msgs::Point &point)
	{
		// 创建一个用于存储目标区域的列表
		MapCommonNS::AreaList ret;
		// 遍历默认区域列表
		for (MapCommonNS::Area &area : default_area_list_)
		{
			// 判断给定点是否在当前区域内（考虑了边缘范围）
			bool isIn = isInArea(point.x, point.y, area, margin_);
			if (isIn)
			{
				// 如果在区域内，创建一个当前区域的副本，并将其添加到目标区域列表中
				MapCommonNS::Area in_area(area);
				ret.push_back(in_area);
			}
		}
		// 返回包含目标区域的列表
		return ret;
	}

	int MapLoader::isInCachedAreaList(MapCommonNS::Area need_area)
	{
		// 获取当前缓存区域列表的长度
		int length = cached_area_list_.size();
		// 遍历缓存区域列表
		for (int i = 0; i < length; i++)
		{
			// 检查每个缓存区域的文件名是否与待检查的区域的文件名相同
			if (need_area.filename == cached_area_list_[i].filename)
			{
				// 如果找到相同的文件名，返回该区域在缓存列表中的索引
				return i;
			}
		}
		// 如果遍历完整个列表都没有找到相同的文件名，返回 -1 表示未找到
		return -1;
	}

	MapCommonNS::AreaList MapLoader::loadAreaList(const std::string &path)
	{
		// 打开文件流，以读取指定路径的文件
		std::ifstream ifs(path.c_str());
		// 用于存储每一行读取的数据
		std::string line;
		// 创建用于存储区域信息的容器
		MapCommonNS::AreaList ret;
		// 逐行读取文件内容
		while (std::getline(ifs, line))
		{
			// 使用istringstream将读取的行分解为逗号分隔的列
			std::istringstream iss(line);
			std::string col;
			std::vector<std::string> cols;
			// 逐列读取每一行的内容
			while (std::getline(iss, col, ','))
				cols.push_back(col);
			// 创建一个 Area 结构体实例，用于存储当前行的区域信息
			MapCommonNS::Area tmp;
			// 将每列的数据转换并存储到 Area 结构体中
			tmp.filename = cols[0];
			tmp.x_min = std::stod(cols[1]);
			tmp.y_min = std::stod(cols[2]);
			tmp.z_min = std::stod(cols[3]);
			tmp.x_max = std::stod(cols[4]);
			tmp.y_max = std::stod(cols[5]);
			tmp.z_max = std::stod(cols[6]);
			// 将当前行的区域信息添加到 AreaList 中
			ret.push_back(tmp);
		}
		// 返回存储所有区域信息的 AreaList
		return ret;
	}

	bool MapLoader::publishStaticPcdMap()
	{
		ros::Duration(3.0).sleep();
		pcl::PointCloud<pcl::PointXYZI>::Ptr pcd_map_filtered_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcd_map_filtered_));
		try
		{
			std::string static_filename = dir_static_map_ + "/static.pcd";
			pcl::io::savePCDFileBinary(static_filename, *pcd_map_filtered_ptr);
		}
		catch (const std::exception &e)
		{
			std::cerr << "An exception occurred: " << e.what() << std::endl;
		}
		// 创建一个sensor_msgs::PointCloud2指针，用于存储点云地图
		sensor_msgs::PointCloud2::Ptr static_map_msg(new sensor_msgs::PointCloud2);
		pcl::toROSMsg(*pcd_map_filtered_ptr, *static_map_msg);
		// 设置点云消息的坐标系为 "map"
		static_map_msg->header.frame_id = "map";
		// 通过发布器发布点云地图消息
		pub_static_points_map_.publish(*static_map_msg);
		return true;
	}

	bool MapLoader::publishDynamicPcdMap()
	{
		bool load_flag = true;
		if (current_dynamic_pcd_map_msg_.width != 0)
		{
			current_dynamic_pcd_map_msg_.header.frame_id = "map";
			pub_dynamic_points_map_.publish(current_dynamic_pcd_map_msg_);
			ROS_INFO("[map_loader] Publish Dynamic Map with %d Points", current_dynamic_pcd_map_msg_.width);
			map_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::READY);
			load_flag = true;
		}
		else
		{
			ROS_ERROR("[map_loader] No Points To Publish %d", current_dynamic_pcd_map_msg_.width);
			map_status_.module_status = static_cast<int>(HtcbotCommonNS::STATUS_TYPE::EXC);
			map_status_.reason = "Map No Points To Publish";
			load_flag = false;
		}
		pub_module_status_.publish(map_status_);
		return load_flag;
	}

	bool MapLoader::isInArea(double x, double y, const MapCommonNS::Area &area, double m)
	{
		return ((area.x_min - m) <= x && x <= (area.x_max + m) && (area.y_min - m) <= y && y <= (area.y_max + m));
	}

}