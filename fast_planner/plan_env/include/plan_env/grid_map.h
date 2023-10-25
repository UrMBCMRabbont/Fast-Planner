#pragma once
#include <costmap_2d/costmap_2d.h>
#include <costmap_converter/costmap_converter_interface.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <boost/functional/hash.hpp>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#define GENERIC_MAX(x, y) ((x) > (y) ? (x) : (y))
#define GENERIC_MIN(x, y) ((x) < (y) ? (x) : (y))
class GridMap {
public:
	int width, height, init_done = 0;
	float resolution, origin_x, origin_y;
	std::vector<int> mapData;
	std::vector<int> obstacle_idx;
	std::vector<std::pair<float, float>> table_seq;
	inline int getInflateOccupancy(Eigen::Vector3d pos, Eigen::Vector3d offset_);
	inline std::vector<int> getObstclesIdx(Eigen::Vector3d pos, Eigen::Vector3d offset_);
	inline Eigen::Vector3d IndexToPos(int idx, Eigen::Vector3d cam_pos);
	inline std::pair<float, float> TablePosCalc(double mx, double my);
	costmap_converter::ObstacleArrayConstPtr obstacles_arr;

private:
	inline int posToIndex(Eigen::Vector3d pos, Eigen::Vector3d offset_);
	inline std::vector<int> WorldToMap(double wx, double wy, Eigen::Vector3d offset_);
	inline std::vector<double> MapToWorld(double my, double mx);
	int grid_idx = 0;
};
inline std::pair<float, float> GridMap::TablePosCalc(double mx, double my) {
	if (!init_done) {
		ROS_INFO("That's why u shit on9!]\n");
	}
	std::pair<float, float> temp;
	temp.first = (mx - origin_x);
	temp.second = -(my - origin_y);
	ROS_INFO("Shit:%f %f || ori:%f %f\n", temp.first, temp.second, origin_x, origin_y);
	return temp;
}

inline std::vector<int> GridMap::WorldToMap(double wx, double wy, Eigen::Vector3d offset_) {
	// ROS_INFO("\nPOS_fastplanner:\nx:%f y:%f\nOrigin:\n%f,%f\nres:%f",wx,wy,origin_x,origin_y,resolution);
	std::vector<int> v;
	if (wx < (1.0 * origin_x) || wy < (1.0 * origin_y)) {
		v.push_back(-1);
		v.push_back(-1);
		return v;
	}
	// int my = int((1.0 * (wx - float(origin_x))) / resolution);
	// int mx = int((1.0 * (wy - float(origin_y))) / resolution);
	int my = int((1.0 * (wx)) / resolution);
	int mx = int((1.0 * (wy)) / resolution);
	// int my = int((1.0 * (wx)) );
	// int mx = int((1.0 * (wy)) );
	// ROS_INFO("\nPOS_after_clac:\nx:%d y:%d\n [%f,%f]",mx,my,origin_x,origin_y);

	if (mx < width && my < height) {
		v.push_back(mx);
		v.push_back(my);
		return v;
	}
}

// 地图-->世界坐标系
inline std::vector<double> GridMap::MapToWorld(double my, double mx) {
	std::vector<double> v;
	if (mx > width || my > height) {
		v.push_back(-1);
		v.push_back(-1);
		return v;
	}
	double wx = (mx * resolution + origin_x);
	double wy = (my * resolution + origin_y);
	if (wx > origin_x && wy > origin_y) {
		v.push_back(wx);
		v.push_back(wy);
		return v;
	}
}
inline int GridMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3d offset_) {
	std::vector<int> temp;
	temp = WorldToMap(pos(0), pos(1), offset_);
	if (temp[0] < 0 || temp[1] < 0) {
		return 0;
	}
	grid_idx = temp[0] * width + temp[1];
	return 1;
}
inline Eigen::Vector3d GridMap::IndexToPos(int idx, Eigen::Vector3d cam_pos) {
	Eigen::Vector3d pos;
	pos(1) = (1.0 * (idx / width)) * resolution;
	pos(0) = (1.0 * (idx % width)) * resolution;
	pos(2) = 0;

	return pos;
}

inline std::vector<int> GridMap::getObstclesIdx(Eigen::Vector3d pos, Eigen::Vector3d offset_) {
	int l = 0, r = obstacle_idx.size() - 1;
	std::vector<int>::iterator start = obstacle_idx.begin() - 1, end;
	std::vector<int> temp;
	temp.clear();
	Eigen::Vector3d grid;
	// smallest grid idx
	grid(0) = GENERIC_MAX(pos(0) - offset_(0), 0);
	grid(1) = GENERIC_MAX(pos(1) - offset_(1), 0);
	if (!posToIndex(grid, grid)) {
		ROS_INFO("You shit at min: %f %f\n", grid(0), grid(1));
		return temp;
	}
	ROS_INFO("You good at min: %f %f\n", grid(0), grid(1));
	int x = grid_idx;

	grid(0) = GENERIC_MIN(pos(0) + offset_(0), width * resolution);
	grid(1) = GENERIC_MIN(pos(1) + offset_(1), height * resolution);
	if (!posToIndex(grid, grid)) {
		ROS_INFO("You shit at max: %f %f\n", grid(0), grid(1));
		return temp;
	}
	ROS_INFO("You good at max: %f %f\n", grid(0), grid(1));
	static int add_zero = 0;
	if (obstacle_idx[0] != 0) {
		obstacle_idx.insert(obstacle_idx.begin(), 0);
		add_zero = 1;
	}
	if (obstacle_idx[obstacle_idx.size() - 1] != (width * height - 1)) {
		obstacle_idx.push_back(width * height - 1);
	}
	// binary search
	while (l <= r) {
		int m = l + (r - l) / 2;

		// Check if x is present at mid
		if (obstacle_idx[m] == x || (obstacle_idx[m] < x && x < obstacle_idx[m + 1])) {
			if (start == obstacle_idx.begin() - 1) {
				start = obstacle_idx.begin() + m;
				l = m;
				r = obstacle_idx.size() - 1;
				x = grid_idx;
				std::cout << "m:" << m << " You can find 1st idx\n";
				continue;
			} else {
				end = obstacle_idx.begin() + m +
					  ((obstacle_idx[m] == x || (obstacle_idx[m] < x && x < obstacle_idx[m + 1])) ? 1 : 0);
				if (start == end) {
					temp.push_back(obstacle_idx[m]);
				} else {
					temp.assign(start, end);
				}
				std::cout << "m:" << m << " You can find 2nd idx\n";
				if (add_zero && temp[0] == 0) {
					temp.erase(temp.begin());
				}
				return temp;
			}
		}

		// If x greater, ignore left half
		if (obstacle_idx[m] < x) l = m + 1;

		// If x is smaller, ignore right half
		else
			r = m - 1;
	}
	ROS_INFO("You shit at u cannot find\n");
	if (start != obstacle_idx.begin() - 1) {
		temp.assign(start, obstacle_idx.end());
	}
	return temp;
}
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, Eigen::Vector3d offset_) {
	if (!posToIndex(pos, offset_)) {
		return 0;
	}
	// ROS_INFO("map grid val:%d",mapData[grid_idx]);
	return (mapData[grid_idx] == 100 ? 1 : 0);
}
extern GridMap global_map;