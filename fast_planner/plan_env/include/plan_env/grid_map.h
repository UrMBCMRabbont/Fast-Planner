#pragma once
#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>
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
class GridMap{
public:
    int origin_x, origin_y, width, height;
    float resolution;
    std::vector<int> mapData;
    inline int getInflateOccupancy(Eigen::Vector3d pos,Eigen::Vector3d offset_);
private:
    inline int posToIndex(Eigen::Vector3d pos, Eigen::Vector3d offset_);
    inline std::vector<int> WorldToMap(double wx, double wy, Eigen::Vector3d offset_);
    inline std::vector<double> MapToWorld(double my, double mx);
    int grid_idx = 0;
};
inline std::vector<int> GridMap::WorldToMap(double wx, double wy, Eigen::Vector3d offset_) {
	ROS_INFO("\nPOS_fastplanner:\nx:%f y:%f\nOrigin:\n%d,%d\nres:%f",wx,wy,origin_x,origin_y,resolution);
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
	ROS_INFO("\nPOS_after_clac:\nx:%d y:%d\n",mx,my);
	
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
inline int GridMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3d offset_){
    std::vector<int> temp;
    temp = WorldToMap(pos(0),pos(1),offset_);
	if(temp[0]<0||temp[1]<0){
		return 0;
	}
    grid_idx = temp[0]*width + temp[1];
	return 1;
}
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, Eigen::Vector3d offset_) {

    if(!posToIndex(pos,offset_)){
        return 0;
    }
	ROS_INFO("map grid val:%d",mapData[grid_idx]);
    return (mapData[grid_idx] == 100? 1 : 0);
}
extern GridMap global_map;