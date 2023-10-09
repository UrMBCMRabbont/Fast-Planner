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
#define GENERIC_MAX(x, y) ((x) > (y) ? (x) : (y))
class GridMap{
public:
    int origin_x, origin_y, width, height;
    float resolution;
    std::vector<int> mapData;
    std::vector<int> obstacle_idx;
    std::vector<std::pair<float,float>> table_seq;
    inline int getInflateOccupancy(Eigen::Vector3d pos,Eigen::Vector3d offset_);
    inline std::vector<int> getObstclesIdx(Eigen::Vector3d pos, Eigen::Vector3d offset_);
    inline Eigen::Vector3d IndexToPos(int idx,Eigen::Vector3d cam_pos);
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
inline Eigen::Vector3d GridMap::IndexToPos(int idx,Eigen::Vector3d cam_pos){
    Eigen::Vector3d pos;
    pos(1) = (1.0*(idx/width))*resolution;
    pos(0) = (1.0*(idx%width))*resolution;
    pos(2) = 0;

	return pos;
}

inline std::vector<int> GridMap::getObstclesIdx(Eigen::Vector3d pos, Eigen::Vector3d offset_){
    int l = 0, r = obstacle_idx.size()-1;
    std::vector<int>::iterator start = obstacle_idx.begin()-1, end;
    std::vector<int> temp;
    temp.clear();
    Eigen::Vector3d grid;
    //smallest grid idx
    grid(0) = GENERIC_MAX(pos(0)-offset_(0)/2, 0);
    grid(1) = GENERIC_MAX(pos(1)-offset_(1)/2, 0);
    if(!posToIndex(grid,grid)){
        return temp;
    }
    int x = grid_idx;

    grid(0) = pos(0)+offset_(0);
    grid(1) = pos(1)+offset_(1);
    if(!posToIndex(grid,grid)){
        return temp;
    }
    //binary search
     while (l <= r) {
        int m = l + (r - l) / 2;
 
        // Check if x is present at mid
        if (obstacle_idx[m] == x||(obstacle_idx[m-1]<x && x<obstacle_idx[m]))
            if(start == obstacle_idx.begin()-1){
                start = obstacle_idx.begin()+m;
                l = m+1;
                r = obstacle_idx.size()-1;
                x = grid_idx;
            }else{
                end = obstacle_idx.begin()+m+(obstacle_idx[m] == x?1:0);
                temp.assign(start,end);
                return temp;
            }
            
 
        // If x greater, ignore left half
        if (obstacle_idx[m] < x)
            l = m + 1;
 
        // If x is smaller, ignore right half
        else
            r = m - 1;
    }
    return temp;
}
inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos, Eigen::Vector3d offset_) {

    if(!posToIndex(pos,offset_)){
        return 0;
    }
	ROS_INFO("map grid val:%d",mapData[grid_idx]);
    return (mapData[grid_idx] == 100? 1 : 0);
}
extern GridMap global_map;