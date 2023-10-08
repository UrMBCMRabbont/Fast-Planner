#pragma once
#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <plan_env/edt_environment.h>
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
#include "plan_env/edt_environment.h"
class GridMap{
public:
    int origin_x, origin_y, width, height;
    float resolution;
    std::vector<int> mapData;
    int getInflateOccupancy(Eigen::Vector3d pos,Eigen::Vector3d offset_);
private:
    int posToIndex(Eigen::Vector3d pos, Eigen::Vector3d offset_);
    std::vector<int> WorldToMap(double wx, double wy, Eigen::Vector3d offset_);
    std::vector<double> MapToWorld(double my, double mx);
};
extern GridMap global_map;