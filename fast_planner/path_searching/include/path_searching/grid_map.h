#pragma once
#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
class GridMap{
public:
    int origin_x, origin_y, resolution, width, height;
    std::vector<int> mapData;
    int getInflateOccupancy(Eigen::Vector3d pos);
private:
    int posToIndex(Eigen::Vector3d pos);
    std::vector<int> WorldToMap(double wx, double wy);
    std::vector<double> MapToWorld(double my, double mx);
};