#pragma once
#include <ros/ros.h>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
class GridMap{
    public:
    int origin_x, origin_y, resolution, width, height;
    std::vector<int> mapData;
    
};