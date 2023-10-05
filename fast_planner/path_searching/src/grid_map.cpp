#include <path_searching/grid_map.h>;
std::vector<int> GridMap::WorldToMap(double wx, double wy, Eigen::Vector3d offset_) {
	ROS_INFO("\nPOS_fastplanner:\nx:%f y:%f\n",wx,wy);
	std::vector<int> v;
	if (wx < (1.0 * origin_x) || wy < (1.0 * origin_y)) {
		v.push_back(-1);
		v.push_back(-1);
		return v;
	}
	int my = int((1.0 * (wx - origin_x-offset_(0))) / resolution);
	int mx = -1*int((1.0 * (wy - origin_y-offset_(1))) / resolution);
	
	if (mx < width && my < height) {
		v.push_back(my);
		v.push_back(mx);
		return v;
	}
}

// 地图-->世界坐标系
std::vector<double> GridMap::MapToWorld(double my, double mx) {
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
int GridMap::posToIndex(Eigen::Vector3d pos, Eigen::Vector3d offset_){
    std::vector<int> temp;
    temp = WorldToMap(pos(0),pos(1),offset_);
    return temp[0]*width + temp[1];
}
int GridMap::getInflateOccupancy(Eigen::Vector3d pos, Eigen::Vector3d offset_) {
    if(mapData[posToIndex(pos,offset_)]<0){
        return 0;
    }
    return (mapData[posToIndex(pos,offset_)] == 100? 1 : 0);
	
}