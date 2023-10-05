#include <path_searching/grid_map.h>;
std::vector<int> GridMap::WorldToMap(double wx, double wy) {
	std::vector<int> v;
	if (wx < (1.0 * origin_x) || wy < (1.0 * origin_y)) {
		v.push_back(-1);
		v.push_back(-1);
		return v;
	}
	int mx = int((1.0 * (wx - origin_x)) / resolution);
	int my = int((1.0 * (wy - origin_y)) / resolution);
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
int GridMap::posToIndex(Eigen::Vector3d pos){
    std::vector<int> temp;
    temp = WorldToMap(pos(0),pos(1));
    return temp[0]*width + temp[1];
}
int GridMap::getInflateOccupancy(Eigen::Vector3d pos) {
    return (mapData[posToIndex(pos)] == 100? 1 : 0);
	
}