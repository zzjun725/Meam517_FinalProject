//
// Created by zzjun on 8/22/22.
//

#include "path_finder/map_tools/map_tool.h"


void GridMap3D::InitMapWithPCL(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr) {
    pcl::PointCloud<pcl::PointXYZ> point_cloud_map;
    pcl::fromROSMsg(*point_cloud_ros_ptr, point_cloud_map);

    CHECK(!point_cloud_map.points.empty());
    point_cloud_map_vis.points.clear();
    clearMapData();

    for (auto pt: point_cloud_map.points) {
        setObstacle(pt.x, pt.y, pt.z);
        Vec3d pt_rounding = coorRounding(Vec3d(pt.x, pt.y, pt.z));
        pt.x = static_cast<float>(pt_rounding.x());
        pt.y = static_cast<float>(pt_rounding.y());
        pt.z = static_cast<float>(pt_rounding.z());
        // std::cout << pt.z;

        point_cloud_map_vis.points.emplace_back(pt);
    }

    point_cloud_map_vis.width = point_cloud_map_vis.size();
    point_cloud_map_vis.height = 1;
    point_cloud_map_vis.is_dense = true;
}

int GridMap3D::getFlattenIdx(Eigen::Vector3i &grid_idx) const {
    return grid_idx[0] * (GRID_Y_SIZE_*GRID_Z_SIZE_) + grid_idx[1]*GRID_Z_SIZE_ + grid_idx[2];
}

int GridMap3D::getFlattenIdx(int grid_idx_x, int grid_idx_y, int grid_idx_z) const {
    return grid_idx_x * (GRID_Y_SIZE_*GRID_Z_SIZE_) + grid_idx_y*GRID_Z_SIZE_ + grid_idx_z;
}

bool GridMap3D::withinMap(Eigen::Vector3i &grid_idx) const {
    int grid_idx_x = grid_idx[0];
    int grid_idx_y = grid_idx[1];
    int grid_idx_z = grid_idx[2];
    return grid_idx_x >= 0 && grid_idx_x < GRID_X_SIZE_ && grid_idx_y >= 0 && grid_idx_y < GRID_Y_SIZE_ &&
           grid_idx_z >= 0 && grid_idx_z < GRID_Z_SIZE_;
}

bool GridMap3D::withinMap(int grid_idx_x, int grid_idx_y, int grid_idx_z) {
    return grid_idx_x >= 0 && grid_idx_x < GRID_X_SIZE_ && grid_idx_y >= 0 && grid_idx_y < GRID_Y_SIZE_ &&
           grid_idx_z >= 0 && grid_idx_z < GRID_Z_SIZE_;
}

void GridMap3D::setObstacle(double pt_x, double pt_y, double pt_z) {
    if (pt_x < x_lower_ || pt_x > x_upper_ ||
        pt_y < y_lower_ || pt_y > y_upper_ ||
        pt_z < z_lower_ || pt_z > z_upper_) {
        return;
    }
    int grid_idx_x = static_cast<int>((pt_x - x_lower_) / resolution_);
    int grid_idx_y = static_cast<int>((pt_y - y_lower_) / resolution_);
    int grid_idx_z = static_cast<int>((pt_z - z_lower_) / resolution_);
    map_data_[getFlattenIdx(grid_idx_x, grid_idx_y, grid_idx_z)] = 1;
}

bool GridMap3D::hasObstacle(Eigen::Vector3i &grid_idx) {
    if (map_data_[getFlattenIdx(grid_idx)]==1)
        return true;
    else
        return false;
}

bool GridMap3D::hasObstacle(int grid_idx_x, int grid_idx_y, int grid_idx_z) {
    if (map_data_[getFlattenIdx(grid_idx_x, grid_idx_y, grid_idx_z)]==1)
        return true;
    else
        return false;
}

bool GridMap3D::infreeSpace(Eigen::Vector3i &grid_idx, bool onlyCenter) {
    if(!withinMap(grid_idx) || hasObstacle(grid_idx))
        return false;
    if(!onlyCenter)
    {
        for(auto dir: search_directions)
        {
            Eigen::Vector3i neighbor_grid_idx = grid_idx + Eigen::Vector3i(dir[0], dir[1], dir[2]);
            if (!withinMap(neighbor_grid_idx) || hasObstacle(neighbor_grid_idx))
                return false;
        }
    }
    return true;
}


Eigen::Vector3d GridMap3D::gridIdx2Coor(const Eigen::Vector3i &grid_idx) const {
    Eigen::Vector3d pt;
    pt.x() = ((double) grid_idx[0] + 0.5) * resolution_ + x_lower_;
    pt.y() = ((double) grid_idx[1] + 0.5) * resolution_ + y_lower_;
    pt.z() = ((double) grid_idx[2] + 0.5) * resolution_ + z_lower_;
    return pt;
}

Eigen::Vector3i GridMap3D::Coor2gridIdx(const Eigen::Vector3d &pt) const {
    // TODO: will treat any out of bound point as in the boundary
    Eigen::Vector3i grid_idx;

    grid_idx[0] = std::min(std::max(int((pt.x() - x_lower_) / resolution_), 0), GRID_X_SIZE_ - 1);
    grid_idx[1] = std::min(std::max(int((pt.y() - y_lower_) / resolution_), 0), GRID_Y_SIZE_ - 1);
    grid_idx[2] = std::min(std::max(int((pt.z() - z_lower_) / resolution_), 0), GRID_Z_SIZE_ - 1);

    return grid_idx;
}


Eigen::Vector3d GridMap3D::coorRounding(const Eigen::Vector3d &pt) {
    return gridIdx2Coor(Coor2gridIdx(pt));
}

void GridMap3D::Init(double x_lower, double x_upper, double y_lower, double y_upper, double z_lower, double z_upper,
                     double resolution) {
    x_lower_ = x_lower;
    x_upper_ = x_upper;
    y_lower_ = y_lower;
    y_upper_ = y_upper;
    z_lower_ = z_lower;
    z_upper_ = z_upper;

    resolution_ = resolution;

    GRID_X_SIZE_ = std::floor((x_upper_ - x_lower_) / resolution_);
    GRID_Y_SIZE_ = std::floor((y_upper_ - y_lower_) / resolution_);
    GRID_Z_SIZE_ = std::floor((z_upper_ - z_lower_) / resolution_);

    clearMapData();
    initSearchDirection();

}



void GridMap3D::initSearchDirection() {
    if (search_directions.size()>1) return;
    for(int x=-1; x<=1; x++)
    {
        for(int y=-1; y<=1; y++)
        {
            for(int z=-1; z<=1; z++)
            {
                if(x==0 && y==0 && z==0) continue;
                search_directions.push_back({x, y, z});
            }
        }
    }
}

void GridMap3D::releaseMemory() {
    delete[] map_data_;
    map_data_ = nullptr;
}


GridMap3D::~GridMap3D() {
    releaseMemory();
}

void GridMap3D::clearMapData() {
    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }
    map_data_ = new uint8_t[GRID_Y_SIZE_ * GRID_X_SIZE_ * GRID_Z_SIZE_];
    memset(map_data_, 0, GRID_X_SIZE_ * GRID_Y_SIZE_  * GRID_Z_SIZE_ * sizeof(uint8_t));
}
