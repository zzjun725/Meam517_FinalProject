//
// Created by zzjun on 8/10/22.
//

#ifndef SRC_MAP_TOOL_H
#define SRC_MAP_TOOL_H
#include "path_finder/utils/type.h"
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <glog/logging.h>
#include "path_finder/map_tools/grid_node.h"

using std::vector;
class GridMap3D;

typedef typename std::shared_ptr<GridMap3D> GridMap3DPTR;

class GridMap3D
{
public:
    int GRID_X_SIZE_{}, GRID_Y_SIZE_{}, GRID_Z_SIZE_{};

    double x_lower_{}, x_upper_{};
    double y_lower_{}, y_upper_{};
    double z_lower_{}, z_upper_{};
    double resolution_{};

    uint8_t *map_data_ = nullptr;

    vector<vector<int>> search_directions{};

    // PCL map
    pcl::PointCloud<pcl::PointXYZ> point_cloud_map_vis;

public:

    GridMap3D()=default;
    ~GridMap3D();
    void Init(double x_lower, double x_upper, double y_lower, double y_upper, double z_lower, double z_upper,
              double resolution);
    void clearMapData();
    void InitMapWithPCL(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr);

    //
    inline int getFlattenIdx(Eigen::Vector3i &grid_idx) const;
    inline int getFlattenIdx(int grid_idx_x, int grid_idx_y, int grid_idx_z) const;
    Eigen::Vector3d gridIdx2Coor(const Eigen::Vector3i &grid_idx) const;
    Eigen::Vector3i Coor2gridIdx(const Eigen::Vector3d &pt) const;
    inline Eigen::Vector3d coorRounding(const Eigen::Vector3d &pt);
    void initSearchDirection();

    //
    inline void setObstacle(double pt_x, double pt_y, double pt_z);
    inline bool withinMap(Eigen::Vector3i &grid_idx) const;
    inline bool withinMap(int grid_idx_x, int grid_idx_y, int grid_idx_z);
    inline bool hasObstacle(Eigen::Vector3i &grid_idx);
    inline bool hasObstacle(int grid_idx_x, int grid_idx_y, int grid_idx_z);
    bool infreeSpace(Eigen::Vector3i &grid_idx, bool onlyCenter);

    //
    void releaseMemory();

};

#endif //SRC_MAP_TOOL_H
