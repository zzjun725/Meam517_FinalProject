//
// Created by zzjun on 8/22/22.
//

#ifndef SRC_CORRIDOR_H
#define SRC_CORRIDOR_H

#include <Eigen/Eigen>
#include "path_finder/map_tools/grid_node.h"
#include "path_finder/map_tools/map_tool.h"
#include "path_finder/utils/type.h"
#include "path_finder/btraj/Cube.h"

#include <unordered_set>

using namespace std;
using namespace Eigen;

class CubeCorridor
{

public:
//    void updateMapWithPCL(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr);
//    void initGridMap(double x_lower, double x_upper, double y_lower, double y_upper, double z_lower, double z_upper,
//                     double resolution);
    CubeCorridor()=delete;
    CubeCorridor(int max_inflate_iter, int step_length);
    void inflate_path(TypeVectorVecd<3>& path);
    Cube inflate_cube(Cube& cube);
    Cube initCube9Grid(Eigen::Vector3d &pt);
    unordered_set<int> get_pathPt_inside_cube(TypeVectorVecd<3>& path, Cube cubic);
    bool pt_inside_cubic(Eigen::Vector3d &pt, Cube &cubic);
    vector<Cube> cubicList{};

    void initGridMap(double x_lower, double x_upper, double y_lower, double y_upper, double z_lower, double z_upper,
                                   double resolution);
    void updateMapWithPCL(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr);

private:
    //inflate
    int _max_inflate_iter;
    int _step_length;

    // for time allocation
    vector<double> times;
    double vx_max;
    double vx_min;
    double vy_max;
    double vy_min;
    double vz_max;
    double vz_min;
    double ax_max;
    double ax_min;
    double ay_max;
    double ay_min;
    double az_max;
    double az_min;

    // global map
    GridMap3DPTR gridmap3Dptr;
    int GRID_X_SIZE_{}, GRID_Y_SIZE_{}, GRID_Z_SIZE_{};
    int _max_x_id{}, _max_y_id{}, _max_z_id{};
    double x_lower_{}, x_upper_{};
    double y_lower_{}, y_upper_{};
    double z_lower_{}, z_upper_{};

public:
    // interface
    Vector3d start;
    Vector3d goal;
    Vector3d start_v;
    Vector3d goal_v;
    Vector3d start_a;
    Vector3d goal_a;



};

#endif //SRC_CORRIDOR_H
