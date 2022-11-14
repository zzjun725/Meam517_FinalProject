//
// Created by zzjun on 8/10/22.
//

#ifndef SRC_A_STAR_3D_H
#define SRC_A_STAR_3D_H

#include "path_finder/utils/type.h"
#include "path_finder/map_tools/grid_node.h"
#include "path_finder/map_tools/map_tool.h"

using std::vector;

class AStar3D;

typedef typename std::shared_ptr<AStar3D> AStar3DPTR;


class AStar3D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    AStar3D() = default;

    ~AStar3D();

    void initGridMap(double x_lower, double x_upper, double y_lower, double y_upper, double z_lower, double z_upper,
                     double resolution);

    void Reset();

    bool Search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &goal_pt);
    GridMap3DPTR gridmap3Dptr;


    TypeVectorVecd<3> GetPath() const;


    TypeVectorVecd<3> GetVisitedNodeCoord() const;


    void updateMapWithPCL(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr);

protected:

    void GetNeighborNodes(const GridNodePtr<3> &curr_node_ptr,
                          std::vector<GridNodePtr<3>> &neighbor_nodes,
                          std::vector<double> &neighbor_edge_costs);

    inline double GetHeuristicValue(GridNodePtr<3> node_1_ptr, GridNodePtr<3> node_2_ptr) const;

    inline double ManhattanHeu(const Vec3d &coord_1, const Vec3d &coord_2) const;

    __attribute__((unused)) inline double EuclideanHeu(const Vec3d &coord_1, const Vec3d &coord_2) const;

    void ReleaseMemory();

protected:
//    GridMap3D gridMap;

    int GRID_X_SIZE_{}, GRID_Y_SIZE_{}, GRID_Z_SIZE_{};
    GridNodePtr<3> terminal_node_ptr_ = nullptr;
    vector<vector<vector<GridNodePtr<3>>>> grid_node_map_{};
    std::multimap<double, GridNodePtr<3>> openset_;

};

#endif //SRC_A_STAR_3D_H

