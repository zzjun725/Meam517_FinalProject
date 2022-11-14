//
// Created by zzjun on 8/10/22.
//
#include <iostream>
#include "path_finder/a_star/a_star_3d.h"
#include "path_finder/map_tools/map_tool.h"
#include <ros/ros.h>


AStar3D::~AStar3D() {
    ReleaseMemory();
}

void AStar3D::Reset() {
    for (int i=0; i < GRID_X_SIZE_; ++i) {
        for (int j=0; j < GRID_Y_SIZE_; ++j) {
           for (int k=0; k < GRID_Z_SIZE_; ++k)
           {
               // std::cout << "reset GridNode at " << i << " " << j << " " << k << std::endl;
               grid_node_map_[i][j][k]->Reset();
           }
        }
    }
    ROS_INFO_STREAM("Reset all nodes for Astar");
}


bool AStar3D::Search(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &goal_pt) {
    // Reset();
    ros::Time start_time = ros::Time::now();

    const Eigen::Vector3i start_grid_index = gridmap3Dptr->Coor2gridIdx(start_pt);
    const Eigen::Vector3i goal_grid_index = gridmap3Dptr->Coor2gridIdx(goal_pt);

//    std::cout<< "init start and goal"<<std::endl;
//
//    std::cout << "Start node grid index: " << "x: " << start_grid_index[0] << "y: " << start_grid_index[1]
//    << "z: " << start_grid_index[2] << std::endl;
//
//    std::cout << "Goal node grid index: " << "x: " << goal_grid_index[0] << "y: " << goal_grid_index[1]
//              << "z: " << goal_grid_index[2] << std::endl;

    GridNodePtr<3> start_node_ptr = grid_node_map_[start_grid_index[0]][start_grid_index[1]][start_grid_index[2]];
    const GridNodePtr<3> goal_node_ptr = grid_node_map_[goal_grid_index[0]][goal_grid_index[1]][start_grid_index[2]];


    start_node_ptr->g_score_ = 0.0;
    start_node_ptr->f_score_ = GetHeuristicValue(start_node_ptr, goal_node_ptr);
    start_node_ptr->status_ = NODE_STATUS::IN_OPENSET;

    openset_.clear();
    openset_.insert(std::make_pair(start_node_ptr->f_score_, start_node_ptr));

    std::vector<GridNodePtr<3>> neighbor_nodes;
    std::vector<double> neighbor_edge_cost;

    GridNodePtr<3> current_node_ptr;
    GridNodePtr<3> neighbor_node_ptr = nullptr;
    while (!openset_.empty()) {
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->status_ = NODE_STATUS::IN_CLOSESET;
        openset_.erase(openset_.begin());

        if (current_node_ptr->grid_index_ == goal_grid_index) {
            ros::Time end_time = ros::Time::now();
            terminal_node_ptr_ = current_node_ptr;
            ROS_INFO("\033[1;32m --> Time in A star is %f ms, path cost %f m \033[0m",
                     (end_time - start_time).toSec() * 1000.0, current_node_ptr->g_score_ * gridmap3Dptr->resolution_);

            return true;
        }

        GetNeighborNodes(current_node_ptr, neighbor_nodes, neighbor_edge_cost);

        for (unsigned int i = 0; i < neighbor_nodes.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes[i];

            if (neighbor_node_ptr->status_ == NODE_STATUS::NOT_VISITED) {
                neighbor_node_ptr->g_score_ = current_node_ptr->g_score_ + neighbor_edge_cost[i];
                neighbor_node_ptr->f_score_ =
                        GetHeuristicValue(neighbor_node_ptr, goal_node_ptr) + neighbor_node_ptr->g_score_;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->status_ = NODE_STATUS::IN_OPENSET;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                continue;
            } else if (neighbor_node_ptr->status_ == NODE_STATUS::IN_OPENSET) {
                double g_score_temp = current_node_ptr->g_score_ + neighbor_edge_cost[i];
                if (neighbor_node_ptr->g_score_ > g_score_temp) {
                    neighbor_node_ptr->g_score_ = g_score_temp;
                    neighbor_node_ptr->f_score_ = g_score_temp + GetHeuristicValue(neighbor_node_ptr, goal_node_ptr);
                    neighbor_node_ptr->parent_node_ = current_node_ptr;

                    auto map_iter = openset_.begin();
                    for (; map_iter != openset_.end(); map_iter++) {
                        if (map_iter->second->grid_index_ == neighbor_node_ptr->grid_index_) {
                            openset_.erase(map_iter);
                            openset_.insert(std::make_pair(neighbor_node_ptr->f_score_, neighbor_node_ptr));
                            break;
                        }
                    }
                }
                continue;
            } else { // In closeset! In fact, this judgment will never take effect,
                // but for code consistency, it is retained!
                continue;
            }
        }

        ros::Time end_time = ros::Time::now();

        //this search timed out!
        if ((end_time - start_time).toSec() * 1000.0 > 2000) {
            ROS_WARN_STREAM("Time out! more than 2000ms");
            return false;
        }
    }

    return false;
}

double AStar3D::GetHeuristicValue(GridNodePtr<3> node_1_ptr, GridNodePtr<3> node_2_ptr) const {
    return ManhattanHeu(node_1_ptr->grid_index_.cast<double>(), node_2_ptr->grid_index_.cast<double>());
}

double AStar3D::ManhattanHeu(const Vec3d &coord_1, const Vec3d &coord_2) const {
    return (coord_1 - coord_2).lpNorm<1>();
}

__attribute__((unused)) double AStar3D::EuclideanHeu(const Vec3d &coord_1, const Vec3d &coord_2) const {
    return (coord_1 - coord_2).norm();
}

//__attribute__((unused)) double AStar3D::DiagonalHeu(const Vec3d &coord_1, const Vec3d &coord_2) const {
//    double dx = std::fabs(coord_2[0] - coord_1[0]);
//    double dy = std::fabs(coord_2[1] - coord_1[1]);
//
//    double h;
//    h = -0.5857864 * std::min(dx, dy) + (dx + dy);
//
//    return tie_breaker_ * h;
//}

TypeVectorVecd<3> AStar3D::GetPath() const {
    TypeVectorVecd<3> path;
    std::vector<GridNodePtr<3>> temp_nodes;

    GridNodePtr<3> grid_node_ptr = terminal_node_ptr_;
    while (grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(grid_node_ptr);
        grid_node_ptr = grid_node_ptr->parent_node_;
    }

    for (const auto &node: temp_nodes) {
        path.emplace_back(node->coordinate_);
    }

    std::reverse(path.begin(), path.end());

    return path;
}



void AStar3D::GetNeighborNodes(const GridNodePtr<3> &curr_node_ptr, std::vector<GridNodePtr<3>> &neighbor_nodes,
                               std::vector<double> &neighbor_edge_costs) {
    neighbor_nodes.clear();
    neighbor_edge_costs.clear();
//    vector<vector<int>> vh_directions={{0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 0, -1}, {0, -1, 0}, {-1, 0, 0}};
//     vector<vector<int>> diag_direnctions={{0, 1, 1}, {0, -1, -1}, {1, 1, 0}, {-1, -1, 0}, {1, 0, 1}, {-1, 0, -1}, {}};
    for (auto dir: gridmap3Dptr->search_directions)
    {
        Eigen::Vector3i neighbor_node_index = curr_node_ptr->grid_index_ + Eigen::Vector3i(dir[0], dir[1], dir[2]);
        if (!gridmap3Dptr->infreeSpace(neighbor_node_index, false))
            continue;

        GridNodePtr<3> neighbor_node_ptr = grid_node_map_[neighbor_node_index[0]][neighbor_node_index[1]][neighbor_node_index[2]];
        if (neighbor_node_ptr->status_ == NODE_STATUS::IN_CLOSESET) {
            continue;
        }
        neighbor_nodes.emplace_back(neighbor_node_ptr);
        double neighbor_edge_cost = (neighbor_node_ptr->grid_index_.cast<double>() -
                                     curr_node_ptr->grid_index_.cast<double>()).norm();
        neighbor_edge_costs.emplace_back(neighbor_edge_cost);
    }
}

void AStar3D::ReleaseMemory() {
    for(int i=0; i<GRID_X_SIZE_; i++)
    {
        for(int j=0; j<GRID_Y_SIZE_; j++)
        {
            for(int k=0; k<GRID_Z_SIZE_; k++)
            {
                delete grid_node_map_[i][j][k];
                grid_node_map_[i][j][k] = nullptr;
            }
        }
    }
    terminal_node_ptr_ = nullptr;
}

TypeVectorVecd<3> AStar3D::GetVisitedNodeCoord() const {
    TypeVectorVecd<3> visited_nodes_coord;
    for (int i = 0; i < GRID_X_SIZE_; i++) {
        for (int j = 0; j < GRID_Y_SIZE_; ++j) {
            for (int k=0; k < GRID_Z_SIZE_; ++k)
            {
                if (grid_node_map_[i][j][k]->status_ == NODE_STATUS::IN_CLOSESET)
                {
                    visited_nodes_coord.emplace_back(grid_node_map_[i][j][k]->coordinate_);
                }
            }
        }
    }

    return visited_nodes_coord;
}

void AStar3D::initGridMap(double x_lower, double x_upper, double y_lower, double y_upper, double z_lower, double z_upper,
                          double resolution) {
    gridmap3Dptr = std::make_shared<GridMap3D>();
    gridmap3Dptr->Init(x_lower, x_upper, y_lower, y_upper, z_lower, z_upper, resolution);

    // for Astar
    GRID_X_SIZE_ = gridmap3Dptr->GRID_X_SIZE_;
    GRID_Y_SIZE_ = gridmap3Dptr->GRID_Y_SIZE_;
    GRID_Z_SIZE_ = gridmap3Dptr->GRID_Z_SIZE_;
    grid_node_map_.resize(GRID_X_SIZE_);
    for(int i=0; i<GRID_X_SIZE_; i++)
    {
        grid_node_map_[i].resize(GRID_Y_SIZE_);
        for(int j=0; j<GRID_Y_SIZE_; j++)
        {
            grid_node_map_[i][j].resize(GRID_Z_SIZE_);
            for(int k=0; k<GRID_Z_SIZE_; k++)
            {
                Eigen::Vector3i tmp_idx(i, j, k);
                Eigen::Vector3d tmp_pt = gridmap3Dptr->gridIdx2Coor(tmp_idx);
                grid_node_map_[i][j][k] = new GridNode<3>(tmp_idx, tmp_pt);
            }
        }
    }
    ROS_INFO("AStar3D: Init Grid Map");
}

void AStar3D::updateMapWithPCL(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr) {
    gridmap3Dptr->InitMapWithPCL(point_cloud_ros_ptr);
}





