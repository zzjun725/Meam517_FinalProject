//
// Created by zzjun on 8/10/22.
//

#ifndef SRC_GRID_NODE_H
#define SRC_GRID_NODE_H
#include <Eigen/Core>
#include <map>

template<int dim>
struct GridNode;

template<int dim>
using GridNodePtr = GridNode<dim> *;

enum NODE_STATUS {
    NOT_VISITED = 0, IN_OPENSET, IN_CLOSESET
};

template<int dim>
struct GridNode {
    // use this macro to make sure the pointer created for the eigen vector is aligned(address related to object size)
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GridNode(const Eigen::Matrix<int, dim, 1> &grid_index,
             const Eigen::Matrix<double, dim, 1> &coordinate) {
        status_ = NODE_STATUS::NOT_VISITED;
        grid_index_ = grid_index;
        coordinate_ = coordinate;
        direction_ = Eigen::Matrix<int, dim, 1>::Zero();
        g_score_ = std::numeric_limits<double>::min();
        f_score_ = std::numeric_limits<double>::min();
        parent_node_ = nullptr;
    }

    void Reset() {
        // std::cout<< "GridNode Reset"<<std::endl;
        status_ = NODE_STATUS::NOT_VISITED;
        direction_ = Eigen::Matrix<int, dim, 1>::Zero();
        g_score_ = std::numeric_limits<double>::min();
        f_score_ = std::numeric_limits<double>::min();
        parent_node_ = nullptr;
    }

    NODE_STATUS status_;
    Eigen::Matrix<double, dim, 1> coordinate_;
    Eigen::Matrix<int, dim, 1> direction_; // Maybe JPS need use it
    Eigen::Matrix<int, dim, 1> grid_index_;

    double g_score_, f_score_;
    GridNodePtr<dim> parent_node_;

    typename std::multimap<double, GridNodePtr<dim>>::iterator node_map_iter_;
};
#endif //SRC_GRID_NODE_H
