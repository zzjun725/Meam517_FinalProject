//
// Created by zzjun on 8/22/22.
//

#ifndef SRC_CUBE_H
#define SRC_CUBE_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <vector>

struct Cube;

struct Cube
{
    //Eigen::Vector3d p1, p2, p3, p4, p5, p6, p7, p8;   // the 8 vertex of a cube
    Eigen::MatrixXd vertex;
    Eigen::Vector3d center; // the center of the cube
    bool valid;    // indicates whether this cube should be deleted

    double t; // time allocated to this cube
    std::vector< std::pair<double, double> > box;
/*
           P4------------P3
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /
        P5------------P6              / x
*/

    // create a cube using 8 vertex and the center point
    Cube( Eigen::MatrixXd vertex_, Eigen::Vector3d center_)
    {
        vertex = vertex_;
        center = center_;
        valid = true;
        t = 0.0;
        box.resize(3);
        setBox();
        setCenter();
    }

    Cube(Cube const &cube)
    {
        vertex = cube.vertex;
        center = cube.center;
        valid = true;
        t = 0.0;
        box.resize(3);
        setBox();
        setCenter();
    }

    // create an inscribe cube of a ball using the center point and the radius of the ball
    void setVertex( Eigen::MatrixXd vertex_, double resolution_)
    {
        vertex = vertex_;
        vertex(0,1) -= resolution_ / 2.0;
        vertex(3,1) -= resolution_ / 2.0;
        vertex(4,1) -= resolution_ / 2.0;
        vertex(7,1) -= resolution_ / 2.0;

        vertex(1,1) += resolution_ / 2.0;
        vertex(2,1) += resolution_ / 2.0;
        vertex(5,1) += resolution_ / 2.0;
        vertex(6,1) += resolution_ / 2.0;

        vertex(0,0) += resolution_ / 2.0;
        vertex(1,0) += resolution_ / 2.0;
        vertex(4,0) += resolution_ / 2.0;
        vertex(5,0) += resolution_ / 2.0;

        vertex(2,0) -= resolution_ / 2.0;
        vertex(3,0) -= resolution_ / 2.0;
        vertex(6,0) -= resolution_ / 2.0;
        vertex(7,0) -= resolution_ / 2.0;

        vertex(0,2) += resolution_ / 2.0;
        vertex(1,2) += resolution_ / 2.0;
        vertex(2,2) += resolution_ / 2.0;
        vertex(3,2) += resolution_ / 2.0;

        vertex(4,2) -= resolution_ / 2.0;
        vertex(5,2) -= resolution_ / 2.0;
        vertex(6,2) -= resolution_ / 2.0;
        vertex(7,2) -= resolution_ / 2.0;
        setBox();
    }

    void setVertex( Eigen::MatrixXd vertex_)
    {
        vertex = vertex_;
        setBox();
        setCenter();
    }

    void setBox()
    {
        box.clear();
        box.resize(3);
        box[0] = std::make_pair( vertex(3, 0), vertex(0, 0) );
        box[1] = std::make_pair( vertex(0, 1), vertex(1, 1) );
        box[2] = std::make_pair( vertex(4, 2), vertex(1, 2) );
    }

    void setCenter()
    {
        double x = (box[0].first + box[0].second)/2;
        double y = (box[1].first + box[1].second)/2;
        double z = (box[2].first + box[2].second)/2;
        center[0] = x;
        center[1] = y;
        center[2] = z;
    }

    double get_x_scale()
    {
        return box[0].second - box[0].first;
    }

    double get_y_scale()
    {
        return box[1].second - box[1].first;
    }

    double get_z_scale()
    {
        return box[2].second - box[2].first;
    }

    void printBox()
    {
        std::cout<<"center of the cube: \n"<<center<<std::endl;
        std::cout<<"vertex of the cube: \n"<<vertex<<std::endl;
    }

    Cube()
    {
        center = Eigen::VectorXd::Zero(3);
        vertex = Eigen::MatrixXd::Zero(8, 3);

        valid = true;
        t = 0.0;
        box.resize(3);
    }

    ~Cube(){}
};


#endif //SRC_CUBE_H

