//
// Created by zzjun on 8/22/22.
//

#include "path_finder/btraj/corridor.h"


/*
           P4------------P3
           /|           /|              ^
          / |          / |              | z
        P1--|---------P2 |              |
         |  P8--------|--p7             |
         | /          | /               /--------> y
         |/           |/               /
        P5------------P6              / x

    Eigen::Vector3i p1{1, -1, 1};
    Eigen::Vector3i p2{1, 1, 1};
    Eigen::Vector3i p3{-1, 1, 1};
    Eigen::Vector3i p4{-1, -1, 1};
    Eigen::Vector3i p5{1, -1, -1};
    Eigen::Vector3i p6{1, 1, -1};
    Eigen::Vector3i p7{-1, 1, -1};
    Eigen::Vector3i p8{-1, -1, -1};
 */


void CubeCorridor::inflate_path(TypeVectorVecd<3> &path) {
    cubicList.clear();
    Cube last_cubic = initCube9Grid(path[0]);
    // last_cubic.printBox();
    last_cubic = inflate_cube(last_cubic);
    // last_cubic.printBox();
    cubicList.push_back(last_cubic);

    // all cubic coor are in map frame
//    cout << "path size is: " << path.size() << endl;
    for(int i=1; i<(int)path.size(); i++)
    {
        if (pt_inside_cubic(path[i], last_cubic))
        {
            cout << "path point " << i <<  " is inside last cubic" << endl;
            continue;
        }
        cout << "create cubic for " << i << "th" << " node" << endl;
        Cube tmpCubic = initCube9Grid(path[i]);
        tmpCubic = inflate_cube(tmpCubic);

        bool lastInsideTmp=true;
        while(!cubicList.empty())
        {
            lastInsideTmp=true;
            last_cubic = cubicList.back();
            unordered_set<int> pathPt_idx_tmpCubic = get_pathPt_inside_cube(path, tmpCubic);
            unordered_set<int> pathPt_idx_lastCubic = get_pathPt_inside_cube(path, last_cubic);
            for(auto pt_idx:pathPt_idx_lastCubic)
            {
                if (pathPt_idx_tmpCubic.find(pt_idx)==pathPt_idx_tmpCubic.end())
                {
                    lastInsideTmp = false;
                    break;
                }
            }
            if (lastInsideTmp)
            {
                cubicList.pop_back();
            }
            else break;
        }

        if(!cubicList.empty())
        {
            bool tmpInsideLast = true;
            unordered_set<int> pathPt_idx_tmpCubic = get_pathPt_inside_cube(path, tmpCubic);
            unordered_set<int> pathPt_idx_lastCubic = get_pathPt_inside_cube(path, last_cubic);
            for(auto pt_idx:pathPt_idx_tmpCubic)
            {
                if (pathPt_idx_lastCubic.find(pt_idx)==pathPt_idx_lastCubic.end())
                {
                    tmpInsideLast = false;
                    break;
                }
            }
            if (tmpInsideLast)
            {
                cout << "skip tmp cubic" << endl;
                continue;
            }
        }

        cubicList.emplace_back(tmpCubic);
        last_cubic = tmpCubic;
        cout << "add tmp cubic" << endl;
    }


}

Cube CubeCorridor::inflate_cube(Cube& cube) {
    Cube cubeMax = cube;
    cout << "original box is: " << endl;
    cubeMax.printBox();

    // Inflate sequence: left, right, front, back, below, above
    MatrixXi vertex_idx(8, 3);
    for (int i = 0; i < 8; i++)
    {
        double coord_x = max(min(cube.vertex(i, 0), x_upper_), x_lower_);
        double coord_y = max(min(cube.vertex(i, 1), y_upper_), y_lower_);
        double coord_z = max(min(cube.vertex(i, 2), z_upper_), z_lower_);
        Vector3d pt(coord_x, coord_y, coord_z);
        // cout << "x: " << coord_x << " y: " << coord_y << " z: " << coord_z << endl;
        // cout << "x: " << cube.vertex(i, 0) << " y: " << cube.vertex(i, 1) << " z: " << cube.vertex(i, 2) << endl;
        Vector3i pt_grid_idx = gridmap3Dptr->Coor2gridIdx(pt);
        if(!gridmap3Dptr->infreeSpace(pt_grid_idx, true))
        {
            ROS_ERROR("[Planning Node] path has node in obstacles !");
            return Cube{};
        }

        vertex_idx.row(i) = pt_grid_idx;
    }

    int id_x, id_y, id_z;

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

    // Y- now is the left side : (p1 -- p4 -- p8 -- p5) face sweep
    // ############################################################################################################
    bool collide;

    MatrixXi vertex_idx_lst = vertex_idx;

    int iter = 0;
    while(iter < _max_inflate_iter)
    {
        collide  = false;
        int y_lo = max(0, vertex_idx(0, 1) - _step_length);
        int y_up = min(_max_y_id, vertex_idx(1, 1) + _step_length);
        // int y_up = vertex_idx(1, 1);
        for(id_y = vertex_idx(0, 1); id_y >= y_lo; id_y-- )
        {
            if(collide)
                break;

            for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
            {
                if(collide)
                    break;

                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    Eigen::Vector3i tmp{id_x, id_y, id_z};
                    if(!gridmap3Dptr->infreeSpace(tmp, false))
                    {
                        collide = true;
                        break;
                    }
                }
            }
        }


        if(collide)
        {
            // cout << "id_y" << id_y << endl;
            vertex_idx(0, 1) = min(id_y+2, vertex_idx(0, 1));
            vertex_idx(3, 1) = min(id_y+2, vertex_idx(3, 1));
            vertex_idx(7, 1) = min(id_y+2, vertex_idx(7, 1));
            vertex_idx(4, 1) = min(id_y+2, vertex_idx(4, 1));
        }
        else
            vertex_idx(0, 1) = vertex_idx(3, 1) = vertex_idx(7, 1) = vertex_idx(4, 1) = id_y + 1;

        // Y+ now is the right side : (p2 -- p3 -- p7 -- p6) face
        // ############################################################################################################
        collide = false;
        for(id_y = vertex_idx(1, 1); id_y <= y_up; id_y++ )
        {
            if(collide)
                break;

            for(id_x = vertex_idx(1, 0); id_x >= vertex_idx(2, 0); id_x-- )
            {
                if(collide)
                    break;

                for(id_z = vertex_idx(1, 2); id_z >= vertex_idx(5, 2); id_z-- )
                {
                    Eigen::Vector3i tmp{id_x, id_y, id_z};
                    if(!gridmap3Dptr->infreeSpace(tmp, false))
                    {
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            // cout << "id_y" << id_y << endl;
            vertex_idx(1, 1) = max(id_y-2, vertex_idx(1, 1));
            vertex_idx(2, 1) = max(id_y-2, vertex_idx(2, 1));
            vertex_idx(6, 1) = max(id_y-2, vertex_idx(6, 1));
            vertex_idx(5, 1) = max(id_y-2, vertex_idx(5, 1));
        }
        else
            vertex_idx(1, 1) = vertex_idx(2, 1) = vertex_idx(6, 1) = vertex_idx(5, 1) = id_y - 1;

        // X + now is the front side : (p1 -- p2 -- p6 -- p5) face
        // ############################################################################################################
        int x_lo = max(0, vertex_idx(3, 0) - _step_length);
        int x_up = min(_max_x_id, vertex_idx(0, 0) + _step_length);

        collide = false;
        for(id_x = vertex_idx(0, 0); id_x <= x_up; id_x++ )
        {
            if(collide)
                break;

            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if(collide)
                    break;

                for(id_z = vertex_idx(0, 2); id_z >= vertex_idx(4, 2); id_z-- )
                {
                    Eigen::Vector3i tmp{id_x, id_y, id_z};
                    if(!gridmap3Dptr->infreeSpace(tmp, false))
                    {
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            // cout << "id_x" << id_x << endl;
            vertex_idx(0, 0) = max(id_x-2, vertex_idx(0, 0));
            vertex_idx(1, 0) = max(id_x-2, vertex_idx(1, 0));
            vertex_idx(5, 0) = max(id_x-2, vertex_idx(5, 0));
            vertex_idx(4, 0) = max(id_x-2, vertex_idx(4, 0));
        }
        else
            vertex_idx(0, 0) = vertex_idx(1, 0) = vertex_idx(5, 0) = vertex_idx(4, 0) = id_x - 1;

        // X- now is the back side : (p4 -- p3 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_x = vertex_idx(3, 0); id_x >= x_lo; id_x-- )
        {
            if(collide)
                break;

            for(id_y = vertex_idx(3, 1); id_y <= vertex_idx(2, 1); id_y++ )
            {
                if(collide)
                    break;

                for(id_z = vertex_idx(3, 2); id_z >= vertex_idx(7, 2); id_z-- )
                {
                    Eigen::Vector3i tmp{id_x, id_y, id_z};
                    if(!gridmap3Dptr->infreeSpace(tmp, false))
                    {
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            // cout << "id_x" << id_x << endl;
            vertex_idx(3, 0) = min(id_x+2, vertex_idx(3, 0));
            vertex_idx(2, 0) = min(id_x+2, vertex_idx(2, 0));
            vertex_idx(6, 0) = min(id_x+2, vertex_idx(6, 0));
            vertex_idx(7, 0) = min(id_x+2, vertex_idx(7, 0));
        }
        else
            vertex_idx(3, 0) = vertex_idx(2, 0) = vertex_idx(6, 0) = vertex_idx(7, 0) = id_x + 1;

        // Z+ now is the above side : (p1 -- p2 -- p3 -- p4) face
        // ############################################################################################################
        collide = false;
        int z_lo = max(0, vertex_idx(4, 2) - _step_length);
        int z_up = min(_max_z_id, vertex_idx(0, 2) + _step_length);
        for(id_z = vertex_idx(0, 2); id_z <= z_up; id_z++ )
        {
            if(collide)
                break;

            for(id_y = vertex_idx(0, 1); id_y <= vertex_idx(1, 1); id_y++ )
            {
                if(collide)
                    break;

                for(id_x = vertex_idx(0, 0); id_x >= vertex_idx(3, 0); id_x-- )
                {
                    Eigen::Vector3i tmp{id_x, id_y, id_z};
                    if(!gridmap3Dptr->infreeSpace(tmp, false))
                    {
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            cout << "id_z" << id_z << endl;
            vertex_idx(0, 2) = max(id_z-2, vertex_idx(0, 2));
            vertex_idx(1, 2) = max(id_z-2, vertex_idx(1, 2));
            vertex_idx(2, 2) = max(id_z-2, vertex_idx(2, 2));
            vertex_idx(3, 2) = max(id_z-2, vertex_idx(3, 2));
        }
        vertex_idx(0, 2) = vertex_idx(1, 2) = vertex_idx(2, 2) = vertex_idx(3, 2) = id_z - 1;

        // now is the below side : (p5 -- p6 -- p7 -- p8) face
        // ############################################################################################################
        collide = false;
        for(id_z = vertex_idx(4, 2); id_z >= z_lo; id_z-- )
        {
            if(collide)
                break;

            for(id_y = vertex_idx(4, 1); id_y <= vertex_idx(5, 1); id_y++ )
            {
                if(collide)
                    break;

                for(id_x = vertex_idx(4, 0); id_x >= vertex_idx(7, 0); id_x-- )
                {
                    Eigen::Vector3i tmp{id_x, id_y, id_z};
                    if(!gridmap3Dptr->infreeSpace(tmp, false))
                    {
                        collide = true;
                        break;
                    }
                }
            }
        }

        if(collide)
        {
            cout << "id_z" << id_z << endl;
            vertex_idx(4, 2) = min(id_z+2, vertex_idx(4, 2));
            vertex_idx(5, 2) = min(id_z+2, vertex_idx(5, 2));
            vertex_idx(6, 2) = min(id_z+2, vertex_idx(6, 2));
            vertex_idx(7, 2) = min(id_z+2, vertex_idx(7, 2));
        }
        else
            vertex_idx(4, 2) = vertex_idx(5, 2) = vertex_idx(6, 2) = vertex_idx(7, 2) = id_z + 1;

        if(vertex_idx_lst == vertex_idx)
            break;

        vertex_idx_lst = vertex_idx;

        MatrixXd vertex_coord(8, 3);
        for(int i = 0; i < 8; i++)
        {
            // TODO: why _max_x_id -1 here?
            int index_x = max(min(vertex_idx(i, 0), _max_x_id - 1), 0);
            int index_y = max(min(vertex_idx(i, 1), _max_y_id - 1), 0);
            int index_z = max(min(vertex_idx(i, 2), _max_z_id - 1), 0);

            Vector3i index(index_x, index_y, index_z);
            Vector3d pos = gridmap3Dptr->gridIdx2Coor(index);
            vertex_coord.row(i) = pos;
        }

        cubeMax.setVertex(vertex_coord);
//        if( isContains(lstcube, cubeMax))
//            return make_pair(lstcube, false);

        iter ++;
//        cout << "inflate at iter: " << iter << endl;
//        cubeMax.printBox();
    }

    return cubeMax;

}


bool CubeCorridor::pt_inside_cubic(Vector3d &pt, Cube &cubic) {
    if(pt.x()>=cubic.box[0].first && pt.x()<=cubic.box[0].second
       && pt.y()>=cubic.box[1].first && pt.y()<=cubic.box[1].second
       && pt.z()>=cubic.box[2].first && pt.z()<=cubic.box[2].second)
        return true;
    else return false;
}

unordered_set<int> CubeCorridor::get_pathPt_inside_cube(TypeVectorVecd<3> &path, Cube cubic) {
    unordered_set<int> pathPt_idx;
    for(int i=0; i<(int)path.size(); i++)
    {
        if (pt_inside_cubic(path[i], cubic))
            pathPt_idx.insert(i);
    }
    return pathPt_idx;
}

Cube CubeCorridor::initCube9Grid(Eigen::Vector3d &pt) {
    Eigen::MatrixXd vertices = Eigen::MatrixXd::Zero(8, 3);
    vector<Eigen::Vector3i> directions;
    directions.emplace_back(1, -1, 1);
    directions.emplace_back(1, 1, 1);
    directions.emplace_back(-1, 1, 1);
    directions.emplace_back(-1, -1, 1);
    directions.emplace_back(1, -1, -1);
    directions.emplace_back(1, 1, -1);
    directions.emplace_back(-1, 1, -1);
    directions.emplace_back(-1, -1, -1);

    for(int i=0; i<(int)directions.size(); i++)
    {
        Eigen::Vector3i center_idx = gridmap3Dptr->Coor2gridIdx(pt);
        Eigen::Vector3d tmp = gridmap3Dptr->gridIdx2Coor(center_idx + directions[i]);
        vertices(i, 0) = tmp.x();
        vertices(i, 1) = tmp.y();
        vertices(i, 2) = tmp.z();
    }
    Cube init_cube(vertices, pt);
    return init_cube;
}

void CubeCorridor::initGridMap(double x_lower, double x_upper, double y_lower, double y_upper, double z_lower, double z_upper,
                          double resolution) {
    gridmap3Dptr = std::make_shared<GridMap3D>();
    gridmap3Dptr->Init(x_lower, x_upper, y_lower, y_upper, z_lower, z_upper, resolution);
    x_lower_ = x_lower;
    x_upper_ = x_upper;
    y_lower_ = y_lower;
    y_upper_ = y_upper;
    z_lower_ = z_lower;
    z_upper_ = z_upper;
    _max_x_id = gridmap3Dptr->GRID_X_SIZE_;
    _max_y_id = gridmap3Dptr->GRID_Y_SIZE_;
    _max_z_id = gridmap3Dptr->GRID_Z_SIZE_;
}

void CubeCorridor::updateMapWithPCL(const sensor_msgs::PointCloud2Ptr &point_cloud_ros_ptr) {
    gridmap3Dptr->InitMapWithPCL(point_cloud_ros_ptr);
}

CubeCorridor::CubeCorridor(int max_inflate_iter, int step_length) {
    _max_inflate_iter = max_inflate_iter;
    _step_length = step_length;

}










