//
// Created by zzjun on 8/11/22.
//

#include "path_finder/a_star/a_star_3d_flow.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "run_astar_3d");
    ros::NodeHandle node_handle("~");


    AStar3DFlow astar_3d_flow(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {

        astar_3d_flow.Run();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}