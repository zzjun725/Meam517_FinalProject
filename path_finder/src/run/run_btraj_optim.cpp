//
// Created by zzjun on 8/25/22.
//

#include "path_finder/btraj/corridor_flow.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "run_btraj_optim");
    ros::NodeHandle node_handle("~");


    CubeCorridor_Flow cubecorridor_flow(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {

        cubecorridor_flow.Run();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}