//
// Created by zzjun on 8/23/22.
//

#include "path_finder/map_tools/map_tool_flow.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "run_map_tool");
    ros::NodeHandle node_handle("~");


    GridMap3D_Flow gridMap3DFlow(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {

        gridMap3DFlow.Run();

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}