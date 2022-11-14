//
// Created by zzjun on 8/23/22.
//

#ifndef SRC_MAP_TOOL_FLOW_H
#define SRC_MAP_TOOL_FLOW_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include "map_tool.h"
#include "path_finder/subscriber/init_points_subscriber.h"
#include "path_finder/subscriber/pointcloud_subscriber.h"

class GridMap3D_Flow {
public:
    GridMap3D_Flow() = default;
    explicit GridMap3D_Flow(ros::NodeHandle &nh);
    void Run();

private:
    void publish_visMap();
    // void publish_Map();

private:
    std::shared_ptr<GridMap3D> gridMap3D_ptr_;
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_;
    // std::shared_ptr<InitPoseSubscriber3D> init_pose_sub_;
    ros::Publisher grid_map_vis_pub_;
    // ros::Publisher grid_map_pub_;

    sensor_msgs::PointCloud2 point_cloud_ros_vis;


    std::deque<sensor_msgs::PointCloud2Ptr> deque_map_;
    sensor_msgs::PointCloud2Ptr current_map_ptr_;
    ros::Time timestamp_;
    bool has_map_=false;


};

#endif //SRC_MAP_TOOL_FLOW_H
