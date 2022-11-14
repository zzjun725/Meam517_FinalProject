//
// Created by zzjun on 8/23/22.
//

#include "path_finder/map_tools/map_tool_flow.h"

GridMap3D_Flow::GridMap3D_Flow(ros::NodeHandle &nh) {
    gridMap3D_ptr_ = std::make_shared<GridMap3D>();
    pointcloud_sub_ = std::make_shared<PointcloudSubscriber>(nh, "/structure_map/global_cloud", 5);
    grid_map_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/grid_map", 1);
    // grid_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/grid_map", 1);

    double map_size_x = nh.param("map/x_size", 40.0);
    double map_size_y = nh.param("map/y_size", 40.0);
    double map_size_z = nh.param("map/z_size", 5.0);
    double resolution = nh.param("map/resolution", 0.1);

    double x_lower = 0;
    double x_upper = map_size_x;
    double y_lower = 0;
    double y_upper = map_size_y;
    double z_lower = 0;
    double z_upper = map_size_z;

    gridMap3D_ptr_->Init(x_lower, x_upper, y_lower, y_upper, z_lower, z_upper, resolution);
}

void GridMap3D_Flow::Run(){
    ros::Rate rate(10);
    pointcloud_sub_->ParseData(deque_map_);
    if (!has_map_)
    {
        if (deque_map_.empty()) {
            return;
        }
        current_map_ptr_ = deque_map_.front();
        deque_map_.pop_front();
        gridMap3D_ptr_->InitMapWithPCL(current_map_ptr_);
        ROS_INFO("MapTool: Init Grid Map");
        has_map_ = true;
        timestamp_ = current_map_ptr_->header.stamp;

        publish_visMap();
    }
    else{
        rate.sleep();
        publish_visMap();
    }
}

void GridMap3D_Flow::publish_visMap() {
    // if (!has_map_) return;
    pcl::toROSMsg(gridMap3D_ptr_->point_cloud_map_vis, point_cloud_ros_vis);
    point_cloud_ros_vis.header.frame_id = "map";
    grid_map_vis_pub_.publish(point_cloud_ros_vis);
}
