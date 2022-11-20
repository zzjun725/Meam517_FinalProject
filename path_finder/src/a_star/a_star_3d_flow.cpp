//
// Created by zzjun on 8/11/22.
//

#include "path_finder/a_star/a_star_3d_flow.h"
#include "path_finder/map_tools/map_tool_flow.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/GridCells.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>

AStar3DFlow::AStar3DFlow(ros::NodeHandle &nh) {
    // definition of all the topics
    astar_searcher_ptr_ = std::make_shared<AStar3D>();
    pointcloud_sub_ = std::make_shared<PointcloudSubscriber>(nh, "/structure_map/global_cloud", 5);
    // init_pose_sub_ = std::make_shared<InitPoseSubscriber3D>(nh, "/initialpose", 5);
    init_pose_sub_ = std::make_shared<InitPoseSubscriber3D>(nh, "/goal", 5);
//    grid_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>("grid_map", 1);
    // path_pub_ = nh.advertise<nav_msgs::Path>("searched_path", 10);
    apath_vis_pub_ = nh.advertise<visualization_msgs::Marker>("astar_path_vis", 10);
    apath_pub_ = nh.advertise<geometry_msgs::PoseArray>("astar_path", 10);
    visited_nodes_pub_ = nh.advertise<nav_msgs::GridCells>("visited_nodes", 10);

    grid_map_vis_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/grid_map", 1);

    start_x_ = nh.param("planning/start_x", 0.0);
    start_y_ = nh.param("planning/start_y", 0.0);
    start_z_ = nh.param("planning/start_z", 0.0);

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

    astar_searcher_ptr_->initGridMap(x_lower, x_upper, y_lower, y_upper, z_lower, z_upper, resolution);

    has_map_ = false;
}

void AStar3DFlow::Run() {
    // update deque_map_ and deque_init_pose_
    pointcloud_sub_->ParseData(deque_map_);
    init_pose_sub_->ParseData(deque_init_pose_);

    // only read map once
    if (!has_map_) {

        if (deque_map_.empty()) {
            return;
        }

        current_map_ptr_ = deque_map_.front();
        deque_map_.pop_front();
        astar_searcher_ptr_->updateMapWithPCL(current_map_ptr_);
        has_map_ = true;
        timestamp_ = current_map_ptr_->header.stamp;
        publish_visMap();
        // astar_searcher_ptr_->Reset();
    }
    deque_map_.clear();

    while (!deque_init_pose_.empty()) {
        ROS_INFO("InitPose Detected, Start A* Search");
        current_init_pose_ptr_ = deque_init_pose_.front();
        deque_init_pose_.pop_front();

        Vec3d start_pt(start_x_, start_y_, start_z_);
        double goal_x = current_init_pose_ptr_->pose.position.x;
        double goal_y = current_init_pose_ptr_->pose.position.y;
        double goal_z = current_init_pose_ptr_->pose.position.z;
        Vec3d goal_pt(
                goal_x,
                goal_y,
                goal_z
        );
        // std::cout<<"goal point: "<<"[x: "<<goal_x<<", y:"<<goal_y<<", z:"<<goal_z<<"]"<<std::endl;

        if (astar_searcher_ptr_->Search(start_pt, goal_pt)) {
            PublishPath(astar_searcher_ptr_->GetPath());
            std::cout << "Path Finder succeed" << std::endl;
        }
//        ros::Duration(0.1).sleep();
        astar_searcher_ptr_->Reset();
    }
    publish_visMap();
}

//bool AStar3DFlow::HasInitPoseData() {
//    return !deque_init_pose_.empty();
//}

geometry_msgs::Point AStar3DFlow::coor2PtMsg(const double &x, const double & y, const double & z){
    geometry_msgs::Point pt;
    pt.x=static_cast<float>(x);
    pt.y=static_cast<float>(y);
    pt.z=static_cast<float>(z);
    return pt;
}

void AStar3DFlow::PublishPath(const TypeVectorVecd<3> &path) {

    visualization_msgs::Marker Apath;
    geometry_msgs::PoseArray path_to_inflate;
    geometry_msgs::Pose temp;
    Apath.header.frame_id = "map";
    Apath.ns="visApath";
    Apath.id=0;
    Apath.header.stamp = ros::Time::now();
    Apath.type = visualization_msgs::Marker::SPHERE_LIST;
    Apath.action = visualization_msgs::Marker::ADD;
    Apath.scale.x = 0.05;
    Apath.scale.y = 0.05;
    Apath.scale.z = 0.05;
    Apath.color.a = 1.0;
    Apath.color.r = 1.0f;
    Apath.color.g = 0.0f;
    Apath.color.b = 0.0f;
    Apath.lifetime=ros::Duration();

    for (const auto &pose: path) {
        Apath.points.push_back(coor2PtMsg(pose.x(), pose.y(), pose.z()));
        temp.position.x = pose.x();
        temp.position.y = pose.y();
        temp.position.z = pose.z();
        path_to_inflate.poses.push_back(temp);
    }

    apath_vis_pub_.publish(Apath);
    apath_pub_.publish(path_to_inflate);
}

void AStar3DFlow::publish_visMap() {
    // if (!has_map_) return;
    pcl::toROSMsg(astar_searcher_ptr_ -> gridmap3Dptr ->point_cloud_map_vis, point_cloud_ros_vis);
    point_cloud_ros_vis.header.frame_id = "map";
    grid_map_vis_pub_.publish(point_cloud_ros_vis);
}

