//
// Created by zzjun on 8/25/22.
//

#include "path_finder/btraj/corridor_flow.h"

CubeCorridor_Flow::CubeCorridor_Flow(ros::NodeHandle &nh) {

    astarPath_sub_ = std::make_shared<AstarPathSubscriber>(nh, "/astar_3d/astar_path", 10);
    pointcloud_sub_ = std::make_shared<PointcloudSubscriber>(nh, "/grid_map", 5);
    corridor_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/cubic_corridors", 100);

    // init GridMap
    double map_size_x = nh.param("map/x_size", 40.0);
    double map_size_y = nh.param("map/y_size", 40.0);
    double map_size_z = nh.param("map/z_size", 5.0);
    double resolution = nh.param("map/resolution", 0.1);

    double x_lower = 0.0;
    double x_upper = map_size_x;
    double y_lower = 0.0;
    double y_upper = map_size_y;
    double z_lower = 0.0;
    double z_upper = map_size_z;

    // init corridor
    int max_inflate_iter = nh.param("btraj/max_inflate_iter", 40);
    int step_length = nh.param("btraj/step_length", 2);
    cubeCorridor_ptr_ = std::make_shared<CubeCorridor>(max_inflate_iter, step_length);

    cubeCorridor_ptr_->initGridMap(x_lower, x_upper, y_lower, y_upper, z_lower, z_upper, resolution);
    has_map_ = false;

}

void CubeCorridor_Flow::Run()
{
    pointcloud_sub_->ParseData(deque_map_);
    astarPath_sub_->ParseData(deque_astar_path_);

    if (!has_map_) {

        if (deque_map_.empty()) {
            return;
        }
        current_map_ptr_ = deque_map_.front();
        deque_map_.pop_front();
        cubeCorridor_ptr_->updateMapWithPCL(current_map_ptr_);
        has_map_ = true;
        ROS_INFO("Init Corridor Grid Map");
    }
    deque_map_.clear();

    while(!deque_astar_path_.empty())
    {
        current_path_ptr_ = deque_astar_path_.front();
        deque_astar_path_.pop_front();

        TypeVectorVecd<3> current_astar_path = poseMsg2vector(current_path_ptr_);
        ROS_INFO("A* Path Detected, Start Corridor inflation with %zu", current_astar_path.size());
        cubeCorridor_ptr_->inflate_path(current_astar_path);
        ROS_INFO("Finish Corridor inflation, get %zu cubics", cubeCorridor_ptr_->cubicList.size());
        publish_corridor_list();
        ros::Duration(1.0).sleep();
    }
    deque_astar_path_.clear();


}

void CubeCorridor_Flow::publish_corridor_list() {
    for(auto & marker : corridor_list_vis.markers)
        marker.action = visualization_msgs::Marker::DELETE;
    corridor_pub_.publish(corridor_list_vis);
    corridor_list_vis.markers.clear();

    ros::Duration(1.0).sleep();

    visualization_msgs::Marker box;
    box.header.frame_id="map";
    box.ns="box";
    box.type=visualization_msgs::Marker::CUBE;
    box.action=visualization_msgs::Marker::ADD;
    box.header.stamp = ros::Time::now();

    box.pose.orientation.x=0.0;
    box.pose.orientation.y=0.0;
    box.pose.orientation.z=0.0;
    box.pose.orientation.w=1.0;

    box.color.a = 0.3;
    box.color.r = 0.0;
    box.color.g = 0.0;
    box.color.b = 1.0;

    for(int i=0;i<(int)cubeCorridor_ptr_->cubicList.size();++i){
        box.id=i;

        box.pose.position.x=cubeCorridor_ptr_->cubicList[i].center[0];
        box.pose.position.y=cubeCorridor_ptr_->cubicList[i].center[1];
        box.pose.position.z=cubeCorridor_ptr_->cubicList[i].center[2];

        box.scale.x=cubeCorridor_ptr_->cubicList[i].get_x_scale();
        box.scale.y=cubeCorridor_ptr_->cubicList[i].get_y_scale();
        box.scale.z=cubeCorridor_ptr_->cubicList[i].get_z_scale();
        ROS_INFO("Publish Cubics index: %d, at %f, %f, %f, scale is %f, %f, %f", i,
                 box.pose.position.x, box.pose.position.y, box.pose.position.z,
                 box.scale.x, box.scale.y, box.scale.z);

        corridor_list_vis.markers.push_back(box);
    }

    corridor_pub_.publish(corridor_list_vis);
}

TypeVectorVecd<3> CubeCorridor_Flow::poseMsg2vector(geometry_msgs::PoseArray_<allocator<void>>::Ptr & pathMsg) {
    TypeVectorVecd<3> astar_path;
    auto n=pathMsg->poses.size();
    for(int i=0; i<(int)n; i++)
    {
        double x=pathMsg->poses[i].position.x;
        double y=pathMsg->poses[i].position.y;
        double z=pathMsg->poses[i].position.z;
        astar_path.push_back({x, y, z});
    }

    return astar_path;
}
