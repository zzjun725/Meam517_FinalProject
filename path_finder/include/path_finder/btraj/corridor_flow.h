//
// Created by zzjun on 8/25/22.
//

#ifndef PATH_FINDER_CORRIDOR_FLOW_H
#define PATH_FINDER_CORRIDOR_FLOW_H

#include <ros/ros.h>
#include <mutex>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include "corridor.h"
#include "path_finder/subscriber/astar_path_subscriber.h"
#include "path_finder/subscriber/pointcloud_subscriber.h"

class CubeCorridor_Flow
{
public:
    CubeCorridor_Flow()=delete;
    explicit CubeCorridor_Flow(ros::NodeHandle &nh);
    visualization_msgs::MarkerArray corridor_list_vis;
    void Run();

private:
    void publish_corridor_list();

private:



    std::shared_ptr<CubeCorridor> cubeCorridor_ptr_;
    std::shared_ptr<AstarPathSubscriber> astarPath_sub_;
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_;
    std::deque<sensor_msgs::PointCloud2Ptr> deque_map_;
    std::deque<geometry_msgs::PoseArrayPtr> deque_astar_path_;

    ros::Publisher corridor_pub_;

    sensor_msgs::PointCloud2Ptr current_map_ptr_;
    geometry_msgs::PoseArrayPtr current_path_ptr_;

    bool has_map_;

    static TypeVectorVecd<3> poseMsg2vector(geometry_msgs::PoseArrayPtr& pathMsg);

};


#endif //PATH_FINDER_CORRIDOR_FLOW_H
