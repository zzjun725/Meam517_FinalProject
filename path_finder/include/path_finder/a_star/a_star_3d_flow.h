//
// Created by zzjun on 8/11/22.
//

#ifndef SRC_A_STAR_3D_FLOW_H
#define SRC_A_STAR_3D_FLOW_H


#include "path_finder/a_star/a_star_3d.h"
#include "path_finder/utils/type.h"
#include "path_finder/map_tools/map_tool_flow.h"
#include "path_finder/subscriber/init_points_subscriber.h"
#include "path_finder/subscriber/pointcloud_subscriber.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <pcl_conversions/pcl_conversions.h>

#include <mutex>

class AStar3DFlow {
public:
    AStar3DFlow() = delete;

    explicit AStar3DFlow(ros::NodeHandle &nh);

    void Run();

private:

    // bool HasInitPoseData();

    void PublishPath(const TypeVectorVecd<3> &path);


private:
    std::shared_ptr<AStar3D> astar_searcher_ptr_;
    std::shared_ptr<PointcloudSubscriber> pointcloud_sub_;
    std::shared_ptr<InitPoseSubscriber3D> init_pose_sub_;

//    ros::Publisher grid_map_pub_;
    ros::Publisher apath_pub_;
    ros::Publisher apath_vis_pub_;
    ros::Publisher visited_nodes_pub_;
    std::deque<sensor_msgs::PointCloud2Ptr> deque_map_;
//    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> deque_init_pose_;
//    geometry_msgs::PoseWithCovarianceStampedPtr current_init_pose_ptr_;
    std::deque<geometry_msgs::PoseStampedPtr> deque_init_pose_;
    geometry_msgs::PoseStampedPtr current_init_pose_ptr_;
    sensor_msgs::PointCloud2Ptr current_map_ptr_;

    ros::Publisher grid_map_vis_pub_;
    sensor_msgs::PointCloud2 point_cloud_ros_vis;

    ros::Time timestamp_;
    double start_x_, start_y_, start_z_;
    bool has_map_;

    geometry_msgs::Point coor2PtMsg(const double &x, const double &y, const double &z);

    void publish_visMap();
};


#endif //SRC_A_STAR_3D_FLOW_H
