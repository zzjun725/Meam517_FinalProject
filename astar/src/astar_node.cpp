#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "astar.h"

using namespace std;
using namespace Eigen;
using namespace sdf_tools;

// Originally from launch file. Use as constants.
double x_size = 50.0;
double y_size = 50.0;
double z_size = 5.0;
double x_local_size = 20;
double y_local_size = 20;
double z_local_size = 5;
double resolution = 0.2;
double inv_resolution;
COLLISION_CELL free_cell(0.0);
COLLISION_CELL obst_cell(1.0);
double MAX_Vel = 2.0;
double MAX_Acc =2.0;

// Useful global variables
bool has_map = false;
bool has_target = false;
bool is_init = true;
bool is_emerg = false;

// Parameters from launch file.
Vector3d start_pt, start_vel, start_acc;
Vector3d end_pt;
Vector3d map_origin;

// ROS related
ros::Subscriber map_sub, pts_sub;
ros::Publisher local_map_vis_pub, grid_path_vis_pub, nodes_vis_pub;

gridPathFinder* path_finder = new gridPathFinder();
CollisionMapGrid* collision_map = new CollisionMapGrid();
CollisionMapGrid* collision_map_local = new CollisionMapGrid();

void rcvWaypointsCallback(const nav_msgs::Path& wp);
void rcvPointCloudCallback(const sensor_msgs::PointCloud2& pcd_map);

vector<pcl::PointXYZ> pointInflate( pcl::PointXYZ pt);
void visGridPath( vector<Vector3d> grid_path);
void visExpNode( vector<GridNodePtr> nodes);
void trajPlanning();

// Astar planning
void rcvWaypointsCallback(const nav_msgs::Path& wp)
{
    ROS_INFO("Start waypoint callbacks");
    if (wp.poses[0].pose.position.z < 0.0) {
        return;
    }
    is_init = false;
    end_pt << wp.poses[0].pose.position.x,
              wp.poses[0].pose.position.y,
              wp.poses[0].pose.position.z;
    has_target = true;
    is_emerg = true;
    ROS_INFO("[Astar Node] receive the way-points");
    trajPlanning(); 
}

Vector3d local_origin;
void rcvPointCloudCallback(const sensor_msgs::PointCloud2& pcd_map)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pcd_map, cloud);
    if((int)cloud.points.size()==0)
    {
        ROS_WARN("Cloud points are empty");
        return;
    }
    delete collision_map_local;
    collision_map->RestMap();

    double local_c_x = (int)((start_pt(0) - x_local_size/2.0)  * inv_resolution + 0.5) * resolution;
    double local_c_y = (int)((start_pt(1) - y_local_size/2.0)  * inv_resolution + 0.5) * resolution;
    double local_c_z = (int)((start_pt(2) - z_local_size/2.0)  * inv_resolution + 0.5) * resolution;
    local_origin << local_c_x, local_c_y, local_c_z;

    Translation3d origin_local_translation( local_origin(0), local_origin(1), local_origin(2));
    Quaterniond origin_local_rotation(1.0, 0.0, 0.0, 0.0);

    Affine3d origin_local_transform = origin_local_translation * origin_local_rotation;
    
    double _buffer_size = 2 * MAX_Vel;
    double _x_buffer_size = x_local_size + _buffer_size;
    double _y_buffer_size = y_local_size + _buffer_size;
    double _z_buffer_size = z_local_size + _buffer_size;

    collision_map_local = new CollisionMapGrid(origin_local_transform, "world", resolution, _x_buffer_size, _y_buffer_size, _z_buffer_size, free_cell);

    vector<pcl::PointXYZ> inflatePts(20);
    pcl::PointCloud<pcl::PointXYZ> cloud_inflation;
    pcl::PointCloud<pcl::PointXYZ> cloud_local;
    
    for (int idx = 0; idx < (int)cloud.points.size(); idx++)
    {   
        auto mk = cloud.points[idx];
        pcl::PointXYZ pt(mk.x, mk.y, mk.z);

        if( fabs(pt.x - start_pt(0)) > x_local_size / 2.0 || fabs(pt.y - start_pt(1)) > y_local_size / 2.0 || fabs(pt.z - start_pt(2)) > z_local_size / 2.0 )
            continue; 
        
        cloud_local.push_back(pt);
    }
    has_map = true;

    cloud_local.width = cloud_local.points.size();
    cloud_local.height = 1;
    cloud_local.is_dense = true;
    cloud_local.header.frame_id = "world";

    sensor_msgs::PointCloud2 inflateMap, localMap;
    
    pcl::toROSMsg(cloud_local, localMap);
    local_map_vis_pub.publish(localMap);
    trajPlanning(); 
}

visualization_msgs::MarkerArray grid_vis; 
void visGridPath( vector<Vector3d> grid_path )
{
    for(auto & mk: grid_vis.markers) 
        mk.action = visualization_msgs::Marker::DELETE;

    grid_path_vis_pub.publish(grid_vis);
    grid_vis.markers.clear();

    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "astar/grid_path";
    mk.type = visualization_msgs::Marker::CUBE;
    mk.action = visualization_msgs::Marker::ADD;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.0;
    mk.color.b = 0.0;

    int idx = 0;
    for(int i = 0; i < int(grid_path.size()); i++)
    {
        mk.id = idx;

        mk.pose.position.x = grid_path[i](0); 
        mk.pose.position.y = grid_path[i](1); 
        mk.pose.position.z = grid_path[i](2);  

        mk.scale.x = resolution;
        mk.scale.y = resolution;
        mk.scale.z = resolution;

        idx ++;
        grid_vis.markers.push_back(mk);
    }

    grid_path_vis_pub.publish(grid_vis);
}

void visExpNode( vector<GridNodePtr> nodes )
{   
    visualization_msgs::Marker node_vis; 
    node_vis.header.frame_id = "world";
    node_vis.header.stamp = ros::Time::now();
    node_vis.ns = "astar/visited_nodes";
    node_vis.type = visualization_msgs::Marker::CUBE_LIST;
    node_vis.action = visualization_msgs::Marker::ADD;
    node_vis.id = 0;

    node_vis.pose.orientation.x = 0.0;
    node_vis.pose.orientation.y = 0.0;
    node_vis.pose.orientation.z = 0.0;
    node_vis.pose.orientation.w = 1.0;
    node_vis.color.a = 0.3;
    node_vis.color.r = 0.0;
    node_vis.color.g = 1.0;
    node_vis.color.b = 0.0;

    node_vis.scale.x = resolution;
    node_vis.scale.y = resolution;
    node_vis.scale.z = resolution;

    geometry_msgs::Point pt;
    for(int i = 0; i < int(nodes.size()); i++)
    {
        Vector3d coord = nodes[i]->coord;
        pt.x = coord(0);
        pt.y = coord(1);
        pt.z = coord(2);

        node_vis.points.push_back(pt);
    }

    nodes_vis_pub.publish(node_vis);
}

void trajPlanning()
{
    if (has_target ==false || has_map == false)
    {
        return;
    }
    path_finder->linkLocalMap(collision_map_local, local_origin);
    ROS_INFO("start_pt: %f, %f, %f", start_pt(0), start_pt(1), start_pt(2));
    path_finder->AstarSearch(start_pt, end_pt);
    vector<Vector3d> gridPath = path_finder->getPath();
    ROS_INFO("%f", gridPath[0].x());
    vector<GridNodePtr> searchedNodes = path_finder->getVisitedNodes();
    path_finder->resetLocalMap();
    visGridPath(gridPath);
    visExpNode(searchedNodes);
    
    // TODO: Corridor  
    // TODO: Trajectory optimization
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle nh("~");
    map_sub = nh.subscribe("/map", 1, rcvPointCloudCallback);
    pts_sub = nh.subscribe("waypoints", 1, rcvWaypointsCallback);

    local_map_vis_pub = nh.advertise<sensor_msgs::PointCloud2>("vis_map_local", 1);
    grid_path_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1);
    nodes_vis_pub     = nh.advertise<visualization_msgs::Marker>("expanded_nodes_vis", 1);

    map_origin << -x_size/2.0, -y_size/2.0, 0.0;

    inv_resolution = 1.0 / resolution;
    int max_x_id = (int)(x_size * inv_resolution);
    int max_y_id = (int)(y_size * inv_resolution);
    int max_z_id = (int)(z_size * inv_resolution);
    Vector3i GLSIZE(max_x_id, max_y_id, max_z_id);

    int max_local_x_id = (int)(x_local_size * inv_resolution);
    int max_local_y_id = (int)(y_local_size * inv_resolution);
    int max_local_z_id = (int)(z_local_size * inv_resolution);
    Vector3i LOSIZE(max_local_x_id, max_local_y_id, max_local_z_id);
    ROS_INFO("Start path finder.");
    path_finder = new gridPathFinder(GLSIZE, LOSIZE);
    path_finder->initGridNodeMap(resolution, map_origin);
    ROS_INFO("Initialize grid node map.");
    Translation3d origin_translation(map_origin(0), map_origin(1), 0.0);
    Quaterniond origin_rotation(1.0, 0.0, 0.0, 0.0);
    Affine3d origin_transform = origin_translation * origin_rotation;
    collision_map = new CollisionMapGrid(origin_transform, "world", resolution, x_size, y_size, z_size, free_cell);
    ros::Rate rate(100);
    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
