//
// Created by zzjun on 8/10/22.
//

#ifndef SRC_INIT_POINTS_SUBSCRIBER_H
#define SRC_INIT_POINTS_SUBSCRIBER_H




#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <deque>
#include <mutex>


class InitPoseSubscriber2D {
public:
    InitPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name,
                         size_t buff_size);

    void ParseData(std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> &pose_data_buff);

private:
    void MessageCallBack(const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> init_poses_;

    std::mutex buff_mutex_;
};


class InitPoseSubscriber3D {
public:
    InitPoseSubscriber3D(ros::NodeHandle &nh, const std::string &topic_name,
                         size_t buff_size);

    void ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff);

private:
    void MessageCallBack(const geometry_msgs::PoseStampedPtr &init_pose_ptr);

private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseStampedPtr> init_poses_;

    std::mutex buff_mutex_;
};



#endif //SRC_INIT_POINTS_SUBSCRIBER_H
