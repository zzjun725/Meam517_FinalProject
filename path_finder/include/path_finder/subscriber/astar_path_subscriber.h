//
// Created by zzjun on 8/25/22.
//

#ifndef PATH_FINDER_ASTAR_PATH_SUBSCRIBER_H
#define PATH_FINDER_ASTAR_PATH_SUBSCRIBER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <deque>
#include <mutex>
#include <thread>
#include <string>

class AstarPathSubscriber {
public:
    AstarPathSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size);
    void ParseData(std::deque<geometry_msgs::PoseArrayPtr> &deque_astarPath_msg_ptr);

private:
    void MessageCallBack(const geometry_msgs::PoseArrayPtr &astarPath_msg_ptr);


private:
    ros::Subscriber subscriber_;
    std::deque<geometry_msgs::PoseArrayPtr> deque_astarPath;
    std::mutex buff_mutex_;

};


#endif //PATH_FINDER_ASTAR_PATH_SUBSCRIBER_H
