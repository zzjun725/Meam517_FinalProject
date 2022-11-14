//
// Created by zzjun on 8/25/22.
//

#include "path_finder/subscriber/astar_path_subscriber.h"

AstarPathSubscriber::AstarPathSubscriber(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(topic_name, buff_size, &AstarPathSubscriber::MessageCallBack, this);
}

void AstarPathSubscriber::ParseData(std::deque<geometry_msgs::PoseArrayPtr> &deque_astarPath_msg_ptr) {
    buff_mutex_.lock();
    if (!deque_astarPath.empty()) {
        // std::cout << "get astar path!" << std::endl;
        deque_astarPath_msg_ptr.insert(
                deque_astarPath_msg_ptr.end(),
                deque_astarPath.begin(),
                deque_astarPath.end()
        );

        deque_astarPath.clear();
    }
    buff_mutex_.unlock();
}

void AstarPathSubscriber::MessageCallBack(const geometry_msgs::PoseArrayPtr &astarPath_msg_ptr) {
    buff_mutex_.lock();
    // std::cout << "call back!" << std::endl;
    deque_astarPath.emplace_back(astarPath_msg_ptr);
    buff_mutex_.unlock();
}
