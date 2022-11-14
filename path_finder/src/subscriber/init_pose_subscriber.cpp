//
// Created by zzjun on 8/11/22.
//

#include "path_finder/subscriber/init_points_subscriber.h"


InitPoseSubscriber2D::InitPoseSubscriber2D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(
            topic_name, buff_size, &InitPoseSubscriber2D::MessageCallBack, this
    );
}

void InitPoseSubscriber2D::ParseData(std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> &pose_data_buff) {
    buff_mutex_.lock();
    if (!init_poses_.empty()) {
        pose_data_buff.insert(pose_data_buff.end(), init_poses_.begin(), init_poses_.end());
        init_poses_.clear();
    }
    buff_mutex_.unlock();
}

void InitPoseSubscriber2D::MessageCallBack(const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_ptr) {
    buff_mutex_.lock();
    init_poses_.emplace_back(init_pose_ptr);
    buff_mutex_.unlock();
}



InitPoseSubscriber3D::InitPoseSubscriber3D(ros::NodeHandle &nh, const std::string &topic_name, size_t buff_size) {
    subscriber_ = nh.subscribe(
            topic_name, buff_size, &InitPoseSubscriber3D::MessageCallBack, this
    );
}

void InitPoseSubscriber3D::ParseData(std::deque<geometry_msgs::PoseStampedPtr> &pose_data_buff) {
    buff_mutex_.lock();
    if (!init_poses_.empty()) {
        pose_data_buff.insert(pose_data_buff.end(), init_poses_.begin(), init_poses_.end());
        init_poses_.clear();
    }
    buff_mutex_.unlock();
}

void InitPoseSubscriber3D::MessageCallBack(const geometry_msgs::PoseStampedPtr &init_pose_ptr) {
    buff_mutex_.lock();
    init_poses_.emplace_back(init_pose_ptr);
    buff_mutex_.unlock();
}

