#include "explorer_camera_controller.h"

using namespace explorer_camera_controller;

CameraController::CameraController() {}
CameraController::~CameraController() {}

bool CameraController::init(hardware_interface::PositionJointInterface *joint_handle_,
                            ros::NodeHandle &root_nh,
                            ros::NodeHandle &controller_nh) {
    // 获取名称列表
    if (!getNameList(controller_nh, "joints", this->camera_joint_names)) {
        ROS_ERROR("can not get name list!");
        return false;
    }

    // 建立描述对象
    for (auto it : camera_joint_names) {
        joint_map.insert(std::make_pair(it, new joint_with_limit(it, controller_nh, *joint_handle_)));
    }

    camera_sub_ = controller_nh.subscribe("explorer_camera", 10, &CameraController::cameraCallBack, this);
    camera_reset_sub_ = controller_nh.subscribe("explorer_camera_reset", 10, &CameraController::resetCommandCallBack, this);
    //来自explorer_joy_control
    ROS_ERROR("camera init successed");
    return true;
}
/*
 *数据的数据刷新
 */
void CameraController::update(const ros::Time &time, const ros::Duration &period) {
    ros::Time now_time = time;
    // 计算与上一次的刷新的时间差
    double duration = fabs((elapsed_time.toSec() - now_time.toSec()));
    elapsed_time = now_time;

    // 位置达到限制,标红
    for (auto name : camera_joint_names) {
        if (joint_map[name]->moveWillDanger()) {
            ROS_ERROR_STREAM(name << " :\t" << joint_map[name]->getNowPose());
        } else {
            ROS_INFO_STREAM(name << " :\t" << joint_map[name]->getNowPose());
        }
    }

    ROS_INFO("\n");

    // 检测是否需要复位
    if (!reset_queue.empty()) {
        std::vector<std::pair<std::string, double>> line;
        // 取出当前的目标
        line = reset_queue.front();
        int get_goal = 0;

        for (auto name : line) {
            joint_map[name.first]->setAim(name.second);

            if (joint_map[name.first]->moveToAim(duration)) {
                // 记录达到目标的数量
                ++get_goal;
            }
        }

        // 全部达到目标,转入下一个位置
        if (get_goal == line.size()) {
            reset_queue.pop();
        }

        return;
    }

    // 建议对map的遍历使用iterator保证效率
    for (auto it : joint_map) {
        it.second->moveToAim(duration);
    }
}

//摄像头舵机移动控制
void CameraController::cameraCallBack(const geometry_msgs::Twist &msg) {
    if (msg.linear.x != 0 || msg.linear.y != 0 || msg.linear.z != 0 ||
        msg.angular.x != 0 || msg.angular.y != 0 || msg.angular.z != 0) {
        while (reset_queue.size()) {
            reset_queue.pop();
        }
    }

    joint_map["front_camera_left_right_joint"]->setAim(joint_map["front_camera_left_right_joint"]->getNowPose() + msg.linear.y);
    joint_map["front_camera_up_down_joint"]->setAim(joint_map["front_camera_up_down_joint"]->getNowPose() + msg.linear.x);
    joint_map["rear_camera_left_right_joint"]->setAim(joint_map["rear_camera_left_right_joint"]->getNowPose() + msg.angular.y);
    joint_map["rear_camera_up_down_joint"]->setAim(joint_map["rear_camera_up_down_joint"]->getNowPose() + msg.angular.x);
}

//舵机位姿重置
void CameraController::resetCommandCallBack(const explorer_msgs::explorer_camera_reset &msg) {
    reset_msg = msg;
    if (reset_msg.front_camera_reset && !(reset_msg.rear_camera_reset)) {
        joint_map["front_camera_left_right_joint"]->setAim(0.0);
        joint_map["front_camera_up_down_joint"]->setAim(0.0);
    }

    if (!(reset_msg.front_camera_reset) && reset_msg.rear_camera_reset) {
        joint_map["rear_camera_left_right_joint"]->setAim(0.0);
        joint_map["rear_camera_up_down_joint"]->setAim(0.0);
    }

    if (reset_msg.front_camera_reset && reset_msg.rear_camera_reset) {
        joint_map["front_camera_left_right_joint"]->setAim(0.0);
        joint_map["front_camera_up_down_joint"]->setAim(0.0);
        joint_map["rear_camera_left_right_joint"]->setAim(0.0);
        joint_map["rear_camera_up_down_joint"]->setAim(0.0);
    }
}

bool CameraController::getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names) {
    XmlRpc::XmlRpcValue name_list;

    if (!controller_nh.getParam(name_param, name_list)) {
        ROS_ERROR("can not get the joint list!");
        return false;
    }

    if (name_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (name_list.size() == 0) {
            ROS_ERROR("did not get the joint name!");
            return false;
        }

        for (int i = 0; i < name_list.size(); i++) {
            if (name_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("get the error name list!");
                return false;
            }
        }

        names.resize(name_list.size());

        for (int i = 0; i < name_list.size(); i++) {
            names[i] = static_cast<std::string>(name_list[i]);
        }
    } else if (name_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        names.push_back(name_list);
    } else {
        ROS_ERROR("the wheel param get error");
        return false;
    }

    return true;
}
