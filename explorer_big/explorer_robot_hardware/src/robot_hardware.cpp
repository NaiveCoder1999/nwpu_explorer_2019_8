/**
 * author : rescuer liao
 * date : 2015-4-20
 * last modified: 2019-8
 **/
#include "robot_hardware.h"
#include <std_msgs/Float64.h>
#include <algorithm>
#include <boost/make_shared.hpp>
#include <sstream>

/*注意频率值frep_ */
ExplorerHardware::ExplorerHardware(ros::NodeHandle node)
    : nh_(node),
      nh_private("~"),
      frep_(30.0),
      last_left_wheel_vel(0.0),
      last_right_wheel_vel(0.0),
      last_front_vice_wheel_pos(0.0),
      last_back_vice_wheel_pos(0.0),
      last_front_camera_pos(0.0),
      last_rear_camera_pos(0.0) {
    robot_drive_pub_ = nh_.advertise<explorer_msgs::explorer_message>("explorer_driver", 10);

    nh_.setCallbackQueue(&callback_queue_);

    //load base wheel joint name
    if (!this->getNameList(this->nh_, "/explorer_drive_controller/left_wheel", base_joint_name_) ||
        !this->getNameList(this->nh_, "/explorer_drive_controller/right_wheel", base_joint_name_)) {
        ROS_ERROR("get vice wheel name fail");
    } else {
        for (auto name : base_joint_name_) {
            ROS_ERROR_STREAM(name);
        }
    }

    //load vice wheel joint name
    if (!this->getNameList(this->nh_, "explorer_vice_wheel_controller/joints", vice_wheel_joint_name_)) {
        ROS_ERROR("get vice wheel name fail");
    }

    //load camera joint name
    if (!this->getNameList(this->nh_, "explorer_camera_controller/joints", camera_joint_name_)) {
        ROS_ERROR("get camera joint name fail");
    }

    // load arm joint name
    if (!this->getNameList(this->nh_, "/explorer_arm_controller/joints", arm_joint_name_)) {
        ROS_ERROR("get arm joints name fail");
    }

    //load the initialize position of the robot vice wheel
    for (auto it : vice_wheel_joint_name_) {
        if (!this->nh_.getParam("explorer_vice_wheel_controller/reset/" + it, vice_wheel_off_pos_[it])) {
            ROS_ERROR_STREAM("get reset position fail: " + ("explorer_vice_wheel_controller/reset/" + it));
            break;
        }
    }

    //load the initialize position of the camera joint
    for (auto it : camera_joint_name_) {
        if (!this->nh_.getParam("explorer_camera_controller/reset/" + it, camera_joint_off_pos_[it])) {
            ROS_ERROR_STREAM("get reset position fail: " + ("explorer_camera_controller/reset/" + it));
            break;
        }
    }

    // load the initialize position of the robot arm(getParam load values)
    for (auto it : arm_joint_name_) {
        if (!this->nh_.getParam("/explorer_arm_controller/reset/" + it, arm_joint_off_pos_[it])) {
            ROS_ERROR_STREAM("get reset position fail: " + ("/explorer_arm_controller/reset/" + it));
            break;
        }
    }

    //Initialize the arm joint handles
    for (auto name : arm_joint_name_) {
        arm_joint_pos_[name] = 0.0;
        this->nh_.getParam("/explorer_arm_controller/reset/" + name, arm_joint_pos_[name]);
        arm_joint_vel_[name] = 0.0;
        arm_joint_eff_[name] = 0.0;
        arm_joint_cmd_[name] = 0.0;
        //joint_pose_[name] = 0.0;

        hardware_interface::JointStateHandle arm_joint_state(
            name,
            &arm_joint_pos_[name],
            &arm_joint_vel_[name],
            &arm_joint_eff_[name]);
        joint_state_interface_.registerHandle(arm_joint_state);

        //绑定JointHandle和arm_joint_cmd_，从controller处的setCommand函数将角度赋值到arm_joint_cmd_
        hardware_interface::JointHandle arm_pose(
            joint_state_interface_.getHandle(name),
            &arm_joint_cmd_[name]);
        joint_pos_interface_.registerHandle(arm_pose);
    }

    //Initialize the base wheel joint handles
    for (auto name : base_joint_name_) {
        base_wheel_eff_[name] = 0.0;
        base_wheel_pos_[name] = 0.0;
        base_wheel_vel_[name] = 0.0;
        base_wheel_cmd_[name] = 0.0;

        hardware_interface::JointStateHandle base_wheel_joint_state(
            name,
            &base_wheel_pos_[name],
            &base_wheel_vel_[name],
            &base_wheel_eff_[name]);
        joint_state_interface_.registerHandle(base_wheel_joint_state);

        //绑定JointHandle和base_wheel_cmd_，从controller处的setCommand函数将速度赋值到base_wheel_cmd_
        hardware_interface::JointHandle base_wheel_vel(
            joint_state_interface_.getHandle(name),
            &base_wheel_cmd_[name]);
        joint_vel_interface_.registerHandle(base_wheel_vel);
    }

    //Initialize the vice wheel joint handles
    for (auto name : vice_wheel_joint_name_) {
        vice_wheel_eff_[name] = 0.0;
        vice_wheel_pos_[name] = 0.0;
        vice_wheel_vel_[name] = 0.0;
        vice_wheel_cmd_[name] = 0.0;
        //joint_pose_[name] = 0.0;

        hardware_interface::JointStateHandle vice_wheel_joint_state(
            name,
            &vice_wheel_pos_[name],
            &vice_wheel_vel_[name],
            &vice_wheel_eff_[name]);
        joint_state_interface_.registerHandle(vice_wheel_joint_state);

        //绑定JointHandle和vice_wheel_cmd_，从controller处的setCommand函数将角度赋值到vice_wheel_cmd_
        hardware_interface::JointHandle vice_wheel_pose(
            joint_state_interface_.getHandle(name),
            &vice_wheel_cmd_[name]);
        joint_pos_interface_.registerHandle(vice_wheel_pose);
    }

    //Initialize the camera joint handles
    for (auto name : camera_joint_name_) {
        camera_joint_eff_[name] = 0.0;
        camera_joint_pos_[name] = 0.0;
        camera_joint_vel_[name] = 0.0;
        camera_joint_cmd_[name] = 0.0;

        hardware_interface::JointStateHandle camera_joint_state(
            name,
            &camera_joint_pos_[name],
            &camera_joint_vel_[name],
            &camera_joint_eff_[name]);
        joint_state_interface_.registerHandle(camera_joint_state);

        //绑定JointHandle和camera_joint_cmd_，从controller处的setCommand函数将角度赋值到vice_wheel_cmd_
        hardware_interface::JointHandle camera_joint_pose(
            joint_state_interface_.getHandle(name),
            &camera_joint_cmd_[name]);
        joint_pos_interface_.registerHandle(camera_joint_pose);
    }

    //此处joint_state_interface_与rviz显示有关
    registerInterface(&joint_state_interface_);
    registerInterface(&joint_pos_interface_);
    registerInterface(&joint_vel_interface_);

    spinner_.reset(new ros::AsyncSpinner(1, &callback_queue_));
    spinner_->start();
    ROS_ERROR("2333");
    //for the sync callback quequ
}

ExplorerHardware::~ExplorerHardware() {}

bool ExplorerHardware::start() {
    for (auto name : vice_wheel_joint_name_) {
        vice_wheel_pos_[name] = vice_wheel_off_pos_[name];
        vice_wheel_vel_[name] = 0.0;
        vice_wheel_eff_[name] = 0.0;
    }

    for (auto name : camera_joint_name_) {
        camera_joint_pos_[name] = camera_joint_off_pos_[name];
        camera_joint_vel_[name] = 0.0;
        camera_joint_eff_[name] = 0.0;
    }

    return true;
}

void ExplorerHardware::stop() {
    spinner_->stop();
}

void ExplorerHardware::write(ros::Time current_time, ros::Duration period) {
    callback_queue_.callAvailable();
    bool base_move = false;
    float left_wheel_vel = std::max(fabs(base_wheel_cmd_["left_up_wheel_base_joint"]),
                                    fabs(base_wheel_cmd_["left_down_wheel_base_joint"]));
    float right_wheel_vel = std::max(fabs(base_wheel_cmd_["right_up_wheel_base_joint"]),
                                     fabs(base_wheel_cmd_["right_down_wheel_base_joint"]));

    float front_vice_wheel_pos = std::max(fabs(vice_wheel_cmd_["left_up_fin_base_joint"]),
                                          fabs(vice_wheel_cmd_["right_up_fin_base_joint"]));
    float back_vice_wheel_pos = std::max(fabs(vice_wheel_cmd_["left_down_fin_base_joint"]),
                                         fabs(vice_wheel_cmd_["right_down_fin_base_joint"]));

    float front_camera_pos = std::max(fabs(camera_joint_cmd_["front_camera_left_right_joint"]),
                                      fabs(camera_joint_cmd_["front_camera_up_down_joint"]));
    float rear_camera_pos = std::max(fabs(camera_joint_cmd_["rear_camera_left_right_joint"]),
                                     fabs(camera_joint_cmd_["rear_camera_up_down_joint"]));

    if (
        (left_wheel_vel || right_wheel_vel) || (last_left_wheel_vel && !left_wheel_vel) || (last_right_wheel_vel && !right_wheel_vel)) {
        pub_vel_cmd();
        base_move = true;
    }

    if ((front_vice_wheel_pos || (last_front_vice_wheel_pos && !front_vice_wheel_pos)) || (back_vice_wheel_pos || (last_back_vice_wheel_pos && !back_vice_wheel_pos))) {
        pub_vice_wheel_cmd();
        base_move = true;
    }

    if ((front_camera_pos || (last_front_camera_pos && !front_camera_pos)) || (rear_camera_pos || (last_rear_camera_pos && !rear_camera_pos))) {
        pub_camera_joint_cmd();
        base_move = true;
    }

    if (!base_move) {
        //no command to control the robot base ,
        //so arm command can send to the robot
        pub_arm_joint_cmd();
    }

    //垃圾操作，强制刷新joint_states里面的四个传动轮让姿态显示实时，解决官方bug...
    for (auto name : base_joint_name_) {
        base_wheel_pos_[name] += 1.0;
    }

    last_left_wheel_vel = left_wheel_vel;
    last_right_wheel_vel = right_wheel_vel;
    last_front_vice_wheel_pos = front_vice_wheel_pos;
    last_back_vice_wheel_pos = back_vice_wheel_pos;
}

void ExplorerHardware::read(ros::Time current_time, ros::Duration period) {
    //get current robot joint state
    callback_queue_.callAvailable();
}

/*
 * 以下为主履带速度数据
 * 各个履带速度分别发送,以满足小explorer和大explorer的要求
 */
void ExplorerHardware::pub_vel_cmd() {
    for (auto name : base_joint_name_) {
        base_wheel_vel_[name] = base_wheel_cmd_[name];
        base_wheel_pos_[name] += 1.0;
        base_wheel_eff_[name] = 0.0;
    }

    explorer_msgs::explorer_message move_message;
    move_message.low_level_id = 1;
    move_message.high_level_id = 1;
    move_message.data.push_back(base_wheel_cmd_["left_up_wheel_base_joint"]);
    //复位指令发送left_down_wheel_base_joint
    move_message.data.push_back(base_wheel_cmd_["right_up_wheel_base_joint"]);
    //        ROS_INFO("pub move order") ;
    robot_drive_pub_.publish(move_message);
    move_message.low_level_id = 2;
    move_message.high_level_id = 2;
    move_message.data.clear();                                                   //清空id1的八位数据，开始发送id2
    move_message.data.push_back(base_wheel_cmd_["left_down_wheel_base_joint"]);  //用于重启底盘
    move_message.data.push_back(base_wheel_cmd_["right_down_wheel_base_joint"]);
    robot_drive_pub_.publish(move_message);
}
/*
 * 以下为副履带的底层数据传输函数
 */
void ExplorerHardware::pub_vice_wheel_cmd() {
    for (auto name : vice_wheel_joint_name_) {
        vice_wheel_pos_[name] += vice_wheel_cmd_[name];
        vice_wheel_vel_[name] = 0.0;
        vice_wheel_eff_[name] = 0.0;
    }

    if (vice_wheel_cmd_["left_up_fin_base_joint"] != 0.0 || vice_wheel_cmd_["right_up_fin_base_joint"] != 0.0) {
        explorer_msgs::explorer_message vice_wheel_message;
        vice_wheel_message.low_level_id = 3;
        vice_wheel_message.high_level_id = 3;
        vice_wheel_message.data.push_back(-(vice_wheel_pos_["left_up_fin_base_joint"]));
        vice_wheel_message.data.push_back(-(vice_wheel_pos_["right_up_fin_base_joint"]));
        vice_wheel_cmd_["left_up_fin_base_joint"] = 0.0;
        vice_wheel_cmd_["right_up_fin_base_joint"] = 0.0;
        robot_drive_pub_.publish(vice_wheel_message);
    }

    if (vice_wheel_cmd_["left_down_fin_base_joint"] != 0.0 || vice_wheel_cmd_["right_down_fin_base_joint"] != 0.0) {
        explorer_msgs::explorer_message vice_wheel_message;
        vice_wheel_message.low_level_id = 4;
        vice_wheel_message.high_level_id = 4;
        vice_wheel_message.data.clear();  //清空id3的八位数据，开始发送id4
        vice_wheel_message.data.push_back(vice_wheel_pos_["left_down_fin_base_joint"]);
        vice_wheel_message.data.push_back(vice_wheel_pos_["right_down_fin_base_joint"]);
        vice_wheel_cmd_["left_down_fin_base_joint"] = 0.0;
        vice_wheel_cmd_["right_down_fin_base_joint"] = 0.0;
        robot_drive_pub_.publish(vice_wheel_message);
    }
}

/*
 * 以下为机械臂的底层数据传输函数
 * 注意! 将机械臂在ros中的移动与显示中的数据整合在一起,以保证计算机"认为"的机械臂位置数据与实际位置尽可能的一致
 */
void ExplorerHardware::pub_arm_joint_cmd() {
    for (auto name : arm_joint_name_) {
        arm_joint_pos_[name] += arm_joint_cmd_[name];  //此处是pos+cmd 也就是相对角度
        arm_joint_vel_[name] = 0.0;
        arm_joint_eff_[name] = 0.0;
        //arm_joint_cmd_[name] = 0.0;
    }

    static double last_arm1_bearing_joint = 0.0;
    //发送底座左右移动信号
    if (arm_joint_cmd_["arm1_bearing_joint"] != 0.0) {
        explorer_msgs::explorer_message arm_rotate_joint_message;
        arm_rotate_joint_message.low_level_id = 5;
        arm_rotate_joint_message.high_level_id = 5;
        arm_rotate_joint_message.data.push_back(arm_joint_pos_["arm1_bearing_joint"]);
        last_arm1_bearing_joint = arm_joint_pos_["arm1_bearing_joint"];
        arm_joint_cmd_["arm1_bearing_joint"] = 0.0;
        robot_drive_pub_.publish(arm_rotate_joint_message);
    }

    static double last_arm2_arm1_joint = 0.0, last_arm3_arm2_joint = 0.0;
    //发送大臂上下和小臂上下移动信号,此处改变方向最快捷
    if (arm_joint_cmd_["arm2_arm1_joint"] != 0.0 || arm_joint_cmd_["arm3_arm2_joint"] != 0.0) {
        explorer_msgs::explorer_message arm_move_joint_message;
        arm_move_joint_message.low_level_id = 6;
        arm_move_joint_message.high_level_id = 6;
        arm_move_joint_message.data.push_back(arm_joint_pos_["arm2_arm1_joint"]);
        arm_move_joint_message.data.push_back(arm_joint_pos_["arm3_arm2_joint"]);
        last_arm2_arm1_joint = arm_joint_pos_["arm2_arm1_joint"];
        last_arm3_arm2_joint = arm_joint_pos_["arm3_arm2_joint"];
        arm_joint_cmd_["arm2_arm1_joint"] = 0.0;
        arm_joint_cmd_["arm3_arm2_joint"] = 0.0;
        robot_drive_pub_.publish(arm_move_joint_message);
    }

    static double last_pt1_arm_joint = 0.0, last_pt2_pt1_joint = 0.0;
    //发送爪子左右和爪子上下移动信号
    if (arm_joint_cmd_["pt1_arm_joint"] != 0.0 || arm_joint_cmd_["pt2_pt1_joint"] != 0.0) {
        explorer_msgs::explorer_message camera_move_joint_message;
        camera_move_joint_message.high_level_id = 7;
        camera_move_joint_message.low_level_id = 7;
        camera_move_joint_message.data.push_back(arm_joint_pos_["pt1_arm_joint"]);
        camera_move_joint_message.data.push_back(-arm_joint_pos_["pt2_pt1_joint"]);
        last_pt1_arm_joint = arm_joint_pos_["pt1_arm_joint"];
        last_pt2_pt1_joint = -arm_joint_pos_["pt2_pt1_joint"];
        arm_joint_cmd_["pt1_arm_joint"] = 0.0;
        arm_joint_cmd_["pt2_pt1_joint"] = 0.0;
        robot_drive_pub_.publish(camera_move_joint_message);
    }

    static double last_gripper_joint = 0.0, last_rotate_joint = 0.0;
    //发送爪子左右和爪子上下移动信号
    if (arm_joint_cmd_["gripper_joint"] != 0.0 || arm_joint_cmd_["rotate_joint"] != 0.0) {
        explorer_msgs::explorer_message paw_move_joint_message;
        paw_move_joint_message.high_level_id = 8;
        paw_move_joint_message.low_level_id = 8;
        paw_move_joint_message.data.push_back(arm_joint_pos_["gripper_joint"]);
        paw_move_joint_message.data.push_back(arm_joint_pos_["rotate_joint"]);
        last_gripper_joint = arm_joint_pos_["gripper_joint"];
        last_rotate_joint = arm_joint_pos_["rotate_joint"];
        arm_joint_cmd_["gripper_joint"] = 0.0;
        arm_joint_cmd_["rotate_joint"] = 0.0;
        robot_drive_pub_.publish(paw_move_joint_message);
    }
}

/*
 * 以下为摄像头舵机的底层数据传输函数
 */
void ExplorerHardware::pub_camera_joint_cmd() {
    for (auto name : camera_joint_name_) {
        camera_joint_pos_[name] += camera_joint_cmd_[name];
        camera_joint_vel_[name] = 0.0;
        camera_joint_eff_[name] = 0.0;
    }

    if (camera_joint_cmd_["front_camera_up_down_joint"] != 0.0 || camera_joint_cmd_["front_camera_left_right_joint"] != 0.0) {
        explorer_msgs::explorer_message camera_joint_message;
        camera_joint_message.low_level_id = 12;
        camera_joint_message.high_level_id = 12;
        camera_joint_message.data.push_back(camera_joint_pos_["front_camera_up_down_joint"]);
        camera_joint_message.data.push_back(camera_joint_pos_["front_camera_left_right_joint"]);
        camera_joint_cmd_["front_camera_up_down_joint"] = 0.0;
        camera_joint_cmd_["front_camera_left_right_joint"] = 0.0;
        robot_drive_pub_.publish(camera_joint_message);
    }

    if (camera_joint_cmd_["rear_camera_up_down_joint"] != 0.0 || camera_joint_cmd_["rear_camera_left_right_joint"] != 0.0) {
        explorer_msgs::explorer_message camera_joint_message;
        camera_joint_message.low_level_id = 13;
        camera_joint_message.high_level_id = 13;
        camera_joint_message.data.push_back(camera_joint_pos_["rear_camera_up_down_joint"]);
        camera_joint_message.data.push_back(camera_joint_pos_["rear_camera_left_right_joint"]);
        camera_joint_cmd_["rear_camera_up_down_joint"] = 0.0;
        camera_joint_cmd_["rear_camera_left_right_joint"] = 0.0;
        robot_drive_pub_.publish(camera_joint_message);
    }

}

ros::Duration ExplorerHardware::getPeriod() {
    return ros::Duration(1 / frep_);
}

ros::Time ExplorerHardware::getCurrentTime() {
    return ros::Time::now();
}

ros::CallbackQueue *ExplorerHardware::getCallBackQuequ() {
    return &callback_queue_;
}
float ExplorerHardware::getFrep() {
    return frep_;
}

/*
 * 从参数服务器(ros.param)中获取名称列表(其实是从diff_wheel_controller中抄过来的)
 * 参数 controller_nh ros::NodeHandle     读取参数服务器数据的节点(注意其命名空间)
 * 参数 name_param    std::string         读取参数服务器数据的名称
 * 参数 names         std::vector<string> 读取参数的保存数组
 */
bool ExplorerHardware::getNameList(ros::NodeHandle controller_nh, const std::string name_param, std::vector<std::string> &names) {
    XmlRpc::XmlRpcValue name_list;

    if (!controller_nh.getParam(name_param, name_list)) {
        ROS_ERROR("can not get param list!!");
        return false;
    }

    if (name_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
        if (name_list.size() == 0) {
            ROS_ERROR("did not get param name");
            return false;
        }

        for (int i = 0; i < name_list.size(); i++) {
            if (name_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                ROS_ERROR("get the error param list");
                return false;
            }
        }

        //names.resize(name_list.size());

        for (int i = 0; i < name_list.size(); i++) {
            names.push_back(static_cast<std::string>(name_list[i]));
        }
    } else if (name_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
        names.push_back(name_list);
    } else {
        ROS_ERROR("the param get error");
        return false;
    }

    return true;
}
