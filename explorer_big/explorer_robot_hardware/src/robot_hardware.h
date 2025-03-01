#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

#include <controller_manager/controller_manager.h>
#include <explorer_msgs/explorer_joint.h>
#include <explorer_msgs/explorer_low_level_data.h>
#include <explorer_msgs/explorer_message.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
const double PI = acos(0.0) * 2;

class ExplorerHardware : public hardware_interface::RobotHW {
   public:
    ExplorerHardware(ros::NodeHandle node);
    ~ExplorerHardware();
    bool start();
    void stop();
    void read(ros::Time current_time, ros::Duration period);
    void write(ros::Time current_time, ros::Duration period);
    ros::Duration getPeriod();
    ros::Time getCurrentTime();
    ros::CallbackQueue *getCallBackQuequ();
    float getFrep();
    bool getNameList(ros::NodeHandle, const ::std::string, ::std::vector<::std::string> &);

   protected:

   private:
    // 用于被joint_state_controller调用,注意这个只能读取不能调用
    ::hardware_interface::JointStateInterface joint_state_interface_;

    // 用于被arm_controller & vice_wheel_controller & camera_controller调用
    ::hardware_interface::PositionJointInterface joint_pos_interface_;

    //被diff_wheel_controller 调用
    ::hardware_interface::VelocityJointInterface joint_vel_interface_;

    ::ros::NodeHandle nh_;
    ::ros::NodeHandle nh_private;
    ::ros::Publisher robot_drive_pub_;

    //::sensor_msgs::JointStateConstPtr current_joint_state_ptr_;

    ::ros::CallbackQueue callback_queue_;

    ::std::vector<::std::string> base_joint_name_;        //base wheel joint name
    ::std::vector<::std::string> vice_wheel_joint_name_;  //vice wheel joint name
    ::std::vector<::std::string> arm_joint_name_;         //arm joint name
    ::std::vector<::std::string> camera_joint_name_;         //arm joint name

    //轮子(履带)的joint_handle
    ::std::map<::std::string, double> base_wheel_pos_;
    ::std::map<::std::string, double> base_wheel_cmd_;
    ::std::map<::std::string, double> base_wheel_eff_;
    ::std::map<::std::string, double> base_wheel_vel_;
    //副履带的joint_handle
    ::std::map<::std::string, double> vice_wheel_pos_;
    ::std::map<::std::string, double> vice_wheel_cmd_;
    ::std::map<::std::string, double> vice_wheel_eff_;
    ::std::map<::std::string, double> vice_wheel_vel_;
    ::std::map<::std::string, double> vice_wheel_off_pos_;  //the initialize position of the robot vice wheel
    //机械臂的joint_handle
    ::std::map<::std::string, double> arm_joint_pos_;  //传向rviz的角度位置信息
    ::std::map<::std::string, double> arm_joint_cmd_;  //从controller传来的命令信息
    ::std::map<::std::string, double> arm_joint_eff_;
    ::std::map<::std::string, double> arm_joint_vel_;
    ::std::map<::std::string, double> arm_joint_off_pos_;  //the initialize position of the robot arm
    //二自由度摄像头的joint_handle
    ::std::map<::std::string, double> camera_joint_pos_;  //传向rviz的角度位置信息
    ::std::map<::std::string, double> camera_joint_cmd_;  //从controller传来的命令信息
    ::std::map<::std::string, double> camera_joint_eff_;
    ::std::map<::std::string, double> camera_joint_vel_;
    ::std::map<::std::string, double> camera_joint_off_pos_;

    bool base_move;

    double last_front_vice_wheel_pos;
    double last_back_vice_wheel_pos;
    double last_left_wheel_vel;
    double last_right_wheel_vel;
    double last_front_camera_pos;
    double last_rear_camera_pos;

    float frep_;

    ::boost::shared_ptr<ros::AsyncSpinner> spinner_;

    void pub_vel_cmd();
    void pub_vice_wheel_cmd();
    void pub_arm_joint_cmd();
    void pub_camera_joint_cmd();

};

#endif  // ROBOT_HARDWARE_H
