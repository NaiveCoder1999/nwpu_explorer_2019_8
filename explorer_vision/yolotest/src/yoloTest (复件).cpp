#include <iostream>
#include <iomanip>
#include <vector>
#include <fstream>

#include <opencv2/opencv.hpp>            // C++
#include <opencv2/highgui/highgui_c.h>   // C
#include <opencv2/imgproc/imgproc_c.h>   // C
#include "yolo_v2_class.hpp" 
//添加qrcode头文件
#include <hector_worldmodel_msgs/ImagePercept.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <cv.h>
//#include<opencv2/core/core.hpp>
//#include<opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>


using namespace std;
using namespace cv;


std::string  names_file = "/home/szj/darknet/data/coco.names";
std::string  cfg_file = "/home/szj/catkin_ws/src/nwpu_explorer/explorer_vision/yolotest/train_weight_cfg/yolov2.cfg";
std::string  weights_file = "/home/szj/catkin_ws/src/nwpu_explorer/explorer_vision/yolotest/train_weight_cfg/yolov2_final.weights";

bool flag_l=false;
bool flag_r=false;
//image_transport::ImageTransport percept_publisher_;
//sensor_msgs::CameraInfoConstPtr right_info;
//sensor_msgs::CameraInfoConstPtr left_info;
sensor_msgs::CameraInfoPtr right_info;
sensor_msgs::CameraInfoPtr left_info;
sensor_msgs::CompressedImagePtr image_right;
sensor_msgs::CompressedImagePtr image_left;


/* 
void leftinfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info){
    if(!flag_l)
    {
        flag_l=true;
        //当镜头的布尔变量为false时
        //代表镜头暂时没有传递数据
        //需要对其进行处理
        //设置布尔变量为true时代表数据接口已经ok
        left_info =camera_info;
        ROS_ERROR("run the left");
    }
}

void rightinfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info){
    if(!flag_r)
    {
        flag_r=true;
        right_info=camera_info;
        ROS_ERROR("run the right");
    }
}
*/


void leftinfoCallback(const sensor_msgs::CameraInfoPtr &camera_info)
{
        //当镜头的布尔变量为false时
        //代表镜头暂时没有传递数据
        //需要对其进行处理
        //设置布尔变量为true时代表数据接口已经ok
        //ROS_ERROR("left_info");
        left_info = camera_info;
}

void rightinfoCallback(const sensor_msgs::CameraInfoPtr &camera_info)
{
        //ROS_ERROR("right_info");
        right_info=camera_info;

}

//两个镜头共用同一个回调函数
//在其中利用布尔变量进行筛选
void imageCallback_right(const sensor_msgs::CompressedImagePtr& image)
{
    //ROS_ERROR("miao miaomia0  right");
    //if(!flag_r)
    if (flag_r)
		ROS_WARN("Detection already running, message omitted");
	else
    {
        //ROS_ERROR("get a right image");
		flag_r = true;
		image_right = image;
    }
    return;
}

void imageCallback_left(const sensor_msgs::CompressedImagePtr& image)
{
    //ROS_ERROR("miao miaomia0     left");
    //if(!flag_r)
    if (flag_l)
		ROS_WARN("Detection already running, message omitted");
	else
    {
        //ROS_ERROR("get a left image");
		flag_l = true;
		image_left = image;
    }
    return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolo_test");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Info);
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh;
    ros::NodeHandle nh_(nh);
    image_transport::ImageTransport image_transport_(nh_);
    ros::Subscriber sub_camera1;
    ros::Subscriber sub_camera2;
    ros::Subscriber info_camera1;
    ros::Subscriber info_camera2;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Rect rect;
 
    std::string pub_qrcode_topic_= "image_percept";
    std::string sub_image_qrcode_topic_camera1_= "/camera_right/image_raw/compressed";
    std::string sub_image_qrcode_topic_camera2_= "/camera_left/image_raw/compressed";
    std::string camera_info1_= "/camera_right/camera_info";
    std::string camera_info2_= "/camera_left/camera_info";
    int orderNum=0;
    ros::Publisher percept_publisher_;
    Detector detector(cfg_file, weights_file);
  
    
    //std::string sub_image_qrcode_topic_camera1_;
    //std::string sub_image_qrcode_topic_camera2_;

    //std::string camera_info1_;
    //std::string camera_info2_;
    //ROS_ERROR("In qrcode_detection.cpp    impl()");

  //priv_nh.getParam("rotation_source_frame", rotation_source_frame_id_);
  //priv_nh.getParam("rotation_target_frame", rotation_target_frame_id_);
  //priv_nh.getParam("rotation_image_size", rotation_image_size_);
  //暂时规避变量名的问题
  /*
  //std::string pub_qrcode_topic_= "image_percept";
    //std::string sub_image_qrcode_topic_camera1_= "/camera_right/image_raw/compressed";
    //std::string sub_image_qrcode_topic_camera2_= "/camera_left/image_raw/compressed";
    //std::string camera_info1_= "/camera_right/camera_info";
    //std::string camera_info2_= "/camera_left/camera_info";
   */
    

    
/* 
    priv_nh.param("pub_qrcode_topic", pub_qrcode_topic_, std::string("image_percept"));
    //priv_nh.getParam("sub_image_qrcode_topic_camera1", sub_image_qrcode_topic_camera1_，std::string("/camera_right/image_raw/compressed"));
    //priv_nh.getParam("sub_image_qrcode_topic_camera2", sub_image_qrcode_topic_camera2_，std::string("/camera_left/image_raw/compressed"));
    priv_nh.Param("sub_image_qrcode_topic_camera1", sub_image_qrcode_topic_camera1_，std::string("/camera_right/image_raw/compressed"));
    priv_nh.Param("sub_image_qrcode_topic_camera2", sub_image_qrcode_topic_camera2_，std::string("/camera_left/image_raw/compressed"));
    priv_nh.param("camera_info1", camera_info1_, std::string("/camera_right/camera_info"));
    priv_nh.param("camera_info2", camera_info2_, std::string("/camera_left/camera_info"));
*/
//!!!!!有问题啊直接从launch获取参数!!!!!!!!!!@@@@@@@@@@@@@@@@@@@@@

    priv_nh.getParam("pub_qrcode_topic", pub_qrcode_topic_);
    priv_nh.getParam("sub_image_qrcode_topic_camera1", sub_image_qrcode_topic_camera1_);
    priv_nh.getParam("sub_image_qrcode_topic_camera2", sub_image_qrcode_topic_camera2_);
    priv_nh.getParam("camera_info1", camera_info1_);
    priv_nh.getParam("camera_info2", camera_info2_);
 
    
    

  //priv_nh.getParam("camera_info1", camera_info1_);
  //priv_nh.getParam("camera_info2", camera_info2_);

  //priv_nh.param("camera_frame",camera_frame_,std::string("camera_color_optical_frame"));
  //priv_nh.param("camera_frame",camera_frame_,std::string("camera_right"));
  //灰度图像显示的场景
  //ROS_ERROR("%s,%s",camera_info1_,camera_info2_);

  //myadd
  // priv_nh.param("sub_image_qrcode_topic_front_left_camera", sub_image_qrcode_topic_front_left_camera_, std::string("/front_left_camera/image_raw"));
  // priv_nh.param("sub_image_qrcode_topic_front_right_camera", sub_image_qrcode_topic_front_right_camera_, std::string("/front_right_camera/image_raw"));
   // priv_nh.param("sub_image_qrcode_topic_back_left_camera", sub_image_qrcode_topic_back_left_camera_, std::string("/back_left_camera/image_raw"));
   // priv_nh.param("sub_image_qrcode_topic_back_right_camera", sub_image_qrcode_topic_back_right_camera_, std::string("/back_right_camera/image_raw"));
   //
   //priv_nh.param("pub_find_qrcode_topic", pub_find_qrcode_topic_, std::string("/find_qrcode"));


    
    
    sub_camera1=nh_.subscribe(sub_image_qrcode_topic_camera1_,10,imageCallback_right);
    info_camera1=nh_.subscribe(camera_info1_,10,rightinfoCallback);
    sub_camera2=nh_.subscribe(sub_image_qrcode_topic_camera2_,10,imageCallback_left);
    info_camera2=nh_.subscribe(camera_info2_,10,leftinfoCallback);
    percept_publisher_ = nh_.advertise<hector_worldmodel_msgs::ImagePercept>(pub_qrcode_topic_, 10);

    
    ros::Rate loop_rate(30);
    while (ros::ok())
    {
        if(flag_r)
        {
            loop_rate.sleep();
            cv_ptr = cv_bridge::toCvCopy(image_right, sensor_msgs::image_encodings::BGR8);
            Mat img_org = cv_ptr->image;
            hector_worldmodel_msgs::ImagePercept percept;
            percept.camera_info = *right_info;
            percept.header = image_right->header;
            percept.info.class_support = 1.0;
            percept.info.object_support = 1.0;
            std::vector<bbox_t> result_vec = detector.detect(img_org);
            for (auto &i : result_vec) 
            {
                //if(i.w>i.h)
                //    continue;
                //对检测的矩形框进行简单筛选
                /*
                if(i.w<430||i.w>445)
                    continue;
                if(i.h<325||i.h>335)
                    continue;
                 */
                //if(float(1.0*i.w/i.h)<1.3||float(1.0*i.w/i.h)>1.4)
                //    continue;
                //if(i.prob <= 0.85)
                //    continue;
                //if(float(1.0*i.w/i.h)>1.32 || float(1.0*i.w/i.h)<1.22 || i.prob<=0.85)
                //    continue;
                
                    ROS_ERROR("%lf",float(1.0*i.w/i.h));
                    percept.x = float(i.x + i.w/2.0);
                    percept.y = float(i.y + i.h/2.0);
                    percept.width = float(i.w);
                    percept.height = float(i.h);
                    orderNum+=1;
                    std::string name = "victim";
                    std::stringstream ss;
                    ss << name << orderNum;
                    percept.info.class_id = ss.str();
                    percept_publisher_.publish(percept);
                    //对图像进行画图操作
                    rect = cv::Rect(i.x, i.y, i.w, i.h);
                    //ROS_ERROR("%d",i.w);
                    //ROS_ERROR("%d",i.h);
                    ROS_ERROR();
                    //cout<<"i.x"<<i.x<<endl;
                    //cout<<"i.y"<<i.y<<endl;
                    //cout<<"i.w"<<i.w<<endl;
                    //cout<<"i.h"<<i.h<<endl;
                    //i中的信息包括左上焦点的坐标和宽度和高度
                    cv::rectangle(img_org, rect, cv::Scalar(0,255,255), 2);
                
                
            }
            resize(img_org, img_org, Size(640, 480));
            imshow("camera_right",img_org);
            waitKey(3);
            flag_r = false;
        }
        else{
            loop_rate.sleep();
        }

        if(flag_l)
        {
            loop_rate.sleep();
            cv_ptr = cv_bridge::toCvCopy(image_left, sensor_msgs::image_encodings::BGR8);
            Mat img_org = cv_ptr->image;
            hector_worldmodel_msgs::ImagePercept percept;
            percept.camera_info = *left_info;
            percept.header = image_left->header;
            percept.info.class_support = 1.0;
            percept.info.object_support = 1.0;
            std::vector<bbox_t> result_vec = detector.detect(img_org);
            for (auto &i : result_vec) 
            {
                //对检测的矩形框进行简单筛选
                //if(i.w>i.h)
                //    continue;
                /*
                if(i.w<430||i.w>445)
                    continue;
                if(i.h<325||i.h>335)
                    continue;
                     */
                //if(float(1.0*i.w/i.h)<1.3||float(1.0*i.w/i.h)>1.4)
                //    continue; 
                //if(i.prob <= 0.85)
                //    continue;
                //if(float(1.0*i.w/i.h)>1.32 || float(1.0*i.w/i.h)<1.22 || i.prob<=0.85)
                //    continue;

                    ROS_ERROR("%lf",i.prob);;
                    percept.x = float(i.x + i.w/2.0);
                    percept.y = float(i.y + i.h/2.0);
                    percept.width = float(i.w);
                    percept.height = float(i.h);
                    orderNum+=1;
                    std::string name = "victim";
                    std::stringstream ss;
                    ss << name << orderNum;
                    percept.info.class_id = ss.str();
                    percept_publisher_.publish(percept);
                    //对图像进行画图操作
                    rect = cv::Rect(i.x, i.y, i.w, i.h);
                    //cout<<"i.x"<<i.x<<endl;
                    //cout<<"i.y"<<i.y<<endl;
                    //cout<<"i.w"<<i.w<<endl;
                    //cout<<"i.h"<<i.h<<endl;
                    //i中的信息包括左上焦点的坐标和宽度和高度
                    cv::rectangle(img_org, rect, cv::Scalar(0,255,255), 2);
                
            }
            resize(img_org, img_org, Size(640, 480));
            imshow("camera_left",img_org);
            waitKey(3);
            flag_l = false;
        }
        else{
            loop_rate.sleep();
        }
        ros::spinOnce();
    }
    
    return 0;
    
}








































/* 
std::vector<std::string> objects_names_from_file(std::string const filename) {
    std::ifstream file(filename);
    std::vector<std::string> file_lines;
    if (!file.is_open()) return file_lines;
    for(std::string line; getline(file, line);) file_lines.push_back(line);
    std::cout << "object names loaded \n";
    return file_lines;
}

int main(int argc, char ** argv)
{
    std::string  names_file = "/home/szj/darknet/data/coco.names";
    std::string  cfg_file = "/home/szj/YoloTest/train_weight_cfg/yolov2-qrcode.cfg";
    std::string  weights_file = "/home/szj/YoloTest/train_weight_cfg/yolov2-voc_2000.weights";
    std::string filename;

    float const thresh = 0.2;
    Detector detector(cfg_file, weights_file);

    //auto obj_names = objects_names_from_file(names_file);
    VideoCapture capture(0);  
    int count = 0;
    while(1){
        // std::cout<<"count: "<<count<<std::endl;
        char ch;
        count++;
        //cv::Mat mat_img = cv::imread("/home/szj/YoloTest/000009.jpg");
	Mat mat_img;
	capture>>mat_img;
	if(!mat_img.data) return -1;

        std::vector<bbox_t> result_vec = detector.detect(mat_img);
	// std::cout<<result_vec.size()<<std::endl;

        cv::Rect rect = draw_boxes(mat_img,result_vec);

        cv::rectangle(mat_img, rect, cv::Scalar(0,255,255), 2);
        cv::imshow("window name", mat_img);
        cv::waitKey(1);
    }

    return 0;
}
*/
