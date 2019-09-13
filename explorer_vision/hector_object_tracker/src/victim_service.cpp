#include "victim_service.h"
#include <iostream>
#include <ros/ros.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/PoseWithCovariance.h>
#include "Object.h"
#include "ObjectModel.h"
#include <hector_worldmodel_msgs/Object.h>
#include <tf/transform_listener.h>
#include <string>

#include <tf/tf.h>
#include <list>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <hector_object_tracker/types.h>
#include <hector_worldmodel_msgs/SetObjectState.h>
#include <hector_worldmodel_msgs/SetObjectName.h>
#include <hector_worldmodel_msgs/AddObject.h>
#include <hector_worldmodel_msgs/GetObjectModel.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

VictimService::VictimService() : merged("map")
{
    ROS_INFO("start init victim service!");
    ros::NodeHandle n;
    victimSub = n.subscribe("object_tracker/percept/pose", 10, &VictimService::VictimInfoCallback, this);
    //pose stamp理解为带时间戳的pose
    //pose表示自由空间中姿势的表，由位置和方向组成
    //开启victim会进入hector_object_tracker.cpp

    victimSer = n.advertiseService("get_victim_models", &VictimService::VictimServiceCallback, this);
    camera_left = new hector_object_tracker::ObjectModel("camera_left");
    camera_right = new hector_object_tracker::ObjectModel("camera_right");
    ROS_INFO("out the init function victim_service");
    last_left.x() = 0;
    last_left.y() = 0;
    last_right.x() = 0;
    last_right.y() = 0;
    //merged=new hector_object_tracker::ObjectModel("map");
}

VictimService::~VictimService() {}

void VictimService::VictimInfoCallback(const geometry_msgs::PoseStamped &pose)
{
    ROS_ERROR("get the info of one point");
    hector_worldmodel_msgs::Object ob;
    ob.header = pose.header;
    ob.pose.pose = pose.pose;
    ob.info.class_id = "victim";
    //固定了class_id的具体种类
    std::stringstream ss;
    ss << pose.header.seq;
    ss >> ob.info.object_id;
    ob.info.name = "victim";
    //尝试更改为qrcode标记
    //ob.info.name="qrcode";
    ob.info.support = 1.0;
    ob.state.state = 2;
    hector_object_tracker::Object *obj = new hector_object_tracker::Object(ob);
    /*if(ob.header.frame_id=="camera_left")
        camera_left->add(hector_object_tracker::ObjectPtr(obj));
    else
        camera_right->add(hector_object_tracker::ObjectPtr(obj));*/
    //ObjectList *ls;
    Eigen::Vector2f curr;
    geometry_msgs::Pose temppose;
    hector_object_tracker::ObjectPtr temp;
    try
    {
        temp = hector_object_tracker::ObjectPtr(obj)->transform(tf, "map", ros::Time());
        temp->getPose(temppose);
        //merged.add(temp);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("transform fail for %s", ex.what());
        return;
    }
    curr.x() = temppose.position.x;
    curr.y() = temppose.position.y;
    merged.lock();
    //此处的curx坐标的原点在光心
    //(突然感觉都是在世界坐标系)
    //坐标轴为实际长度
    //dist用来限制同一镜头下两帧之间的差别，是否需要添加此次的标记
    if (ob.header.frame_id == "camera_left")
    {
        ROS_ERROR("left :last x and last y: %f,%f", last_left.x(), last_left.y());
        float dist = (curr.x() - last_left.x()) * (curr.x() - last_left.x()) + (curr.y() - last_left.y()) * (curr.y() - last_left.y());
        ROS_ERROR("distance:%f", dist);
        //if (dist > 1)
        if (dist > 0.4)
        {
            try
            {
                hector_object_tracker::ObjectPtr fuck = hector_object_tracker::ObjectPtr(obj);
                camera_left->add(fuck);
                ROS_WARN("ojbk");
                //merged.merge(fuck,tf,std::string("camera_left"));
                merged.add(temp);
                ROS_WARN("ok");
            }
            catch (...)
            {
                ROS_ERROR("add merged fail!!!!");
                merged.unlock();
                return;
            }
            last_left.x() = curr.x();
            last_left.y() = curr.y();
        }
    }
    else if (ob.header.frame_id == "camera_right")
    {
        ROS_ERROR("right :last x and last y: %f,%f", last_right.x(), last_right.y());
        float dist = (curr.x() - last_right.x()) * (curr.x() - last_right.x()) + (curr.y() - last_right.y()) * (curr.y() - last_right.y());
        ROS_ERROR("distance:%f", dist);
        //if (dist > 1)
        if (dist > 0.45)
        {
            try
            {
                hector_object_tracker::ObjectPtr fuck = hector_object_tracker::ObjectPtr(obj);
                ROS_INFO("");
                camera_right->add(fuck);
                ROS_WARN("ojbk");
                //merged.merge(fuck,tf,std::string("camera_left"));
                //merged.add(temp);
                merged.add(temp);
                ROS_WARN("ok");
            }
            catch (...)
            {
                ROS_ERROR("add merged fail!!!!");
                merged.unlock();
                return;
            }
            last_right.x() = curr.x();
            last_right.y() = curr.y();
        }
    }
    else
    {
        ROS_ERROR("frame error!!!");
        return;
    }

    /* 
    //if(ob.header.frame_id=="camera/color")
    if(ob.header.frame_id=="camera_left")
    {
        ROS_ERROR("left :last x and last y: %f,%f",last_left.x(),last_left.y());
        float dist=(curr.x()-last_left.x())*(curr.x()-last_left.x())+(curr.y()-last_left.y())*(curr.y()-last_left.y());
        ROS_ERROR("distance:%f",dist);
        //if(dist>1)
        //if(dist>0.5)
        if(dist>0.6)
        //这个地方设置了最近的距离限制！！！！！！！！！！！！！！！！！！！！！！
        //if(1)
        {
            try
            {
            hector_object_tracker::ObjectPtr fuck=hector_object_tracker::ObjectPtr(obj);
            camera_left->add(fuck);
            ROS_WARN("ojbk");
            //merged.merge(fuck,tf,std::string("camera_left"));
            merged.add(temp);
            ROS_WARN("ok");
            }
            catch(...){
                ROS_ERROR("add merged fail!!!!");
                merged.unlock();
                return;
            }
            last_left.x()=curr.x();
            last_left.y()=curr.y();
        }
    }
    else {
        ROS_ERROR("frame error!!!");
        return;
        }
*/

    merged.unlock();
    //int num=camera_right->modelsize()+camera_left->modelsize();
    ROS_ERROR("the size of model is %d", merged.modelsize());
}

bool VictimService::VictimServiceCallback(hector_worldmodel_msgs::GetObjectModel::Request &request, hector_worldmodel_msgs::GetObjectModel::Response &response)
{
    /*oldObjs.push_back(*merged);
    merged=new hector_object_tracker::ObjectModel("map");
    //merged.initMerged();
    //ROS_INFO("the size before merged is %d",merged->modelsize());
    ROS_ERROR("output the all victim info");
    merged->mergeWith(*camera_left,tf,std::string("camera_left"));
    merged->mergeWith(*camera_right,tf,std::string("camera_right"));
    ROS_INFO("the size of output model is %d",merged->modelsize());
    merged->getMessage(response.model);
    int num=camera_right->modelsize()+camera_left->modelsize();
    ROS_INFO("the size of model is %d",num);
    return true;*/
    merged.lock();
    ROS_ERROR("output the all victim info");
    ROS_ERROR("the size of output model is %d", merged.modelsize());
    merged.getMessage(response.model);
    //merge同时存储了左摄和右摄的object相关信息
    merged.unlock();
    return true;
}

int main(int argc, char **argv)
{
    ROS_INFO("start victim_service!");
    ros::init(argc, argv, "victim_service");
    VictimService vc;
    ros::spin();
}