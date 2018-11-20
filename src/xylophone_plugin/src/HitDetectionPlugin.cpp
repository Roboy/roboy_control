
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <iostream>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <functional>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3.h"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

//TF stuff
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <common_utilities/rviz_visualization.hpp>


using namespace gazebo;

class HitDetectionPlugin : public ModelPlugin, public rviz_visualization {

public:
    HitDetectionPlugin() : ModelPlugin() {

        //printf("Hello World!\n");

    }

public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) {

        std::cerr << "\nThe Hit Detection Plugin plugin is attached to model[" <<
                  _model->GetName() << "]\n";
///////////////////////////////
        if (_model->GetJointCount() == 0) {
            std::cerr << "Invalid joint count, Hit Detection plugin not loaded\n";
            return;
        }

        this->model = _model;

        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "HitDetection",
                      ros::init_options::NoSigintHandler);
        }
        nh.reset(new ros::NodeHandle());
        hit_detection_pub = nh->advertise<std_msgs::String>("xylophone/hitdetection", 1);
        boost::shared_ptr<ros::AsyncSpinner> spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
        spinner->start();

        physics::Joint_V joints = model->GetJoints();
//        for (auto joint : joints) {
//            prevVelocity[joint->GetName()] = 0.0;
//            prevTime[joint->GetName()] = 0.0;
//        }

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&HitDetectionPlugin::OnUpdate, this, _1));


        this->model->SetGravityMode(0);
        ROS_WARN("hit detection ready, hello");
        last_update = ros::Time::now();
    }


public:
    void OnUpdate(const common::UpdateInfo & /*_info*/) {
        std::string model_name = model->GetName();
        physics::Link_V links = model->GetLinks();

        if((ros::Time::now()-last_update).toSec()>0.01) {
            tf::Transform transform;
            int i = 0;
            for (auto link : links) {
                std::string link_name = link->GetName();
                auto linkPose = link->GetWorldPose();
                transform.setOrigin(tf::Vector3(linkPose.pos.x, linkPose.pos.y, linkPose.pos.z));
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", link_name));
                Eigen::Vector3d pos(linkPose.pos.x, linkPose.pos.y, linkPose.pos.z);
                Eigen::Quaterniond quat(linkPose.rot.w, linkPose.rot.x, linkPose.rot.y, linkPose.rot.z);
                quat.normalize();
                publishMesh("xylophone_plugin", "xylophone/meshes/CAD", (link_name + ".stl").c_str(), pos, quat, 0.001,
                            "world", "xyl", i++, 1, COLOR(1,1,1,0.8));
                // ROS_INFO_STREAM("  " << link_name<< " "<<link->GetWorldPose());
            }
            last_update = ros::Time::now();
            ROS_INFO_STREAM_THROTTLE(1,"here");
        }

        physics::Joint_V joints = model->GetJoints();
        for (auto joint : joints) {
            std::string joint_name = joint->GetName();
            auto accel = GetAcceleration(joint);
            if (accel > 0.1) {
                std_msgs::String msg;
                msg.data = joint_name;

                hit_detection_pub.publish(msg);
            }
        }
    }

public:
    double GetAcceleration(physics::JointPtr joint) {
        double current_time = this->model->GetWorld()->GetSimTime().Double();
        double current_vel = joint->GetVelocity(0);
//        double accel = (current_vel - prevVelocity[joint->GetName()]) / (current_time - prevTime[joint->GetName()]);
//        prevVelocity[joint->GetName()] = current_vel;
//        prevTime[joint->GetName()] = current_time;
        return 0;

    }
    ros::Time last_update;

private:
    event::ConnectionPtr updateConnection;
    physics::ModelPtr model;
    ros::NodeHandlePtr nh;
    ros::Publisher hit_detection_pub;


private:

    //subscribes to tf broadcaster (?)
    tf::TransformListener listener;
    tf::TransformBroadcaster br;
};

GZ_REGISTER_MODEL_PLUGIN(HitDetectionPlugin)