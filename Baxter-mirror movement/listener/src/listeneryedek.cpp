#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>


int main(int argc, char** argv) {


ros::init(argc, argv, "my_skeleton_tf_listener");

ros::NodeHandle node;

// publisher declaration
ros::Publisher torso_joint = node.advertise<geometry_msgs::Point>("torso_joint", 1);

ros::Publisher left_hand_joint = node.advertise<geometry_msgs::Point>("left_hand_joint", 1);

ros::Publisher right_hand_joint = node.advertise<geometry_msgs::Point>("right_hand_joint", 1);


// listener
tf::TransformListener listener;

ros::Rate rate(50.0); // frequency of operation

while (node.ok())
{
    // Transforms declared for each joint
    tf::StampedTransform  transform_torso, transform_left_hand, transform_right_hand;
    try
    {
        // each joint frame to reference frame transforms

        listener.lookupTransform("/torso_1", "/openni_depth_frame",ros::Time(0), transform_torso);

        listener.lookupTransform("/left_hand_1", "/openni_depth_frame",ros::Time(0), transform_left_hand);

        listener.lookupTransform("/right_hand_1", "/openni_depth_frame",ros::Time(0), transform_right_hand);

    }
        catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.10).sleep();
        continue;
    }

    // geometry points declaration for storing 3D coordinates of joints and then published later
    geometry_msgs::Point  torso_pose, left_hand_pose, right_hand_pose,baxter_right_hand,baxter_left_hand ;

    // joint position extraction and store

    // torso joint
    torso_pose.x = transform_torso.getOrigin().x();
    torso_pose.y = transform_torso.getOrigin().y();
    torso_pose.z = transform_torso.getOrigin().z();

    // left hand joint
    left_hand_pose.x = transform_left_hand.getOrigin().x();
    left_hand_pose.y = transform_left_hand.getOrigin().y();
    left_hand_pose.z = transform_left_hand.getOrigin().z();

    // right hand joint
    right_hand_pose.x = transform_right_hand.getOrigin().x();
    right_hand_pose.y = transform_right_hand.getOrigin().y();
    right_hand_pose.z = transform_right_hand.getOrigin().z();
    double  scale=0.3;

    // joint positions publish
    baxter_right_hand.x=(0.5)*(right_hand_pose.z-(torso_pose.z));
    baxter_right_hand.y=(-0.5)*(right_hand_pose.x-(torso_pose.x));
    baxter_right_hand.z=(0.5)*(right_hand_pose.y-(torso_pose.y+0.6));

    baxter_left_hand.x=(0.5)*(left_hand_pose.z-(torso_pose.z));
    baxter_left_hand.y=(0.5)*(left_hand_pose.x-(torso_pose.x+0.6));
    baxter_left_hand.z=(0.5)*(left_hand_pose.y-(torso_pose.y+0.6));

    if (baxter_left_hand.y<-1) {
      baxter_left_hand.y=-1;
    }

    if (baxter_right_hand.y<-1) {
      baxter_right_hand.y=-1;
    }


    torso_joint.publish(torso_pose);

    left_hand_joint.publish(baxter_left_hand);

    right_hand_joint.publish(baxter_right_hand);


    rate.sleep();
}

return 0;

}
