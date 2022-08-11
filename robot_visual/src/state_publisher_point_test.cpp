#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <modbus/joint_state.h>
#include <math.h>
#include <string.h>

class arm_state
{
    public:
        float joint_angle_1=0.0, joint_angle_2=-90.0 , joint_angle_3=0.0 , joint_angle_4=-90.0 , joint_angle_5=0.0, joint_angle_6=0.0;
        void joint_Callback(const modbus::joint_state::ConstPtr& state);
};
void arm_state::joint_Callback(const modbus::joint_state::ConstPtr& state)
{
    // ROS_INFO("I heard: [%f, %f, %f, %f, %f, %f]", state->position[0],state->position[1],state->position[2],state->position[3],state->position[4],state->position[5]);
    joint_angle_1 = state->position[0];
    joint_angle_2 = state->position[1];
    joint_angle_3 = state->position[2];
    joint_angle_4 = state->position[3];
    joint_angle_5 = state->position[4];
    joint_angle_6 = state->position[5];
}
void publish_state(ros::Publisher& pub, float joint_angle_1, float joint_angle_2, float joint_angle_3, float joint_angle_4, float joint_angle_5, float joint_angle_6)
{
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "j1";
    joint_state.position[0] = tfRadians(joint_angle_1);
    joint_state.name[1] = "j2";
    joint_state.position[1] = tfRadians(joint_angle_2+90);
    joint_state.name[2] = "j3";
    joint_state.position[2] = tfRadians(joint_angle_3);
    joint_state.name[3] = "j4";
    joint_state.position[3] = tfRadians(-(joint_angle_4+90));
    joint_state.name[4] = "j5";
    joint_state.position[4] = tfRadians(-joint_angle_5);
    joint_state.name[5] = "j6";
    joint_state.position[5] = tfRadians(joint_angle_6);
    pub.publish(joint_state);
    ros::Duration(2).sleep();
}

int main(int argc, char** argv) {
    arm_state state;
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Subscriber sub = n.subscribe("/modbus_joint_states", 1000, &arm_state::joint_Callback, &state);

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(30);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    geometry_msgs::TransformStamped odom_trans;
    sensor_msgs::JointState joint_state;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "axis";

    while (ros::ok()) {
        ros::Duration(2).sleep();
        
        // 1
        ROS_INFO("G1");
        publish_state(joint_pub, 30,-90,0,-90,0,0);
        ROS_INFO("G3");
        publish_state(joint_pub, 150,-90,0,-90,0,0);
        ROS_INFO("G4");
        publish_state(joint_pub, 30,-90,0,-90,0,0); 
        // 1,2
        ROS_INFO("G5");
        publish_state(joint_pub, 30,-110,0,-90,0,0); 
        ROS_INFO("G6");
        publish_state(joint_pub, 150,-70,0,-90,0,0); 
        // 1, 3
        ROS_INFO("G8");
        publish_state(joint_pub, 30,-90,-45,-90,0,0); 
        ROS_INFO("G9");
        publish_state(joint_pub, 150,-90,45,-90,0,0);
        ROS_INFO("G10");
        publish_state(joint_pub, 30,-90,0,-90,0,0); 
        // 1, 2, 3
        ROS_INFO("A1");
        publish_state(joint_pub, 30,-110,20,-90,0,0);
        ROS_INFO("A2");
        publish_state(joint_pub, 150,-70,-20,-90,0,0); 
        // 1, 4
        ROS_INFO("G11");
        publish_state(joint_pub, 30,-110,0,-45,0,0); 
        ROS_INFO("G12");
        publish_state(joint_pub, 150,-70,0,-135,0,0); 
        ROS_INFO("G13");
        publish_state(joint_pub, 30,-90,0,-90,0,0); 
        // 7 字形
        ROS_INFO("B1");
        publish_state(joint_pub, -45,-55,-90,-90,0,0); 
        ROS_INFO("B5");
        publish_state(joint_pub, -45,-55,-90,-30,60,0); 
        ROS_INFO("B2");
        publish_state(joint_pub, -135, -125,90,-90,0,0); 
        ROS_INFO("B6");
        publish_state(joint_pub, -135, -125,90,-150,-60,0); 
        ROS_INFO("B3");
        publish_state(joint_pub, -20,-55,-90,-110, 118.216,0); 
        ROS_INFO("B4");
        publish_state(joint_pub, 45,-55,-90,-30,30,0); 
        ROS_INFO("B7");
        publish_state(joint_pub, 200, -125, 90,-60,-60,0); 
        ROS_INFO("B8");
        publish_state(joint_pub, 130, -125, 90,-140,-52.9,0); 

        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
