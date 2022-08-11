#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <modbus/joint_state.h>
#include <math.h>
#include <string.h>
//#include <robot_visual/conio.h>
// # define KEY_X 120
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

    // char key = getch();
    // int value = key;
    while (ros::ok()) {

        // if(value == KEY_X)
        // {
            //update joint_state

            //math.radians()
            ros::Duration(0.5).sleep();
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(6);
            joint_state.position.resize(6);
            joint_state.name[0] ="j1";
            joint_state.position[0] = tfRadians(state.joint_angle_1);
            joint_state.name[1] ="j2";
            joint_state.position[1] = tfRadians(state.joint_angle_2+90);
            joint_state.name[2] ="j3";
            joint_state.position[2] = tfRadians(state.joint_angle_3);
            joint_state.name[3] ="j4";
            joint_state.position[3] = tfRadians(-(state.joint_angle_4+90));
            joint_state.name[4] ="j5";
            joint_state.position[4] = tfRadians(-(state.joint_angle_5));
            joint_state.name[5] ="j6";
            joint_state.position[5] = tfRadians(state.joint_angle_6);
            joint_pub.publish(joint_state);
            ros::Duration(0.5).sleep();
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(6);
            joint_state.position.resize(6);
            joint_state.name[0] ="j1";
            joint_state.position[0] = tfRadians(state.joint_angle_1+90);
            joint_state.name[1] ="j2";
            joint_state.position[1] = tfRadians(state.joint_angle_2+90);
            joint_state.name[2] ="j3";
            joint_state.position[2] = tfRadians(state.joint_angle_3+90);
            joint_state.name[3] ="j4";
            joint_state.position[3] = tfRadians(-(state.joint_angle_4+90));
            joint_state.name[4] ="j5";
            joint_state.position[4] = tfRadians(-(state.joint_angle_5));
            joint_state.name[5] ="j6";
            joint_state.position[5] = tfRadians(state.joint_angle_6);
            joint_pub.publish(joint_state);
            ros::Duration(0.5).sleep();
            joint_state.header.stamp = ros::Time::now();
            joint_state.name.resize(6);
            joint_state.position.resize(6);
            joint_state.name[0] ="j1";
            joint_state.position[0] = tfRadians(state.joint_angle_1);
            joint_state.name[1] ="j2";
            joint_state.position[1] = tfRadians(state.joint_angle_2+90);
            joint_state.name[2] ="j3";
            joint_state.position[2] = tfRadians(state.joint_angle_3);
            joint_state.name[3] ="j4";
            joint_state.position[3] = tfRadians(-(state.joint_angle_4+90));
            joint_state.name[4] ="j5";
            joint_state.position[4] = tfRadians(-(state.joint_angle_5));
            joint_state.name[5] ="j6";
            joint_state.position[5] = tfRadians(state.joint_angle_6);

            // update transform
            // (moving in a circle with radius=2)
            // odom_trans.header.stamp = ros::Time::now();
            // odom_trans.transform.translation.x = cos(angle)*2;
            // odom_trans.transform.translation.y = sin(angle)*2;
            // odom_trans.transform.translation.z = .7;
            // odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(angle+M_PI/2);

            //send the joint state and transform
            joint_pub.publish(joint_state);
            // broadcaster.sendTransform(odom_trans);

            // Create new robot state
            // tilt += tinc;
            // if (tilt<-.5 || tilt>0) tinc *= -1;
            // height += hinc;
            // if (height>.2 || height<0) hinc *= -1;
            // swivel += degree;
            // angle += degree/4;

            // This will adjust as needed per iteration
        // }
        
        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}