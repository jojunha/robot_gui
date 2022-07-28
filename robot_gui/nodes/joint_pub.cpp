#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "kist_msgs/arm_state.h"


double joint_angle[15];

void StateCB(const kist_msgs::arm_state::ConstPtr& msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joint_pub");
  ros::NodeHandle nh;

  ros::Subscriber angles_sub_ = nh.subscribe<kist_msgs::arm_state>("/arm_states", 1, StateCB);
  ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

  ros::Rate loop_rate(60);


  while (ros::ok())
  {
    sensor_msgs::JointState msg;

    msg.name.push_back("j0");
    msg.name.push_back("j1");
    msg.name.push_back("j2");
    msg.name.push_back("j3");
    msg.name.push_back("j4");
    msg.name.push_back("j5");
    msg.name.push_back("j6");
    msg.name.push_back("j7");
    msg.name.push_back("j8");
    msg.name.push_back("j9");
    msg.name.push_back("j10");
    msg.name.push_back("j11");
    msg.name.push_back("j12");
    msg.name.push_back("j13");
    msg.name.push_back("j14");

    msg.position.push_back(0.0);
    for(int i = 1; i<15; i++){
      msg.position.push_back(joint_angle[i]);
    }

    msg.header.stamp = ros::Time::now();

    joint_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

void StateCB(const kist_msgs::arm_state::ConstPtr& msg){
  for(int i = 0; i < 15; i++){
    joint_angle[i] = msg->joint_angle[i];
  }
}