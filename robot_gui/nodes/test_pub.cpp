#include "ros/ros.h"
#include "kist_msgs/arm_state.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_pub");

  ros::NodeHandle n("~");

  ros::Publisher joint_pub = n.advertise<kist_msgs::arm_state>("/arm_states", 100);

  ros::Rate loop_rate(10);

  double count = 0;
  while (ros::ok())
  {

    kist_msgs::arm_state state;
    for(int i = 0; i < 15; i++){
      state.joint_angle[i] = rand() % (10 + i) + 0.1;
      state.joint_velocity[i] = rand() % (20 + i) + 0.2;
      state.joint_torque[i] = rand() % (40 + i) + 0.7;

      state.joint_angle_des[i] = rand() % (10 + i) + 0.1;
    }

    for(int i = 0; i < 6; i++){
      state.left_hand[i] = rand() % (10 + i) + 0.1;
      state.right_hand[i] = rand() % (15 + i) + 0.1;

      state.left_hand_des[i] = rand() % (10 + i) + 0.1;
      state.right_hand_des[i] = rand() % (15 + i) + 0.1;
    }

    state.time = count;
    joint_pub.publish(state);

    ros::spinOnce();

    loop_rate.sleep();
    count += 0.1;
  }


  return 0;
}
