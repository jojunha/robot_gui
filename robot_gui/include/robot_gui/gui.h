#ifndef GUI_H
#define GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <cmath>

#include "kist_msgs/arm_state.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/UInt32.h>

#define degree_to_radian (M_PI/180)
#define radian_to_degree (180/M_PI)


namespace Ui {
class gui;
}

class gui : public QWidget
{
  Q_OBJECT

public:
  explicit gui(QWidget *parent = nullptr);
  ~gui();

private:
  Ui::gui *ui;
  QTimer *ros_timer;

  ros::NodeHandle nh;

  ros::Subscriber state_sub; // robot state sub
  ros::Publisher command_pub; // control command pub
  ros::Publisher joint_pub; // joint angles pub to robot_state_publisher

  QString fileName = "/home/kist/catkin_ws/src/robot_gui/saved_graph/save.png"; // save directory

  QVector<double> g0_x, g0_y;
  QVector<double> g1_x, g1_y;
  QVector<double> g2_x, g2_y;

  const int x_range = 100;  // xAxis range
  const int y_range = 5;  // yAxis range

  int angle_mode = 0; // angle unit defalut=0
                      // 0->radian, 1->degree

  kist_msgs::arm_state arm_state; // recieve topic

  bool rviz_mode = false; 

  char hand_command[14][20] = {"zero torque", "4 finger grasp", "3 finger grasp", "2 finger grasp", "envelop grasp", "5", "6", 
                            "7", "8","9","home","current", "grasp ready", "grasp like motion"}; // hand command
                            
  int _hand_mode = 0; // index of hand_command

  void StateCB(const kist_msgs::arm_state::ConstPtr& msg); // robot state callback function

  void show_arm_state();
  void show_hand_state();
  void show_graph();

  void init_plot();
  void update_plot(int graph_num, double x_, double y_);

  void pub_joint();

public slots:

  void graph_on_clicked();

  void save_clicked();

  void torque_off_clicked();
  void gravity_clicked();
  void initial_clicked();
  void task_clicked();

  void hand_on_clicked();
  void hand_off_clicked();
  void emergency_clicked();

  void angle_unit_clicked(); // radian, degree

  void spinOnce(); // for ROS callback


};

#endif // GUI_H
