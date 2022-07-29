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

  void StateCB(const kist_msgs::arm_state::ConstPtr& msg);

  void show_state();
  void show_graph();

  void init_plot();
  void update_plot(int graph_num, double x_, double y_);

  void pub_joint();

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

  kist_msgs::arm_state arm_state;

  bool rviz_mode = false;


public slots:

  void graph_on_clicked();

  void save_clicked();

  void torque_off_clicked();
  void gravity_clicked();
  void initial_clicked();
  void task_clicked();

  void spinOnce();

  void angle_unit_clicked();

};

#endif // GUI_H
