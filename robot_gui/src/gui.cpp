#include "gui.h"
#include "ui_gui.h"
#include <QDebug>

using namespace std;

gui::gui(QWidget *parent) :
  QWidget(parent),
  ui(new Ui::gui)
{
  ui->setupUi(this);

  ros_timer = new QTimer(this);
  connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
  ros_timer->start(60);  // ROS Commnication Hz

  connect(ui->graph0,SIGNAL(clicked()),this,SLOT(graph_on_clicked()));
  connect(ui->graph1,SIGNAL(clicked()),this,SLOT(graph_on_clicked()));
  connect(ui->graph2,SIGNAL(clicked()),this,SLOT(graph_on_clicked()));

  connect(ui->save,SIGNAL(clicked()),this,SLOT(save_clicked()));

  connect(ui->torque_off,SIGNAL(clicked()),this,SLOT(torque_off_clicked()));
  connect(ui->gravity,SIGNAL(clicked()),this,SLOT(gravity_clicked()));
  connect(ui->initial,SIGNAL(clicked()),this,SLOT(initial_clicked()));
  connect(ui->task,SIGNAL(clicked()),this,SLOT(task_clicked()));

  connect(ui->hand_torque_off,SIGNAL(clicked()),this,SLOT(hand_off_clicked()));
  connect(ui->hand_torque_on,SIGNAL(clicked()),this,SLOT(hand_on_clicked()));
  connect(ui->emergency,SIGNAL(clicked()),this,SLOT(emergency_clicked()));

  connect(ui->radian,SIGNAL(clicked()),this,SLOT(angle_unit_clicked()));
  connect(ui->degree,SIGNAL(clicked()),this,SLOT(angle_unit_clicked()));

  ui->radian->setChecked(true);
  ui->tabWidget->setStyleSheet("background-color: rgb(240, 240, 240)");
  ui->tabWidget->setStyleSheet("QTabBar::tab { background-color: rgb(180,180,180)");
  ui->tabWidget->tabBar()->setStyleSheet("QTabBar::tab:selected {\
                                           color: rgb(0,0,0);\
                                           background-color: rgb(240,240,240);\
                                       }");

  state_sub = nh.subscribe<kist_msgs::arm_state>("/arm_states", 1, &gui::StateCB, this);
  command_pub = nh.advertise<std_msgs::UInt32>("/gui_command", 10);
  joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);

  nh.param<bool>("rviz_mode", rviz_mode, false);

  gui::init_plot();

  sleep(2);
  gui::pub_joint();
  ui->state->setText("Torque OFF");
  ui->state_2->setText("Torque OFF");

}

gui::~gui()
{
  delete ui;
}

void gui::spinOnce(){
  if(ros::ok()){
    ros::spinOnce();
  }
  else{
    QApplication::quit();
  }
}

void gui::angle_unit_clicked(){
  if(ui->radian->isChecked()){
    angle_mode = 0; // radian mode
    cout << "Radian Mode" << endl;
  }
  if(ui->degree->isChecked()){
    angle_mode = 1; // degree mode
    cout << "Degree Mode" << endl;
  }
}

void gui::graph_on_clicked(){
  if(!(ui->graph0->isChecked())){
    cout << "graph0 reset!" << endl;
    g0_x.clear();
    g0_y.clear();
  }
  if(!(ui->graph1->isChecked())){
    cout << "graph1 reset!" << endl;
    g1_x.clear();
    g1_y.clear();
  }
  if(!(ui->graph2->isChecked())){
    cout << "graph2 reset!" << endl;
    g2_x.clear();
    g2_y.clear();
  }
}

void gui::save_clicked(){
  ui->plot->savePng(fileName,  0, 0, 1.0, -1);
}

void gui::torque_off_clicked(){
  std_msgs::UInt32 msg;
  msg.data = 1;
  command_pub.publish(msg);
  ui->state->setText("Torque OFF");
}

void gui::gravity_clicked(){
  std_msgs::UInt32 msg;
  msg.data = 2;
  command_pub.publish(msg);
  ui->state->setText("Gravity Com.");
}

void gui::initial_clicked(){
  std_msgs::UInt32 msg;
  msg.data = 3;
  command_pub.publish(msg);
  ui->state->setText("Initial Pose");
}

void gui::task_clicked(){
  std_msgs::UInt32 msg;
  msg.data = 4;
  command_pub.publish(msg);
  ui->state->setText("Task");
}

void gui::hand_on_clicked(){
  std_msgs::UInt32 msg;
  msg.data = 5;
  command_pub.publish(msg);
  ui->state_2->setText("Torque ON");
}

void gui::hand_off_clicked(){
  std_msgs::UInt32 msg;
  msg.data = 6;
  command_pub.publish(msg);
  ui->state_2->setText("Torque OFF");
}

void gui::emergency_clicked(){
  std_msgs::UInt32 msg;
  msg.data = 7;
  command_pub.publish(msg);
  ui->state->setText("Torque OFF");
  ui->state_2->setText("Torque OFF");
}


void gui::StateCB(const kist_msgs::arm_state::ConstPtr& msg){

  arm_state = *msg; // state update

  show_arm_state();
  show_hand_state();
  show_graph();

  if(_hand_mode != msg->hand_mode)
  {
    _hand_mode = msg->hand_mode;
    ui->state_2->setText(hand_command[_hand_mode]);
  }

  if(rviz_mode){
    pub_joint();
  }

}


void gui::show_arm_state(){

  ui->time->setNum(arm_state.time);

  // radian mode
  if(angle_mode == 0){
    for(int i = 0; i < 15; i++){
      ui->joint_table->setItem(i,0,new QTableWidgetItem(QString::number(arm_state.joint_angle[i])));
      ui->joint_table->item(i,0)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->joint_table->setItem(i,1,new QTableWidgetItem(QString::number(arm_state.joint_velocity[i])));
      ui->joint_table->item(i,1)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->joint_table->setItem(i,2,new QTableWidgetItem(QString::number(arm_state.joint_torque[i])));
      ui->joint_table->item(i,2)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    }

    for(int i = 0; i < 6; i++){
      ui->end_effector->setItem(0,i,new QTableWidgetItem(QString::number(arm_state.left_hand[i])));
      ui->end_effector->item(0,i)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->end_effector->setItem(1,i,new QTableWidgetItem(QString::number(arm_state.right_hand[i])));
      ui->end_effector->item(1,i)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    }
  }
  else{ // degree mode
    for(int i = 0; i < 15; i++){
      ui->joint_table->setItem(i,0,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.joint_angle[i])));
      ui->joint_table->item(i,0)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->joint_table->setItem(i,1,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.joint_velocity[i])));
      ui->joint_table->item(i,1)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->joint_table->setItem(i,2,new QTableWidgetItem(QString::number(arm_state.joint_torque[i])));
      ui->joint_table->item(i,2)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    }

    for(int i = 0; i < 6; i++){
      ui->end_effector->setItem(0,i,new QTableWidgetItem(QString::number(arm_state.left_hand[i])));
      ui->end_effector->item(0,i)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->end_effector->setItem(1,i,new QTableWidgetItem(QString::number(arm_state.right_hand[i])));
      ui->end_effector->item(1,i)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    }

    for(int i = 4; i < 6; i++){
      ui->end_effector->setItem(0,i,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.left_hand[i])));
      ui->end_effector->item(0,i)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->end_effector->setItem(1,i,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.right_hand[i])));
      ui->end_effector->item(1,i)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    }
  }
}


void gui::show_hand_state(){
  // radian mode
  if(angle_mode == 0){
    for(int i = 0; i < 4; i++){
      ui->hand_table->setItem(i,0,new QTableWidgetItem(QString::number(arm_state.hand_angle[i])));
      ui->hand_table->item(i,0)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->hand_table->setItem(i,1,new QTableWidgetItem(QString::number(arm_state.hand_angle[i+4])));
      ui->hand_table->item(i,1)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->hand_table->setItem(i,2,new QTableWidgetItem(QString::number(arm_state.hand_angle[i+8])));
      ui->hand_table->item(i,2)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->hand_table->setItem(i,3,new QTableWidgetItem(QString::number(arm_state.hand_angle[i+12])));
      ui->hand_table->item(i,3)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    }
  }
  else{ // degree mode
    for(int i = 0; i < 4; i++){
      ui->hand_table->setItem(i,0,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.hand_angle[i])));
      ui->hand_table->item(i,0)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->hand_table->setItem(i,1,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.hand_angle[i+4])));
      ui->hand_table->item(i,1)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->hand_table->setItem(i,2,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.hand_angle[i+8])));
      ui->hand_table->item(i,2)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);

      ui->hand_table->setItem(i,3,new QTableWidgetItem(QString::number(radian_to_degree * arm_state.hand_angle[i+12])));
      ui->hand_table->item(i,3)->setTextAlignment(Qt::AlignCenter | Qt::AlignVCenter);
    }
  }
}


void gui::show_graph(){
  if(ui->graph0->isChecked()){
    double error = 0;
    for(int i = 0 ; i < 15; i++){
      error += pow(arm_state.joint_angle_des[i] - arm_state.joint_angle[i],2);
    }
    error /= 15;
    update_plot(0, arm_state.time, error);
  }

  if(ui->graph1->isChecked()){
    double error = 0;
    for(int i = 0 ; i < 6; i++){
      error += pow(arm_state.left_hand_des[i] - arm_state.left_hand[i],2);
    }
    error /= 6;
    update_plot(1, arm_state.time, error);
  }

  if(ui->graph2->isChecked()){
    double error = 0;
    for(int i = 0 ; i < 6; i++){
      error += pow(arm_state.right_hand_des[i] - arm_state.right_hand[i],2);
    }
    error /= 6;
    update_plot(2, arm_state.time, error);
  }
}

void gui::init_plot(){
  ui->plot->addGraph();
  ui->plot->addGraph();
  ui->plot->addGraph();

  ui->plot->xAxis->setLabel("X");
  ui->plot->yAxis->setLabel("Y");

  // legend name
  ui->plot->graph(0)->setPen(QPen(Qt::red));
  ui->plot->graph(0)->setName("Joint Error");

  ui->plot->graph(1)->setPen(QPen(Qt::blue));
  ui->plot->graph(1)->setName("Left Hand");

  ui->plot->graph(2)->setPen(QPen(Qt::black));
  ui->plot->graph(2)->setName("Right Hand");

  ui->plot->xAxis->setRange(0, x_range);
  ui->plot->yAxis->setRange(0, y_range);

  ui->graph0->setChecked(true);
  ui->graph1->setChecked(true);
  ui->graph2->setChecked(true);

  ui->plot->legend->setVisible(true);
  ui->plot->legend->setBrush(QColor(255, 255, 255, 100));

  ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

  ui->plot->replot();
}

void gui::update_plot(int graph_num, double _x, double _y){
  if(graph_num == 0){
    g0_x.append(_x);
    g0_y.append(_y);
    ui->plot->graph(0)->setData(g0_x, g0_y);
  }
  if(graph_num == 1){
    g1_x.append(_x);
    g1_y.append(_y);
    ui->plot->graph(1)->setData(g1_x, g1_y);
  }
  if(graph_num == 2){
    g2_x.append(_x);
    g2_y.append(_y);
    ui->plot->graph(2)->setData(g2_x, g2_y);
  }

  ui->plot->replot();
}

// For rviz
void gui::pub_joint(){
  sensor_msgs::JointState joint_state;

  joint_state.name.push_back("j0");
  joint_state.name.push_back("j1");
  joint_state.name.push_back("j2");
  joint_state.name.push_back("j3");
  joint_state.name.push_back("j4");
  joint_state.name.push_back("j5");
  joint_state.name.push_back("j6");
  joint_state.name.push_back("j7");
  joint_state.name.push_back("j8");
  joint_state.name.push_back("j9");
  joint_state.name.push_back("j10");
  joint_state.name.push_back("j11");
  joint_state.name.push_back("j12");
  joint_state.name.push_back("j13");
  joint_state.name.push_back("j14");

  joint_state.position.push_back(0.0);

  for(int i = 1; i<15; i++){
    joint_state.position.push_back(arm_state.joint_angle[i]);
  }
  joint_state.header.stamp = ros::Time::now();

  joint_pub.publish(joint_state);
}
