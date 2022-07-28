#include <QApplication>
#include <QIcon>
#include "gui.h"


int main(int argc, char *argv[])
{

  ros::init(argc, argv, "GUI",ros::init_options::AnonymousName);
  QApplication a(argc, argv);

  a.setQuitOnLastWindowClosed(false);

  gui d;

  // set the window title as the node name
  d.setWindowTitle(QString::fromStdString("ROBOT GUI"));

  // load the icon from our qrc file and set it as the application icon
  QIcon icon(":/icons/kist.png");

  d.setWindowIcon(icon);

  d.show();

// a.exec -> event 처리
  return a.exec();
}
