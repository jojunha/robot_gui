#include <QApplication>
#include <QIcon>
#include "gui.h"


int main(int argc, char *argv[])
{

  ros::init(argc, argv, "GUI");
  QApplication a(argc, argv);


  a.setQuitOnLastWindowClosed(false);

  gui g;
  // set the window title as the node name
  g.setWindowTitle(QString::fromStdString("ROBOT GUI"));

  // load the icon from our qrc file and set it as the application icon
  QIcon icon(":/icons/kist.png");

  g.setWindowIcon(icon);

  g.show();

// a.exec -> event 처리
  return a.exec();
}
