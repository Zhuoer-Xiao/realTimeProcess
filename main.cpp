#include "realTimeProcess.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    realTimeProcess w;
    w.setWindowState(Qt::WindowMaximized);
    w.setWindowTitle(u8"实时三维点云处理系统");
    w.show();
    return a.exec();
}
