#include "mainwindow.h"
#include "Client.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //MainWindow w;
   // w.show();
    Client c;
    c.show();
    return a.exec();
}
