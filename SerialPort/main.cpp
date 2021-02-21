#include "widget.h"
#include <QApplication>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);


    Widget w;
    w.show();

    return a.exec();
}
