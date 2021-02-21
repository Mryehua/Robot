#ifndef WIDGET_H
#define WIDGET_H
#include <QWidget>
#include <QDebug>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTime>

namespace Ui {
class Widget;
}

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = 0);
    ~Widget();

private slots:
    void Read_Date();       //读取串口数据
    void find_port();       //查找可用串口
    void sleep(int msec);      //延时函数

    void on_send_button_clicked();
    void on_open_port_clicked();
    void on_close_port_clicked();
    void on_clear_button_1_clicked();
    void on_clear_button2_clicked();
    void on_receive_modl_clicked();
    void on_send_modl_clicked();

private:
    Ui::Widget *ui;
    QSerialPort *serialport;
    bool textstate_receive;
    bool textstate_send;
};

#endif // WIDGET_H
