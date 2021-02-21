#ifndef SERVERWIDGET_H
#define SERVERWIDGET_H

#include <QWidget>
#include<QTcpSocket>  //通信套接字
#include<QTcpServer>  //监听套接字

namespace Ui {
class ServerWidget;
}

class ServerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ServerWidget(QWidget *parent = nullptr);
    ~ServerWidget();

private slots:
    void on_pushButtonsend_clicked();

    void on_pushButtonclose_clicked();

private:
    Ui::ServerWidget *ui;
    QTcpServer *tcpserver;
    QTcpSocket *tcpsocket;
};

#endif // SERVERWIDGET_H
