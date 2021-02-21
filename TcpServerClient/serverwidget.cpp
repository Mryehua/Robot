#include "serverwidget.h"
#include "ui_serverwidget.h"

ServerWidget::ServerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ServerWidget)
{
    ui->setupUi(this);
    this->setWindowTitle("服务端");
    tcpserver = NULL;
    tcpsocket = NULL;
    //指定父对象，便于回收内存
    tcpserver =new QTcpServer(this);


    tcpserver->listen(QHostAddress::Any,8888);
    connect(tcpserver,&QTcpServer::newConnection,
            [=]()
    {
        //去除建立好连接的套接字
        tcpsocket = tcpserver->nextPendingConnection();
        //显示对方ip、端口信息
        QString IP=tcpsocket->peerAddress().toString();
        qint16 port =tcpsocket->peerPort();
        QString temp =QString("[%1:%2]:成功连接").arg(IP).arg(port);
        ui->textEditRead->setText(temp);

        connect(tcpsocket,&QTcpSocket::readyRead,
                [=]()
        {
            //从通信的套接字中取出内容
            QByteArray array = tcpsocket->readAll();
            ui->textEditRead->append(array); //防止数据被覆盖
        }
        );

    }
    );


}

ServerWidget::~ServerWidget()
{
    delete ui;
}

void ServerWidget::on_pushButtonsend_clicked()
{
    if (NULL==tcpsocket)
    {
        return;
    }
    //获取编辑区的内容
    QString str = ui->textEditWrite->toPlainText();
    //向获取的套接字中的ip与端口发送编辑区的内容
    tcpsocket->write(str.toUtf8().data());

}

void ServerWidget::on_pushButtonclose_clicked()
{
    if (NULL==tcpsocket)
    {
        return;
    }
    //主动和客户端断开连接
    tcpsocket->disconnectFromHost();
    tcpsocket->close();
    tcpsocket =NULL;
}
